"""Supervisor (Hito 3): estados explicitos y el UNICO modulo que llama a Move y StopMove.

    ESPERA --(n_seguir medidas buenas)--> SEGUIMIENTO <--> LINEA_PERDIDA --(t_perdida_s)--> PARADA
    SEGUIMIENTO --(barra alcanzada)--> FIN
    SEGUIMIENTO --(esquina, solo nivel 4)--> GIRO_ESQUINA --(90 grados y linea vista)--> SEGUIMIENTO
    cualquiera --(vigilante, FSM, tiempo maximo, Ctrl+C, excepcion)--> PARADA

Se llama una vez por ciclo de ordenes (20 Hz) con paso(t, edad_fotograma). En cada ciclo pide el
estado de la linea a la estimacion, decide, manda (o en simulacro solo anota) la orden y le dice a la
estimacion que vx se mando. En simulacro recorre los mismos estados y deja la orden que habria
mandado, pero no llama nunca a Move ni a StopMove (ni siquiera crea el LocoClient).

Fin (6.4.4): la barra se mide mientras se ve (hasta ~0.3 m) y despues se cuenta con la estima de
avance; se manda StopMove cuando, con lo que el robot anda al frenar (v x t_frenado_s), la puntera
quedaria `fin_objetivo_m` mas alla del borde cercano de la barra. El PDF admite pasarse hasta 30 cm.
Con la barra (o la esquina del nivel 4) a menos de `final_ciego_m`, no ver la linea es lo esperado y no
cuenta como perdida.

Esquina (6.4.5, nivel 4): cuando el centro de giro llega a la esquina (contando el frenado), StopMove,
espera a que pare, gira 90 grados en el sitio sobre la IMU, olvida la linea vieja y vuelve a
SEGUIMIENTO cuando ve la nueva; si no la ve en `t_perdida_s`, PARADA.
"""

import math

from .estimacion import envolver
from .mensajes import Decision, Orden

ESPERA = "ESPERA"
SEGUIMIENTO = "SEGUIMIENTO"
LINEA_PERDIDA = "LINEA_PERDIDA"
GIRO_ESQUINA = "GIRO_ESQUINA"
FIN = "FIN"
PARADA = "PARADA"
TERMINALES = (FIN, PARADA)


class Supervisor:
    def __init__(self, cfg, robot, vigilante, control, estimador, nivel=1, simulacro=True):
        s = cfg["supervisor"]
        self.cfg = s
        self.robot = robot
        self.vigilante = vigilante
        self.control = control
        self.estimador = estimador
        self.nivel = nivel
        self.simulacro = simulacro
        self.estado = ESPERA
        self.motivo = ""
        self.t_inicio = None
        self.t_estado = None
        self.aviso_externo = None       # p. ej. el hilo de la FSM: motivo de parada
        self.tiempos = {}               # s en cada estado
        self.transiciones = []          # (t, de, a, motivo)
        self.ordenes_enviadas = 0
        self._t_ultimo = None
        self._giro = None
        self._parado = False

    @property
    def terminado(self):
        return self.estado in TERMINALES

    # --- transiciones -----------------------------------------------------------------------------
    def empezar(self, t):
        self.t_inicio = self.t_estado = self._t_ultimo = t
        self._cambiar(t, ESPERA, "SEGUIR")

    def _cambiar(self, t, nuevo, motivo):
        if nuevo != self.estado or not self.transiciones:
            self.transiciones.append((round(t - self.t_inicio, 3), self.estado, nuevo, motivo))
        self.estado, self.motivo, self.t_estado = nuevo, motivo, t

    def detener(self, t, motivo):
        """Parada desde fuera (Ctrl+C, excepcion): StopMove y PARADA."""
        if not self.terminado:
            self._cambiar(t, PARADA, motivo)
        self._stop()
        return self._decision(Orden(), {})

    def _stop(self):
        if self._parado:
            return
        self._parado = True
        if not self.simulacro and getattr(self.robot, "marcha_habilitada", False):
            self.robot.parar()

    def quieto(self):
        """Velocidad 0 otra vez (StopMove es SetVelocity(0, 0, 0) con 1 s de duracion): tras FIN o PARADA,
        para mantener el robot quieto y de pie aunque se pierda un mensaje. Nunca en simulacro."""
        if not self.simulacro and getattr(self.robot, "marcha_habilitada", False):
            return self.robot.parar()
        return None

    def _mover(self, orden):
        self._parado = False
        if not self.simulacro:
            self.robot.mover(orden.vx, orden.vy, orden.vyaw)   # dura 1 s: si esto muere, el robot para solo
            self.ordenes_enviadas += 1

    def _decision(self, orden, detalle):
        return Decision(estado=self.estado, motivo=self.motivo, orden_enviada=orden, simulacro=self.simulacro,
                        detalle=detalle)

    # --- ciclo -------------------------------------------------------------------------------------------
    def paso(self, t, edad_fotograma):
        if self._t_ultimo is not None:
            self.tiempos[self.estado] = self.tiempos.get(self.estado, 0.0) + (t - self._t_ultimo)
        self._t_ultimo = t
        if self.terminado:
            self._stop()
            return self._decision(Orden(), {})
        est = self.estimador.estado(t, self.control.L)
        detalle = {"est": est}

        motivo = self.vigilante.motivo_parada(edad_fotograma)
        if motivo is None and self.aviso_externo:
            motivo = self.aviso_externo
        if motivo is None and t - self.t_inicio > self.cfg["duracion_max_s"]:
            motivo = f"duracion maxima ({self.cfg['duracion_max_s']:.0f} s)"
        if motivo:
            self._cambiar(t, PARADA, motivo)
            self._stop()
            self.estimador.orden(t, 0.0)
            return self._decision(Orden(), detalle)

        orden = Orden()
        if self.estado == ESPERA:
            orden = self._espera(t, est)
        elif self.estado in (SEGUIMIENTO, LINEA_PERDIDA):
            orden, extra = self._seguir(t, est)
            detalle.update(extra)
        elif self.estado == GIRO_ESQUINA:
            orden, extra = self._esquina(t, est)
            detalle.update(extra)

        if self.terminado or self.estado == ESPERA:
            self._stop()
            orden = Orden()
        elif orden is None:                 # StopMove pedido por el propio estado (frenar en la esquina)
            self._stop()
            orden = Orden()
        else:
            self._mover(orden)
            detalle["enviada"] = not self.simulacro
        self.estimador.orden(t, orden.vx)
        return self._decision(orden, detalle)

    def _espera(self, t, est):
        s = self.cfg
        if est.hay_linea and est.edad_s < 0.2 and est.racha >= s["n_seguir"] and est.confianza >= s["conf_seguir"]:
            self.control.reiniciar()
            self._cambiar(t, SEGUIMIENTO, f"linea vista (confianza {est.confianza:.2f})")
        elif t - self.t_inicio > s["t_inicio_max_s"]:
            self._cambiar(t, PARADA, f"sin linea clara {s['t_inicio_max_s']:.0f} s despues de SEGUIR")
        return Orden()

    def _seguir(self, t, est):
        s = self.cfg
        # fin: la puntera quedaria fin_objetivo_m mas alla de la barra tras frenar
        if est.dist_fin is not None and est.v * s["t_frenado_s"] - est.dist_fin >= s["fin_objetivo_m"]:
            self._cambiar(t, FIN, f"barra: {est.dist_fin:+.2f} m de la puntera a {est.v:.2f} m/s")
            return None, {}
        # esquina (nivel 4): el centro de giro, sobre la esquina tras frenar
        if self.nivel == 4 and est.esquina is not None:
            d_centro = est.esquina[0] + self.estimador.x_puntera + self.estimador.d_giro
            if est.v * s["t_frenado_s"] >= d_centro:
                self._giro = {"fase": "frenando", "t": t, "sentido": est.esquina[1]}
                self._cambiar(t, GIRO_ESQUINA, f"esquina a {est.esquina[0]:+.2f} m, sentido {est.esquina[1]:+d}")
                return None, {}
        final_ciego = (est.dist_fin is not None and est.dist_fin < s["final_ciego_m"]) or \
            (self.nivel == 4 and est.esquina is not None and est.esquina[0] < s["final_ciego_m"])
        if final_ciego and est.edad_s > s["t_final_max_s"]:
            self._cambiar(t, PARADA, f"tramo final a ciegas de mas de {s['t_final_max_s']:.0f} s")
            return None, {}
        perdida = est.edad_s > s["t_aviso_s"] and not final_ciego
        if self.estado == SEGUIMIENTO and perdida:
            self._cambiar(t, LINEA_PERDIDA, f"sin medida buena desde hace {est.edad_s:.2f} s")
        elif self.estado == LINEA_PERDIDA and not perdida:
            self._cambiar(t, SEGUIMIENTO, "linea recuperada" if not final_ciego else "final a ciegas hacia la barra")
        if self.estado == LINEA_PERDIDA and est.edad_s > s["t_perdida_s"]:
            self._cambiar(t, PARADA, f"linea perdida mas de {s['t_perdida_s']:.1f} s")
            return None, {}
        factor = s["vx_perdida"] if self.estado == LINEA_PERDIDA else 1.0
        orden, extra = self.control.calcular(t, est, factor_vx=factor)
        return orden, extra

    def _esquina(self, t, est):
        s, g = self.cfg, self._giro
        if g["fase"] == "frenando":
            if t - g["t"] >= s["t_parado_s"]:
                g.update(fase="girando", yaw0=self.robot.imu().yaw, t=t, t_estable=None)
                g["yaw_obj"] = envolver(g["yaw0"] + g["sentido"] * math.pi / 2)
            return None, {"giro": "frenando"}
        if g["fase"] == "girando":
            err = envolver(g["yaw_obj"] - self.robot.imu().yaw)
            if abs(err) < math.radians(s["giro_esquina_tol_deg"]):
                g["t_estable"] = g["t_estable"] or t
                if t - g["t_estable"] >= s["giro_esquina_estable_s"]:
                    g.update(fase="buscando", t=t)
                    self.estimador.reiniciar()
                    self.control.reiniciar()
                    return None, {"giro": "hecho", "error_deg": math.degrees(err)}
                return None, {"giro": "estable", "error_deg": math.degrees(err)}
            g["t_estable"] = None
            orden = self.control.girar(t, err, s["giro_esquina_kp"], s["giro_esquina_min"])
            return orden, {"giro": "girando", "error_deg": math.degrees(err)}
        # buscando la linea nueva
        if est.hay_linea and est.racha >= s["n_seguir"] and est.confianza >= s["conf_seguir"]:
            self.control.reiniciar()
            self._cambiar(t, SEGUIMIENTO, "linea nueva tras la esquina")
        elif t - g["t"] > s["t_perdida_s"]:
            self._cambiar(t, PARADA, "sin linea tras girar en la esquina")
        return None, {"giro": "buscando"}
