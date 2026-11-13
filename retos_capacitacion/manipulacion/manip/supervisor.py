"""Supervisor (seccion 5 del PDF): maquina de estados explicita, unico que cambia el peso de arm_sdk y el unico
que da un objeto por terminado; ejecuta la salida segura.

    ESPERA -> ENTRADA (rampa de 12 s) -> A_OBSERVACION (los dos brazos, en espejo)
      -> por cada diana u objeto: OBSERVAR -> PLANIFICAR -> MOVER -> DESCENSO -> (TOCAR | CERRAR -> TRANSPORTE
         -> SOLTAR) -> VOLVER
         agrupados por brazo: todos los de un brazo y luego los del otro (el que espera, en la observacion)
      -> SALIDA (observacion -> reposo, peso a 0 en >= 5 s) -> FIN
    cualquier estado con peso -> SALIDA_SEGURA: congelar, abrir la mano, subir a la altura de seguridad, salir
      por la pose de observacion al reposo y solo entonces bajar el peso.
La salida por la observacion es una ampliacion de la del PDF ("subirlo a la altura de seguridad y solo
entonces bajar el peso a 0"): en el gemelo, ir al reposo desde encima de la mesa barre su borde, y con el peso
en 0 la politica devuelve el brazo a su postura por donde quiera (lo advierte el propio PDF). Se puede quitar
con supervisor.salida_por_observacion: false.

Percepcion nueva antes de cada agarre (nivel 3: el plan no se calcula una sola vez). En simulacro el brazo no
publica en rt/arm_sdk (brazo.py) y la mano abre y cierra en el aire.
"""

import math
import time

import numpy as np

from .cinematica import TareaIK, Banda, T_punto
from .geometria import aplicar
from .mensajes import JUNTAS_BRAZO, LADOS, Parada, Plan, PuntoPaso
from .planificacion import centro_agarre


# puntos de paso a la altura de seguridad o de transporte: su tramo desde la observacion (o hacia ella) es limpio
PASOS_ALTOS = ("sobre", "subir", "levantar", "sobre_bandeja", "retirar_de_bandeja")


class Contacto(Exception):
    """La yema toco la diana: se para el descenso."""


class Supervisor:
    def __init__(self, cfg, robot, brazo, manos, percepcion, planificador, fuente, vigilante, registro,
                 nivel=1, simulacro=False, dianas=None, asignacion=None, detector_dianas=None, log=print):
        self.cfg, self.robot, self.brazo, self.manos = cfg, robot, brazo, manos
        self.per, self.pl, self.fuente, self.vig, self.reg = percepcion, planificador, fuente, vigilante, registro
        self.nivel, self.simulacro = nivel, simulacro
        self.dianas = dianas or []
        self.asignacion = asignacion or {}
        self.detector_dianas = detector_dianas
        self.log = log
        self.estado = "ESPERA"
        self.motivo_estado = ""
        self._plan_activo = None      # el plan que se esta ejecutando: la salida segura sale por sus puntos altos
        self.al_estado = []           # funciones f(estado) a cada cambio de estado (pruebas: --parada-en)
        self.resultados = []
        self.c = cfg["contacto"]
        self.s = cfg["supervisor"]
        self._ultimo_fotograma = None

    # --- utilidades -----------------------------------------------------------------------------------
    def _a(self, estado, motivo="", detalle=None):
        self.estado, self.motivo_estado = estado, motivo
        self.reg.evento(estado, motivo, detalle)
        for f in self.al_estado:
            f(estado)

    def q(self, lado):
        return self.robot.q()[JUNTAS_BRAZO[lado]]

    def comprobar(self):
        self.vig.comprobar()

    def contexto_registro(self):
        giro = getattr(self.vig, "giro", None)
        return {"estado": self.estado, "peso": self.brazo.peso, "obj": self.brazo.consigna(),
                "deriva_yaw": giro.deriva() if giro is not None and giro.imu0 is not None else None}

    def _mover(self, lado, punto, extra=None, al_parar="congelar"):
        """Un punto de paso con el brazo `lado`; `extra` se llama en cada ciclo (contacto, sujetar...)."""
        def chequeo():
            self.comprobar()
            if extra:
                extra()
        self.brazo.ir_a(self.brazo.pose_brazo(lado, punto.q), punto.duracion, chequeo, al_parar=al_parar)

    def _guiar(self, lado, guia, altura, fin, extra=None, cada=0.02, integrar=True):
        """La yema por la recta de `guia` (manip/guia.py) en lazo cerrado: cada `cada` s la consigna (fijar, sin
        trayectoria) es la postura de la recta a altura(t) mas la correccion que integra (con integrar) lo que la
        medida se aparta.
        Acaba con fin(t) True (devuelve True) o si arnes.py no acepta fijar (False: con --arnes, un arnes.py anterior
        al despliegue). extra(): en cada ciclo (el contacto, que lo corta con una excepcion)."""
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            self.comprobar()
            if fin(t):
                return True
            if extra:
                extra()
            q = guia.consigna(altura(t), self.q(lado), cada, integrar)
            if not self.brazo.fijar(self.brazo.pose_brazo(lado, q)):
                self.log(f"  OJO: no se pudo fijar la consigna ({getattr(self.brazo, 'ultimo_rechazo', None)}): el toque "
                         "sigue sin guiar (con --arnes: un arnes.py anterior al despliegue no conoce 'fijar'; reinicialo)")
                return False
            time.sleep(max(0.0, cada - (time.monotonic() - t0 - t)))

    def _abrir_manos(self, lados=None):
        """Las manos abiertas del todo (los seis DOF a 1000: la mano abierta del gemelo, con la que se comprueban
        la entrada, la observacion y la salida). Al empezar la tirada (si no, heredan la postura de la anterior:
        2026-11-11, el indice derecho siguio extendido toda la prueba) y al volver de tocar."""
        lados = list(lados or self.manos)
        for lado in lados:
            self.manos[lado].preforma([1000] * 6, vigilar=self.comprobar)
            self.pl.mano_gemelo(lado, [1000] * 6)
        self._a("MANOS", f"abiertas: {', '.join(lados)}")

    def _ambos(self, planes, comprobar=None):
        """Mueve los dos brazos a la vez por sus puntos de paso, uno a uno (entrada y salida en espejo: los planes
        tienen los mismos puntos; el reposo <-> observacion, el de paso_inicio si lo hay)."""
        for k in range(max(len(p.puntos_de_paso) for p in planes)):
            pose, dur = {}, 0.0
            for p in planes:
                if k < len(p.puntos_de_paso):
                    pose.update(self.brazo.pose_brazo(p.mano, p.puntos_de_paso[k].q))
                    dur = max(dur, p.puntos_de_paso[k].duracion)
            self.brazo.ir_a(pose, dur, comprobar or self.comprobar)

    # --- percepcion -----------------------------------------------------------------------------------
    def _fotogramas(self, n):
        """Hasta n fotogramas seguidos y distintos (el primero, fresco); como mucho ~1.5 s."""
        f = self.fuente.fresco(timeout_s=3.0)
        if f is None:
            return []
        fs, limite = [f], time.monotonic() + 1.5
        while len(fs) < n and time.monotonic() < limite:
            g = self.fuente.siguiente(0.5)
            if g is None:
                break
            if g is fs[-1] or g.n == fs[-1].n:      # el lector continuo devuelve el ultimo hasta que llega otro (otro n:
                                                     # con la profundidad parada, el color seguia con el mismo n, §68)
                time.sleep(0.01)
                continue
            fs.append(g)
        return fs

    def observar(self):
        fs = self._fotogramas(int(self.cfg["percepcion"].get("fotogramas_observar", 1)))
        if not fs:
            raise Parada("camara sin fotogramas al observar")
        f = fs[-1]
        self._ultimo_fotograma = f
        self._n_observar = getattr(self, "_n_observar", 0) + 1
        self.reg.imagen(f"observar_{self._n_observar}", f.color)        # lo que vio (§55: sin ella no se sabia por que)
        # y su IR y su profundidad (16 bits, mm): el prisma salia o no segun la profundidad de su tapa (§75)
        if getattr(f, "ir", None) is not None:
            self.reg.imagen(f"observar_{self._n_observar}_ir", f.ir)
        if getattr(f, "prof", None) is not None:
            self.reg.imagen(f"observar_{self._n_observar}_prof", f.prof)
        q = self.robot.q()
        esc = self.per.procesar_varios(fs, {l: q[JUNTAS_BRAZO[l]] for l in LADOS})
        self.pl.poner_escena(esc)
        # el yaw, en grados (modulo la simetria de la clase): §57, sin el no se sabia con que giro se planifico
        self._a("OBSERVAR", f"{len(esc.objetos)} objetos", {
            "objetos": [(o.clase, round(o.confianza, 2), np.round(o.p, 3).tolist(), round(math.degrees(o.yaw), 1))
                        + (("tapa en color",) if o.por_color else ()) for o in esc.objetos],
            "fotogramas": len(fs), "ms": round(esc.ms, 1)})
        return esc, f

    # --- tirada ---------------------------------------------------------------------------------------
    def ejecutar(self):
        motivo_fin = "completado"
        try:
            self.vig.iniciar()
            self._a("ENTRADA", getattr(self.brazo, "texto_entrada", None) or
                    f"peso 0 -> 1 en {self.brazo.t_entrada:.0f} s siguiendo la postura medida")
            self.brazo.rampa_peso(1.0, comprobar=self.comprobar)
            self._abrir_manos()
            planes = [self.pl.entre_reposo_y_observacion(l, True) for l in LADOS]
            malos = [p.motivo for p in planes if not p.comprobado]
            if malos:
                raise Parada(f"reposo -> observacion no pasa la comprobacion: {malos}")
            self._a("A_OBSERVACION", "los dos brazos a la pose de observacion")
            self._ambos(planes)
            if self.nivel == 0:
                self._medir_mesa()
            elif self.nivel == 1:
                self._nivel1()
            else:
                self._niveles_objetos()
            self._salida()
        except Parada as e:
            motivo_fin = f"salida segura: {e}"
            self.salida_segura(str(e))
        except KeyboardInterrupt:
            motivo_fin = "salida segura: Ctrl+C"
            self.salida_segura("Ctrl+C")
        return motivo_fin

    def recuperar(self):
        """Con arnes.py (manipulacion.py --arnes --recuperar): unos brazos que se quedaron donde sea (manipulacion.py
        murio a medias) vuelven a 0 por la salida segura: abrir las manos, subir, por la observacion y al reposo."""
        self.vig.iniciar()
        self._a("ENTRADA", "recuperar: tomar los brazos donde esten")
        try:
            self.brazo.rampa_peso(1.0)
        except Parada as e:
            self._a("FIN", f"no se pueden tomar los brazos: {e}")
            return f"recuperar: {e}"
        self.salida_segura("recuperar los brazos")
        return "recuperados"

    def _salida(self):
        self._a("SALIDA", "observacion -> reposo")
        planes = [self.pl.entre_reposo_y_observacion(l, False) for l in LADOS]
        if all(p.comprobado for p in planes):
            self._ambos(planes)
        self.brazo.rampa_peso(0.0)
        self._a("FIN", "peso 0.00")

    # --- toque de prueba: la altura de la mesa --------------------------------------------------------
    def _medir_mesa(self):
        """manipulacion.py --medir-mesa (RESULTADOS §53): la yema de cada mano baja en un punto libre de la mesa
        (prueba_mesa.puntos) hasta que la fuerza del indice la nota, mas honda que en el nivel 1 (prueba_mesa.limite_m:
        rozando, la fuerza no cambia; apretando, sube enseguida). Lo que la yema queda por debajo (o encima) del plano
        corregido es lo que hay que sumar a la correccion de altura de este [g]; manipulacion.py la guarda en
        montaje.yaml (mesa.toque). Deja el resultado en self.medida_mesa."""
        pm = self.cfg.get("prueba_mesa") or {}
        esc, _ = self.observar()
        medidas = []
        for n, (lado, xy) in enumerate((pm.get("puntos") or {"izq": [0.48, 0.10], "der": [0.48, -0.10]}).items(), 1):
            xy = tuple(float(v) for v in xy)
            cerca = [o.clase for o in esc.objetos if math.hypot(o.p[0] - xy[0], o.p[1] - xy[1]) < float(pm.get("libre_m", 0.08))]
            if cerca:
                self._a("PLANIFICAR", f"punto {n} ({lado}): {cerca} a menos de {100 * float(pm.get('libre_m', 0.08)):.0f} cm; "
                        "se salta")
                continue
            plan = self.pl.tocar(lado, xy, self.q(lado))
            if not plan.comprobado:
                self._a("PLANIFICAR", f"punto {n} con la mano {lado}: {plan.motivo}")
                continue
            r = self._tocar(n, xy, plan, prueba=True)
            r["vale"] = r["resultado"] == "tocada" and "fuerza" in r.get("por", "")
            medidas.append(r)
        validas = [r for r in medidas if r["vale"]]
        res = {"puntos": [{k: r.get(k) for k in ("mano", "diana", "por", "altura_mm", "fuerza_gf", "vale")} for r in medidas],
               "correccion_antes_m": round(float(getattr(self, "correccion_altura", 0.0)), 4)}
        if validas:
            h = [r["altura_mm"] / 1000.0 for r in validas]
            res["correccion_m"] = round(res["correccion_antes_m"] + float(np.mean(h)), 4)
            res["diferencia_mm"] = round(1000 * (max(h) - min(h)), 1)
            self._a("MESA", f"la mesa a {', '.join(f'{x * 1000:+.1f}' for x in h)} mm del plano corregido: correccion "
                    f"{1000 * res['correccion_antes_m']:.1f} -> {1000 * res['correccion_m']:.1f} mm", res)
        else:
            self._a("MESA", "sin medida: ningun punto lo noto la fuerza (o no hubo plan)", res)
        self.medida_mesa = res

    # --- nivel 1 --------------------------------------------------------------------------------------
    def _nivel1(self):
        dianas = list(self.dianas)
        if not dianas and self.detector_dianas is not None:
            _, f = self.observar()
            dianas = self.detector_dianas(f)
            self._a("OBSERVAR", f"{len(dianas)} dianas", {"dianas": [np.round(d, 3).tolist() for d in dianas]})
        # agrupadas por brazo (el de su lado), empezando por el de la primera; dentro de cada brazo, en su orden
        if dianas:
            primero = "izq" if dianas[0][1] >= 0 else "der"
            dianas = sorted(dianas, key=lambda d: ("izq" if d[1] >= 0 else "der") != primero)
        for i, xy in enumerate(dianas):
            preferida = "izq" if xy[1] >= 0 else "der"
            plan = None
            for lado in (preferida, "der" if preferida == "izq" else "izq"):
                p = self.pl.tocar(lado, xy, self.q(lado))
                if p.comprobado:
                    plan = p
                    break
                self._a("PLANIFICAR", f"diana {i + 1} con la mano {lado}: {p.motivo}")
            if plan is None:
                self.resultados.append({"diana": list(xy), "resultado": "sin plan"})
                continue
            self.resultados.append(self._tocar(i + 1, xy, plan))

    def _tocar(self, n, xy, plan, prueba=False):
        lado, mano = plan.mano, self.manos[plan.mano]
        self._plan_activo = plan
        que = "punto" if prueba else "diana"
        self._a("PLANIFICAR", f"{que} {n} con la mano {lado}", plan.detalle)
        mano.preforma(self.pl.objetos["preformas"][plan.detalle.get("preforma", "senalar")], vigilar=self.comprobar)
        pts = {p.nombre: p for p in plan.puntos_de_paso}            # 'subir': el ultimo, el alto
        bajada = [p for p in plan.puntos_de_paso if p.nombre == "bajar"]
        self._a("MOVER", f"sobre {'el punto' if prueba else 'la diana'} {n}")
        self._mover(lado, pts["sobre"])
        self._mover(lado, pts["aproximar"])
        # la yema medida: los encoders del brazo y la cinematica con los dedos de la preforma, sobre la mesa del [g]
        cin_t = self.pl.cin_preforma(plan.detalle.get("preforma", "senalar"))
        sup = float(plan.detalle.get("superficie_m", 0.0))
        h_toque = sup + float(self.c.get("altura_toque_m", 0.0))
        if prueba:                                         # el toque de prueba: que pare la fuerza
            h_toque = sup + float((self.cfg.get("prueba_mesa") or {}).get("limite_m", -0.025))
        plano = self.pl.plano
        en_mesa = self.pl.punto(xy, 0.0)

        def yema(q=None):
            return cin_t.fk(lado, self.q(lado) if q is None else q, "yema")[:3, 3]

        def altura_yema():
            return float(plano.altura(yema()))

        def a_diana_mm():
            """Donde esta la yema medida respecto de la diana, en la mesa (x, y del torso, mm)."""
            return np.round(1000 * (plano.proyectar(yema()) - en_mesa)[:2], 1).tolist()
        guia = None
        if self.c.get("guiado", True) and bajada:
            from .guia import Guia
            Q = [pts["aproximar"].q] + [p.q for p in bajada]
            guia = Guia(Q, [float(plano.altura(yema(q))) for q in Q], ganancia=self.c.get("guiado_ganancia", 1.5),
                        paso_max=self.c.get("guiado_paso_max_rad", 0.005), corr_max=self.c.get("guiado_corr_max_rad", 0.15),
                        salto_max=self.c.get("guiado_salto_max_rad", 0.04))
        en_xy = plano.proyectar(en_mesa)

        def ajustar(h, extra=None):
            """Quieto con la yema a la altura h de la recta, integrando la correccion hasta que la yema medida esta en
            la vertical de la diana (ajuste_tol_m en xy, 0.3 s seguidos) o como mucho `maximo` s. None si arnes.py
            no acepta fijar; si no, {antes_mm, despues_mm, s}."""
            r, dentro, tol = {"antes_mm": a_diana_mm()}, [0], float(self.c.get("ajuste_tol_m", 0.003))
            maximo = float(self.c.get("ajuste_max_s", 5.0)) if h >= guia.arriba else float(self.c.get("pausa_max_s", 3.0))

            def listo(t):
                d = float(np.linalg.norm((plano.proyectar(yema()) - en_xy)[:2]))
                dentro[0] = dentro[0] + 1 if d < tol else 0
                return dentro[0] >= 15 or t > maximo
            t0 = time.monotonic()
            if not self._guiar(lado, guia, lambda t: h, listo, extra=extra, integrar=True):
                return None
            r.update(despues_mm=a_diana_mm(), s=round(time.monotonic() - t0, 1))
            return r
        ajuste = {}
        if guia is not None:
            # quieto en aproximar, la yema medida a su sitio sobre la diana antes de bajar
            ajuste = ajustar(guia.arriba)
            if ajuste is None:
                guia, ajuste = None, {}
        base = mano.linea_base(n=5)
        a0 = float(mano.leer().angulo[3])                  # el indice en su preforma, antes de bajar
        self.vig.manos_vigiladas = {lado}
        self._a("DESCENSO", "despacio hasta la mesa (la yema medida) o hasta notarla" + (", guiada" if guia else ""), {
            "base_gf": base.round(0).tolist(), "indice": a0, "yema_mm": round(1000 * altura_yema(), 1),
            "para_en_mm": round(1000 * h_toque, 1), "guiada": guia is not None, "ajuste": ajuste})
        contacto = {"t": None, "f": 0.0, "abre": 0.0, "tactil": 0.0, "seguidas": 0, "por": "", "h": None}
        usar_tactil = self.c.get("usar_tactil", False)
        umbral_ang = float(self.c.get("umbral_angulo", 1e9))
        lecturas = int(self.c.get("lecturas_contacto", 2))
        por_posicion = self.c.get("parar_por_posicion", True)

        def notar():
            """Para al notar la mesa: la fuerza del indice se aparta de su base mas de umbral_gf (al tocar BAJA:
            RESULTADOS §43) o la mesa lo abre, en `lecturas` lecturas seguidas (hay lecturas sueltas corruptas); y,
            de limite, al llegar la yema medida a contacto.altura_toque_m sobre la superficie."""
            e = mano.leer()
            f = float(e.fuerza[3] - e.base[3])
            abre = float(e.angulo[3]) - a0
            t = mano.tactil_yema() if usar_tactil else 0.0
            h = altura_yema()
            if por_posicion and h <= h_toque:
                contacto.update(t=time.monotonic(), f=f, abre=abre, tactil=t, por="posicion", h=h, xy=a_diana_mm())
                raise Contacto()
            por = [n for n, si in (("fuerza", abs(f) > self.c["umbral_gf"]), ("angulo", abre > umbral_ang),
                                   ("tactil", usar_tactil and t > self.c.get("umbral_tactil", 60))) if si]
            contacto["seguidas"] = contacto["seguidas"] + 1 if por else 0
            if contacto["seguidas"] >= lecturas:
                contacto.update(t=time.monotonic(), f=f, abre=abre, tactil=t, por="+".join(por), h=h, xy=a_diana_mm())
                raise Contacto()
        resultado, fijada, pausa = "sin contacto", None, None
        try:
            hecho = False
            if guia is not None:
                # la consigna baja por la recta, corregida con lo medido, a vel_descenso_m_s hasta pausa_m sobre la
                # superficie; alli, quieta, se reajusta la vertical; y el final, a vel_final_m_s (el brazo baja a
                # saltos: al tocar, poca marcha y la fuerza lo nota antes)
                v, v_f = float(self.c["vel_descenso_m_s"]), float(self.c.get("vel_final_m_s", self.c["vel_descenso_m_s"]))
                h_p = min(guia.arriba, max(guia.abajo, sup + float(self.c.get("pausa_m", 0.0))))
                hecho = self._guiar(lado, guia, lambda t: guia.arriba - v * t, lambda t: t > (guia.arriba - h_p) / v,
                                    extra=notar)
                if hecho and h_p < guia.arriba:
                    pausa = ajustar(h_p, extra=notar)
                    hecho = pausa is not None
                if hecho:
                    t_fin = (h_p - guia.abajo) / v_f + 1.0
                    hecho = self._guiar(lado, guia, lambda t: h_p - v_f * t, lambda t: t > t_fin, extra=notar)
            if not hecho:
                # sin guiar: de una vez al final de la recta (punto a punto, cada tramo dura como poco t_min_mov_s:
                # 16 s por cm con la escala 2, §51); al notarlo se detiene donde va la consigna (congelado en lo
                # medido, el brazo, que va ~1 cm por encima de su consigna, subiria)
                fin = PuntoPaso(q=bajada[-1].q, duracion=sum(p.duracion for p in bajada), nombre="bajar")
                self._mover(lado, fin, extra=notar, al_parar="detener")
        except Contacto:
            resultado = "tocada"
            if guia is not None:
                # quieto donde toco: la recta a la altura medida del contacto (la consigna deja de bajar: no sigue lo
                # que iba retrasado), con la correccion
                h_c = float(contacto["h"])
                fijada = self._guiar(lado, guia, lambda t: h_c, lambda t: t > float(self.c.get("mantener_s", 1.0)),
                                     extra=mano.leer)                    # la mano vigilada: lecturas frescas
            else:
                self.brazo.detener()
        q_contacto = self.q(lado)
        e = mano.leer()
        f_max = float(max(contacto["f"], e.fuerza[3] - e.base[3]))
        time.sleep(0.3)                                    # quieto: donde se queda de verdad la yema
        h_quieta = altura_yema()
        self._a("TOCAR", resultado, {"fuerza_gf": round(f_max), "abre": round(contacto["abre"]), "por": contacto["por"],
                                      "tactil": contacto["tactil"],
                                      "yema_al_parar_mm": None if contacto["h"] is None else round(1000 * contacto["h"], 1),
                                      "yema_diana_al_parar_mm": contacto.get("xy"), "consigna_fijada": fijada, "pausa": pausa,
                                      "yema_quieta_mm": round(1000 * h_quieta, 1), "yema_diana_quieta_mm": a_diana_mm()})
        if self._ultimo_fotograma is not None:
            self.reg.imagen(f"{que}_{n}", self._ultimo_fotograma.color)
        self.vig.manos_vigiladas = set()
        self._a("VOLVER", "subir y a la observacion")
        subida = False
        if guia is not None:
            # por la misma recta hasta aproximar (sin arrastrar la yema por la mesa), guiada
            v = float(self.c.get("vel_subida_m_s", 0.02))
            h0 = altura_yema()
            t_fin = (guia.arriba - h0) / v + 0.5
            subida = self._guiar(lado, guia, lambda t: h0 + v * t, lambda t: t > t_fin)
        if not subida:
            self._mover(lado, pts["aproximar"])            # sin guiar: de una vez (no punto a punto, §51)
        self._mover(lado, pts["subir"])
        self._mover(lado, pts["observacion"])
        self._abrir_manos([lado])                         # en la observacion, la mano abierta (la del gemelo)
        self._plan_activo = None
        return {"diana": list(xy), "mano": lado, "resultado": resultado, "fuerza_gf": round(f_max),
                "q_contacto": q_contacto.round(4).tolist(), "yema_diana_mm": contacto.get("xy"), "por": contacto["por"],
                "altura_mm": None if contacto["h"] is None else round(1000 * contacto["h"], 1)}

    # --- niveles 2 y 3 ----------------------------------------------------------------------------------
    def _niveles_objetos(self):
        pendientes = dict(self.asignacion)
        intentos = {k: 0 for k in pendientes}
        en_curso = None             # el brazo que esta trabajando: sigue con sus objetos antes de pasar al otro
        while pendientes:
            esc, f = self.observar()
            vistos = {o.clase: o for o in esc.objetos if o.clase in pendientes and o.confianza >= self.cfg["percepcion"]["confianza_min"]}
            if not vistos:
                c_min = self.cfg["percepcion"]["confianza_min"]
                dudosos = [f"{o.clase} ({o.confianza:.2f}) en ({o.p[0]:.3f}, {o.p[1]:+.3f})" for o in esc.objetos]
                self._a("PLANIFICAR", f"no veo ninguno de {sorted(pendientes)} con confianza >= {c_min}"
                        + (f"; vistos: {', '.join(dudosos)}" if dudosos else ""))
                break
            hecho = False
            # agrupados por brazo: primero los del brazo en curso (al empezar, el del objeto mas cercano); dentro de
            # un brazo, el mas cercano primero. Si ninguno del brazo en curso tiene plan, pasa al otro.
            def orden(kv):
                lado_kv = self.pl.lado_de_bandeja(pendientes[kv[0]])
                return (en_curso is not None and lado_kv != en_curso, float(kv[1].p[0]))
            for clase, o in sorted(vistos.items(), key=orden):
                bandeja = pendientes[clase]
                lado = self.pl.lado_de_bandeja(bandeja)
                plan = self.pl.coger(o, bandeja, self.q(lado), lado)
                if not plan.comprobado:
                    self._a("PLANIFICAR", f"{clase} -> {bandeja}: {plan.motivo}", {"rechazos": plan.detalle.get("rechazos", [])})
                    continue
                en_curso = lado
                ok = self._coger(plan, f)
                intentos[clase] += 1
                self.resultados.append({"objeto": clase, "bandeja": bandeja, "mano": lado,
                                        "resultado": "colocado" if ok else "fallido", "intento": intentos[clase],
                                        "agarre": plan.detalle.get("agarre")})
                if ok or intentos[clase] >= self.s.get("intentos_por_objeto", 2):
                    pendientes.pop(clase)
                hecho = True
                break
            if not hecho:
                self._a("PLANIFICAR", f"ningun objeto con plan valido: {sorted(vistos)}")
                break

    @staticmethod
    def _tramos(puntos):
        """[(nombre, [PuntoPaso])]: los puntos seguidos con el mismo nombre juntos (varios: una recta del plan)."""
        sal = []
        for p in puntos:
            if sal and sal[-1][0] == p.nombre:
                sal[-1][1].append(p)
            else:
                sal.append((p.nombre, [p]))
        return sal

    def _recta_guiada(self, lado, q_desde, puntos, punto_mano, guia_previa=None, seguida=True, extra=None,
                      ajuste_inicio=False, ajuste_fin=False):
        """Una recta del plan de coger (q_desde y `puntos`, con el mismo nombre) guiada por lo medido como la bajada
        del toque (manip/guia.py): el punto `punto_mano` (marco de la mano) por la recta, a vel_recta_m_s de su
        nombre; con ajuste_inicio, antes quieto hasta estar en la recta (contacto.ajuste_tol_m, perpendicular a
        ella); con ajuste_fin, al final hasta estar en el ultimo punto. guia_previa: la del tramo anterior: su
        correccion y, si `seguida` (sin un ir_a en medio), su ultima consigna. Devuelve la Guia (None sin guiar: con
        fijar rechazado, o contacto.guiado false, de una vez con ir_a)."""
        from .guia import Guia
        nombre = puntos[-1].nombre
        cin = self.pl.cin
        if self.c.get("guiado", True):
            Q = [np.asarray(q_desde, float)] + [p.q for p in puntos]
            P = np.array([aplicar(cin.fk(lado, q, "mano"), punto_mano) for q in Q])
            kw = {}
            if guia_previa is not None:
                kw = {"corr": guia_previa.corr, "ultima": guia_previa.ultima if seguida else None}
            g = Guia.por_puntos(Q, P, ganancia=self.c.get("guiado_ganancia", 1.5), paso_max=self.c.get("guiado_paso_max_rad", 0.005),
                                corr_max=self.c.get("guiado_corr_max_rad", 0.15), salto_max=self.c.get("guiado_salto_max_rad", 0.04), **kw)
            u = (P[-1] - P[0]) / max(1e-9, float(np.linalg.norm(P[-1] - P[0])))
            tol = float(self.c.get("ajuste_tol_m", 0.003))

            def medido():
                return aplicar(cin.fk(lado, self.q(lado), "mano"), punto_mano)

            def quieto(h, error, maximo):
                dentro = [0]

                def listo(t):
                    dentro[0] = dentro[0] + 1 if error() < tol else 0
                    return dentro[0] >= 15 or t > maximo
                return self._guiar(lado, g, lambda t: h, listo, extra=extra)

            def perp():
                e = medido() - P[0]
                return float(np.linalg.norm(e - (e @ u) * u))
            ok = True
            if ajuste_inicio:
                ok = quieto(g.H[0], perp, float(self.c.get("ajuste_max_s", 5.0)))
            v = float((self.pl.p.get("vel_recta_m_s") or {}).get(nombre, 0.02))
            if ok:
                ok = self._guiar(lado, g, lambda t: g.H[0] - v * t, lambda t: t > g.largo / v + 0.5, extra=extra)
            if ok and ajuste_fin:
                ok = quieto(g.H[-1], lambda: float(np.linalg.norm(medido() - P[-1])), float(self.c.get("pausa_max_s", 3.0)))
            if ok:
                return g
        # sin guiar: de una vez (punto a punto, cada tramo dura como poco t_min_mov_s)
        self._mover(lado, PuntoPaso(q=puntos[-1].q, duracion=sum(p.duracion for p in puntos), nombre=nombre), extra=extra)
        return None

    def _coger(self, plan, f):
        lado, mano, ag = plan.mano, self.manos[plan.mano], plan.agarre
        clase = plan.objeto.clase
        self._plan_activo = plan
        self._a("PLANIFICAR", f"{clase} -> bandeja {plan.bandeja} con la mano {lado}", plan.detalle)
        if f is not None:
            self.reg.imagen(f"agarre_{clase}_antes", f.color)
        mano.preforma(ag.preforma, vigilar=self.comprobar)
        punto = T_punto(centro_agarre(self.pl.objetos, clase, plan.detalle["agarre"]), lado)
        tramos = self._tramos(plan.puntos_de_paso)
        nombres = [n for n, _ in tramos]
        k = nombres.index("agarrar")

        def desde(i):
            return tramos[i - 1][1][-1].q
        self._a("MOVER", f"a {clase}")
        for _, pts in tramos[:k]:
            for p in pts:
                self._mover(lado, p)
        self._a("DESCENSO", "al agarre" + (", guiado" if self.c.get("guiado", True) else ""))
        g = self._recta_guiada(lado, desde(k), tramos[k][1], punto, ajuste_inicio=True, ajuste_fin=True)
        self.vig.manos_vigiladas = {lado}
        self._a("CERRAR", f"FORCE_SET {ag.force_set}")
        e = mano.cerrar(ag.force_set, vigilar=self.comprobar)
        ok, det = mano.agarre(e, ag.force_set)
        en_agarre = np.round(1000 * (aplicar(self.pl.cin.fk(lado, self.q(lado), "mano"), punto)
                                     - aplicar(self.pl.cin.fk(lado, tramos[k][1][-1].q, "mano"), punto)), 1)
        self._a("CERRAR", "agarre comprobado" if ok else "agarre fallido",
                {"dedos": {int(k_): v for k_, v in det.items()}, "fuerza_gf": (e.fuerza - e.base).round(0).tolist(),
                 "angulo": e.angulo.round(0).tolist(), "mano_desde_el_plan_mm": en_agarre.tolist()})
        fot = self.fuente.siguiente(0.5) if hasattr(self.fuente, "siguiente") else None
        if fot is not None:
            self.reg.imagen(f"agarre_{clase}_cerrado", fot.color)
        k_lev = nombres.index("levantar")
        if not ok:
            mano.abrir(vigilar=self.comprobar)
            self.vig.manos_vigiladas = set()
            self._a("VOLVER", "agarre fallido: abrir, subir y a la observacion")
            self._recta_guiada(lado, desde(k_lev), tramos[k_lev][1], punto, guia_previa=g)
            self._mover(lado, plan.puntos_de_paso[-1])
            self._plan_activo = None
            return False
        # FORCE_SET y la consigna cerrada cada 1 s; la lectura, mas a menudo que mano_max_s del vigilante (con la
        # mano vigilada, una lectura mas vieja que eso es "mano sin respuesta")
        t_sujetar, t_leer = [time.monotonic()], [time.monotonic()]
        cada_leer = min(0.2, self.vig.mano_max_s / 2)
        # si el objeto se escapa (Mano.escapado en dos lecturas seguidas) el brazo no se para: sigue su plan, sin
        # movimientos que no se hayan comprobado; al acabar de levantar abre y vuelve a la observacion, y mas tarde
        # suelta en la bandeja como siempre. El intento cuenta como fallido (antes era una Parada que no saltaba)
        escape = {"seguidas": 0, "en": None}

        def vigilar_escape(e2):
            if escape["en"] is not None:
                return
            escape["seguidas"] = escape["seguidas"] + 1 if mano.escapado(e2, ag.force_set) else 0
            if escape["seguidas"] >= 2:
                escape["en"] = self.estado
                self._a(self.estado, f"{clase} se escapa de la mano",
                        {"angulo": e2.angulo.round(0).tolist(), "fuerza_gf": (e2.fuerza - e2.base).round(0).tolist()})

        def sujetar():
            ahora = time.monotonic()
            if ahora - t_sujetar[0] > 1.0:
                mano.sujetar(ag.force_set)
                t_sujetar[0] = ahora
            if ahora - t_leer[0] > cada_leer:
                t_leer[0] = ahora
                vigilar_escape(mano.leer())
        mano.leer()                          # la foto y el registro de arriba han envejecido la ultima lectura
        self._a("TRANSPORTE", f"a la bandeja {plan.bandeja}")
        g = self._recta_guiada(lado, desde(k_lev), tramos[k_lev][1], punto, guia_previa=g, extra=sujetar)
        vigilar_escape(mano.leer())
        if escape["en"] is None:
            vigilar_escape(mano.leer())      # (dos lecturas seguidas)
        if escape["en"] is not None:
            mano.abrir(vigilar=self.comprobar)
            self.vig.manos_vigiladas = set()
            self._a("VOLVER", f"{clase} se escapo al levantar: abrir y a la observacion")
            self._mover(lado, plan.puntos_de_paso[-1])
            self._plan_activo = None
            return False
        k_baj = nombres.index("bajar_a_bandeja")
        for _, pts in tramos[k_lev + 1:k_baj]:
            for p in pts:
                self._mover(lado, p, extra=sujetar)
        g = self._recta_guiada(lado, desde(k_baj), tramos[k_baj][1], punto, guia_previa=g, seguida=False, extra=sujetar,
                               ajuste_inicio=True)
        self._a("SOLTAR", f"{clase} en la bandeja {plan.bandeja}" if escape["en"] is None
                else f"abrir sobre la bandeja {plan.bandeja}: {clase} se escapo en {escape['en']}")
        # solo los dedos que sujetan: los recogidos por la preforma (dos o tres dedos) se abren ya retirada la mano.
        # Abiertos aqui, el medio recogido se estiraba junto al cilindro y lo empujaba contra la pared (§70)
        fs, pre = plan.agarre.force_set, plan.agarre.preforma
        sujetan = [k for k in range(5) if fs[k] is not None and fs[k] > 0]
        recogidos = [k for k in range(5) if k not in sujetan and pre[k] is not None and 0 <= pre[k] < 500]
        mano.abrir(vigilar=self.comprobar, dedos=sujetan if recogidos else None)
        self.vig.manos_vigiladas = set()
        self._a("VOLVER", "retirar y a la observacion")
        k_ret = nombres.index("retirar_de_bandeja")
        self._recta_guiada(lado, desde(k_ret), tramos[k_ret][1], punto, guia_previa=g)
        if recogidos:
            mano.abrir(vigilar=self.comprobar)
        for _, pts in tramos[k_ret + 1:]:
            for p in pts:
                self._mover(lado, p)
        self._plan_activo = None
        return escape["en"] is None

    # --- salida segura ----------------------------------------------------------------------------------
    def salida_segura(self, motivo):
        """Congelar, abrir la mano, subir a la altura de seguridad, (salir por la observacion al reposo) y solo
        entonces bajar el peso. Si algo grave pasa a mitad (lowstate o motor), directo a la rampa de bajada."""
        self._a("SALIDA_SEGURA", motivo)
        if hasattr(self.brazo, "en_salida_segura"):   # con arnes.py: sus movimientos se aceptan tras su parada
            self.brazo.en_salida_segura()
        self.brazo.congelar()
        if self.brazo.peso <= 0.01:
            self._a("FIN", "salida segura sin peso: nada que mover")
            return
        for lado, mano in self.manos.items():
            try:
                mano.abrir()
            except Exception as e:                     # sigue: subir y bajar el peso no depende de la mano
                self._a("SALIDA_SEGURA", f"la mano {lado} no abre: {e}")

        def grave():
            if self.robot.edad_lowstate() > self.s["lowstate_max_s"]:
                raise Parada("rt/lowstate perdido en la salida segura")
            if self.robot.estado().motores_en_fallo:
                raise Parada("motor en fallo en la salida segura")
        try:
            for lado in LADOS:
                q = self.q(lado)
                M = self.pl.cin.fk(lado, q, "mano")
                if not self.pl.sobre_mesa(M[:3, 3], 0.05):
                    continue
                h = float(self.pl.plano.altura(M[:3, 3]))
                altura = self.cfg["planificacion"]["altura_segura_m"]
                if h >= altura:
                    continue
                R = M[:3, :3]
                t = TareaIK(posicion=M[:3, 3] + (altura - h + 0.02) * self.pl.plano.n, tol_pos=0.01,
                            bandas=[Banda(np.array([1.0, 0, 0]), R[:, 0], math.cos(math.radians(15)), 1.0),
                                    Banda(np.array([0, 1.0, 0]), R[:, 1], math.cos(math.radians(15)), 1.0)])
                r = self.pl.cin.ik(lado, t, q0=q)
                if r.ok:
                    self._a("SALIDA_SEGURA", f"subir la mano {lado} a {altura * 100:.0f} cm")
                    self.brazo.ir_a(self.brazo.pose_brazo(lado, r.q), None, grave)
            if self.s.get("salida_por_observacion", True):
                for lado in LADOS:
                    self._mover_seguro(lado, self.pl.observacion[lado], grave)
                # al reposo solo los brazos que llegaron a la observacion: desde otro sitio barrerian la mesa
                planes = []
                for lado in LADOS:
                    if np.abs(self.q(lado) - self.pl.observacion[lado]).max() < 0.1:
                        p = self.pl.entre_reposo_y_observacion(lado, False)
                        if p.comprobado:
                            planes.append(p)
                    else:
                        self._a("SALIDA_SEGURA", f"brazo {lado} fuera de la observacion: no va al reposo")
                if planes:
                    self._a("SALIDA_SEGURA", f"observacion -> reposo ({', '.join(sorted(p.mano for p in planes))})")
                    self._ambos(planes, grave)
        except Parada as e:
            self._a("SALIDA_SEGURA", f"{e}: directo a bajar el peso")
        self._a("SALIDA_SEGURA", f"peso -> 0 en {self.brazo.t_salida:.0f} s")
        self.brazo.rampa_peso(0.0)
        self._a("FIN", "peso 0.00 tras la salida segura")

    def _mover_seguro(self, lado, q_obj, grave):
        """A la observacion solo por tramos que pasan la comprobacion: directo o, si no, pasando por un punto alto
        del plan en curso (sobre el objeto, la bandeja o la diana), el mas cercano primero. Directo, desde encima
        de la mesa, la muneca puede girar bajando la mano hasta la mesa. Si ninguno pasa, se queda arriba."""
        q = self.q(lado)
        if np.abs(q - q_obj).max() < 0.05:
            return
        vias = [None]
        if self._plan_activo is not None and self._plan_activo.mano == lado:
            altos = [p.q for p in self._plan_activo.puntos_de_paso if p.nombre in PASOS_ALTOS]
            vias += sorted(altos, key=lambda v: float(np.abs(v - q).max()))
        motivos = []
        for via in vias:
            pts, qa = [], q
            for nombre, qb in ([] if via is None else [("via", via)]) + [("observacion", q_obj)]:
                pts.append(PuntoPaso(q=qb, duracion=self.pl.duracion(qa, qb), nombre=nombre))
                qa = qb
            plan = self.pl.comprobar(Plan(objeto=None, mano=lado, tarea="observacion", puntos_de_paso=pts),
                                     q_inicio=q, sin_objetos=True)
            if plan.comprobado:
                self._a("SALIDA_SEGURA", f"brazo {lado} a la observacion" +
                        ("" if via is None else f" por un punto alto del plan (directo: {motivos[0]})"))
                for p in plan.puntos_de_paso:
                    self.brazo.ir_a(self.brazo.pose_brazo(lado, p.q), p.duracion, grave)
                return
            motivos.append(plan.motivo)
        self._a("SALIDA_SEGURA", f"brazo {lado}: no hay camino limpio a la observacion ({motivos[0]}); se queda")
