"""El regulador de fuerza (inspire_hand_interface/Control; copia fija en regulador/) en el agarre del reto.

Un agarre regulado (LazoAgarre), dentro de Mano.cerrar y hasta Mano.abrir:
  1. APROXIMACION, en el hilo del supervisor (que sigue llamando a vigilar(): el brazo sujeto en el agarre, §87).
     Cada dedo regulado cierra rapido hasta su angulo de aproximacion (justo antes del objeto) y lento desde ahi,
     sin parar (la conmutacion de velocidad del Exp 2). Al tocar, ese dedo se CONGELA donde esta (como `_aproxima`
     del banco): el primero en llegar no empuja el objeto ni el brazo mientras llegan los demas. Un dedo que llega
     a su tope de cierre sin tocar nada se queda alli.
  2. APRIETE y SOSTENER, en un hilo propio: el lazo en coordenadas (A3/A4/B1), no un lazo de fuerza por dedo:
       apriete = (F_pulgar + F_dedos) / 2     -> la fuerza de agarre, a la consigna del objeto
       balance = F_pulgar - F_dedos           -> a la registrada al establecer el agarre (no a 0: offsets de sensor)
       reparto = F_indice - F_medio (modo 2)  -> idem
     PI en gramos de cada coordenada -> cambio de fuerza por dedo -> J^-1 (en el modo 1, la K del RLS) -> comando,
     con el vector escalado a dq_max y el cuanto por dedo (5 cerrando, 3 abriendo) con acumulador de resto.
     Solo se decide con muestra fresca (Disparador) y fuera del periodo refractario.
     Los detectores de A1: el actuador cede (POS retrocede contra su comando) -> AFLOJAR; el objeto se escapa (la
     fuerza cae en todos los dedos con POS quieto) -> APRETAR (sube la consigna). La guarda (techo por dedo) afloja.
     Todos los dedos sin fuerza, o el pulgar o todos los dedos vacios en su tope: el objeto se perdio (`perdido`).
En el brazo sin regulador (observador=True) el mismo hilo solo mira y apunta: las mismas columnas, coordenadas y
detectores, sin escribir nada en la mano. Asi los dos brazos del banco se miden igual.

Fuerzas en gf sobre el cero (`mano.cero`: la mano abierta, sin carga, >= 10 s desde la ultima suelta).
"""

import threading
import time
from collections import deque
from types import SimpleNamespace

import numpy as np

from nucleo import PI, Bitacora, DetectorEscape, DetectorResbalon, Disparador, EstimadorRLS, Guarda
from hand_modbus import CURRENT, POS_ACT
from manip.mano import ANGLE_ACT, FORCE_ACT, ManoSinRespuesta, bytes_de

TEMP = 1618                   # por TCP, un byte por DOF: 3 registros (como ERROR y STATUS)
T, I, M = 4, 3, 2
DEDOS_MODO = {1: (T, I), 2: (T, I, M)}
NOMBRE_DEDO = {"pulgar": T, "indice": I, "medio": M, "anular": 1, "menique": 0}
NOMBRE = {v: k for k, v in NOMBRE_DEDO.items()}


class BitacoraAgarre(Bitacora):
    """El CSV del banco (las mismas columnas, para sus herramientas) y lo propio del agarre en el robot."""
    COLS = Bitacora.COLS + ["t_mono", "fase", "F_ref", "apriete", "balance", "reparto", "err_r", "u_r",
                            "ANG_T", "ANG_I", "ANG_M", "F_W"]


class LazoAgarre:
    def __init__(self, mano, p, modo, fuerza_gf, aprox, cierre, ruta_csv=None, fase=None, log=print,
                 observador=False, reloj=time.monotonic, dormir=time.sleep):
        """mano: ManoRegulada (regs, escribir, mover, cero, lado). p: config/regulador.yaml. aprox: {dof: angulo};
        cierre: la consigna de cierre por DOF de la mano (Mano.cierre: el tope de cada dedo). fase(): el estado del
        supervisor (CERRAR, TRANSPORTE...), para el registro y las medidas por tramo."""
        self.mano, self.p, self.modo = mano, p, int(modo)
        self.dedos = DEDOS_MODO[self.modo]
        self.grupo = self.dedos[1:]
        self.fuerza_obj = float(fuerza_gf)
        self.f_ref = float(fuerza_gf)
        # apretando (el objeto se escapa) no mas de fuerza_max_frac de la del objeto (y, al apretar, ni tan cerca del
        # techo de la guarda que la salte un dedo: _tope_apretar)
        self.f_max = float(fuerza_gf) * float(p["reacciones"]["fuerza_max_frac"])
        self.aprox = {int(d): int(a) for d, a in (aprox or {}).items()}
        self.cierre = [int(c) for c in cierre]
        self.fase = fase or (lambda: "")
        self.log = log
        self.observador = observador
        self.reloj, self.dormir = reloj, dormir
        self.cero = np.asarray(mano.cero, float).copy()
        self.t0 = reloj()
        c, r, s = p["control"], p["reacciones"], p["seguridad"]
        self.c, self.r, self.s = c, r, s
        mk = lambda kp, ki, banda: PI(kp, ki, c["fuga"], banda, c["paso_cierra"], c["paso_abre"], c["dq_max"], 0.0)  # noqa: E731
        self.pi_a = mk(c["kp_apriete"], c["ki_apriete"], c["banda_apriete_gf"])
        self.pi_b = mk(c["kp_balance"], 0.0, c["banda_balance_gf"])
        self.pi_r = mk(c["kp_reparto"], 0.0, c["banda_reparto_gf"])
        # el cuanto vive en el dedo: un PI por dedo solo para cuantizar (con su resto acumulado)
        self.q = {d: PI(0.0, 0.0, 1.0, 0.0, c["paso_cierra"], c["paso_abre"], c["dq_max"], 0.0) for d in self.dedos}
        g = p["ganancias"]
        self.ratio = np.array([g["pos_por_u"]["pulgar"] if d == T else g["pos_por_u"]["dedo"] for d in self.dedos])
        J = np.array(g["modo1"] if self.modo == 1 else g["modo2"], float)
        self.J = J
        # el desacoplo: J^-1 (A3), la K del RLS (A4) o solo la diagonal (B1: un cambio de fuerza por dedo entre su
        # propia ganancia, el kq del banco; con tres dedos la J medida no es autoconsistente)
        self.desacoplo = str(c.get(f"desacoplo_modo{self.modo}", "rls" if self.modo == 1 else "diagonal"))
        diag = np.diag(1.0 / np.diag(J))
        self.Jinv = diag if self.desacoplo == "diagonal" or abs(np.linalg.det(J)) < 1e-6 else np.linalg.inv(J)
        self.est = None
        if self.modo == 1 and self.desacoplo == "rls":
            rT, rI = self.ratio
            self.est = EstimadorRLS(((J[0, 0] / rT, J[0, 1] / rI), (J[1, 0] / rT, J[1, 1] / rI)),
                                    lam=c["rls_lam"], min_pos=c["rls_min_pos"], ridge=c["rls_ridge"])
        self.disp = {d: Disparador(d, p["lectura"].get("disparo_plazo_s", 0.040)) for d in self.dedos}
        self.resbalon = DetectorResbalon(list(self.dedos), r["resbalon_counts"], r["resbalon_ventana_s"])
        self.escape = DetectorEscape(list(self.dedos), r["escape_gf"], r["escape_frac"], r["resbalon_ventana_s"])
        self.guarda = self._nueva_guarda()
        self.bit = BitacoraAgarre(ruta_csv) if ruta_csv else None
        # estado
        self.cmds = {}                     # ANGLE_SET de cada dedo regulado (lo ultimo mandado)
        self.t_abre = {}                   # cuando se le mando abrir a cada dedo (el detector de resbalon, en espera)
        self.contacto = {}                 # dof -> (t desde el inicio, angulo) al tocar
        self.al_tope = set()               # dedos que llegaron a su tope sin tocar
        self.t_accion = None
        self.pend = None                   # la accion por evaluar (RLS): (t, pos, F)
        self.b0 = self.r0 = None           # balance y reparto registrados al establecer
        self.t_en_banda = None
        self.t_establecido = None
        self.establecido = threading.Event()
        self.perdido = False
        self.vigilar_perdida = False
        self.t_inicio_lazo = self.t0
        self.t_bajo = None
        self.fallo = None
        self.eventos = []
        self.guardas = deque()
        self.picos = {}                    # por dedo, en el cierre
        self.por_fase = {}                 # fase -> [apriete...] (las medidas por tramo)
        self.n_acciones = 0
        self.n_lecturas = 0
        self._ult = None                   # la ultima muestra
        self._f_prev = [None] * 6
        self._ang = None
        self._cur = [0] * 6
        self._temps = None
        self._t_temp = -1e9
        self._t_buena = reloj()
        self._hilo = None
        self._parar = threading.Event()

    # --- utilidades ---------------------------------------------------------------------------------
    def _nueva_guarda(self):
        s = self.s
        return Guarda(SimpleNamespace(techo_fuerza=s["techo_gf"], fuerza_ciclos=s["fuerza_ciclos"],
                                      corriente_max=s["corriente_max_ma"], corriente_ciclos=s["corriente_ciclos"],
                                      temp_max=s["temp_max_c"], timeout_s=1e9), list(self.dedos))

    def _tope_apretar(self):
        """La consigna maxima al apretar: fuerza_max_frac de la del objeto, y que el dedo mas cargado quede a 0.9 del
        techo de la guarda. Con apriete A y balance B (pulgar - dedos), el pulgar lleva ~A + B/2 y los de enfrente
        ~A - B/2 (en el modo 2, repartidos entre dos): el mas cargado, ~A + |B|/2 (modo 1) o A + max(B, 0)/2 (modo 2)."""
        b = 0.0 if self.b0 is None else (abs(self.b0) if self.modo == 1 else max(self.b0, 0.0))
        return max(self.f_ref, min(self.f_max, 0.9 * float(self.s["techo_gf"]) - b / 2.0))

    def _evento(self, tipo, texto, **detalle):
        t = self.reloj() - self.t0
        self.eventos.append({"t": round(t, 3), "fase": self.fase(), "tipo": tipo, "texto": texto, **detalle})
        self.log(f"  [regulador {self.mano.lado}] {texto}")
        return tipo

    def _mandar(self, nuevos):
        """ANGLE_SET de los dedos de `nuevos` ({dof: angulo}); los demas, sin tocar (-1)."""
        v = [-1] * 6
        for d, a in nuevos.items():
            a = int(min(max(a, self.cierre[d] if self.cierre[d] >= 0 else 0), 1000))
            if d in self.cmds and a > self.cmds[d]:
                self.t_abre[d] = self.reloj()
            self.cmds[d] = a
            v[d] = a
        self.mano.mover(v)

    def _muestra(self, con_angulo=None):
        """(t, pos, F sobre el cero, fresca_f) y, cada angulo_cada lecturas, el angulo (self._ang)."""
        lp = self.p["lectura"]
        f = self.mano.regs(FORCE_ACT, 6)
        pos = self.mano.regs(POS_ACT, 6)
        t = self.reloj()
        self.n_lecturas += 1
        if con_angulo or (con_angulo is None and self.n_lecturas % int(lp["angulo_cada"]) == 0) or self._ang is None:
            self._ang = self.mano.regs(ANGLE_ACT, 6)
        if self.n_lecturas % int(lp["corriente_cada"]) == 0:
            self._cur = self.mano.regs(CURRENT, 6).tolist()
        if t - self._t_temp > float(lp["temperatura_cada_s"]):
            self._t_temp = t
            try:
                self._temps = bytes_de(self.mano._crudos(TEMP, 3).registers)
            except ManoSinRespuesta:
                self._temps = None
        fresca = [self._f_prev[d] is not None and f[d] != self._f_prev[d] for d in range(6)]
        self._f_prev = list(f)
        self._t_buena = t
        F = f - self.cero
        self._ult = (t, pos, F)
        return t, pos, F, fresca

    def coordenadas(self, F):
        """(apriete, balance, reparto) de las fuerzas F (gf sobre el cero)."""
        fg = float(sum(F[d] for d in self.grupo))
        a = (float(F[T]) + fg) / 2.0
        b = float(F[T]) - fg
        r = float(F[I] - F[M]) if self.modo == 2 else 0.0
        return a, b, r

    def _fila(self, t, pos, F, fresca, evento="", **kw):
        fase = self.fase()
        a, b, r = self.coordenadas(F)
        self.por_fase.setdefault(fase, []).append(a)
        for d in self.dedos:
            self.picos.setdefault(fase, {})
            self.picos[fase][d] = max(self.picos[fase].get(d, -1e9), float(F[d]))
        if self.bit is None:
            return
        g = lambda v, d: "" if v is None else int(round(float(v[d])))   # noqa: E731
        self.bit.fila(t=f"{t - self.t0:.4f}", t_mono=f"{t:.4f}", fase=fase,
                      fresca_f="".join("1" if fresca[d] else "0" for d in self.dedos),
                      F_T=g(F, T), F_I=g(F, I), F_M=g(F, M) if self.modo == 2 else "",
                      F_grip=int(round(min(float(F[d]) for d in self.dedos))),
                      POS_T=g(pos, T), POS_I=g(pos, I), POS_M=g(pos, M) if self.modo == 2 else "",
                      cmd_T=self.cmds.get(T, ""), cmd_I=self.cmds.get(I, ""), cmd_M=self.cmds.get(M, ""),
                      ANG_T=g(self._ang, T), ANG_I=g(self._ang, I), ANG_M=g(self._ang, M),
                      I_mA=g(self._cur, T), temp=max(self._temps) if self._temps else "",
                      F_ref="" if self.observador else round(self.f_ref), apriete=round(a), balance=round(b),
                      reparto=round(r) if self.modo == 2 else "", evento=evento, **kw)

    # --- 1. aproximacion (hilo del supervisor) ---------------------------------------------------------
    def aproximar(self, firmware, velocidades, vigilar=None):
        """Cierra los dedos regulados (rapido hasta aprox, lento despues) y congela cada uno al tocar. `firmware`:
        los DOF que cierran por FORCE_SET (el anular de la esfera): ya tienen su consigna, aqui no se tocan.
        velocidades: SPEED_SET de los 6 DOF para los que no son regulados. Devuelve {dof: "contacto"|"tope"|"tiempo"}."""
        a = self.p["aproximacion"]
        v_rap, v_len = int(a["vel_rapida"]), int(a["vel_lenta"])
        vel = [int(x) for x in velocidades]
        t_ini = self.reloj()
        _, _, _, _ = self._muestra(con_angulo=True)
        ang0 = self._ang.copy()
        lento = set()
        for d in self.dedos:
            lento_ya = d in self.aprox and ang0[d] <= self.aprox[d]
            vel[d] = v_len if (lento_ya or d not in self.aprox) else v_rap
            if vel[d] == v_len:
                lento.add(d)
        self.mano.escribir_speed(vel)
        self._mandar({d: self.cierre[d] for d in self.dedos})
        hist = {d: deque() for d in self.dedos}
        t_mov = {d: None for d in self.dedos}
        hecho = {}
        t_vig = -1e9
        periodo = 1.0 / float(self.p["lectura"]["sondeo_hz"])
        while len(hecho) < len(self.dedos):
            t_vuelta = self.reloj()
            t, pos, F, fresca = self._muestra(con_angulo=True)
            ang = self._ang
            ev = ""
            cambia = False
            for d in self.dedos:
                if d in hecho:
                    continue
                if t_mov[d] is None and ang[d] < ang0[d] - 3:
                    t_mov[d] = t
                if d not in lento and d in self.aprox and ang[d] <= self.aprox[d]:
                    lento.add(d)
                    vel[d] = v_len
                    cambia = True
                h = hist[d]
                h.append((t, float(F[d])))
                while h and t - h[0][0] > float(a["ventana_contacto_s"]):
                    h.popleft()
                arrancado = t_mov[d] is not None and t - t_mov[d] >= float(a["t_ignorar_s"])
                subida = float(F[d]) - min(x for _, x in h)
                if (arrancado and subida >= float(a["subida_contacto_gf"])) or float(F[d]) >= float(a["contacto_abs_gf"]):
                    # CONGELAR donde esta: el dedo sigue viajando hacia el ultimo comando y se pasaria (banco: 711 g
                    # pedidos 150). El angulo, el de ahora mismo
                    ahora = self.mano.regs(ANGLE_ACT, 6)
                    self._mandar({d: int(ahora[d])})
                    hecho[d] = "contacto"
                    self.contacto[d] = (round(t - t_ini, 3), int(ahora[d]), round(float(F[d])))
                    ev += f"contacto_{NOMBRE[d]} "
                elif abs(ang[d] - self.cierre[d]) <= int(a["tope_tol"]):
                    hecho[d] = "tope"
                    self.al_tope.add(d)
                    ev += f"tope_{NOMBRE[d]} "
            if cambia:
                self.mano.escribir_speed(vel)
            self._fila(t, pos, F, fresca, ev.strip())
            if t - t_ini > float(a["t_max_s"]):
                ahora = self.mano.regs(ANGLE_ACT, 6)
                for d in self.dedos:
                    if d not in hecho:
                        self._mandar({d: int(ahora[d])})
                        hecho[d] = "tiempo"
                break
            if vigilar is not None and t - t_vig >= 0.02:
                t_vig = t
                vigilar()
            resto = periodo - (self.reloj() - t_vuelta)
            if resto > 0:
                self.dormir(resto)
        # la velocidad del lazo: la lenta (cada escalon del lazo es pequeño y lo marca el cuanto, no la velocidad)
        for d in self.dedos:
            vel[d] = v_len
        self.mano.escribir_speed(vel)
        self._evento("aproximacion", "aproximacion: " + ", ".join(
            f"{NOMBRE[d]} {hecho[d]}" + (f" en {self.contacto[d][1]} ({self.contacto[d][0]:.2f} s)" if d in self.contacto
                                         else "") for d in self.dedos), dedos={NOMBRE[d]: hecho[d] for d in self.dedos})
        return hecho

    # --- 2. el lazo (hilo propio) ------------------------------------------------------------------------
    def arrancar(self):
        self.t_inicio_lazo = self.reloj()
        if not self.cmds and not self.observador:
            ang = self.mano.regs(ANGLE_ACT, 6)
            self.cmds = {d: int(ang[d]) for d in self.dedos}
        self._hilo = threading.Thread(target=self._bucle, name=f"regulador_{self.mano.lado}", daemon=True)
        self._hilo.start()
        return self

    def parar(self):
        self._parar.set()
        if self._hilo is not None and self._hilo is not threading.current_thread():
            self._hilo.join(timeout=2.0)
        if self.bit is not None:
            self.bit.cierra()
            self.bit = None

    def esperar_establecido(self, vigilar=None, t_max=None):
        """En el hilo del supervisor (Mano.cerrar): hasta que el apriete se establece, se pierde o pasa t_max."""
        t_max = float(self.c["t_establecer_max_s"] if t_max is None else t_max)
        t0 = self.reloj()
        while self.reloj() - t0 < t_max and not self.establecido.is_set() and not self.perdido and self.fallo is None:
            if vigilar is not None:
                vigilar()
            self.dormir(0.02)
        self.vigilar_perdida = True          # de aqui en adelante el agarre esta hecho: perderlo es escaparse
        return self.establecido.is_set()

    def _bucle(self):
        periodo = 1.0 / float(self.p["lectura"]["sondeo_hz"])
        while not self._parar.is_set():
            t_vuelta = self.reloj()
            try:
                t, pos, F, fresca = self._muestra()
            except ManoSinRespuesta as e:
                if self.reloj() - self._t_buena > float(self.p["lectura"]["sin_lectura_max_s"]):
                    self.fallo = f"sin lecturas de la mano {self.mano.lado} ({e})"
                    self._evento("fallo", self.fallo)
                    return
                self.dormir(0.01)
                continue
            try:
                self._paso(t, pos, F, fresca)
            except ManoSinRespuesta as e:
                self.fallo = f"la mano {self.mano.lado} no acepta ordenes ({e})"
                self._evento("fallo", self.fallo)
                return
            except Exception as e:                                  # un error del lazo no debe dejar el agarre a medias
                self.fallo = f"error en el regulador: {type(e).__name__}: {e}"
                self._evento("fallo", self.fallo)
                return
            resto = periodo - (self.reloj() - t_vuelta)
            if resto > 0:
                self.dormir(resto)

    def _paso(self, t, pos, F, fresca):
        ev = []
        Fl = [float(x) for x in F]
        pl = [float(x) for x in pos]                 # listas: los detectores del banco hacen `if p:`
        # un dedo al que se le mando abrir retrocede por obediencia, no porque ceda. El detector del banco lo descuenta
        # si el comando de abrir cae en su ventana, pero a velocidad lenta el dedo sigue retrocediendo despues y daba
        # resbalones falsos en cadena (cada uno aflojaba otra vez). Mientras dura la ventana y el viaje, sin historia
        espera = float(self.r["resbalon_ventana_s"]) + 0.3
        for d, ta in self.t_abre.items():
            if t - ta < espera:
                self.resbalon.hist[d].clear()
                self.resbalon.cmd_hist[d].clear()
        # los detectores, tambien en el observador (solo se apuntan)
        r = self.resbalon.actualiza(t, pl, self.cmds, Fl, float(self.r["contacto_min_gf"]))
        e = self.escape.actualiza(t, pl, Fl)
        if self.observador:
            if r:
                ev.append(self._evento("resbalon", f"el actuador cede ({NOMBRE[r['dof']]}, {r['counts']:.0f} cuentas)"))
            if e:
                ev.append(self._evento("escape", "el objeto se escapa (cae la fuerza en todos los dedos)"))
            self._fila(t, pos, F, fresca, " ".join(ev))
            return
        if self.perdido:
            self._fila(t, pos, F, fresca)
            return
        # la guarda: un dedo por encima del techo -> aflojar (y si se repite, fallo). Tras aflojar, el refractario
        # antes de volver a mirar: el dedo tarda en abrir y la guarda saltaba otra vez con la misma fuerza
        m = None
        if not self.guardas or t - self.guardas[-1] >= float(self.c["refractario_s"]) + 0.2:
            m = self.guarda.revisa(Fl, self._cur, self._temps)
        if m:
            self.guarda = self._nueva_guarda()
            self.guardas.append(t)
            while self.guardas and t - self.guardas[0] > 2.0:
                self.guardas.popleft()
            if len(self.guardas) > int(self.s["guardas_max"]):
                self.fallo = f"la guarda salta una y otra vez: {m}"
                ev.append(self._evento("fallo", self.fallo))
                self._fila(t, pos, F, fresca, " ".join(ev))
                return
            self.f_ref *= (1.0 - float(self.r["aflojar_frac"]))
            self._mandar({d: self.cmds[d] + 2 * int(self.c["paso_abre"]) for d in self.dedos})
            self.t_accion = t
            ev.append(self._evento("guarda", f"guarda: {m}: aflojar, consigna {self.f_ref:.0f} gf"))
        if r:
            d = r["dof"]
            self.f_ref *= (1.0 - float(self.r["aflojar_frac"]))
            self._mandar({d: self.cmds[d] + 2 * int(self.c["paso_abre"])})
            self.t_accion = t
            ev.append(self._evento("resbalon", f"el actuador cede ({NOMBRE[d]}, {r['counts']:.0f} cuentas): aflojar, "
                                               f"consigna {self.f_ref:.0f} gf"))
        if e:
            nueva = min(self.f_ref * (1.0 + float(self.r["apretar_frac"])), self._tope_apretar())
            ev.append(self._evento("escape", f"el objeto se escapa: apretar, consigna {self.f_ref:.0f} -> {nueva:.0f} gf"))
            self.f_ref = nueva
            self.t_accion = None                 # sin esperar al refractario: es urgente
        # perdido: sin fuerza en todos, o el pulgar o todos los dedos vacios en su tope. Solo con el agarre ya
        # establecido o comprobado (antes, un pulgar en su tope sin fuerza aun puede cargarse al apretar los dedos; un
        # agarre en vacio lo ve Mano.agarre al volver de cerrar)
        cmin = float(self.r["contacto_min_gf"])
        vacio = lambda d: Fl[d] < cmin and self.cmds.get(d, 1000) <= self.cierre[d] + 2   # noqa: E731
        bajo = all(Fl[d] < cmin for d in self.dedos) or vacio(T) or all(vacio(d) for d in self.grupo)
        # ... o el apriete por los suelos aunque el lazo cierre: sin objeto, la flexion propia de los dedos al cerrarse
        # sube su fuerza (el pulgar, ~100 gf sobre el cero ya cerca de su tope) y el umbral fijo no la ve
        a = self.coordenadas(F)[0]
        flojo = a < float(self.r["frac_perdido"]) * self.f_ref
        armado = self.t_establecido is not None or self.vigilar_perdida
        if armado and (bajo or flojo):
            self.t_bajo = self.t_bajo if self.t_bajo is not None else t
            espera = float(self.r["t_perdido_s"] if bajo else self.r["t_perdido_lento_s"])
            if t - self.t_bajo >= espera:
                self.perdido = True
                ev.append(self._evento("perdido", f"el objeto se perdio (apriete {a:.0f} gf de {self.f_ref:.0f}): " + ", ".join(
                    f"{NOMBRE[d]} {Fl[d]:.0f} gf" for d in self.dedos)))
                self._fila(t, pos, F, fresca, " ".join(ev))
                return
        else:
            self.t_bajo = None
        self._control(t, pos, F, fresca, ev)
        self._fila(t, pos, F, fresca, " ".join(ev))

    def _control(self, t, pos, F, fresca, ev):
        toca = [self.disp[d].toca(t, fresca) for d in self.dedos]
        if not any(x[0] for x in toca):
            return
        dt = max(x[1] for x in toca if x[0])
        a, b, r = self.coordenadas(F)
        # el RLS (modo 1): el efecto de la accion anterior, cuando ya ha asentado
        if self.est is not None and self.pend is not None and t - self.pend[0] >= float(self.c["rls_espera_s"]):
            _, p0, f0 = self.pend
            self.est.actualiza((pos[T] - p0[0], pos[I] - p0[1]), (F[T] - f0[0], F[I] - f0[1]))
            self.pend = None
        # establecido: el apriete en su banda establecido_s -> se registran balance y reparto y entran sus lazos
        if abs(self.f_ref - a) <= float(self.c["banda_apriete_gf"]):
            self.t_en_banda = self.t_en_banda if self.t_en_banda is not None else t
            if self.t_establecido is None and t - self.t_en_banda >= float(self.c["establecido_s"]):
                self.t_establecido = t
                self.b0, self.r0 = b, r
                self.establecido.set()
                ev.append(self._evento("establecido", f"agarre establecido: apriete {a:.0f} gf (consigna {self.f_ref:.0f}), "
                                                      f"balance {b:+.0f}" + (f", reparto {r:+.0f}" if self.modo == 2 else "")
                                                      + f" a los {t - self.t_inicio_lazo:.2f} s"))
        else:
            self.t_en_banda = None
        if self.t_accion is not None and t - self.t_accion < float(self.c["refractario_s"]):
            return
        u_a = self.pi_a.fuerza_pedida(self.f_ref - a, dt)
        u_b = self.pi_b.fuerza_pedida(self.b0 - b, dt) if self.b0 is not None else 0.0
        u_r = self.pi_r.fuerza_pedida(self.r0 - r, dt) if (self.r0 is not None and self.modo == 2) else 0.0
        if u_a == 0.0 and u_b == 0.0 and u_r == 0.0:
            return
        dF_T = u_a + u_b / 2.0
        dF_G = u_a - u_b / 2.0
        if self.modo == 1:
            dF = np.array([dF_T, dF_G])
        else:
            dF = np.array([dF_T, dF_G / 2.0 + u_r / 2.0, dF_G / 2.0 - u_r / 2.0])
        dq = None
        if self.est is not None:
            iK = self.est.inversa()
            if iK is not None:               # K^-1 da delta de POS; entre cuentas por unidad, el comando
                dq = np.array([(iK[0][0] * dF[0] + iK[0][1] * dF[1]) / self.ratio[0],
                               (iK[1][0] * dF[0] + iK[1][1] * dF[1]) / self.ratio[1]])
        if dq is None:
            dq = self.Jinv @ dF
        mx = float(np.abs(dq).max())
        if mx > float(self.c["dq_max"]):     # escalar el vector, no recortar cada dedo (deshace el desacoplo)
            dq *= float(self.c["dq_max"]) / mx
        nuevos = {}
        for d, x in zip(self.dedos, dq):
            n = self.q[d].cuantiza(float(x), t)
            if n:
                nuevos[d] = self.cmds[d] - n            # n > 0 cierra: ANGLE_SET baja
        if nuevos:
            self._mandar(nuevos)
            self.t_accion = t
            self.n_acciones += 1
            if self.est is not None and self.pend is None:
                self.pend = (t, (pos[T], pos[I]), (F[T], F[I]))

    # --- medidas ------------------------------------------------------------------------------------
    def resumen(self):
        """Lo que se apunta de cada agarre (ensayos del banco y resumen de la tirada)."""
        fases = {}
        for f, v in self.por_fase.items():
            if not v:
                continue
            x = np.array(v)
            fases[f] = {"n": len(x), "mediana": round(float(np.median(x))), "min": round(float(x.min())),
                        "max": round(float(x.max())), "inicio": round(float(np.median(x[:10]))),
                        "final": round(float(np.median(x[-10:])))}
        picos = {f: {NOMBRE[d]: round(v) for d, v in p.items()} for f, p in self.picos.items()}
        return {"modo": self.modo, "regulado": not self.observador, "consigna_gf": self.fuerza_obj,
                "consigna_final_gf": round(self.f_ref), "contacto": {NOMBRE[d]: v for d, v in self.contacto.items()},
                "al_tope": [NOMBRE[d] for d in sorted(self.al_tope)],
                "t_establecido_s": None if self.t_establecido is None else round(self.t_establecido - self.t_inicio_lazo, 2),
                "apriete_por_fase": fases, "picos_por_fase": picos, "perdido": self.perdido, "fallo": self.fallo,
                "acciones": self.n_acciones, "lecturas": self.n_lecturas,
                "K": None if self.est is None else [[round(x, 2) for x in fila] for fila in self.est.K],
                "eventos": [x for x in self.eventos if x["tipo"] != "aproximacion"]}
