"""Cuerpo: las 27 juntas por rt/lowcmd con el robot en el arnes (PLAN, Hito 0). Sustituye a brazo.py
(rt/arm_sdk) desde que el control de marcha de FSM 201 daba pasos con los brazos (2026-10-15).

Un hilo publica a `hz` (500) un LowCmd_ con las 27 juntas, el CRC y el mode_machine leido:
  - consigna: la postura de cada junta (trayectorias de minima sacudida evaluadas en el propio hilo);
  - kp, kd: piernas de config (legs_hold), brazos y cintura del juego de gains.yaml (tuned_gff), por un
    factor de 0 a 1 (rampa de toma y de suelta: con el factor en 0 solo queda kd = amortiguacion);
  - tau: el par que sostiene la postura (manip/dinamica.py: colgado y de pie mezclados con `apoyo`, una
    rampa que manda el procedimiento, NO la carga medida: con ella el par de las piernas y la carga estimada
    se realimentan y el robot rebota) mas el estabilizador: par de los tobillos con la inclinacion y la
    velocidad angular de la IMU respecto de una referencia tomada al encenderlo.
La carga medida en los pies (del par de las piernas, con la tara tomada colgado) solo se muestra y apaga el
estabilizador al colgar.
Vigilancia en el propio hilo: un motor en fallo, el lowstate viejo o L2+B en el mando -> amortiguacion rapida
(en Debug el robot no hace nada con L2+B: lo hace este hilo; medido el 2026-10-16); la inclinacion fuera de
limites -> estabilizador fuera y la postura sostenida (la cuerda recoge al robot).
Si se deja de publicar, los motores mantienen la ultima orden ~0.2 s y luego quedan sin par (medido): un
programa que muere deja caer al robot en la cuerda.
Sin publicador (simulacro o simulacion) el comando va a `destino(cmd)`.
"""

import math
import os
import threading
import time

import numpy as np

from . import config, mando
from .brazo import VEL_TOPE, T_MIN_TOPE, duracion_movimiento, minima_sacudida
from .dinamica import G, Dinamica
from .mensajes import JUNTAS_BRAZO

N = 27
PIERNAS = list(range(12))
CINTURA = 12
BRAZOS = list(range(13, 27))
TOBILLO_PITCH, TOBILLO_ROLL = (4, 10), (5, 11)
RUTA_GAINS = "h1_2_joint_control/config/gains.yaml"
JUNTAS_GAINS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")


def leer_ganancias(nombre, ruta=None):
    """{junta 12-26: (kp, kd)} del juego `nombre` de gains.yaml."""
    ruta = ruta or os.path.join(config.ruta_repo(), RUTA_GAINS)
    if not os.path.exists(ruta):
        raise FileNotFoundError(f"No esta {ruta} (en el PC2 lo copia ./desplegar.sh)")
    juegos = config._leer(ruta)["sets"]
    if nombre not in juegos:
        raise KeyError(f"'{nombre}' no esta en {RUTA_GAINS}: {', '.join(juegos)}")
    J = juegos[nombre]["joints"]
    sal = {CINTURA: (float(J["waist_yaw"]["kp"]), float(J["waist_yaw"]["kd"]))}
    for lado, pre in (("izq", "L_"), ("der", "R_")):
        for j, n in zip(JUNTAS_BRAZO[lado], JUNTAS_GAINS):
            sal[j] = (float(J[pre + n]["kp"]), float(J[pre + n]["kd"]))
    return sal


def grados(v):
    return np.radians(np.asarray(v, float))


class Rampa:
    """Valor que va hacia `meta` a velocidad constante (1/T por segundo)."""

    def __init__(self, valor=0.0):
        self.valor, self.meta, self.vel = float(valor), float(valor), 1.0

    def a(self, meta, segundos):
        self.meta = float(meta)
        self.vel = abs(self.meta - self.valor) / max(segundos, 1e-3)

    def avanzar(self, dt):
        d = self.meta - self.valor
        self.valor = self.meta if abs(d) <= self.vel * dt else self.valor + math.copysign(self.vel * dt, d)
        return self.valor

    @property
    def quieta(self):
        return self.valor == self.meta


class Cuerpo:
    def __init__(self, robot, cfg, dinamica=None, publicar=True, destino=None, reloj=time.monotonic,
                 dormir=time.sleep, ruta_gains=None):
        c = cfg["cuerpo"]
        self.robot, self.c = robot, c
        self.reloj, self._dormir = reloj, dormir
        self.destino = destino
        self.din = dinamica or Dinamica()
        self.hz = float(c["hz"])
        self.dt = 1.0 / self.hz
        b = cfg["brazo"]
        self.t_min = max(float(b["t_min_mov_s"]), T_MIN_TOPE)
        self.vel_max = min(float(b["vel_max_rad_s"]), VEL_TOPE)
        m = float(b["margen_limites_rad"])
        self.limites = {int(j): (lo + m, hi - m) for j, (lo, hi) in b["limites"].items()}
        # ganancias
        self.kp, self.kd = np.zeros(N), np.zeros(N)
        self.kp[:12] = np.tile(np.asarray(c["piernas"]["kp"], float), 2)
        self.kd[:12] = np.tile(np.asarray(c["piernas"]["kd"], float), 2)
        for j, (kp, kd) in leer_ganancias(c["ganancias_brazos"], ruta_gains).items():
            self.kp[j], self.kd[j] = kp, kd
        # posturas
        self.postura_pie = grados(c["postura_pie_deg"])
        self.brazos_cero = grados(c["brazos_cero_deg"])
        self.brazos_colgado = grados(c["brazos_colgado_deg"])
        e = c["estabilizador"]
        self.K, self.D, self.tau_estab_max = float(e["kp_nm_rad"]), float(e["kd_nm_s_rad"]), float(e["tau_max_nm"])
        self.carga_min, self.carga_apagar, self.t_estab = float(e["carga_min"]), float(e["carga_apagar"]), float(e["rampa_s"])
        # integral (roll, pitch). En roll no: el torso lo sostienen las piernas (dos apoyos), el par del tobillo
        # casi no lo mueve y la integral solo se acumulaba hasta el tope (+20 N m con la cuerda, 2026-10-16)
        self.Ki = np.array([float(e.get("ki_roll_nm_rad_s", 0.0)), float(e.get("ki_nm_rad_s", 0.0))])
        self.integral_max = float(e.get("integral_max_nm", 0.0))
        # banda muerta: con la cuerda llevando parte del peso, la cuerda fija la inclinacion a medio grado y la
        # integral se acumulaba hasta el tope peleando contra ella (2026-10-16)
        self.integral_banda = math.radians(float(e.get("integral_banda_deg", 0.0)))
        # referencia fija (nivelar): la que tuvo el robot en FSM 201 con la misma postura; None = la medida
        self.ref_objetivo = [None if e.get(k) is None else math.radians(float(e[k])) for k in ("ref_roll_deg", "ref_pitch_deg")]
        self.vel_ref = math.radians(float(e.get("ref_vel_deg_s", 0.3)))
        # la referencia solo avanza si el torso la sigue: si no, el robot real se atascaba con los tobillos al
        # limite y luego saltaba de +8 a -5 grados (2026-10-16)
        self.ref_seguimiento = math.radians(float(e.get("ref_seguimiento_deg", 1.0)))
        sesgo = e.get("sesgo_tobillo_nm", {}) or {}
        self.sesgo_tobillo = np.array([float(sesgo.get("roll", 0.0)), float(sesgo.get("pitch", 0.0))])
        tmax = e.get("tau_tobillo_max_nm", {}) or {}
        self.tau_tobillo_max = np.array([float(tmax.get("roll", 30.0)), float(tmax.get("pitch", 50.0))])
        self._saturado = np.zeros(2)                # -1/0/+1 por eje (roll, pitch) en el ultimo ciclo
        lim = c["limites"]
        self.inclinacion_max = math.radians(float(lim["inclinacion_max_deg"]))
        self.edad_max = float(lim["edad_lowstate_max_s"])
        self.temp_max = float(lim["temperatura_max_c"])
        self.escala_carga = float(c["carga"]["escala"])
        self.alfa_carga = 1.0 - math.exp(-10 * self.dt / float(c["carga"]["filtro_s"]))   # se mide 1 de cada 10
        # estado (bajo el cerrojo)
        self.cerrojo = threading.Lock()
        msg, _ = robot.ultimo()
        self.mode_machine = int(msg.mode_machine)
        self.q_des = np.array([msg.motor_state[i].q for i in range(N)], float)
        self._tray = None
        self.f_kp, self.f_ff, self.f_estab = Rampa(0.0), Rampa(0.0), Rampa(0.0)
        self.apoyo = Rampa(0.0)                     # 0 colgado, 1 de pie: el par de sosten
        self.ref = np.zeros(2)                      # roll, pitch de referencia del estabilizador
        self.integral = np.zeros(2)                 # N m por tobillo (roll, pitch): lo que falta al modelo de pie
        self.nivelado = False                       # nivelar(): la referencia va a la fija y la integral actua
        # orientacion con la que se calcula el par de sosten: la IMU muy filtrada (o la referencia con el
        # estabilizador encendido). Con la inclinacion instantanea el par anula la gravedad que el estabilizador
        # necesita, el conjunto queda muy rigido y casi sin amortiguar, y oscila (simulado el 2026-10-15)
        self.rp_ff = None
        self.alfa_rp = 1.0 - math.exp(-10 * self.dt / 2.0)
        self.carga = 0.0                            # filtrada y escalada
        self.carga_cruda = 0.0
        self.fz = {"izq": 0.0, "der": 0.0}
        self.fz_crudo = {"izq": 0.0, "der": 0.0}    # sin tara ni escala, para calibrar
        self.tara = {"izq": 0.0, "der": 0.0}        # lo que marca cada pie colgado (tarar())
        self._fz_hist = []
        self.tau_ff = np.zeros(N)
        self.tau_estab = np.zeros(N)
        self.amortiguado, self.motivo = False, ""
        self.avisos = []                            # (t, texto) de lo que hizo la vigilancia
        self.ultimo_cmd = None
        self.estado = {}                            # lo ultimo medido, para la pantalla
        self.al_ciclo = None                        # funcion(cuerpo, msg) para el registro
        self.n, self.dt_max, self.errores = 0, 0.0, 0
        self._t_prev = None
        self._imu_hist = []
        self._pub = None
        if publicar:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
            from unitree_sdk2py.utils.crc import CRC
            self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            self._pub.Init()
            self._crc = CRC()
        self._vivo = False
        self._hilo = None

    # --- hilo ---------------------------------------------------------------------------------------
    def iniciar(self, hilo=True):
        """Empieza a publicar la postura medida con kp 0 (solo kd: lo mismo que la amortiguacion)."""
        self._vivo = True
        if hilo:
            self._hilo = threading.Thread(target=self._bucle, daemon=True)
            self._hilo.start()

    def _bucle(self):
        t_sig = self.reloj()
        while self._vivo:
            try:
                self.ciclo()
            except Exception as ex:                  # el hilo no puede morir: se sigue mandando lo ultimo bueno
                self.errores += 1
                if self.errores == 1 or self.errores % 500 == 0:
                    import traceback
                    self._aviso(f"ERROR en el ciclo ({self.errores}): {ex!r} {traceback.format_exc(limit=3)}")
                if self.ultimo_cmd is not None and self._vivo:
                    try:
                        self._enviar(self.ultimo_cmd)
                    except Exception:
                        pass
            t_sig += self.dt
            espera = t_sig - self.reloj()
            if espera > 0:
                self._dormir(espera)
            elif espera < -0.05:                     # muy retrasado: no recuperar a rafagas
                t_sig = self.reloj()

    def parar(self):
        """Deja de publicar. Solo amortiguado del todo (con kp > 0 los motores quedarian con la ultima orden)."""
        if self.f_kp.valor > 0.0:
            raise RuntimeError(f"parar() con el factor de kp en {self.f_kp.valor:.2f}: primero soltar()")
        self._vivo = False
        if self._hilo is not None:
            self._hilo.join(timeout=1.0)

    # --- un ciclo -------------------------------------------------------------------------------------
    def ciclo(self):
        ahora = self.reloj()
        dt = self.dt if self._t_prev is None else min(ahora - self._t_prev, 0.05)
        if self._t_prev is not None:
            self.dt_max = max(self.dt_max, ahora - self._t_prev)
        self._t_prev = ahora
        msg, t_msg = self.robot.ultimo()
        edad = ahora - t_msg
        ms = msg.motor_state
        q = np.array([ms[i].q for i in range(N)])
        tau = np.array([ms[i].tau_est for i in range(N)])
        imu = msg.imu_state
        roll, pitch = float(imu.rpy[0]), float(imu.rpy[1])
        gx, gy = float(imu.gyroscope[0]), float(imu.gyroscope[1])
        with self.cerrojo:
            self.n += 1
            # vigilancia
            fallos = [i for i in range(N) if ms[i].motorstate != 0]
            if fallos and not self.amortiguado:
                self._amortiguar(float(self.c["t_soltar_fallo_s"]), f"motores en fallo: {fallos}")
            if edad > self.edad_max and not self.amortiguado:
                self._amortiguar(float(self.c["t_soltar_fallo_s"]), f"rt/lowstate de hace {edad:.2f} s")
            if not self.amortiguado and mando.mascara(msg.wireless_remote) & mando.L2B == mando.L2B:
                self._amortiguar(float(self.c["t_soltar_fallo_s"]), "L2+B del mando")
            temp = max(int(ms[i].temperature[0]) for i in range(N))
            # trayectoria
            if self._tray is not None:
                t0, T, ini, fin = self._tray
                x = min(1.0, (ahora - t0) / T)
                self.q_des = ini + (fin - ini) * minima_sacudida(x)
                if x >= 1.0:
                    self._tray = None
            # carga y par de sosten (a hz/10)
            if self.n % 10 == 1:
                fz, _ = self.din.carga_pies(q, tau, roll, pitch)
                self.fz_crudo = fz
                self._fz_hist = [h for h in self._fz_hist if ahora - h[0] <= 2.0] + [(ahora, fz)]
                self.fz = {k: (v - self.tara[k]) * self.escala_carga for k, v in fz.items()}
                self.carga_cruda = (self.fz["izq"] + self.fz["der"]) / (self.din.masa * G)
                self.carga += self.alfa_carga * (self.carga_cruda - self.carga)
                rp = np.array([roll, pitch])
                self.rp_ff = rp if self.rp_ff is None else self.rp_ff + self.alfa_rp * (rp - self.rp_ff)
                r_ff, p_ff = self.ref if self.f_estab.meta > 0.0 else self.rp_ff
                apoyo = self.apoyo.avanzar(10 * dt)
                self.tau_ff = self.din.sosten(self.q_des, apoyo, r_ff, p_ff)
                # lo que el robot real pide de pie y el modelo no da (~5 grados entre el URDF y el robot en
                # cadera/rodilla: +13 N m de pitch por tobillo con la cuerda floja, 2026-10-16)
                self.tau_ff[list(TOBILLO_ROLL)] += apoyo * self.sesgo_tobillo[0]
                self.tau_ff[list(TOBILLO_PITCH)] += apoyo * self.sesgo_tobillo[1]
                self._imu_hist = [h for h in self._imu_hist if ahora - h[0] <= 3.0] + [(ahora, roll, pitch)]
            # estabilizador
            f_e = self.f_estab.avanzar(dt)
            if f_e > 0.0 and f_e == self.f_estab.meta and self.carga < self.carga_apagar:
                self._apagar_estab(1.0, f"carga {self.carga:.2f} < {self.carga_apagar}: estabilizador fuera")
            e_r, e_p = roll - self.ref[0], pitch - self.ref[1]
            if self.f_estab.meta > 0.0 and max(abs(e_r), abs(e_p)) > self.inclinacion_max:
                self._apagar_estab(0.2, f"inclinacion {math.degrees(e_r):+.1f}/{math.degrees(e_p):+.1f} grados: "
                                        "estabilizador fuera, se sostiene la postura")
            # nivelado (lo pide el procedimiento con la cuerda floja, no la carga medida, que se mueve con el par
            # de los tobillos): la referencia va despacio a la fija y la integral quita lo que el modelo de pie no
            # explica (sin ella el robot real se quedo 3.4 grados adelante, 2026-10-16); si no, se mantienen
            if self.nivelado and f_e >= 1.0 and self.f_estab.meta > 0.0 and self.apoyo.valor >= 1.0:
                err = np.array([e_r, e_p])
                for k, obj in enumerate(self.ref_objetivo):
                    if obj is not None and abs(err[k]) < self.ref_seguimiento:
                        self.ref[k] += float(np.clip(obj - self.ref[k], -self.vel_ref * dt, self.vel_ref * dt))
                e_r, e_p = roll - self.ref[0], pitch - self.ref[1]
                for k, ek in enumerate((e_r, e_p)):
                    if abs(ek) <= self.integral_banda:
                        continue
                    if self._saturado[k] == 0 or np.sign(ek) != self._saturado[k]:   # sin acumular saturado
                        self.integral[k] = float(np.clip(self.integral[k] + self.Ki[k] * ek * dt,
                                                         -self.integral_max, self.integral_max))
            f_kp, f_ff = self.f_kp.avanzar(dt), self.f_ff.avanzar(dt)
            self.tau_estab[:] = 0.0
            self._saturado[:] = 0
            if f_e > 0.0:
                u = np.array([self.K * e_r + self.D * gx + self.integral[0], self.K * e_p + self.D * gy + self.integral[1]])
                uc = np.clip(u, -self.tau_estab_max, self.tau_estab_max)
                self._saturado[:] = np.sign(u) * (u != uc)
                for k, juntas in enumerate((TOBILLO_ROLL, TOBILLO_PITCH)):
                    for j in juntas:
                        # el total del tobillo (sosten + estabilizador) dentro de su limite: el PD pone el resto
                        # hasta el maximo del motor (60 N m pitch, 40 roll)
                        ff = self.tau_ff[j] * f_ff
                        t = float(np.clip(ff + f_e * uc[k], -self.tau_tobillo_max[k], self.tau_tobillo_max[k]))
                        if t != ff + f_e * uc[k]:
                            self._saturado[k] = np.sign(uc[k]) or self._saturado[k]
                        self.tau_estab[j] = t - ff
            cmd = {"q": self.q_des.copy(), "dq": np.zeros(N), "kp": self.kp * f_kp, "kd": self.kd.copy(),
                   "tau": self.tau_ff * f_ff + self.tau_estab}
            self.estado = {"t": ahora, "q": q, "tau": tau, "roll": roll, "pitch": pitch, "edad": edad, "temp": temp,
                           "carga": self.carga, "fz": dict(self.fz), "fz_crudo": dict(self.fz_crudo),
                           "f_kp": f_kp, "f_estab": f_e, "integral": self.integral.copy(),
                           "apoyo": self.apoyo.valor}
            self.ultimo_cmd = cmd
        if self._vivo:
            self._enviar(cmd)
        if self.al_ciclo is not None:
            self.al_ciclo(self, msg)

    def _enviar(self, cmd):
        if self._pub is not None:
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            m = unitree_hg_msg_dds__LowCmd_()
            m.mode_pr = 0
            m.mode_machine = self.mode_machine
            for i in range(N):
                c = m.motor_cmd[i]
                c.mode, c.q, c.dq = 1, float(cmd["q"][i]), 0.0
                c.kp, c.kd, c.tau = float(cmd["kp"][i]), float(cmd["kd"][i]), float(cmd["tau"][i])
            m.crc = self._crc.Crc(m)
            self._pub.Write(m)
        if self.destino is not None:
            self.destino(cmd)

    # --- ordenes (desde el hilo principal) ------------------------------------------------------------
    def _aviso(self, texto):
        self.avisos.append((self.reloj(), texto))

    def _amortiguar(self, segundos, motivo):
        if self.amortiguado and self.f_kp.meta == 0.0 and self.f_kp.valor <= self.f_kp.vel * segundos:
            return                                  # ya se suelta igual o mas deprisa
        self.amortiguado, self.motivo = True, motivo
        self._tray = None
        self.f_kp.a(0.0, segundos)
        self.f_ff.a(0.0, segundos)
        self.f_estab.a(0.0, min(segundos, 0.2))
        self._aviso(f"amortiguacion: {motivo}")

    def _apagar_estab(self, segundos, motivo):
        self.f_estab.a(0.0, segundos)
        self._aviso(motivo)

    def tomar(self, segundos=None):
        """kp y el par de sosten de 0 a 1 sosteniendo la postura medida (sin mover nada)."""
        T = float(segundos or self.c["t_toma_s"])
        with self.cerrojo:
            if self.amortiguado:
                raise RuntimeError(f"amortiguado ({self.motivo})")
            self.q_des = self.robot_q()
            self.f_kp.a(1.0, T)
            self.f_ff.a(1.0, T)
        return T

    def soltar(self, segundos=None, motivo="fin"):
        """kp y el par a 0 (queda kd: amortiguacion), desde donde este."""
        with self.cerrojo:
            self._amortiguar(float(segundos or self.c["t_soltar_s"]), motivo)

    def robot_q(self):
        msg, _ = self.robot.ultimo()
        return np.array([msg.motor_state[i].q for i in range(N)], float)

    def recortar(self, q27):
        q = np.array(q27, float)
        for j, (lo, hi) in self.limites.items():
            q[j] = min(max(q[j], lo), hi)
        return q

    def mover(self, fin, segundos=None, juntas=None, esperar=True, comprobar=None):
        """Minima sacudida de la consigna actual a `fin` (27 valores, o {junta: q}) en las `juntas` dadas
        (todas si None). Duracion >= max(8 s, la pedida, la que deja el pico en 0.5 rad/s). Devuelve T."""
        with self.cerrojo:
            if self.amortiguado:
                raise RuntimeError(f"amortiguado ({self.motivo})")
            ini = self.q_des.copy()
            obj = ini.copy()
            if isinstance(fin, dict):
                for j, v in fin.items():
                    obj[j] = v
            else:
                sel = range(N) if juntas is None else juntas
                for j in sel:
                    obj[j] = fin[j]
            obj = self.recortar(obj)
            T = duracion_movimiento(float(np.abs(obj - ini).max()), self.t_min, self.vel_max, 1.0, segundos)
            self._tray = (self.reloj(), T, ini, obj)
        if esperar:
            self.esperar(comprobar)
        return T

    def moviendo(self):
        with self.cerrojo:
            return self._tray is not None

    def esperar(self, comprobar=None, cada=0.02):
        while self.moviendo():
            if comprobar:
                comprobar()
            self._dormir(cada)

    def congelar(self, juntas=BRAZOS):
        """La consigna de esas juntas pasa a la postura medida y se para la trayectoria."""
        q = self.robot_q()
        with self.cerrojo:
            self._tray = None
            for j in juntas:
                self.q_des[j] = q[j]

    def consigna(self):
        with self.cerrojo:
            return self.q_des.copy()

    def tarar(self, maximo=0.25):
        """Colgado, con los pies sin tocar nada: lo que marca cada pie (media de ~2 s) pasa a ser su cero.
        Colgado del arnes el modelo deja ~-40 N por pie (2026-10-16): sin tara la carga sale ~10 % baja.
        Devuelve la tara (N, sin escala); RuntimeError si parece que los pies se apoyan."""
        with self.cerrojo:
            h = self._fz_hist
            if not h or h[-1][0] - h[0][0] < 1.0:
                raise RuntimeError("faltan medidas de la carga (menos de 1 s)")
            t = {k: float(np.mean([f[k] for _, f in h])) for k in ("izq", "der")}
            parte = (t["izq"] + t["der"]) / (self.din.masa * G)
            if abs(parte) > maximo:
                raise RuntimeError(f"colgado, los pies marcan el {parte * self.escala_carga:.0%} del peso: "
                                   "o se apoyan o la medida no vale")
            self.tara = t
        return t

    def nivelar(self, si=True):
        """De pie con la cuerda floja: la referencia del estabilizador va a la fija (ref_*_deg, a ref_vel_deg_s) y
        la integral actua. nivelar(False) (al colgar) deja las dos donde esten. Devuelve los segundos que tarda."""
        with self.cerrojo:
            if si and (self.f_estab.meta == 0.0 or self.apoyo.meta < 1.0):
                raise RuntimeError("nivelar solo de pie: con el estabilizador y el par de pie")
            if si and not self.nivelado:             # sale de donde esta el torso: sin un salto en el error
                h = [x[1:] for x in self._imu_hist if self._imu_hist[-1][0] - x[0] <= 1.0]
                self.ref = np.array(h).mean(axis=0) if h else self.ref
            self.nivelado = bool(si)
            d = [abs(o - r) for o, r in zip(self.ref_objetivo, self.ref) if o is not None]
        return max(d, default=0.0) / self.vel_ref

    def apoyar(self, de_pie, segundos=3.0):
        """El par de sosten pasa a de pie (True) o a colgado (False) en `segundos`."""
        with self.cerrojo:
            self.apoyo.a(1.0 if de_pie else 0.0, segundos)
        return segundos

    def estabilizador(self, encender, segundos=None):
        """Encender toma como referencia la inclinacion media de los ultimos 3 s (colgado a medias el robot se
        balancea despacio en la cuerda); exige carga >= carga_min."""
        T = float(segundos or self.t_estab)
        with self.cerrojo:
            if encender:
                if self.amortiguado:
                    raise RuntimeError(f"amortiguado ({self.motivo})")
                if self.carga < self.carga_min:
                    raise RuntimeError(f"los pies llevan el {self.carga:.0%} del peso (< {self.carga_min:.0%})")
                h = np.array([x[1:] for x in self._imu_hist] or [(0.0, 0.0)])
                self.ref = h.mean(axis=0)
                self.integral = np.zeros(2)
                self.nivelado = False
                self.f_estab.a(1.0, T)
            else:
                self.f_estab.a(0.0, T)
        return T
