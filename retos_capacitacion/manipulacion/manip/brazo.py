"""Brazo (seccion 5 del PDF): trayectorias articulares -> rt/arm_sdk a frecuencia fija, con rampas de peso.

Patron de caja_cuadrado.py: un hilo publica a `hz` la consigna de la cintura (12) y los dos brazos (13-26)
con el peso de mezcla en motor_cmd[27].q, el CRC y el mode_machine leido del lowstate. Los topes de la
seccion 10 se aplican aqui, diga lo que diga la configuracion (que puede bajarlos, no subirlos):
  - kp del brazo <= 150; la cintura se sujeta en la q que tenia al empezar (kp 200, como el ejemplo oficial)
  - cada movimiento dura >= max(8 s, t_min_mov_s) y su velocidad articular de pico (minima sacudida:
    1.875 dq / T) no pasa de 0.5 rad/s: si hace falta, la duracion se alarga
  - rampa de entrada de 12 s siguiendo la postura medida con un tope de 2 grados; salida en >= 5 s
En simulacro NO se crea el publicador de rt/arm_sdk: lo que se publicaria va a `destino` (el robot simulado
o el registro). Solo el supervisor cambia el peso.
"""

import threading
import time

import numpy as np

from .mensajes import CINTURA, JUNTAS_BRAZO

KP_TOPE = 150.0
VEL_TOPE = 0.5
T_MIN_TOPE = 8.0
T_SALIDA_MIN = 5.0
T_ENTRADA_MIN = 12.0
PESO_ARM_SDK = 27
JUNTAS = list(range(12, 27))
BRAZOS = list(range(13, 27))


def minima_sacudida(x):
    return x ** 3 * (10 - 15 * x + 6 * x * x)


def duracion_movimiento(dq_max, t_min, vel_max, escala=1.0, pedida=None):
    """>= max(8 s, t_min), la pedida y la que deja el pico de velocidad (1.875 dq / T) en vel_max <= 0.5."""
    return max(max(t_min, T_MIN_TOPE), pedida or 0.0, 1.875 * dq_max / min(vel_max, VEL_TOPE)) * max(1.0, escala)


class Brazo:
    def __init__(self, robot, cfg, simulacro=False, destino=None, escala_tiempo=1.0, dormir=time.sleep, sincrono=False):
        """dormir: la espera entre consignas (las pruebas pasan una que no espera; en el robot, time.sleep).
        sincrono: en la simulacion acelerada, `destino` recibe cada consigna en el mismo paso (no solo el hilo)."""
        b = cfg["brazo"]
        self.robot = robot
        self.simulacro = simulacro
        self._dormir = dormir
        self._sincrono = sincrono and destino is not None
        self.destino = destino
        self.hz = float(b["hz"])
        self.kp = {j: (b["kp_cintura"] if j == CINTURA else min(b["kp"], KP_TOPE)) for j in JUNTAS}
        self.kd = {j: (b["kd_cintura"] if j == CINTURA else b["kd"]) for j in JUNTAS}
        self.t_min = max(float(b["t_min_mov_s"]), T_MIN_TOPE)
        self.vel_max = min(float(b["vel_max_rad_s"]), VEL_TOPE)
        self.t_entrada = max(float(b["t_rampa_entrada_s"]), T_ENTRADA_MIN)
        self.t_salida = max(float(b["t_rampa_salida_s"]), T_SALIDA_MIN)
        self.tope_seguir = float(b["tope_seguir_rad"])
        self.escala_tiempo = max(1.0, float(escala_tiempo))
        m = float(b["margen_limites_rad"])
        self.limites = {int(j): (lo + m, hi - m) for j, (lo, hi) in b["limites"].items()}
        q0 = robot.q()
        self.q_inicial = q0.copy()
        self.obj = {j: float(q0[j]) for j in JUNTAS}       # consigna actual
        self.peso = 0.0
        self.cerrojo = threading.Lock()
        self.publicados = 0
        self._pub = None
        if not simulacro:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
            from unitree_sdk2py.utils.crc import CRC
            self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self._pub.Init()
            self._crc = CRC()
        self.mode_machine = robot.mode_machine()
        self._vivo = True
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    # --- publicacion ---------------------------------------------------------------------------------
    def _bucle(self):
        periodo = 1.0 / self.hz
        while self._vivo:
            t = time.monotonic()
            with self.cerrojo:
                obj, peso = dict(self.obj), self.peso
            if self._pub is not None:
                self._publicar(obj, peso)
            if self.destino is not None:
                self.destino(obj, peso, self.kp)
            self.publicados += 1
            time.sleep(max(0.0, periodo - (time.monotonic() - t)))

    def _publicar(self, obj, peso):
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = self.mode_machine
        for j, q in obj.items():
            c = cmd.motor_cmd[j]
            c.mode, c.q, c.dq, c.tau, c.kp, c.kd = 1, q, 0.0, 0.0, self.kp[j], self.kd[j]
        cmd.motor_cmd[PESO_ARM_SDK].q = float(min(max(peso, 0.0), 1.0))
        cmd.crc = self._crc.Crc(cmd)
        self._pub.Write(cmd)

    # --- utilidades -----------------------------------------------------------------------------------
    def recortar(self, j, q):
        lo, hi = self.limites[j]
        return min(max(float(q), lo), hi)

    def duracion(self, ini, fin, pedida=None):
        """La duracion de un movimiento de `ini` a `fin` ({j: q}): >= t_min, la pedida y la que deja el pico
        de velocidad en vel_max; todo por la escala de tiempo (2 en las primeras tiradas)."""
        dq = max((abs(fin[j] - ini[j]) for j in fin), default=0.0)
        return duracion_movimiento(dq, self.t_min, self.vel_max, self.escala_tiempo, pedida)

    def consigna(self):
        with self.cerrojo:
            return dict(self.obj)

    def pose_brazo(self, lado, q7):
        return {j: float(v) for j, v in zip(JUNTAS_BRAZO[lado], q7)}

    def error(self, lados=("izq", "der")):
        q = self.robot.q()
        with self.cerrojo:
            return max(abs(q[j] - self.obj[j]) for l in lados for j in JUNTAS_BRAZO[l])

    # --- movimientos ---------------------------------------------------------------------------------
    def rampa_peso(self, hasta, segundos=None, comprobar=None):
        """Rampa del peso de arm_sdk. Al subir, la consigna sigue la postura medida sin separarse mas de
        tope_seguir de la inicial (caja_cuadrado.py: con la postura fija la politica dio pasos; copiandola
        sin tope dejo caer el codo 18 grados). Al bajar, >= t_salida."""
        subir = hasta > self.peso
        segundos = max(segundos or 0.0, self.t_entrada if subir else self.t_salida)
        with self.cerrojo:
            q_ini = dict(self.obj)
            desde = self.peso
        pasos = max(1, int(segundos * self.hz))
        for k in range(1, pasos + 1):
            with self.cerrojo:
                if subir and self.tope_seguir > 0:
                    q = self.robot.q()
                    for j in BRAZOS:
                        self.obj[j] = q_ini[j] + max(-self.tope_seguir, min(self.tope_seguir, q[j] - q_ini[j]))
                self.peso = desde + (hasta - desde) * k / pasos
            self._paso()
            if comprobar:
                comprobar()
            self._dormir(1.0 / self.hz)
        return segundos

    def ir_a(self, pose, segundos=None, comprobar=None):
        """Minima sacudida desde la consigna actual hasta `pose` {junta: q} (recortada a los limites).
        Devuelve la duracion usada."""
        with self.cerrojo:
            ini = dict(self.obj)
        fin = dict(ini)
        for j, q in pose.items():
            if j == CINTURA:
                continue                      # la cintura no se mueve: se sujeta donde estaba
            fin[j] = self.recortar(j, q)
        T = self.duracion(ini, fin, segundos)
        pasos = max(1, int(T * self.hz))
        for k in range(1, pasos + 1):
            s = minima_sacudida(k / pasos)
            with self.cerrojo:
                for j in BRAZOS:
                    self.obj[j] = ini[j] + (fin[j] - ini[j]) * s
            self._paso()
            if comprobar:
                comprobar()
            self._dormir(1.0 / self.hz)
        return T

    def _paso(self):
        if self._sincrono:
            with self.cerrojo:
                obj, peso = dict(self.obj), self.peso
            self.destino(obj, peso, self.kp)

    def seguir(self, lado, puntos, comprobar=None, al_llegar=None):
        """Recorre los puntos de paso (manip.mensajes.PuntoPaso) del brazo `lado`. al_llegar(i) tras cada uno."""
        for i, p in enumerate(puntos):
            self.ir_a(self.pose_brazo(lado, p.q), p.duracion, comprobar)
            if al_llegar:
                al_llegar(i)

    def congelar(self):
        """La consigna pasa a la postura medida: el brazo se queda donde esta, sin seguir empujando."""
        q = self.robot.q()
        with self.cerrojo:
            for j in BRAZOS:
                self.obj[j] = self.recortar(j, q[j])

    def parar(self):
        """Deja de publicar. Solo con el peso ya en 0 (si no, el brazo quedaria con la ultima consigna)."""
        if self.peso > 0.01:
            raise RuntimeError(f"parar() con el peso de arm_sdk en {self.peso:.2f}: primero rampa_peso(0)")
        self._vivo = False
        self._hilo.join(timeout=1.0)
