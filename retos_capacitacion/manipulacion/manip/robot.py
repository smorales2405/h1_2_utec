"""Entrada del robot: rt/lowstate (q, dq, par, IMU, mando, motores) y la FSM. Solo lee: no publica nada.

Con brazo.py, el unico modulo que importa el SDK de Unitree. Corre como usuario unitree, nunca con sudo
(como root el DDS no arranca). No hay LocoClient: el PDF prohibe Move (seccion 10).
"""

import math
import re
import subprocess
import threading
import time

import numpy as np

from . import mando as mando_mod
from .mensajes import EstadoRobot, Imu

TOPIC_LOWSTATE = "rt/lowstate"
LEE_FSM = "/home/unitree/robotics40/lectores_h1_2/build/lee_fsm"   # solo se ejecuta, no se toca
FSM_VALIDA = 201


def yaw_de(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def estado_de(msg, t) -> EstadoRobot:
    s = msg.imu_state
    ms = msg.motor_state
    return EstadoRobot(t=t, q=np.array([ms[i].q for i in range(27)]), dq=np.array([ms[i].dq for i in range(27)]),
                       tau=np.array([ms[i].tau_est for i in range(27)]),
                       imu=Imu(t=t, tick=msg.tick, roll=s.rpy[0], pitch=s.rpy[1], yaw=yaw_de(s.quaternion),
                               gx=s.gyroscope[0], gy=s.gyroscope[1], gz=s.gyroscope[2]),
                       mando=mando_mod.decodificar(msg.wireless_remote),
                       motores_en_fallo=[i for i in range(27) if ms[i].motorstate != 0],
                       mode_machine=int(msg.mode_machine))


def leer_fsm(iface="eth0"):
    """(fsm o None, texto). lee_fsm pregunta GetFsmId por RPC: solo lectura."""
    try:
        out = subprocess.run([LEE_FSM, iface], capture_output=True, text=True, timeout=15).stdout
    except Exception as e:
        return None, f"no se pudo ejecutar lee_fsm: {e}"
    m = re.search(r"GetFsmId\s*->\s*ret=(-?\d+)\s+valor=(-?\d+)", out)
    if not m:
        return None, f"salida de lee_fsm sin GetFsmId: {out.strip()[:200]}"
    return (int(m.group(2)) if int(m.group(1)) == 0 else None), out.strip()


class Robot:
    def __init__(self, iface="eth0"):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        self.iface = iface
        ChannelFactoryInitialize(0, iface)
        self._cerrojo = threading.Lock()
        self._msg = None
        self._t_msg = 0.0
        self.n_lowstate = 0
        self._oyentes = []
        self._sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self._sub.Init(self._al_recibir, 10)

    def _al_recibir(self, msg):
        ahora = time.monotonic()
        with self._cerrojo:
            self._msg, self._t_msg = msg, ahora
            self.n_lowstate += 1
            oyentes = list(self._oyentes)
        for f in oyentes:
            f(msg, ahora)

    def al_lowstate(self, funcion):
        """funcion(msg, t) se llama desde el hilo del DDS con cada rt/lowstate (~500 Hz)."""
        with self._cerrojo:
            self._oyentes.append(funcion)

    def esperar_lowstate(self, timeout_s=3.0):
        t0 = time.monotonic()
        while self.ultimo()[0] is None and time.monotonic() - t0 < timeout_s:
            time.sleep(0.05)
        return self.ultimo()[0] is not None

    def ultimo(self):
        with self._cerrojo:
            return self._msg, self._t_msg

    def edad_lowstate(self):
        msg, t = self.ultimo()
        return math.inf if msg is None else time.monotonic() - t

    def estado(self) -> EstadoRobot:
        msg, t = self.ultimo()
        return estado_de(msg, t)

    def q(self) -> np.ndarray:
        msg, _ = self.ultimo()
        return np.array([msg.motor_state[i].q for i in range(27)])

    def mode_machine(self):
        return int(self.ultimo()[0].mode_machine)

    def leer_fsm(self):
        return leer_fsm(self.iface)


class LectorFsm:
    """Lee la FSM en segundo plano cada `cada_s` (lee_fsm tarda ~1 s): el vigilante consulta el ultimo valor."""

    def __init__(self, robot, cada_s=2.0):
        self.robot, self.cada_s = robot, cada_s
        self.fsm, self.t = None, 0.0
        self._fin = threading.Event()
        self._hilo = threading.Thread(target=self._bucle, daemon=True)

    def iniciar(self):
        self.fsm, _ = self.robot.leer_fsm()
        self.t = time.monotonic()
        self._hilo.start()
        return self.fsm

    def _bucle(self):
        while not self._fin.wait(self.cada_s):
            fsm, _ = self.robot.leer_fsm()
            self.fsm, self.t = fsm, time.monotonic()

    def parar(self):
        self._fin.set()
