"""Entrada/salida del robot: rt/lowstate (IMU, motores, mando), la FSM y el LocoClient.

Es el unico modulo que importa el SDK de Unitree. Corre como usuario unitree, nunca con sudo:
como root el DDS no arranca (fs.protected_regular=2 y /tmp/cdds.LOG es de unitree).

La marcha no se habilita al crear el objeto: hace falta habilitar_marcha(), que solo llama el
supervisor fuera del simulacro. mover() recorta siempre a los topes de la seccion 10 del PDF,
diga lo que diga la configuracion.
"""

import math
import re
import subprocess
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from . import mando as mando_mod
from .geometria import yaw_de
from .mensajes import Imu, Mando

TOPIC_LOWSTATE = "rt/lowstate"
LEE_FSM = "/home/unitree/robotics40/lectores_h1_2/build/lee_fsm"   # solo se ejecuta, no se toca
FSM_MARCHA = (201, 204)

# Topes duros del reto (seccion 10): la configuracion puede bajarlos, no subirlos.
VX_TOPE = 0.4
VY_TOPE = 0.2
VYAW_TOPE = 0.5


def recortar(v, tope):
    return max(-tope, min(tope, v))


class Robot:
    def __init__(self, iface="eth0"):
        self.iface = iface
        ChannelFactoryInitialize(0, iface)
        self._cerrojo = threading.Lock()
        self._msg = None
        self._t_msg = 0.0
        self.n_lowstate = 0
        self._oyentes = []
        self._sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self._sub.Init(self._al_recibir, 10)
        self._loco = None

    # --- rt/lowstate ---------------------------------------------------------------------
    def _al_recibir(self, msg: LowState_):
        ahora = time.monotonic()
        with self._cerrojo:
            self._msg = msg
            self._t_msg = ahora
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

    def imu(self) -> Imu:
        msg, t = self.ultimo()
        s = msg.imu_state
        return Imu(t=t, tick=msg.tick, roll=s.rpy[0], pitch=s.rpy[1], yaw=yaw_de(s.quaternion),
                   gx=s.gyroscope[0], gy=s.gyroscope[1], gz=s.gyroscope[2])

    def mando(self) -> Mando:
        return mando_mod.decodificar(self.ultimo()[0].wireless_remote)

    def motores_en_fallo(self):
        msg, _ = self.ultimo()
        return [i for i, m in enumerate(msg.motor_state[:27]) if m.motorstate != 0]

    def temp_max(self):
        msg, _ = self.ultimo()
        return max(m.temperature[0] for m in msg.motor_state[:27])

    # --- FSM -------------------------------------------------------------------------------
    def leer_fsm(self):
        """(fsm o None, texto). lee_fsm pregunta GetFsmId por RPC: solo lectura."""
        try:
            out = subprocess.run([LEE_FSM, self.iface], capture_output=True, text=True, timeout=15).stdout
        except Exception as e:
            return None, f"no se pudo ejecutar lee_fsm: {e}"
        m = re.search(r"GetFsmId\s*->\s*ret=(-?\d+)\s+valor=(-?\d+)", out)
        if not m:
            return None, f"salida de lee_fsm sin GetFsmId: {out.strip()[:200]}"
        return (int(m.group(2)) if int(m.group(1)) == 0 else None), out.strip()

    # --- marcha ----------------------------------------------------------------------------
    def habilitar_marcha(self):
        from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
        self._loco = LocoClient()
        self._loco.SetTimeout(10.0)   # el defecto de 1 s da 3104
        self._loco.Init()

    @property
    def marcha_habilitada(self):
        return self._loco is not None

    def mover(self, vx, vy, vyaw):
        """Move dura 1 s en el robot: si este proceso muere, el robot para solo."""
        if self._loco is None:
            raise RuntimeError("marcha no habilitada")
        # continous_move=True dejaria la orden 864000 s: nunca
        self._loco.Move(recortar(vx, VX_TOPE), recortar(vy, VY_TOPE), recortar(vyaw, VYAW_TOPE),
                        continous_move=False)

    def parar(self):
        """Velocidad 0 durante 1 s: el robot deja de trasladarse y de girar y se queda de pie. Es lo
        mismo que StopMove del SDK (SetVelocity(0, 0, 0)), pero devuelve el codigo del RPC."""
        if self._loco is not None:
            return self._loco.SetVelocity(0.0, 0.0, 0.0, 1.0)
        return None
