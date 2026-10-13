"""Robot simulado con la interfaz de manip.robot.Robot, para probar brazo, vigilante y supervisor en la PC.

No es fisica: con el peso de arm_sdk en 1 cada junta del brazo sigue su consigna con un retraso de primer
orden y se queda `caida_rad` corta hacia su postura de reposo (con kp 100 el robot real se queda ~5 grados
corto); con el peso en 0 vuelve a la postura de la marcha. La IMU y las piernas no se mueven salvo que una
prueba lo pida (girar(), dar_paso()). Publica "lowstate" a 500 Hz en un hilo, como el DDS.
"""

import math
import threading
import time
from types import SimpleNamespace

import numpy as np

from manip import mando as mando_mod
from manip.mensajes import Mando
from sim.escena import POSTURA_PIE


def _quat_yaw(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


class RobotSimulado:
    def __init__(self, postura=None, tau_s=0.25, caida_rad=0.08, hz=500.0, fsm=201, instantaneo=False):
        """instantaneo: el brazo esta en la consigna en cuanto llega (simulacion acelerada)."""
        self._q = np.array(postura or POSTURA_PIE, float)
        self.reposo = self._q.copy()
        self.tau_s, self.caida_rad, self.hz = tau_s, caida_rad, hz
        self.fsm = fsm
        self.instantaneo = instantaneo
        self.yaw, self.roll, self.pitch = 0.0, 0.0, 0.0
        self.mando = Mando(0, 0.0, 0.0, 0.0, 0.0)
        self.fallos = []
        self._obj = None            # {indice: q} de arm_sdk
        self._peso = 0.0
        self._cerrojo = threading.Lock()
        self._oyentes = []
        self._t = time.monotonic()
        self._vivo = True
        self.n_lowstate = 0
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    # --- lo que publica brazo.py ------------------------------------------------------------------
    def recibir_armsdk(self, obj, peso, kp):
        with self._cerrojo:
            self._obj, self._peso, self._kp = dict(obj), float(peso), dict(kp)
            if self.instantaneo:
                for j in range(12, 27):
                    self._q[j] = (1 - self._peso) * self.reposo[j] + self._peso * self._obj.get(j, self.reposo[j])

    # --- perturbaciones para las pruebas ---------------------------------------------------------
    def girar(self, grados):
        self.yaw += math.radians(grados)

    def dar_paso(self, rad=0.2):
        self._q[3] += rad              # la rodilla izquierda: lo que delata un paso

    # --- bucle -------------------------------------------------------------------------------------
    def _bucle(self):
        dt = 1.0 / self.hz
        while self._vivo:
            with self._cerrojo:
                obj, peso = self._obj, self._peso
                a = min(1.0, dt / self.tau_s)
                for j in range(12, 27):
                    o = self.reposo[j] if obj is None else obj.get(j, self.reposo[j])
                    if j > 12:          # el brazo se queda corto hacia el reposo
                        d = o - self.reposo[j]
                        o = o - math.copysign(min(abs(d), self.caida_rad), d) * (abs(d) > 0.1)
                    meta = (1 - peso) * self.reposo[j] + peso * o
                    self._q[j] += a * (meta - self._q[j])
                self.n_lowstate += 1
                self._t = time.monotonic()
                oyentes = list(self._oyentes)
            msg = self._mensaje()
            for f in oyentes:
                f(msg, self._t)
            time.sleep(dt)

    def _mensaje(self):
        ms = [SimpleNamespace(q=float(self._q[i]) if i < 27 else 0.0, dq=0.0, tau_est=0.0, temperature=[35, 35],
                              motorstate=(1 if i in self.fallos else 0)) for i in range(35)]
        imu = SimpleNamespace(rpy=[self.roll, self.pitch, self.yaw], quaternion=_quat_yaw(self.yaw),
                              gyroscope=[0.0, 0.0, 0.0], accelerometer=[0.0, 0.0, 9.81])
        return SimpleNamespace(tick=self.n_lowstate, motor_state=ms, imu_state=imu, mode_machine=6,
                               wireless_remote=list(mando_mod.codificar(self.mando)))

    # --- interfaz de manip.robot.Robot ----------------------------------------------------------------
    def al_lowstate(self, funcion):
        with self._cerrojo:
            self._oyentes.append(funcion)

    def esperar_lowstate(self, timeout_s=3.0):
        return True

    def ultimo(self):
        return self._mensaje(), self._t

    def edad_lowstate(self):
        return time.monotonic() - self._t

    def estado(self):
        from manip.robot import estado_de
        return estado_de(self._mensaje(), self._t)

    def q(self):
        with self._cerrojo:
            return self._q.copy()

    def mode_machine(self):
        return 6

    def leer_fsm(self):
        return self.fsm, f"GetFsmId -> ret=0 valor={self.fsm} (simulado)"

    def parar(self):
        self._vivo = False
        self._hilo.join(timeout=1.0)

