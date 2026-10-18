"""Mano Inspire simulada con la interfaz del cliente Modbus de pymodbus, para probar manip/mano.py sin robot.

Cada DOF va hacia su ANGLE_SET a una velocidad proporcional a SPEED_SET. La fuerza leida es el residual de
flexion del propio dedo (70-230 gf al cerrar, caracterizacion del indice) mas el contacto: si hay un
objeto, a partir de su angulo de contacto la fuerza sube 12 gf por cuenta (k_c medida) y el firmware para el
dedo al llegar a FORCE_SET. Como en banco, un dedo que llego a FORCE_SET deja de aceptar ANGLE_SET hasta
que FORCE_SET quede por encima de su fuerza. Se puede pedir que un dedo no responda (error 5) hasta CLEAR.
"""

import time
from types import SimpleNamespace

import numpy as np

from manip import mano as m

CUENTAS_POR_S_POR_SPEED = 3.0      # SPEED_SET calibra el actuador: k = 2.99-3.05 (caracterizacion)
K_CONTACTO = 12.0                  # gf por cuenta contra un objeto


class ManoSimulada:
    def __init__(self):
        self.ang = np.full(6, 1000.0)
        self.obj = np.full(6, 1000.0)
        self.fset = np.full(6, 2000.0)
        self.speed = np.full(6, 1000.0)
        self.contacto = {}             # dof -> angulo de contacto (objeto en la mano)
        self.atascado = set()          # dofs que no responden hasta CLEAR
        self.error = np.zeros(6)
        self.enclavado = np.zeros(6, bool)
        self._t = time.monotonic()
        self.escrituras = []
        self.conectada = True

    def connect(self):
        return self.conectada

    def close(self):
        pass

    def poner_objeto(self, angulos_contacto):
        """{dof: angulo al que el dedo toca el objeto}; {} = sin objeto."""
        self.contacto = dict(angulos_contacto)

    def _fuerza(self):
        f = 70 + 160 * (1000 - self.ang) / 1000            # residual de flexion
        for k, a in self.contacto.items():
            if self.ang[k] < a:
                f[k] += K_CONTACTO * (a - self.ang[k])
        return f

    def _avanzar(self):
        ahora = time.monotonic()
        dt = ahora - self._t
        self._t = ahora
        for k in range(6):
            if k in self.atascado:
                continue
            paso = CUENTAS_POR_S_POR_SPEED * self.speed[k] * dt
            d = self.obj[k] - self.ang[k]
            nuevo = self.ang[k] + max(-paso, min(paso, d))
            if d < 0 and k in self.contacto and nuevo < self.contacto[k]:
                # contra el objeto: avanza hasta que la fuerza llega a FORCE_SET y para ahi
                # f(a) = 70 + 0.16 (1000 - a) + K (contacto - a) = FORCE_SET  ->  a
                tope = (230 + K_CONTACTO * self.contacto[k] - self.fset[k]) / (K_CONTACTO + 0.16)
                if nuevo <= tope:
                    nuevo = tope
                    self.enclavado[k] = True
            self.ang[k] = nuevo
        f = self._fuerza()
        self.enclavado &= f >= self.fset - 1

    # --- interfaz de pymodbus ------------------------------------------------------------------------
    def read_holding_registers(self, a, n, slave=1):
        self._avanzar()
        if a == m.ANGLE_ACT:
            v = self.ang
        elif a == m.FORCE_ACT:
            v = self._fuerza()
        elif a in m.BLOQUES_BYTE:
            # un byte por DOF, dos por registro (el primero en el alto); lo que sigue al bloque da 0xFFFF
            by = self.error if a == m.ERROR else np.zeros(6)
            v = [(int(by[2 * k]) << 8) | int(by[2 * k + 1]) for k in range(3)] + [0xFFFF] * 3
        elif a == m.FORCE_SET:
            v = self.fset
        else:
            v = np.zeros(n)
        regs = [int(round(x)) & 0xFFFF for x in np.asarray(v)[:n]]
        return SimpleNamespace(registers=regs, isError=lambda: False)

    def write_registers(self, a, valores, slave=1):
        self._avanzar()
        self.escrituras.append((a, list(valores)))
        v = [x - 0x10000 if x >= 0x8000 else x for x in valores]
        if a == m.ANGLE_SET:
            for k, x in enumerate(v):
                if x >= 0 and not self.enclavado[k]:
                    self.obj[k] = x
        elif a == m.FORCE_SET:
            self.fset = np.array(v, float)
            self.enclavado &= self._fuerza() >= self.fset - 1
        elif a == m.SPEED_SET:
            self.speed = np.array(v, float)
        elif a == m.CLEAR:
            self.atascado.clear()
            self.error[:] = 0
        return SimpleNamespace(isError=lambda: False)
