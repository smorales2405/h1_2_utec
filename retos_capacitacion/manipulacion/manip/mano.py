"""Mano (seccion 5 del PDF): ordenes de alto nivel (preforma, cerrar, sujetar, abrir) -> Modbus TCP, y estado.

Inspire RH56DFTP, izquierda 192.168.124.211 y derecha .210 (al reves que la documentacion de Unitree).
DOF 0-5: menique, anular, medio, indice, flexion del pulgar, rotacion del pulgar; 0 cerrado, 1000 abierto.
Lo aprendido (memoria del proyecto, caja_cuadrado.py e inspire_hand_interface/Caracterizacion):
  - cada bloque de registros se lee por separado (1546..1620 de una vez da basura)
  - FORCE_SET (<= 800 gf) se escribe ANTES de cada cierre y la consigna queda en cerrado del todo: el
    firmware aprieta hasta esa fuerza y la mantiene (fijarla en el angulo actual suelta el objeto)
  - SPEED <= 100 al cerrar: el sobreimpulso lo manda la velocidad (+202 gf a 100, +637 a 250 con FS 500)
  - si FORCE_ACT llega a FORCE_SET el dedo deja de aceptar ANGLE_SET (medido en banco): para abrir se sube
    FORCE_SET primero y se comprueba con ANGLE_ACT que abrio; si no, se borra el error (1004) y se reintenta
  - la linea base de fuerza, con la mano abierta y >= 10 s desde la ultima suelta (el cero se corre 21-46 gf)
  - 1005 (SAVE) graba en la memoria de la mano: nunca se escribe
"""

import struct
import threading
import time

import numpy as np

from .mensajes import EstadoMano, Parada

ANGLE_SET, FORCE_SET, SPEED_SET = 1486, 1498, 1522
ANGLE_ACT, FORCE_ACT, ERROR, ESTADO = 1546, 1582, 1606, 1612
CLEAR, SAVE = 1004, 1005
BLOQUES_BYTE = (ERROR, ESTADO)                # un byte por DOF: 3 registros (inspire_sdkpy los lee asi)
TACTIL_INDICE = ((4110, 9), (4128, 96))     # punta y yema del indice (zonas de caja_cuadrado.py)
DEDOS = (0, 1, 2, 3)
PULGAR, ROTACION = 4, 5


class ManoSinRespuesta(Parada):
    pass


def bytes_de(registros):
    """ERROR, STATUS y TEMP van en bytes, dos DOF por registro y el primero en el byte alto (como inspire_sdkpy).
    Son 3 registros, no 6: los 3 siguientes quedan fuera del bloque y la mano los da como 0xFFFF (-1)."""
    return [(int(r) >> s) & 0xFF for r in registros for s in (8, 0)]


class Mano:
    def __init__(self, lado, cfg, cliente=None, dormir=time.sleep):
        """cliente: un ModbusTcpClient (o el simulado de sim/mano_simulada.py en las pruebas)."""
        m = cfg["mano"]
        self.lado = lado
        self.p = m
        self.fs_max = min(int(m["force_set_max"]), 800)
        self.fs_abierta = min(int(m["force_set_abierta"]), self.fs_max)
        self.corte = int(m["corte_gf"])
        self.speed_cierre = min(int(m["speed_cierre"]), 100)
        self.speed_abrir = int(m["speed_abrir"])
        self.cierre = [int(x) for x in m.get("cierre", [0, 0, 0, 0, 300, -1])]
        self._dormir = dormir
        if cliente is None:
            from pymodbus.client import ModbusTcpClient
            cliente = ModbusTcpClient(cfg["red"]["manos"][lado], port=cfg["red"]["puerto_manos"], timeout=1)
        self.c = cliente
        if not self.c.connect():
            raise ManoSinRespuesta(f"no conecta con la mano {lado}")
        self.cerrojo = threading.Lock()
        self.base = np.zeros(6)
        self.base_tactil = None
        self.force_set = np.full(6, -1.0)
        self.t_ultima_suelta = -1e9
        self.t_ultima_lectura = 0.0
        self.ultimo = None
        self.escribir(SPEED_SET, [self.speed_cierre] * 6)
        self.escribir_force_set([self.fs_abierta] * 6)

    # --- registros -----------------------------------------------------------------------------------
    def _crudos(self, a, n):
        with self.cerrojo:
            r = self.c.read_holding_registers(a, n, 1)
        if r is None or (hasattr(r, "isError") and r.isError()):
            raise ManoSinRespuesta(f"mano {self.lado}: fallo al leer {a}")
        return r

    def regs(self, a, n, signo=True):
        if a in BLOQUES_BYTE:
            return np.array(bytes_de(self._crudos(a, 3).registers), dtype=float)
        r = self._crudos(a, n)
        f = "h" if signo else "H"
        return np.array(struct.unpack(f">{n}{f}", struct.pack(f">{n}H", *r.registers)), dtype=float)

    def escribir(self, a, v):
        if a == SAVE or a <= SAVE < a + len(v):
            raise ValueError("1005 (SAVE) graba en la mano: nunca")
        with self.cerrojo:
            r = self.c.write_registers(a, [int(x) & 0xFFFF for x in v], 1)
        if r is not None and hasattr(r, "isError") and r.isError():
            raise ManoSinRespuesta(f"mano {self.lado}: fallo al escribir {a}")

    def escribir_force_set(self, fs):
        """gf por DOF, recortados al tope; -1 (no cierra) deja el de mano abierta."""
        v = [self.fs_abierta if x is None or x < 0 else min(int(x), self.fs_max) for x in fs]
        self.escribir(FORCE_SET, v)
        self.force_set = np.array(v, float)

    def mover(self, angulos):
        """ANGLE_SET de los 6 DOF; -1 = no tocar ese DOF."""
        self.escribir(ANGLE_SET, [-1 if a is None or a < 0 else int(min(max(a, 0), 1000)) for a in angulos])

    def borrar_error(self):
        self.escribir(CLEAR, [1])

    def leer(self) -> EstadoMano:
        t = time.monotonic()
        e = EstadoMano(t=t, lado=self.lado, angulo=self.regs(ANGLE_ACT, 6), fuerza=self.regs(FORCE_ACT, 6),
                       base=self.base.copy(), force_set=self.force_set.copy(), error=self.regs(ERROR, 6),
                       estado=self.regs(ESTADO, 6))
        self.t_ultima_lectura, self.ultimo = t, e
        return e

    def tactil_yema(self):
        """Lo que sube el tactil de la punta y la yema del indice sobre su base (maximo, unidades del sensor)."""
        v = np.concatenate([self.regs(a, n, signo=False) for a, n in TACTIL_INDICE])
        if self.base_tactil is None:
            return 0.0
        return float((v - self.base_tactil).max())

    # --- ordenes ------------------------------------------------------------------------------------
    def _esperar_quieta(self, timeout_s, tol=6, quieta_s=0.3, vigilar=None):
        """Espera a que ANGLE_ACT deje de cambiar. Vigila la fuerza: por encima del corte, Parada."""
        t0 = time.monotonic()
        prev, desde = None, None
        while True:
            e = self.leer()
            if vigilar:
                vigilar()
            if (e.fuerza - e.base).max() > self.corte:
                self.mover([1000, 1000, 1000, 1000, 1000, -1])
                raise Parada(f"mano {self.lado}: {int(e.fuerza.max())} gf > corte de {self.corte}")
            if prev is not None and np.abs(e.angulo - prev).max() <= tol:
                desde = desde or time.monotonic()
                if time.monotonic() - desde >= quieta_s:
                    return e
            else:
                desde = None
            prev = e.angulo
            if time.monotonic() - t0 > timeout_s:
                return e
            self._dormir(0.05)

    def preforma(self, angulos, vigilar=None):
        """Lleva la mano a una preforma. Si cambia la rotacion del pulgar, primero abre su flexion (si gira
        con los dedos cerrados, choca con ellos)."""
        e = self.leer()
        if angulos[ROTACION] >= 0 and abs(e.angulo[ROTACION] - angulos[ROTACION]) > 50:
            self.escribir_force_set([self.fs_abierta] * 6)
            self.escribir(SPEED_SET, [self.speed_abrir] * 6)
            self.mover([-1, -1, -1, -1, 1000, -1])
            self._esperar_quieta(2.0, vigilar=vigilar)
            self.mover([-1, -1, -1, -1, -1, angulos[ROTACION]])
            self._esperar_quieta(2.0, vigilar=vigilar)
        self.escribir(SPEED_SET, [self.speed_abrir] * 6)
        self.mover(angulos)
        return self._esperar_quieta(3.0, vigilar=vigilar)

    def cerrar(self, force_set, vigilar=None):
        """Cierra los DOF con force_set > 0: FORCE_SET primero, velocidad lenta, consigna cerrada del todo."""
        self.escribir_force_set(force_set)
        self.escribir(SPEED_SET, [self.speed_cierre] * 6)
        self.mover([self.cierre[k] if force_set[k] is not None and force_set[k] > 0 else -1 for k in range(6)])
        return self._esperar_quieta(4.0, vigilar=vigilar)

    def sujetar(self, force_set):
        """Reafirma FORCE_SET y la consigna cerrada (cada ~1 s mientras se lleva el objeto)."""
        self.escribir_force_set(force_set)
        self.mover([self.cierre[k] if force_set[k] is not None and force_set[k] > 0 else -1 for k in range(6)])

    def abrir(self, vigilar=None):
        """Abre la flexion del pulgar y los dedos, sin girar el pulgar, y comprueba que abrio."""
        for intento in range(2):
            self.escribir_force_set([self.fs_abierta] * 6)      # desenclava los dedos que llegaron a FORCE_SET
            self.escribir(SPEED_SET, [self.speed_abrir] * 6)
            self.mover([1000, 1000, 1000, 1000, 1000, -1])
            e = self._esperar_quieta(self.p["t_abrir_s"], vigilar=vigilar)
            if e.angulo[:5].min() >= self.p["abierta_min"]:
                self.t_ultima_suelta = time.monotonic()
                return e
            self.borrar_error()
            self._dormir(0.2)
        raise ManoSinRespuesta(f"mano {self.lado}: no abre (angulos {e.angulo.astype(int).tolist()})")

    def linea_base(self, n=10):
        """Fuerza (y tactil del indice) con la mano abierta y sin carga. Exige >= t_base_s desde la ultima
        suelta: tras sujetar, el cero se corre 21-46 gf y tarda ~8 s en volver."""
        espera = self.p["t_base_s"] - (time.monotonic() - self.t_ultima_suelta)
        if espera > 0:
            self._dormir(espera)
        f = np.mean([self.leer().fuerza for _ in range(n)], 0)
        self.base = f
        self.base_tactil = np.mean([np.concatenate([self.regs(a, k, signo=False) for a, k in TACTIL_INDICE])
                                    for _ in range(3)], 0)
        return f

    def agarre(self, e: EstadoMano, force_set, vacio_max=60, fuerza_min=80):
        """La pista del PDF: un dedo que se detiene lejos de 0 con fuerza tiene algo delante; uno que llega a
        0 esta vacio. (ok, {dof: "lleno" | "vacio" | "flojo"}). Hacen falta el pulgar y al menos un dedo."""
        detalle = {}
        for k in range(5):
            if force_set[k] is None or force_set[k] <= 0:
                continue
            neta = e.fuerza[k] - e.base[k]
            if e.angulo[k] <= max(vacio_max, self.cierre[k] + vacio_max if self.cierre[k] > 0 else 0):
                detalle[k] = "vacio"
            elif neta >= fuerza_min:
                detalle[k] = "lleno"
            else:
                detalle[k] = "flojo"
        llenos = [k for k, v in detalle.items() if v == "lleno"]
        return (PULGAR in llenos and any(k in llenos for k in DEDOS)), detalle

    def cerrar_conexion(self):
        with self.cerrojo:
            self.c.close()
