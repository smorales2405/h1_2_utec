"""Registro de cada tirada: CSV de rt/lowstate a ~100 Hz, CSV por fotograma y resumen JSON.

Todo va a datos/<tipo>_<fecha>/ con el tiempo en time.monotonic(), el mismo reloj que t_rx de los
fotogramas, para poder cruzar los ficheros. El CSV de lowstate sigue las columnas de
datos_cuadrado/ (cuadrado.py) y anade el mando, que sirve para contar intervenciones.
"""

import csv
import json
import math
import os
import threading
import time

from . import mando as mando_mod
from .geometria import yaw_de

NOMBRES = ["L_HipYaw", "L_HipPitch", "L_HipRoll", "L_Knee", "L_AnklePitch", "L_AnkleRoll",
           "R_HipYaw", "R_HipPitch", "R_HipRoll", "R_Knee", "R_AnklePitch", "R_AnkleRoll",
           "WaistYaw",
           "L_ShoulderPitch", "L_ShoulderRoll", "L_ShoulderYaw", "L_Elbow",
           "L_WristRoll", "L_WristPitch", "L_WristYaw",
           "R_ShoulderPitch", "R_ShoulderRoll", "R_ShoulderYaw", "R_Elbow",
           "R_WristRoll", "R_WristPitch", "R_WristYaw"]


def carpeta_tirada(base, tipo, nombre=None):
    sello = time.strftime("%Y%m%d_%H%M%S")
    carpeta = os.path.join(base, f"{tipo}_{sello}" + (f"_{nombre}" if nombre else ""))
    os.makedirs(carpeta, exist_ok=True)
    return carpeta


def guardar_json(ruta, datos):
    with open(ruta, "w") as f:
        json.dump(datos, f, indent=2, ensure_ascii=False, default=str)


class CsvSimple:
    """CSV con columnas fijas; escribir() acepta un dict y es seguro entre hilos."""

    def __init__(self, ruta, columnas):
        self.columnas = list(columnas)
        self._f = open(ruta, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.columnas)
        self._cerrojo = threading.Lock()
        self.filas = 0

    def escribir(self, fila: dict):
        valores = [_fmt(fila.get(c, "")) for c in self.columnas]
        with self._cerrojo:
            self._w.writerow(valores)
            self.filas += 1

    def cerrar(self):
        with self._cerrojo:
            self._f.flush()
            self._f.close()


def _fmt(v):
    # 12 cifras: time.monotonic() pasa de 10^4 s con el robot encendido unas horas y con 6
    # cifras los instantes se redondeaban a 10 ms (ordenes.csv del 2026-10-03)
    if isinstance(v, float):
        return "" if math.isnan(v) else f"{v:.12g}"
    if isinstance(v, bool):
        return int(v)
    return v


class RegistroLowstate:
    """Se engancha con robot.al_lowstate(reg.al_recibir) y guarda 1 de cada `cada` mensajes.
    `contexto()` devuelve un dict con lo que se este haciendo (estado, orden) para cada fila."""

    def __init__(self, ruta, cada=5, contexto=None):
        self.cada = cada
        self.contexto = contexto or (lambda: {})
        self._n = 0
        cab = ["t", "tick", "estado", "vx_cmd", "vy_cmd", "vyaw_cmd",
               "yaw_deg", "roll_deg", "pitch_deg", "qw", "qx", "qy", "qz",
               "gx", "gy", "gz", "ax", "ay", "az", "botones", "lx", "ly", "rx", "ry"]
        for n in NOMBRES:
            cab += [f"{n}_q", f"{n}_dq", f"{n}_tau", f"{n}_T", f"{n}_err"]
        self._f = open(ruta, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(cab)
        self._cerrojo = threading.Lock()
        self._cerrado = False

    def al_recibir(self, msg, t):
        self._n += 1
        if self._n % self.cada:
            return
        ctx = self.contexto()
        imu = msg.imu_state
        m = mando_mod.decodificar(msg.wireless_remote)
        fila = [f"{t:.4f}", msg.tick, ctx.get("estado", ""),
                f"{ctx.get('vx', 0.0):.3f}", f"{ctx.get('vy', 0.0):.3f}", f"{ctx.get('vyaw', 0.0):.3f}",
                f"{math.degrees(yaw_de(imu.quaternion)):.3f}",
                f"{math.degrees(imu.rpy[0]):.3f}", f"{math.degrees(imu.rpy[1]):.3f}"]
        fila += [f"{v:.5f}" for v in imu.quaternion]
        fila += [f"{v:.5f}" for v in imu.gyroscope]
        fila += [f"{v:.4f}" for v in imu.accelerometer]
        fila += [m.botones, f"{m.lx:.3f}", f"{m.ly:.3f}", f"{m.rx:.3f}", f"{m.ry:.3f}"]
        for s in msg.motor_state[:27]:
            fila += [f"{s.q:.5f}", f"{s.dq:.4f}", f"{s.tau_est:.3f}", s.temperature[0], s.motorstate]
        with self._cerrojo:
            if not self._cerrado:
                self._w.writerow(fila)

    def cerrar(self):
        with self._cerrojo:
            self._cerrado = True
            self._f.flush()
            self._f.close()
