"""Registro de cada tirada (seccion 5 del PDF): CSV a 50 Hz, imagenes de cada agarre y resumen JSON.

Todo va a datos/<tipo>_<fecha>[_nombre]/ con el tiempo en time.monotonic(), el mismo reloj que t_rx de
los fotogramas, para cruzar los ficheros:
    lowstate.csv   rt/lowstate a 50 Hz: IMU, mando, las 27 juntas (q, dq, tau, temperatura, estado), la
                   consigna de los brazos y la cintura, el peso de arm_sdk y el estado del supervisor
    manos.csv      cada lectura de las manos: angulo, fuerza, base, FORCE_SET, error y estado por DOF
    eventos.csv    decisiones del supervisor y avisos
    agarre_*.png   imagen de color de cada agarre (antes de bajar y al cerrar)
    resumen.json   argumentos, calibracion usada, motivo de fin y metricas
"""

import csv
import json
import math
import os
import threading
import time

import cv2
import numpy as np

from . import mando as mando_mod

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


def _json_por_defecto(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return str(x)


def guardar_json(ruta, datos):
    with open(ruta, "w") as f:
        json.dump(datos, f, indent=2, ensure_ascii=False, default=_json_por_defecto)


def yaw_de(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _fmt(v):
    # 12 cifras: time.monotonic() pasa de 10^4 s con el robot encendido unas horas
    if isinstance(v, (float, np.floating)):
        return "" if math.isnan(v) else f"{float(v):.12g}"
    if isinstance(v, bool):
        return int(v)
    return v


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
            if self._f.closed:
                return
            self._w.writerow(valores)
            self.filas += 1

    def cerrar(self):
        with self._cerrojo:
            if not self._f.closed:
                self._f.flush()
                self._f.close()


COLUMNAS_MANO = (["t", "lado"] + [f"ang{k}" for k in range(6)] + [f"f{k}" for k in range(6)]
                 + [f"base{k}" for k in range(6)] + [f"fset{k}" for k in range(6)]
                 + [f"err{k}" for k in range(6)] + [f"est{k}" for k in range(6)] + ["tactil_yema"])


def fila_mano(e):
    """EstadoMano -> fila de manos.csv."""
    fila = {"t": e.t, "lado": e.lado, "tactil_yema": e.tactil_yema if e.tactil_yema is not None else ""}
    for k in range(6):
        fila[f"ang{k}"] = int(e.angulo[k])
        fila[f"f{k}"] = int(e.fuerza[k])
        fila[f"base{k}"] = int(e.base[k])
        fila[f"fset{k}"] = int(e.force_set[k])
        fila[f"err{k}"] = int(e.error[k])
        fila[f"est{k}"] = int(e.estado[k])
    return fila


class RegistroLowstate:
    """Se engancha con robot.al_lowstate(reg.al_recibir) y guarda 1 de cada `cada` mensajes (500 Hz / 10 =
    50 Hz). `contexto()` devuelve {estado, peso, obj: {indice: q}} para cada fila."""

    def __init__(self, ruta, cada=10, contexto=None):
        self.cada = cada
        self.contexto = contexto or (lambda: {})
        self._n = 0
        cab = ["t", "tick", "estado", "peso", "yaw_deg", "roll_deg", "pitch_deg", "qw", "qx", "qy", "qz",
               "gx", "gy", "gz", "ax", "ay", "az", "botones", "lx", "ly", "rx", "ry"]
        for n in NOMBRES:
            cab += [f"{n}_q", f"{n}_dq", f"{n}_tau", f"{n}_T", f"{n}_err"]
        cab += [f"{NOMBRES[j]}_obj" for j in range(12, 27)]
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
        fila = [f"{t:.4f}", msg.tick, ctx.get("estado", ""), f"{ctx.get('peso', 0.0):.3f}",
                f"{math.degrees(yaw_de(imu.quaternion)):.3f}",
                f"{math.degrees(imu.rpy[0]):.3f}", f"{math.degrees(imu.rpy[1]):.3f}"]
        fila += [f"{v:.5f}" for v in imu.quaternion]
        fila += [f"{v:.5f}" for v in imu.gyroscope]
        fila += [f"{v:.4f}" for v in imu.accelerometer]
        fila += [m.botones, f"{m.lx:.3f}", f"{m.ly:.3f}", f"{m.rx:.3f}", f"{m.ry:.3f}"]
        for s in msg.motor_state[:27]:
            fila += [f"{s.q:.5f}", f"{s.dq:.4f}", f"{s.tau_est:.3f}", s.temperature[0], s.motorstate]
        obj = ctx.get("obj") or {}
        fila += [f"{obj[j]:.5f}" if j in obj else "" for j in range(12, 27)]
        with self._cerrojo:
            if not self._cerrado:
                self._w.writerow(fila)

    def cerrar(self):
        with self._cerrojo:
            self._cerrado = True
            self._f.flush()
            self._f.close()


class Registro:
    """La carpeta de una tirada con sus CSV, imagenes y resumen."""

    def __init__(self, base, tipo, nombre=None):
        self.carpeta = carpeta_tirada(base, tipo, nombre)
        self.manos = CsvSimple(os.path.join(self.carpeta, "manos.csv"), COLUMNAS_MANO)
        self.eventos = CsvSimple(os.path.join(self.carpeta, "eventos.csv"), ["t", "estado", "motivo", "detalle"])
        self.lowstate = None
        self.resumen = {"inicio": time.strftime("%Y-%m-%d %H:%M:%S"), "eventos": []}
        self._t0 = time.monotonic()

    def enganchar_lowstate(self, robot, contexto, cada=10):
        self.lowstate = RegistroLowstate(os.path.join(self.carpeta, "lowstate.csv"), cada, contexto)
        robot.al_lowstate(self.lowstate.al_recibir)

    def mano(self, estado_mano):
        self.manos.escribir(fila_mano(estado_mano))

    def evento(self, estado, motivo, detalle=None, imprimir=True):
        t = time.monotonic()
        self.eventos.escribir({"t": t, "estado": estado, "motivo": motivo,
                               "detalle": json.dumps(detalle, default=_json_por_defecto) if detalle else ""})
        self.resumen["eventos"].append([round(t - self._t0, 2), estado, motivo])
        if imprimir:
            print(f"[{t - self._t0:7.1f} s] {estado}: {motivo}", flush=True)

    def imagen(self, nombre, img):
        if img is not None:
            cv2.imwrite(os.path.join(self.carpeta, f"{nombre}.png"), img)

    def cerrar(self, **resumen):
        self.resumen.update(resumen)
        self.resumen["duracion_s"] = round(time.monotonic() - self._t0, 2)
        for c in (self.lowstate, self.manos, self.eventos):
            if c is not None:
                c.cerrar()
        guardar_json(os.path.join(self.carpeta, "resumen.json"), self.resumen)
