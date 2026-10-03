"""Fuentes de fotogramas con la misma interfaz: la camara en vivo (ZMQ) o un dataset grabado.

    fuente.siguiente(timeout_s) -> Fotograma | None    (None = no llego nada a tiempo / fin)
    fuente.meta                                        intrinsecos y escala de profundidad

La percepcion solo ve Fotograma, asi corre igual sobre el dataset que en vivo (seccion 5 del PDF).

Formato del dataset (carpeta datos/<tipo>_<fecha>/):
    fotogramas.csv   n, t_cam, t_rx, emisor, ir, color, prof (nombres de fichero o vacio)
    meta.json        intrinsecos del primer fotograma
    ir/*.png  color/*.jpg  prof/*.png (16 bit)
"""

import csv
import json
import os
import queue
import threading
import time

import cv2
import numpy as np

from .mensajes import Fotograma
from . import fotograma_zmq


class FuenteZmq:
    """Suscriptor de camara_servidor.py. Con solo_ultimo=True descarta lo atrasado (control);
    con solo_ultimo=False entrega todo lo que cabe en la cola (grabacion)."""

    def __init__(self, direccion="tcp://127.0.0.1:5556", solo_ultimo=True, cola=4):
        import zmq  # solo en el robot (teleop_venv); la PC no lo necesita para el dataset
        self._zmq = zmq
        self._ctx = zmq.Context.instance()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.RCVHWM, max(2, cola))
        self._sub.setsockopt(zmq.LINGER, 0)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.connect(direccion)
        self._solo_ultimo = solo_ultimo
        self.meta = {}
        self.recibidos = 0

    def siguiente(self, timeout_s=1.0):
        zmq = self._zmq
        if not self._sub.poll(int(timeout_s * 1000)):
            return None
        partes = self._sub.recv_multipart()
        if self._solo_ultimo:
            while True:
                try:
                    partes = self._sub.recv_multipart(zmq.NOBLOCK)
                except zmq.Again:
                    break
        f = fotograma_zmq.decodificar(partes)
        self.meta = f.meta
        self.recibidos += 1
        return f

    def cerrar(self):
        self._sub.close()


class EscritorDataset:
    """Escribe fotogramas en un hilo aparte para no frenar al que los recibe. Si el disco no
    da abasto descarta (y cuenta) en vez de acumular retraso."""

    def __init__(self, carpeta, cola=60):
        self.carpeta = carpeta
        os.makedirs(carpeta, exist_ok=True)
        self._f = open(os.path.join(carpeta, "fotogramas.csv"), "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["n", "t_cam", "t_rx", "emisor", "ir", "color", "prof"])
        self._cola = queue.Queue(maxsize=cola)
        self._meta_escrita = False
        self.escritos = 0
        self.descartados = 0
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def escribir(self, f: Fotograma):
        try:
            self._cola.put_nowait(f)
        except queue.Full:
            self.descartados += 1

    def _bucle(self):
        while True:
            f = self._cola.get()
            if f is None:
                return
            self._guardar(f)

    def _guardar(self, f: Fotograma):
        if not self._meta_escrita:
            with open(os.path.join(self.carpeta, "meta.json"), "w") as fm:
                json.dump(f.meta, fm, indent=1)
            self._meta_escrita = True
        nombres = {}
        for flujo, ext, params in (("ir", "png", [cv2.IMWRITE_PNG_COMPRESSION, 1]),
                                   ("color", "jpg", [cv2.IMWRITE_JPEG_QUALITY, 95]),
                                   ("prof", "png", [cv2.IMWRITE_PNG_COMPRESSION, 1])):
            arr = getattr(f, flujo)
            if arr is None:
                nombres[flujo] = ""
                continue
            os.makedirs(os.path.join(self.carpeta, flujo), exist_ok=True)
            nombres[flujo] = f"{flujo}/{f.n:07d}.{ext}"
            cv2.imwrite(os.path.join(self.carpeta, nombres[flujo]), arr, params)
        self._w.writerow([f.n, f"{f.t_cam:.6f}", f"{f.t_rx:.6f}", int(f.emisor),
                          nombres["ir"], nombres["color"], nombres["prof"]])
        self.escritos += 1

    def cerrar(self):
        self._cola.put(None)
        self._hilo.join(timeout=30)
        self._f.close()


class FuenteDataset:
    """Lee un dataset grabado. Con tiempo_real=True respeta los intervalos de t_rx (repeticion
    del simulacro); si no, entrega los fotogramas tan rapido como se pidan."""

    def __init__(self, carpeta, tiempo_real=False):
        self.carpeta = carpeta
        with open(os.path.join(carpeta, "fotogramas.csv")) as f:
            self._filas = list(csv.DictReader(f))
        ruta_meta = os.path.join(carpeta, "meta.json")
        self.meta = {}
        if os.path.exists(ruta_meta):
            with open(ruta_meta) as fm:
                self.meta = json.load(fm)
        self._i = 0
        self._tiempo_real = tiempo_real
        self._t0_real = None
        self._t0_datos = None

    def __len__(self):
        return len(self._filas)

    def siguiente(self, timeout_s=None):
        if self._i >= len(self._filas):
            return None
        fila = self._filas[self._i]
        self._i += 1
        f = Fotograma(n=int(fila["n"]), t_cam=float(fila["t_cam"]), t_rx=float(fila["t_rx"]),
                      emisor=bool(int(fila["emisor"])), meta=self.meta)
        if fila["ir"]:
            f.ir = cv2.imread(os.path.join(self.carpeta, fila["ir"]), cv2.IMREAD_GRAYSCALE)
        if fila["color"]:
            f.color = cv2.imread(os.path.join(self.carpeta, fila["color"]), cv2.IMREAD_COLOR)
        if fila["prof"]:
            f.prof = cv2.imread(os.path.join(self.carpeta, fila["prof"]), cv2.IMREAD_UNCHANGED)
        if self._tiempo_real:
            if self._t0_real is None:
                self._t0_real, self._t0_datos = time.monotonic(), f.t_rx
            espera = (f.t_rx - self._t0_datos) - (time.monotonic() - self._t0_real)
            if espera > 0:
                time.sleep(espera)
        return f

    def __iter__(self):
        while True:
            f = self.siguiente()
            if f is None:
                return
            yield f

    def cerrar(self):
        pass


class ImuDataset:
    """La IMU del lowstate.csv de un dataset, interpolada a cualquier instante (p. ej. el t_rx de
    un fotograma: los dos usan time.monotonic() del PC2)."""

    def __init__(self, ruta_csv):
        t, roll, pitch, yaw = [], [], [], []
        with open(ruta_csv) as f:
            for fila in csv.DictReader(f):
                t.append(float(fila["t"]))
                roll.append(float(fila["roll_deg"]))
                pitch.append(float(fila["pitch_deg"]))
                yaw.append(float(fila["yaw_deg"]))
        self.t = np.array(t)
        self.roll = np.radians(roll)
        self.pitch = np.radians(pitch)
        self.yaw = np.unwrap(np.radians(yaw))   # sin saltos de +-180: se puede interpolar

    def en(self, t):
        """(roll, pitch, yaw) en rad en `t`; fuera del registro, el extremo mas cercano."""
        return (float(np.interp(t, self.t, self.roll)), float(np.interp(t, self.t, self.pitch)),
                float(np.interp(t, self.t, self.yaw)))


def prof_en_metros(f: Fotograma) -> np.ndarray:
    """Profundidad en metros (0 = sin dato)."""
    escala = float(f.meta.get("escala_prof", 0.001))
    return f.prof.astype(np.float32) * escala
