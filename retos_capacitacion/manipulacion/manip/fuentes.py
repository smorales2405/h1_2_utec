"""Fuentes de fotogramas con la misma interfaz: la camara en vivo (ZMQ), un dataset grabado o la camara
virtual del gemelo (sim/camara_virtual.py, que produce los mismos Fotograma).

    fuente.siguiente(timeout_s) -> Fotograma | None    (None = no llego nada a tiempo / fin)
    fuente.meta                                        intrinsecos, escala y extrinseco profundidad -> color

La percepcion solo ve Fotograma: corre igual sobre el dataset que en vivo (seccion 5 del PDF).

Formato del dataset (carpeta datos/<tipo>_<fecha>/), el de seguidor_linea:
    fotogramas.csv   n, t_cam, t_rx, emisor, ir, color, prof (nombres de fichero o vacio)
    meta.json        metadatos del primer fotograma
    ir/*.png  color/*.png  prof/*.png (16 bit)
El color va en PNG (sin perdidas): los bordes de los ArUco y de los objetos se miden en el.
"""

import csv
import json
import os
import queue
import threading
import time

import cv2
import numpy as np

from . import fotograma_zmq
from .geometria import Intrinsecos
from .mensajes import Fotograma


class FuenteZmq:
    """Suscriptor de camara_servidor.py. Con solo_ultimo=True descarta lo atrasado; con False entrega
    todo lo que cabe en la cola (grabacion)."""

    def __init__(self, direccion="tcp://127.0.0.1:5556", solo_ultimo=True, cola=4):
        import zmq
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

    def fresco(self, edad_max_s=0.2, timeout_s=3.0):
        """El primer fotograma reciente (t_rx a menos de `edad_max_s`): mientras no se lee, las colas de
        ZMQ se llenan y, llenas, tiran los NUEVOS (seguidor_linea, 2026-10-07)."""
        t0 = time.monotonic()
        while True:
            resto = timeout_s - (time.monotonic() - t0)
            if resto <= 0:
                return None
            f = self.siguiente(timeout_s=resto)
            if f is not None and time.monotonic() - f.t_rx < edad_max_s:
                return f

    def cerrar(self):
        self._sub.close()


class LectorContinuo:
    """Lee la fuente en un hilo y guarda el ultimo fotograma: el vigilante sabe en todo momento la edad del
    ultimo (camara sin fotogramas, seccion 10) aunque el supervisor solo mire al observar."""

    def __init__(self, fuente):
        self.fuente = fuente
        self.meta = getattr(fuente, "meta", {})
        self.ultimo, self.t = None, time.monotonic()
        self._fin = threading.Event()
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def _bucle(self):
        while not self._fin.is_set():
            f = self.fuente.siguiente(0.5)
            if f is not None:
                self.ultimo, self.t = f, time.monotonic()
                self.meta = f.meta

    def edad(self):
        return time.monotonic() - self.t

    def fresco(self, edad_max_s=0.3, timeout_s=3.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if self.ultimo is not None and self.edad() < edad_max_s and self.t > t0 - edad_max_s:
                return self.ultimo
            time.sleep(0.02)
        return None

    def siguiente(self, timeout_s=1.0):
        return self.fresco(timeout_s=timeout_s)

    def cerrar(self):
        self._fin.set()
        self._hilo.join(timeout=1.0)


class EscritorDataset:
    """Escribe fotogramas en un hilo aparte; si el disco no da abasto descarta (y cuenta)."""

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
            guardar_meta(self.carpeta, f.meta)
            self._meta_escrita = True
        nombres = {}
        for flujo in ("ir", "color", "prof"):
            arr = getattr(f, flujo)
            if arr is None:
                nombres[flujo] = ""
                continue
            os.makedirs(os.path.join(self.carpeta, flujo), exist_ok=True)
            nombres[flujo] = f"{flujo}/{f.n:07d}.png"
            cv2.imwrite(os.path.join(self.carpeta, nombres[flujo]), arr, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        self._w.writerow([f.n, f"{f.t_cam:.6f}", f"{f.t_rx:.6f}", int(f.emisor),
                          nombres["ir"], nombres["color"], nombres["prof"]])
        self.escritos += 1

    def cerrar(self):
        self._cola.put(None)
        self._hilo.join(timeout=30)
        self._f.close()


def guardar_meta(carpeta, meta):
    with open(os.path.join(carpeta, "meta.json"), "w") as fm:
        json.dump(meta, fm, indent=1, default=lambda x: np.asarray(x).tolist())


class FuenteDataset:
    """Lee un dataset grabado. Con tiempo_real=True respeta los intervalos de t_rx."""

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


def prof_en_metros(f: Fotograma) -> np.ndarray:
    """Profundidad en metros (0 = sin dato), en el marco del IR izquierdo."""
    return f.prof.astype(np.float32) * float(f.meta.get("escala_prof", 0.001))


def intrinsecos(f: Fotograma, flujo: str) -> Intrinsecos:
    return Intrinsecos.de_meta(f.meta[flujo])


def T_color_prof(meta) -> np.ndarray:
    """Extrinseco de fabrica profundidad -> color (4x4). Las capturas sin el (servidor del seguidor)
    usan el tipico de la D435i: el color 15 mm a la derecha del IR izquierdo, sin giro."""
    if "prof_a_color" in meta:
        return np.asarray(meta["prof_a_color"], float)
    M = np.eye(4)
    M[0, 3] = 0.015
    return M
