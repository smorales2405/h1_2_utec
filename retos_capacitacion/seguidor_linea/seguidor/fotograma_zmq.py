"""Fotograma <-> mensaje ZMQ multiparte: una cabecera JSON y un bloque crudo por flujo.

Lo usan camara_servidor.py (root) y FuenteZmq (usuario). Sin compresion: por 127.0.0.1 el
IR crudo son 300 KB por fotograma, 9 MB/s a 30 fps.
"""

import json

import numpy as np

from .mensajes import Fotograma

FLUJOS = ("ir", "color", "prof")


def codificar(f: Fotograma):
    cab = {"n": f.n, "t_cam": f.t_cam, "t_rx": f.t_rx, "emisor": f.emisor, "meta": f.meta, "flujos": []}
    partes = []
    for nombre in FLUJOS:
        arr = getattr(f, nombre)
        if arr is None:
            continue
        arr = np.ascontiguousarray(arr)
        cab["flujos"].append([nombre, list(arr.shape), arr.dtype.str])
        partes.append(arr.tobytes())
    return [json.dumps(cab).encode()] + partes


def decodificar(partes) -> Fotograma:
    cab = json.loads(bytes(partes[0]))
    if len(cab["flujos"]) != len(partes) - 1:
        raise ValueError(f"mensaje con {len(partes) - 1} bloques y {len(cab['flujos'])} flujos en la cabecera")
    f = Fotograma(n=cab["n"], t_cam=cab["t_cam"], t_rx=cab["t_rx"], emisor=cab["emisor"], meta=cab["meta"])
    for (nombre, forma, tipo), datos in zip(cab["flujos"], partes[1:]):
        setattr(f, nombre, np.frombuffer(datos, dtype=np.dtype(tipo)).reshape(forma))
    return f
