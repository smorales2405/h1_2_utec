#!/usr/bin/env python3
"""Graba la calibracion camara-torso (6.1.3) en el robot: mientras herramientas/arnes.py recorre las poses ([k]),
guarda una muestra cada vez que un brazo se queda quieto fuera de 0 en una pose nueva: la imagen de color, la
profundidad y las q del brazo medidas (rt/lowstate) y la mandada (la pose mas cercana de poses_calibracion.yaml).

    ./ejecutar.sh herramientas/grabar_calibracion.py          # terminal 2, ANTES de pulsar [k] en arnes.py
Solo lee la camara (camara_servidor.sh con profundidad) y rt/lowstate: no manda nada. En otro proceso que
arnes.py para no cargar su lazo de 500 Hz. Ctrl+C al terminar ([k] acaba con "calibracion recorrida"). Luego, en
la PC: ./desplegar.sh --traer y python3 herramientas/calibrar_camara_torso.py datos/calibracion_<fecha>
"""

import argparse
import collections
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import config  # noqa: E402
from manip.registro import carpeta_tirada, guardar_json  # noqa: E402

JUNTAS = {"izq": slice(13, 20), "der": slice(20, 27)}


class Grabador:
    """Decide cuando guardar: un brazo a mas de `lejos_de_cero` rad de 0 en alguna junta, quieto `t_quieto` s
    (ninguna de sus juntas se mueve mas de `tol` rad en ese tiempo) y a mas de `min_cambio` rad de la ultima
    muestra de ese brazo. El final de una minima sacudida de 8 s aun se mueve ~0.016 rad en su ultimo segundo:
    no cuenta como quieto."""

    def __init__(self, t_quieto=1.0, tol=0.004, min_cambio=0.05, lejos_de_cero=0.10):
        self.t_quieto, self.tol, self.min_cambio, self.lejos = t_quieto, tol, min_cambio, lejos_de_cero
        self.hist = collections.deque()
        self.ultima = {"izq": None, "der": None}

    def al_estado(self, t, q27):
        self.hist.append((t, np.asarray(q27, float).copy()))
        while self.hist and self.hist[0][0] < t - 3 * self.t_quieto:
            self.hist.popleft()

    def lado_quieto(self, t):
        """El brazo que toca guardar ahora (o None) y su q (7)."""
        ventana = [q for tq, q in self.hist if tq >= t - self.t_quieto]
        if len(ventana) < 3 or self.hist[0][0] > t - self.t_quieto:
            return None, None
        Q = np.array(ventana)
        for lado, js in JUNTAS.items():
            q = Q[-1, js]
            if np.abs(q).max() < self.lejos:
                continue
            if np.ptp(Q[:, js], axis=0).max() > self.tol:
                continue
            u = self.ultima[lado]
            if u is not None and np.abs(q - u).max() < self.min_cambio:
                continue
            return lado, q
        return None, None

    def guardada(self, lado, q):
        self.ultima[lado] = np.asarray(q, float).copy()


def pose_cercana(poses, lado, q, tope=0.15):
    """(indice, q mandada) de la pose de `poses[lado]` mas cercana a q, o (None, None) si ninguna esta a < tope."""
    if not poses or lado not in poses:
        return None, None
    P = np.asarray(poses[lado], float)
    d = np.abs(P - q).max(axis=1)
    k = int(np.argmin(d))
    return (k, P[k]) if d[k] < tope else (None, None)


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--t-quieto", type=float, default=1.0, help="s sin moverse antes de guardar")
    ap.add_argument("--max", type=int, default=60, help="termina al llegar a tantas muestras")
    a = ap.parse_args()
    from manip.fuentes import FuenteZmq, guardar_meta
    from manip.robot import Robot
    fuente = FuenteZmq(cfg["red"]["zmq_fotogramas"])
    if fuente.fresco(timeout_s=5.0) is None:
        sys.exit("No llegan fotogramas: ./camara_servidor.sh")
    robot = Robot(cfg["red"]["iface"])
    if not robot.esperar_lowstate(3.0):
        sys.exit("No llega rt/lowstate")
    try:
        poses = config._leer(os.path.join(config.CONFIG, "poses_calibracion.yaml")).get("poses")
    except OSError:
        poses = None
    carpeta = carpeta_tirada(config.ruta_datos(cfg), "calibracion")
    for sub in ("color", "prof"):
        os.makedirs(os.path.join(carpeta, sub), exist_ok=True)
    g = Grabador(t_quieto=a.t_quieto)
    muestras, t_linea, meta = [], 0.0, False
    print(f"grabando en {carpeta}: pulsa [k] en arnes.py; Ctrl+C al acabar", flush=True)
    try:
        while len(muestras) < a.max:
            f = fuente.siguiente(1.0)
            ahora = time.monotonic()
            if robot.edad_lowstate() > 0.5:
                print("rt/lowstate parado", flush=True)
                time.sleep(0.5)
                continue
            g.al_estado(ahora, robot.q())
            if f is None or f.color is None:
                continue
            lado, q = g.lado_quieto(ahora)
            if lado is not None:
                k = sum(m["lado"] == lado for m in muestras)
                nombre = f"{lado}_{k:02d}"
                cv2.imwrite(os.path.join(carpeta, "color", nombre + ".png"), f.color)
                fila = {"n": len(muestras), "lado": lado, "q": [round(float(v), 5) for v in q],
                        "color": f"color/{nombre}.png", "t": round(ahora, 3)}
                if f.prof is not None:
                    cv2.imwrite(os.path.join(carpeta, "prof", nombre + ".png"), f.prof)
                    fila["prof"] = f"prof/{nombre}.png"
                kp, qc = pose_cercana(poses, lado, q)
                if kp is not None:
                    fila["pose"], fila["q_cmd"] = kp, [round(float(v), 5) for v in qc]
                muestras.append(fila)
                g.guardada(lado, q)
                if not meta:
                    guardar_meta(carpeta, f.meta)
                    meta = True
                guardar_json(os.path.join(carpeta, "muestras.json"), muestras)
                print(f"muestra {len(muestras)}: brazo {lado}, pose {kp if kp is not None else '?'}", flush=True)
            if ahora - t_linea > 5.0:
                t_linea = ahora
                print(f"  {len(muestras)} muestras", flush=True)
    except KeyboardInterrupt:
        pass
    guardar_json(os.path.join(carpeta, "muestras.json"), muestras)
    n = {l: sum(m["lado"] == l for m in muestras) for l in JUNTAS}
    print(f"{len(muestras)} muestras ({n['izq']} izq, {n['der']} der) -> {carpeta}\n"
          f"En la PC: ./desplegar.sh --traer y python3 herramientas/calibrar_camara_torso.py datos/{os.path.basename(carpeta)}")


if __name__ == "__main__":
    main()
