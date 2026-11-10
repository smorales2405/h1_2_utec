"""herramientas/vibracion_camara.py con imagenes sinteticas: dos ArUco de la mesa que se mueven juntos (la camara)
con una senoide conocida mas un ruido propio de cada uno (la deteccion)."""

import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "herramientas"))
import vibracion_camara as vc  # noqa: E402

from manip import calibracion as cb  # noqa: E402
from manip import config  # noqa: E402
from manip.geometria import Intrinsecos  # noqa: E402
from manip.mensajes import Fotograma  # noqa: E402

CFG = config.cargar()
INTR = Intrinsecos(600.0, 600.0, 320.0, 240.0)


def secuencia(amp_px=1.5, f_hz=2.0, ruido_px=0.0, fps=30.0, segundos=4.0, semilla=0):
    rng = np.random.default_rng(semilla)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, CFG["calibracion"]["diccionario"]))
    lado = 40
    marcas = {i: np.pad(cv2.aruco.generateImageMarker(d, i, lado), 8, constant_values=255) for i in (2, 3)}
    sitio = {2: (180.0, 200.0), 3: (420.0, 210.0)}
    meta = {"color": INTR.a_dict()}
    for n in range(int(segundos * fps)):
        t = n / fps
        img = np.full((480, 640), 200, np.uint8)
        comun = amp_px * math.sin(2 * math.pi * f_hz * t)
        for i, (x, y) in sitio.items():
            dx, dy = comun + rng.normal(0, ruido_px), 0.5 * comun + rng.normal(0, ruido_px)
            M = np.float32([[1, 0, x + dx], [0, 1, y + dy]])
            capa = cv2.warpAffine(marcas[i], M, (640, 480), flags=cv2.INTER_LINEAR, borderValue=255)
            mascara = cv2.warpAffine(np.ones_like(marcas[i]) * 255, M, (640, 480), flags=cv2.INTER_LINEAR)
            img = np.where(mascara > 0, capa, img).astype(np.uint8)
        yield Fotograma(n=n, t_cam=t, t_rx=t, emisor=False, meta=meta, color=cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))


def medir(**kw):
    det = cb.detector(CFG["calibracion"]["diccionario"])
    filas = []
    for f in secuencia(**kw):
        filas += vc.medir_fotograma(f, det, INTR, 0.05)
    return vc.analizar(filas, [], INTR, distancia_zona_m=0.8)


def test_movimiento_conocido():
    r = medir(amp_px=1.5, f_hz=2.0, ruido_px=0.0)
    assert r["marcadores"] == [2, 3] and r["fotogramas_comunes"] == 120
    h, v = r["camara"]["horizontal"], r["camara"]["vertical"]
    assert abs(h["px"]["std"] - 1.5 / math.sqrt(2)) < 0.1             # senoide: amplitud / raiz(2)
    assert abs(v["px"]["std"] - 0.75 / math.sqrt(2)) < 0.1
    assert abs(h["frecuencia_hz"] - 2.0) < 0.3 and abs(h["amplitud_px"] - 1.5) < 0.4
    assert abs(h["grados"]["std"] - math.degrees(h["px"]["std"] / 600.0)) < 1e-9
    assert r["ruido_px"]["horizontal"]["por_marcador"] < 0.1


def test_el_ruido_no_es_movimiento():
    r = medir(amp_px=0.0, ruido_px=0.3)
    h = r["camara"]["horizontal"]
    assert 0.15 < r["ruido_px"]["horizontal"]["por_marcador"] < 0.5
    assert h["neto_px"] < 0.12                                           # todo era ruido de la deteccion


def test_imu_en_la_misma_ventana():
    det = cb.detector(CFG["calibracion"]["diccionario"])
    filas = []
    for f in secuencia(amp_px=2.0, f_hz=1.0, ruido_px=0.0, segundos=3.0):
        filas += vc.medir_fotograma(f, det, INTR, 0.05)
    # una IMU a 100 Hz cuyo pitch es el giro vertical de la camara (0.5 * 2 px senoide / 600 px)
    # y un yaw que deriva 0.5 grados/min con la camara sin girar
    imu = [{"t": k / 100.0, "roll": 0.0, "pitch": math.atan2(1.0 * math.sin(2 * math.pi * 1.0 * k / 100.0), 600.0),
            "yaw": math.radians(0.5 / 60.0 * k / 100.0), "gx": 0.0, "gy": 0.0, "gz": 0.0, "carga": 0.72,
            "int_pitch": 0.0} for k in range(300)]
    r = vc.analizar(filas, imu, INTR)
    c = r["imu"]["camara_frente_a_imu"]
    assert c["r_pitch"] > 0.95 and abs(c["pendiente_pitch"] - 1.0) < 0.15
    assert abs(r["imu"]["pitch"]["std"] - math.degrees(1.0 / 600.0) / math.sqrt(2)) < 0.01
    assert abs(r["imu"]["tendencia_grados_min"]["yaw"] - 0.5) < 0.01


def test_deriva_del_yaw_sin_que_gire_la_camara():
    """La IMU deriva en yaw (2026-11-11 en el robot: -0.45 grados/min) y la camara no gira: es la IMU."""
    det = cb.detector(CFG["calibracion"]["diccionario"])
    filas = []
    for f in secuencia(amp_px=0.0, ruido_px=0.1, segundos=4.0):
        filas += vc.medir_fotograma(f, det, INTR, 0.05)
    imu = [{"t": k / 100.0, "roll": 0.0, "pitch": 0.0, "yaw": math.radians(-0.45 / 60.0 * k / 100.0), "gx": 0.0,
            "gy": 0.0, "gz": 0.0, "carga": 0.84, "int_pitch": 0.1} for k in range(400)]
    r = vc.analizar(filas, imu, INTR)
    assert abs(r["imu"]["tendencia_grados_min"]["yaw"] + 0.45) < 0.01
    assert abs(r["camara"]["horizontal"]["tendencia_grados_min"]) < 0.05
    assert any("es la IMU, no el cuerpo" in x and "11 min" in x for x in vc.informe(r))
