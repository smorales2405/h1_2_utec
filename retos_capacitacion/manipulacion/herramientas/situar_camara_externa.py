#!/usr/bin/env python3
"""Donde poner la camara externa (la Logitech BRIO de la laptop): cuanto subirla o bajarla, acercarla o alejarla del
robot, inclinarla y girarla para que el agarre de los objetos se vea bien (RESULTADOS §63). En la PC, con la camara
conectada y el robot en reposo (que no tape el tablero); no mueve nada del robot.

    python3 herramientas/situar_camara_externa.py              # en vivo: las indicaciones se rehacen mientras la mueves
    python3 herramientas/situar_camara_externa.py --una        # una sola medida (~2 s)
    python3 herramientas/situar_camara_externa.py --ventana    # ademas, la imagen con lo que se quiere ver (q: salir)
    python3 herramientas/situar_camara_externa.py --guardar    # al acabar, el zoom que lo encuadra, en manipulacion.yaml

Como mide: en 4K y sin zoom, la pose de la camara respecto de la mesa con los ArUco 6 y 7 (planos, junto al labio
frontal; docs/aruco_6_7_camara_externa.pdf) o, si falta alguno, con los 8 y 9 de las asas. El sitio del 8 y el 9
se mide la primera vez desde el 6 y el 7 (datos/externa/marcadores_asas.json; --medir-asas lo rehace). Lo que se quiere ver: la zona de recogida ampliada y las bandejas del montaje del robot
(config/montaje.yaml, por ssh), de la mesa a `situar.altura_m`.
- Altura y adelante/atras: la camara, vista desde el centro de la zona, con una elevacion dentro de `elevacion_deg`
  y a una distancia a la que lo que se quiere ver llene el 1080p con un zoom dentro de `zoom`.
- Inclinacion y giro: los que lo centran en la imagen desde donde esta la camara.
Al acabar, la imagen anotada y la medida quedan en datos/externa/situar_camara.jpg y .json, y la camara vuelve a sus
controles de grabacion.
"""

import argparse
import glob
import itertools
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
import yaml

AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, "herramientas"))
from camara_externa import DEST, ROBOT_SSH, cfg_camara, configurar, dispositivo  # noqa: E402
from manip import montaje as mt  # noqa: E402
from manip.geometria import aplicar, inv  # noqa: E402

ANCHO, ALTO = 3840, 2160            # se mide en 4K sin zoom: el 1080p con zoom Z es el centro de esta imagen / Z
ARRIBA = np.array([0.0, 0.0, 1.0])
CONFIG = os.path.join(AQUI, "config", "manipulacion.yaml")
EXTERNA = os.path.join(AQUI, "datos", "externa")
ASAS = os.path.join(EXTERNA, "marcadores_asas.json")
COLORES = {"azul": (255, 120, 0), "roja": (0, 0, 255), "blanca": (255, 255, 255), "naranja": (0, 140, 255)}


# --- la escena: la mesa en su marco fisico ------------------------------------------------------------------------
def montaje_robot():
    """(montaje, de donde): el del robot (config/montaje.yaml por ssh) o, sin robot, el ultimo traido
    (datos/montaje_robot_*.yaml) o el de esta copia."""
    try:
        r = subprocess.run(["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", ROBOT_SSH,
                            f"cat {DEST}/config/montaje.yaml"], capture_output=True, text=True, timeout=20)
        if r.returncode == 0 and r.stdout.strip():
            return yaml.safe_load(r.stdout), "el del robot"
    except (OSError, subprocess.SubprocessError):
        pass
    for f in sorted(glob.glob(os.path.join(AQUI, "datos", "montaje_robot_*.yaml")), reverse=True) + \
            [os.path.join(AQUI, "config", "montaje.yaml")]:
        with open(f) as fh:
            m = yaml.safe_load(fh)
        if m and "mesa" in m and "mesa_fisica" in m:
            return m, os.path.relpath(f, AQUI) + " (sin robot)"
    sys.exit("no hay montaje con la mesa medida y mesa_fisica")


def esquinas_planas(c, lado, k):
    """Las 4 esquinas (en el orden de OpenCV: arriba-izquierda y en el sentido del reloj) de un ArUco plano sobre la
    mesa centrado en c, con su 'arriba' hacia el labio frontal (+x) girado k cuartos de vuelta."""
    a = k * math.pi / 2
    arriba = np.array([math.cos(a), math.sin(a), 0.0])
    der = np.cross(arriba, ARRIBA)
    h = lado / 2
    return np.array([c - h * der + h * arriba, c + h * der + h * arriba, c + h * der - h * arriba, c - h * der - h * arriba])


class Escena:
    """La mesa en su marco fisico (de wincha, m): origen en el centro del labio cercano (el del robot), x hacia el
    labio frontal (el de la camara), y a la izquierda del robot (a la derecha de quien mira desde la camara), z
    arriba. El marco del montaje es este escalado con lo que ve la camara del robot (montaje.escala_mesa)."""

    def __init__(self, montaje, sit, margen_zona):
        fis = mt.fisica(montaje)
        self.montaje, self.fondo, self.ancho = montaje, fis["fondo"], fis["ancho"]
        mesa = montaje["mesa"]
        self.esc = mt.escala_mesa({int(k): v for k, v in (mesa.get("aruco_torso") or {}).items()}, fis)
        self.Ti = inv(mt.T_torso_mesa(montaje))
        # lo que se quiere ver: la zona ampliada y las bandejas, de la mesa a altura_m
        zr, m = montaje["zona_recogida"], margen_zona
        xs, ys = (zr["x"][0] - m, zr["x"][1] + m), (zr["y"][0] - m, zr["y"][1] + m)
        self.zona = np.array([self.de_torso((x, y)) for x, y in ((xs[0], ys[0]), (xs[1], ys[0]), (xs[1], ys[1]), (xs[0], ys[1]))])
        self.bandejas = {}
        for nombre, b in sorted(montaje.get("bandejas", {}).items()):
            h, c = b["hueco_m"] / 2 + b["pared_m"], b["centro"]
            self.bandejas[nombre] = (b.get("color"), b.get("alto_m", 0.04), np.array(
                [self.de_torso((c[0] + sx * h, c[1] + sy * h)) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]))
        base = np.vstack([self.zona] + [v[2] for v in self.bandejas.values()])
        self.altura = float(sit["altura_m"])
        self.roi = np.vstack([base, base + [0.0, 0.0, self.altura]])
        self.centro = self.zona.mean(0) + [0.0, 0.0, 0.03]      # desde donde se mide la elevacion: el cubo
        # los ArUco planos (6 y 7), por su distancia a los labios
        p = sit["planos"]
        x, y = self.fondo - p["al_labio_frontal_m"], self.ancho / 2 - p["al_labio_lateral_m"]
        self.planos = {int(p["ids"][0]): np.array([x, y, 0.0]), int(p["ids"][1]): np.array([x, -y, 0.0])}
        self.lado_planos = float(p["lado_m"])
        self.ids_asas = [int(i) for i in sit["asas"]["ids"]]
        self.lado_asas = float(sit["asas"]["lado_m"])

    def de_torso(self, xy):
        p = aplicar(self.Ti, mt.punto_en_mesa(self.montaje, xy, 0.0))
        return np.array([p[0] / self.esc, p[1] / self.esc, 0.0])


# --- la camara: pose, angulos y proyeccion --------------------------------------------------------------------------
def intrinsecos(f):
    return np.array([[f, 0.0, ANCHO / 2], [0.0, f, ALTO / 2], [0.0, 0.0, 1.0]])


def _unit(v):
    return v / np.linalg.norm(v)


def angulos(R, t):
    """De la pose (R, t: mesa -> camara) a (C, inclinacion, giro, ladeo): C, el centro de la camara en la mesa (m);
    inclinacion, grados hacia abajo; giro, grados hacia +y (a la derecha de quien mira desde la camara); ladeo,
    grados con el lado derecho de la camara mas alto (vista desde detras; la escena se ve girada al reves)."""
    Rcm = R.T
    C = -Rcm @ np.asarray(t, float).ravel()
    xc, zc = Rcm[:, 0], Rcm[:, 2]
    x0 = _unit(np.cross(zc, ARRIBA))
    y0 = np.cross(zc, x0)
    return (C, math.degrees(math.asin(-zc[2])), math.degrees(math.atan2(zc[1], -zc[0])),
            math.degrees(math.atan2(xc @ -y0, xc @ x0)))


def R_de(incl, giro, ladeo=0.0):
    """R (mesa -> camara) de una camara que mira hacia el robot con esos angulos (grados, como en angulos())."""
    p, g, l = (math.radians(v) for v in (incl, giro, ladeo))
    zc = np.array([-math.cos(p) * math.cos(g), math.cos(p) * math.sin(g), -math.sin(p)])
    x0 = _unit(np.cross(zc, ARRIBA))
    y0 = np.cross(zc, x0)
    xc = math.cos(l) * x0 - math.sin(l) * y0
    return np.column_stack([xc, np.cross(zc, xc), zc]).T


def proyectar(R, C, P, K):
    Pc = (np.asarray(P, float) - C) @ R.T
    z = Pc[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = Pc[:, :2] / z[:, None] * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    return uv, z


def zoom_que_cabe(R, C, roi, K, margen):
    """El zoom mas alto con el que `roi` cabe en la imagen (recorte centrado, como el zoom de la camara) con
    `margen` alrededor (fraccion de media imagen); < 1: no cabe ni sin zoom. 0 si algo queda detras."""
    uv, z = proyectar(R, C, roi, K)
    if (z <= 0.05).any():
        return 0.0
    ax = max(np.abs(uv[:, 0] - K[0, 2]).max() * (1 + margen), 1e-9)
    ay = max(np.abs(uv[:, 1] - K[1, 2]).max() * (1 + margen), 1e-9)
    return float(min(ANCHO / 2 / ax, ALTO / 2 / ay))


def apuntar(C, roi, K, vueltas=5):
    """(inclinacion, giro) que centran `roi` en la imagen desde C, sin ladeo."""
    d = roi.mean(0) - C
    giro, incl = math.degrees(math.atan2(d[1], -d[0])), math.degrees(math.atan2(-d[2], math.hypot(d[0], d[1])))
    for _ in range(vueltas):
        uv, z = proyectar(R_de(incl, giro), C, roi, K)
        if (z <= 0).any():
            break
        du = (uv[:, 0].min() + uv[:, 0].max()) / 2 - K[0, 2]
        dv = (uv[:, 1].min() + uv[:, 1].max()) / 2 - K[1, 2]
        giro += math.degrees(math.atan(du / K[0, 0]))
        incl += math.degrees(math.atan(dv / K[1, 1]))
    return incl, giro


def elevacion(C, centro):
    """(grados sobre la mesa, distancia en m) de la camara vista desde `centro`."""
    d = C - centro
    return math.degrees(math.atan2(d[2], math.hypot(d[0], d[1]))), float(np.linalg.norm(d))


def zoom_apuntada(C, esc, K, margen):
    incl, giro = apuntar(C, esc.roi, K)
    return zoom_que_cabe(R_de(incl, giro), C, esc.roi, K, margen)


# --- la pose con los ArUco ------------------------------------------------------------------------------------------
def detector():
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), p)


def detectar(det, img, ids):
    c, i, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    if i is None:
        return {}
    return {int(k): cc.reshape(4, 2).astype(np.float64) for k, cc in zip(i.ravel(), c) if int(k) in ids}


def _error(O, I, rv, tv, K):
    pr, _ = cv2.projectPoints(O, rv, tv, K, None)
    return float(np.sqrt(((pr.reshape(-1, 2) - I) ** 2).sum(1).mean()))


def pose_camara(vistos, esc, asas, K, giros=None):
    """Pose de la camara con los ArUco vistos ({id: (4, 2)}): el 6 y el 7 donde estan (probando sus cuatro giros: no
    se sabe como se pegaron) y, si falta alguno, las asas de `asas` ({id: (4, 3)}). Las asas medidas desde una vista
    se apartan 3-6 px vistas desde otra (de un ArUco de 5 cm solo, la profundidad y el giro salen peor): con el 6 y
    el 7 no se usan. Hacen falta dos ArUco. Devuelve dict (R, t, error_px, ids, giros) o None."""
    planos = sorted(i for i in esc.planos if i in vistos)
    con_asas = [] if len(planos) == 2 else sorted(i for i in asas if i in vistos)
    if len(planos) + len(con_asas) < 2:
        return None
    rv = tv = None
    obj, img = [], []
    giros = dict(giros or {})
    if len(planos) == 2:
        I = np.vstack([vistos[i] for i in planos])
        mejor = None
        for ks in itertools.product(range(4), repeat=2):
            O = np.vstack([esquinas_planas(esc.planos[i], esc.lado_planos, k) for i, k in zip(planos, ks)])
            ok, r, t = cv2.solvePnP(O, I, K, None, flags=cv2.SOLVEPNP_IPPE)
            if ok:
                e = _error(O, I, r, t, K)
                if mejor is None or e < mejor[0]:
                    mejor = (e, ks, r, t, O)
        if mejor is None:
            return None
        _, ks, rv, tv, O = mejor
        giros.update(dict(zip(planos, ks)))
        obj.append(O)
        img.append(I)
    else:
        for i in planos:        # uno solo: con el giro de la ultima vez que se vieron los dos
            obj.append(esquinas_planas(esc.planos[i], esc.lado_planos, giros.get(i, 0)))
            img.append(vistos[i])
    for i in con_asas:
        obj.append(np.asarray(asas[i], float))
        img.append(vistos[i])
    O, I = np.vstack(obj), np.vstack(img)
    if rv is None:
        ok, rv, tv = cv2.solvePnP(O, I, K, None, flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            return None
    rv, tv = cv2.solvePnPRefineLM(O, I, K, None, rv, tv)
    R, _ = cv2.Rodrigues(rv)
    C, incl, _, _ = angulos(R, tv)
    if C[0] < esc.fondo or C[2] <= 0:       # la camara, del lado del labio frontal y por encima de la mesa
        return None
    return {"R": R, "t": tv.ravel(), "error_px": _error(O, I, rv, tv, K), "ids": planos + con_asas, "giros": giros}


def asas_desde(vistos, pose, K, esc):
    """Esquinas (mesa) de los ArUco de las asas vistos, desde la pose de la camara: cada uno con su tamano."""
    h = esc.lado_asas / 2
    O = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
    sal = {}
    for i in esc.ids_asas:
        if i not in vistos:
            continue
        ok, r, t = cv2.solvePnP(O, vistos[i], K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            Pc = O @ cv2.Rodrigues(r)[0].T + t.ravel()
            sal[i] = (Pc - pose["t"]) @ pose["R"]
    return sal


def leer_asas():
    try:
        with open(ASAS) as f:
            d = json.load(f)
        return {int(k): np.array(v, float) for k, v in d["esquinas"].items()}, {int(k): v for k, v in d.get("giros", {}).items()}
    except (OSError, ValueError, KeyError):
        return {}, {}


def guardar_asas(esquinas, giros, n):
    os.makedirs(EXTERNA, exist_ok=True)
    with open(ASAS, "w") as f:
        json.dump({"fecha": time.strftime("%Y-%m-%d %H:%M"), "fotogramas": n, "marco": "mesa fisica (situar_camara_externa.py)",
                   "esquinas": {str(k): np.round(v, 4).tolist() for k, v in sorted(esquinas.items())},
                   "giros": {str(k): int(v) for k, v in sorted(giros.items())}}, f, indent=1)


# --- las indicaciones -----------------------------------------------------------------------------------------------
def rango(C, eje, valores, esc, K, sit):
    """(min, max) de los valores de C[eje] (lo demas como esta) con la camara dentro del rango, o None."""
    ok = []
    for v in valores:
        Cv = np.array(C, float)
        Cv[eje] = v
        el, _ = elevacion(Cv, esc.centro)
        if not sit["elevacion_deg"][0] <= el <= sit["elevacion_deg"][1]:
            continue
        if sit["zoom"][0] <= zoom_apuntada(Cv, esc, K, sit["margen"]) <= sit["zoom"][1]:
            ok.append(v)
    return (min(ok), max(ok)) if ok else None


def objetivo(esc, K, sit):
    """El sitio ideal: de frente al robot, a la elevacion objetivo y a la distancia del zoom objetivo."""
    th = math.radians(sit["elevacion_objetivo_deg"])
    u = np.array([math.cos(th), 0.0, math.sin(th)])
    lo, hi = 0.2, 6.0
    for _ in range(40):         # el zoom que hace falta crece con la distancia
        mid = (lo + hi) / 2
        if zoom_apuntada(esc.centro + mid * u, esc, K, sit["margen"]) < sit["zoom_objetivo"]:
            lo = mid
        else:
            hi = mid
    C = esc.centro + hi * u
    C[1] = esc.centro[1]
    return C


def evaluar(pose, esc, K, sit, zoom_cfg):
    """Lo que hay que mover (dict) para la pose medida."""
    C, incl, giro, ladeo = pose["C"], pose["incl"], pose["giro"], pose["ladeo"]
    el, dist = elevacion(C, esc.centro)
    incl_a, giro_a = apuntar(C, esc.roi, K)
    Co = objetivo(esc, K, sit)
    R = R_de(incl, giro, ladeo)
    return {
        "C": C, "incl": incl, "giro": giro, "ladeo": ladeo, "elevacion": el, "distancia": dist,
        "zoom_apuntada": zoom_apuntada(C, esc, K, sit["margen"]),
        "rango_altura": rango(C, 2, np.arange(0.0, 2.0, 0.01), esc, K, sit),
        "rango_x": rango(C, 0, np.arange(esc.fondo + 0.05, esc.fondo + 3.0, 0.01), esc, K, sit),
        "objetivo": Co, "incl_obj": incl_a, "giro_obj": giro_a,
        "zoom_cabe": zoom_que_cabe(R, C, esc.roi, K, sit["margen"]), "zoom_cfg": zoom_cfg / 100.0,
    }


def entrar(v, r, margen=0.03):
    """Lo que hay que mover v (m) para quedar dentro de r = (min, max) con `margen`; 0 si ya esta dentro."""
    lo, hi = r
    if hi - lo < 2 * margen:
        return (lo + hi) / 2 - v
    if v < lo:
        return lo + margen - v
    if v > hi:
        return hi - margen - v
    return 0.0


def propuesta(ev):
    """(dh, dx) en m: el movimiento mas corto que deja la camara dentro del rango: solo la altura, solo adelante/atras
    o las dos al sitio ideal."""
    C, ra, rx = ev["C"], ev["rango_altura"], ev["rango_x"]
    if ra is not None and rx is not None and ra[0] <= C[2] <= ra[1] and rx[0] <= C[0] <= rx[1]:
        return 0.0, 0.0
    opciones = [(ev["objetivo"][2] - C[2], ev["objetivo"][0] - C[0])]
    if ra is not None:
        opciones.append((entrar(C[2], ra), 0.0))
    if rx is not None:
        opciones.append((0.0, entrar(C[0], rx)))
    return min(opciones, key=lambda o: abs(o[0]) + abs(o[1]))


def texto(ev, pose, esc, sit, en_vivo=False):
    C, Co = ev["C"], ev["objetivo"]
    tol = float(sit["tol_deg"])
    lado = "derecha" if C[1] > 0 else "izquierda"
    lin = [f"Camara: {100 * C[2]:.0f} cm sobre la mesa, a {100 * (C[0] - esc.fondo):.0f} cm del labio frontal "
           f"({C[0]:.2f} m del labio del robot) y {100 * abs(C[1]):.0f} cm a tu {lado} del eje del robot. "
           f"ArUco {' '.join(map(str, pose['ids']))}, error {pose['error_px']:.1f} px.",
           f"Desde el centro de la zona se ve a {ev['elevacion']:.0f} grados de elevacion (bien: "
           f"{sit['elevacion_deg'][0]}-{sit['elevacion_deg'][1]}) y a {ev['distancia']:.2f} m: la zona ampliada y las "
           f"bandejas llenarian el 1080p con zoom x{ev['zoom_apuntada']:.2f} (bien: {sit['zoom'][0]}-{sit['zoom'][1]}).", ""]
    ra, rx = ev["rango_altura"], ev["rango_x"]
    dh, dx = propuesta(ev)

    def mover(d, v, r, mas, menos, otra):
        if abs(d) >= 0.005:
            return f"{mas if d > 0 else menos} {100 * abs(d):.0f} cm"
        return "bien" if r is not None and r[0] <= v <= r[1] else f"dejala (basta con {otra})"

    def lado_giro(g):
        return f"{abs(g):.1f} a tu {'der.' if g > 0 else 'izq.'}"
    di, dg = ev["incl_obj"] - ev["incl"], ev["giro_obj"] - ev["giro"]
    filas = [("", "ahora", "bien (lo demas como esta)", "que hacer"),
             ("Altura", f"{100 * C[2]:.0f} cm", f"{100 * ra[0]:.0f} a {100 * ra[1]:.0f} cm" if ra else "ninguna",
              mover(dh, C[2], ra, "SUBELA", "BAJALA", "adelante/atras")),
             ("Adelante/atras", f"{100 * (C[0] - esc.fondo):.0f} cm del labio",
              f"{100 * (rx[0] - esc.fondo):.0f} a {100 * (rx[1] - esc.fondo):.0f} cm" if rx else "ninguna",
              mover(dx, C[0], rx, "ALEJALA del robot", "ACERCALA al robot", "la altura")),
             ("Inclinacion", f"{ev['incl']:.1f} abajo", f"{ev['incl_obj']:.1f} abajo +-{tol:g}",
              "bien" if abs(di) <= tol else f"INCLINALA {abs(di):.1f} grados hacia {'abajo' if di > 0 else 'arriba'}"),
             ("Giro", lado_giro(ev["giro"]), f"{lado_giro(ev['giro_obj'])} +-{tol:g}",
              "bien" if abs(dg) <= tol else f"GIRALA {abs(dg):.1f} grados a tu {'derecha' if dg > 0 else 'izquierda'}")]
    for f in filas:
        lin.append(f"  {f[0]:<16}{f[1]:<20}{f[2]:<28}{f[3]}")
    lin += ["", f"Ideal: {100 * Co[2]:.0f} cm sobre la mesa y a {100 * (Co[0] - esc.fondo):.0f} cm del labio frontal, de "
            f"frente al robot ({sit['elevacion_objetivo_deg']} grados de elevacion, zoom x{sit['zoom_objetivo']})."]
    if abs(ev["ladeo"]) > tol:
        lin.append(f"Ladeo {abs(ev['ladeo']):.1f} grados: baja el lado {'derecho' if ev['ladeo'] > 0 else 'izquierdo'} "
                   "de la camara (mirando hacia el robot).")
    if abs(C[1]) > 0.10:
        lin.append(f"Esta {100 * abs(C[1]):.0f} cm a tu {lado} del eje del robot: mejor de frente (hasta 10 cm).")
    if dh or dx:
        lin.append("Primero la altura y la distancia; la inclinacion y el giro, desde donde quede"
                   + (" (se rehacen solos)." if en_vivo else " (vuelve a medir)."))
    elif abs(di) <= tol and abs(dg) <= tol and abs(ev["ladeo"]) <= tol:
        lin.append("La camara esta bien puesta.")
    z = ev["zoom_cabe"]
    if z < 1.0:
        lin.append("Para grabar: tal como apunta, la zona y las bandejas no caben ni sin zoom.")
    else:
        zc = ev["zoom_cfg"]
        lin.append(f"Para grabar (1080p): tal como apunta, zoom_absolute {zoom_absoluto(z)} (en manipulacion.yaml, "
                   f"{int(round(100 * zc))}: {'cabe' if zc <= z else 'se sale'}).")
    return "\n".join(lin)


def zoom_absoluto(z):
    return int(min(500, max(100, math.floor(100 * z))))


# --- la imagen anotada ----------------------------------------------------------------------------------------------
def anotar(img, vistos, pose, esc, K, ev, escala=0.5):
    im = cv2.resize(img, None, fx=escala, fy=escala, interpolation=cv2.INTER_AREA)
    for i, c in vistos.items():
        p = (c * escala).astype(np.int32)
        cv2.polylines(im, [p], True, (0, 255, 0), 2)
        cv2.putText(im, str(i), tuple(p[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    if pose is not None:
        R = R_de(pose["incl"], pose["giro"], pose["ladeo"])
        C = pose["C"]

        def caja(base, alto, color, grosor=2):
            arriba = base + [0.0, 0.0, alto]
            uv, z = proyectar(R, C, np.vstack([base, arriba]), K)
            if (z <= 0).any():
                return
            p = (uv * escala).astype(np.int32)
            cv2.polylines(im, [p[:4]], True, color, grosor)
            cv2.polylines(im, [p[4:]], True, color, 1)
            for k in range(4):
                cv2.line(im, tuple(p[k]), tuple(p[k + 4]), color, 1)
        caja(esc.zona, esc.altura, (0, 255, 0))
        for color, alto, base in esc.bandejas.values():
            caja(base, alto, COLORES.get(color, (200, 200, 200)))
        # el 1080p con el zoom de manipulacion.yaml (cian) y con el que encuadra (amarillo)
        for z, color, rotulo, abajo in ((ev["zoom_cfg"], (255, 255, 0), "zoom actual", False),
                                        (ev["zoom_cabe"], (0, 255, 255), "zoom que encuadra", True)):
            if z >= 1.0:
                w, h = ANCHO / z * escala / 2, ALTO / z * escala / 2
                cx, cy = ANCHO * escala / 2, ALTO * escala / 2
                cv2.rectangle(im, (int(cx - w), int(cy - h)), (int(cx + w), int(cy + h)), color, 2)
                cv2.putText(im, f"{rotulo} x{z:.2f}", (int(cx - w) + 6, int(cy + h) - 10 if abajo else int(cy - h) + 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    return im


# --- captura ----------------------------------------------------------------------------------------------------------
class Captura(threading.Thread):
    """Lee la camara sin parar y se queda con el ultimo fotograma (OpenCV decodifica el 4K a ~10 fps)."""

    def __init__(self, dev):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ok = self.cap.isOpened() and int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) == ANCHO
        self.img, self.n, self.seguir = None, 0, True
        self.lock = threading.Lock()

    def run(self):
        while self.seguir:
            ok, im = self.cap.read()
            if ok:
                with self.lock:
                    self.img, self.n = im, self.n + 1
            else:
                time.sleep(0.05)

    def ultimo(self):
        with self.lock:
            return self.img, self.n

    def parar(self):
        self.seguir = False
        self.join(timeout=2)
        self.cap.release()


def mediana(hist):
    C = np.median([h["C"] for h in hist], axis=0)
    return {"C": C, "incl": float(np.median([h["incl"] for h in hist])), "giro": float(np.median([h["giro"] for h in hist])),
            "ladeo": float(np.median([h["ladeo"] for h in hist])), "error_px": float(np.median([h["error_px"] for h in hist])),
            "ids": hist[-1]["ids"]}


def guardar_zoom(z):
    with open(CONFIG) as f:
        txt = f.read()
    i = txt.index("\ncamara_externa:")
    nuevo, n = re.subn(r"(\n    zoom_absolute: *)\d+", rf"\g<1>{z}", txt[i:], count=1)
    if n != 1:
        return False
    with open(CONFIG, "w") as f:
        f.write(txt[:i] + nuevo)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--una", action="store_true", help="una sola medida")
    ap.add_argument("--ventana", action="store_true", help="la imagen anotada en una ventana (q: salir)")
    ap.add_argument("--guardar", action="store_true", help="al acabar, el zoom que encuadra en manipulacion.yaml")
    ap.add_argument("--medir-asas", action="store_true", help="vuelve a medir el sitio del 8 y el 9 desde el 6 y el 7")
    a = ap.parse_args()

    c = cfg_camara()
    sit = c["situar"]
    with open(CONFIG) as f:
        margen_zona = float(yaml.safe_load(f)["percepcion"]["margen_zona_m"])
    dev = dispositivo(c["nombre"])
    if dev is None:
        sys.exit(f"No hay camara '{c['nombre']}'")
    montaje, de_donde = montaje_robot()
    esc = Escena(montaje, sit, margen_zona)
    K = intrinsecos(float(c["focal_4k_px"]))
    print(f"Montaje: {de_donde} ({montaje.get('fecha', '?')}). Camara {dev} en 4K sin zoom.")
    asas, giros = ({}, {}) if a.medir_asas else leer_asas()
    medir_asas = not asas
    muestras_asas = {}

    # 4K sin zoom: el zoom antes que pan/tilt; lo demas (foco fijo, exposicion), como al grabar
    controles = dict(c["controles"], zoom_absolute=100, pan_absolute=0, tilt_absolute=0)
    configurar(dev, controles)
    cap = Captura(dev)
    if not cap.ok:
        configurar(dev, c["controles"])
        sys.exit(f"No se puede abrir {dev} en 4K (la esta usando otro programa?)")
    cap.start()
    det = detector()
    ids = set(esc.planos) | set(esc.ids_asas)
    hist = deque(maxlen=5)
    ult_n, ult_print, inicio = 0, 0.0, time.time()
    validas, img, vistos, pose_m, ev = 0, None, {}, None, None
    try:
        while True:
            im, n = cap.ultimo()
            if im is None or n == ult_n or time.time() - inicio < 1.5:     # la exposicion, ajustandose
                time.sleep(0.02)
                if time.time() - inicio > 10 and im is None:
                    sys.exit("la camara no da imagen")
                continue
            ult_n, img = n, im
            vistos = detectar(det, img, ids)
            p = pose_camara(vistos, esc, asas, K, giros)
            if p is not None and p["error_px"] < 5.0:
                giros = p["giros"]
                C, incl, giro, ladeo = angulos(p["R"], p["t"])
                h = {"C": C, "incl": incl, "giro": giro, "ladeo": ladeo, "error_px": p["error_px"], "ids": p["ids"]}
                if hist and (np.linalg.norm(C - hist[-1]["C"]) > 0.03 or abs(incl - hist[-1]["incl"]) > 2
                             or abs(giro - hist[-1]["giro"]) > 2):
                    hist.clear()        # se esta moviendo: sin arrastrar las medidas de antes
                hist.append(h)
                validas += 1
                if medir_asas and all(i in vistos for i in esc.planos) and p["error_px"] < 2.0:
                    for i, e in asas_desde(vistos, p, K, esc).items():
                        muestras_asas.setdefault(i, []).append(e)
                    if muestras_asas and min(len(v) for v in muestras_asas.values()) >= 10:
                        asas = {i: np.median(v, axis=0) for i, v in muestras_asas.items()}
                        guardar_asas(asas, giros, min(len(v) for v in muestras_asas.values()))
                        medir_asas = False
                        print("ArUco de las asas medidos desde el 6 y el 7: " + ", ".join(
                            f"{i} con el centro en x {100 * v.mean(0)[0]:.1f}, y {100 * v.mean(0)[1]:+.1f} cm, borde de abajo a "
                            f"{100 * v[:, 2].min():.1f} cm sobre la mesa" for i, v in sorted(asas.items()))
                            + f" ({os.path.relpath(ASAS, AQUI)})")
            elif not a.una and time.time() - ult_print > 1.0:
                print("\033[H\033[J" if sys.stdout.isatty() else "", end="")
                print(f"No veo dos ArUco del 6 al 9 (vistos: {sorted(vistos) or 'ninguno'}): que la camara mire al tablero.")
                ult_print = time.time()
            listo = hist and (len(hist) >= 3 or a.una)
            if listo and (not a.una or validas >= 10) and time.time() - ult_print > 0.5:
                pose_m = mediana(hist)
                ev = evaluar(pose_m, esc, K, sit, c["controles"]["zoom_absolute"])
                if not a.una and sys.stdout.isatty():
                    print("\033[H\033[J", end="")
                print(texto(ev, pose_m, esc, sit, en_vivo=not a.una))
                if not a.una:
                    print("\n(en vivo: Ctrl+C para terminar)")
                ult_print = time.time()
                if a.una:
                    break
            if a.una and time.time() - inicio > 15:
                print(f"No veo dos ArUco del 6 al 9 (vistos: {sorted(vistos) or 'ninguno'}): que la camara mire al tablero.")
                break
            if a.ventana and img is not None:
                cv2.imshow("situar camara externa", anotar(img, vistos, pose_m, esc, K, ev) if ev else
                           cv2.resize(img, None, fx=0.5, fy=0.5))
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        print()
    finally:
        cap.parar()
        configurar(dev, c["controles"])
        if a.ventana:
            cv2.destroyAllWindows()
    if img is None:
        return 1
    os.makedirs(EXTERNA, exist_ok=True)
    salida = os.path.join(EXTERNA, "situar_camara.jpg")
    cv2.imwrite(salida, anotar(img, vistos, pose_m, esc, K, ev) if ev else cv2.resize(img, None, fx=0.5, fy=0.5))
    if ev is None:
        print(f"Sin medida. Imagen: {os.path.relpath(salida, AQUI)}")
        return 1
    with open(os.path.join(EXTERNA, "situar_camara.json"), "w") as f:
        json.dump({"fecha": time.strftime("%Y-%m-%d %H:%M:%S"), "montaje": de_donde, "focal_4k_px": float(K[0, 0]),
                   "ids": pose_m["ids"], "error_px": round(pose_m["error_px"], 2),
                   "camara_m": np.round(pose_m["C"], 4).tolist(), "inclinacion_deg": round(ev["incl"], 2),
                   "giro_deg": round(ev["giro"], 2), "ladeo_deg": round(ev["ladeo"], 2),
                   "elevacion_deg": round(ev["elevacion"], 2), "distancia_m": round(ev["distancia"], 3),
                   "zoom_apuntada": round(ev["zoom_apuntada"], 3), "objetivo_m": np.round(ev["objetivo"], 4).tolist(),
                   "inclinacion_objetivo_deg": round(ev["incl_obj"], 2), "giro_objetivo_deg": round(ev["giro_obj"], 2),
                   "zoom_que_encuadra": round(ev["zoom_cabe"], 3)}, f, indent=1)
    print(f"Imagen: {os.path.relpath(salida, AQUI)} (verde, la zona ampliada hasta {100 * esc.altura:.0f} cm; las bandejas "
          "en su color; cian, el zoom actual; amarillo, el que encuadra)")
    if a.guardar:
        if ev["zoom_cabe"] < 1.0:
            print("--guardar: no cabe ni sin zoom; no cambio manipulacion.yaml")
        elif guardar_zoom(zoom_absoluto(ev["zoom_cabe"])):
            print(f"manipulacion.yaml: zoom_absolute {c['controles']['zoom_absolute']} -> {zoom_absoluto(ev['zoom_cabe'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
