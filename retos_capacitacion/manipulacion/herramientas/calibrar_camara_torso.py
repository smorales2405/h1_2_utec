#!/usr/bin/env python3
"""Calibracion camara-torso (6.1.3 y 6.1.4) a partir de una grabacion de grabar_calibracion.py (o sintetica).

Detecta el ArUco del dorso en el color de cada muestra, ajusta T_torso_color y los dos marcadores por
minimos cuadrados de reproyeccion con la q MEDIDA, y da el residuo (pixeles y mm). Con --consigna ajusta
tambien con la q mandada, para ver que se calibra en realidad con el brazo ~5 grados corto (6.1.4).
Escribe config/calibracion_<fecha>.yaml (salvo --no-guardar). Si la grabacion trae verdad.yaml (sintetica),
compara con ella.

    python3 herramientas/calibrar_camara_torso.py datos/calibracion_<fecha>
"""

import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import calibracion as cb  # noqa: E402
from manip import config  # noqa: E402
from manip.cinematica import Cinematica  # noqa: E402
from manip.fuentes import T_color_prof  # noqa: E402
from manip.geometria import Intrinsecos, R_a_rotvec, inv  # noqa: E402


def diferencia(A, B):
    """(mm, grados) entre dos poses 4x4."""
    D = inv(A) @ B
    return 1000 * float(np.linalg.norm(A[:3, 3] - B[:3, 3])), math.degrees(float(np.linalg.norm(R_a_rotvec(D[:3, :3]))))


def profundidad_en_marcador(carpeta, cin, cal, meta, ventana=3):
    """Con la camara ya calibrada (el color): donde tiene que estar el centro de cada marcador y lo que mide la
    profundidad alli (mediana de (2*ventana+1)^2 pixeles validos). [(n, lado, z esperada, z medida)] de las
    muestras con prof/. Si la profundidad mide de mas (la mesa ~3 % mas grande, RESULTADOS §22), sale aqui."""
    with open(os.path.join(carpeta, "muestras.json")) as f:
        filas = json.load(f)
    ip = Intrinsecos.de_meta(meta["prof"])
    T_prof_torso = inv(T_color_prof(meta)) @ inv(cal.T_torso_color)
    sal = []
    for m in filas:
        if "prof" not in m or m["lado"] not in cal.T_dorso:
            continue
        P = cv2.imread(os.path.join(carpeta, m["prof"]), cv2.IMREAD_UNCHANGED)
        if P is None:
            continue
        p = T_prof_torso @ (cin.fk(m["lado"], np.array(m["q"]), "mano") @ cal.T_dorso[m["lado"]])[:, 3]
        u, v = (int(round(c)) for c in ip.proyectar(p[None, :3])[0])
        if not (ventana <= u < P.shape[1] - ventana and ventana <= v < P.shape[0] - ventana):
            continue
        w = P[v - ventana:v + ventana + 1, u - ventana:u + ventana + 1].astype(float)
        w = w[w > 0] * float(meta.get("escala_prof", 0.001))
        if len(w) >= 5:
            sal.append((m["n"], m["lado"], float(p[2]), float(np.median(w))))
    return sal


def leer(carpeta, cfg, usar="q"):
    with open(os.path.join(carpeta, "meta.json")) as f:
        meta = json.load(f)
    with open(os.path.join(carpeta, "muestras.json")) as f:
        filas = json.load(f)
    det = cb.detector(cfg["calibracion"]["diccionario"])
    ids = cfg["calibracion"]["ids"]
    muestras, perdidas = [], []
    for m in filas:
        img = cv2.imread(os.path.join(carpeta, m["color"]))
        d = cb.detectar(img, det, {ids[m["lado"]]})
        if ids[m["lado"]] not in d:
            perdidas.append(m["n"])
            continue
        muestras.append({"lado": m["lado"], "q": np.array(m[usar] if usar in m else m["q"]),
                         "esquinas": d[ids[m["lado"]]], "n": m["n"]})
    return meta, muestras, perdidas


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("grabacion")
    ap.add_argument("--consigna", action="store_true", help="ademas, ajustar con la q mandada (6.1.4)")
    ap.add_argument("--no-guardar", action="store_true")
    a = ap.parse_args()
    cin = Cinematica.desde_config(cfg)
    meta, muestras, perdidas = leer(a.grabacion, cfg)
    print(f"{len(muestras)} muestras con el marcador visible; sin el: {perdidas or 'ninguna'}")
    if len(muestras) < cfg["calibracion"]["poses_min"]:
        sys.exit(f"hacen falta al menos {cfg['calibracion']['poses_min']}")
    intr = Intrinsecos.de_meta(meta["color"])
    cal0 = cb.desde_urdf(cin, T_color_prof(meta))
    lado_m = cfg["calibracion"]["marcador_lado_m"]
    cal, inf = cb.ajustar(muestras, cin, intr, cal0, lado_m)
    print(f"residuo: {cal.residuo_px:.2f} px de media ({inf['error_px_max']:.2f} max), "
          f"{cal.residuo_mm:.1f} mm de media ({inf['error_mm_max']:.1f} max) en el centro del marcador")
    mm, deg = diferencia(cal0.T_torso_color, cal.T_torso_color)
    print(f"camara respecto del URDF: {mm:.1f} mm y {deg:.2f} grados")
    for lado in inf["lados"]:
        mm, deg = diferencia(cb.T_DORSO_DEFECTO[lado], cal.T_dorso[lado])
        print(f"dorso {lado} respecto de la semilla: {mm:.1f} mm y {deg:.1f} grados")
    T_kabsch = cb.kabsch_centros(muestras, cin, intr, cal, lado_m)
    mm, deg = diferencia(cal.T_torso_color, T_kabsch)
    print(f"Kabsch de los centros (con el dorso ajustado) frente al ajuste completo: {mm:.1f} mm, {deg:.2f} grados")
    extra = {"grabacion": os.path.basename(os.path.normpath(a.grabacion)), "sin_marcador": perdidas,
             "error_px_por_muestra": inf["error_px_por_muestra"], "error_mm_por_muestra": inf["error_mm_por_muestra"]}
    prof = profundidad_en_marcador(a.grabacion, cin, cal, meta)
    if prof:
        ze, zm = np.array([p[2] for p in prof]), np.array([p[3] for p in prof])
        r = zm / ze
        b, c = np.polyfit(ze, r, 1)
        print(f"profundidad en el marcador / la esperada: {np.median(r):.4f} de mediana (p10-p90 "
              f"{np.percentile(r, 10):.4f}-{np.percentile(r, 90):.4f}) en {len(r)} muestras a {ze.min():.2f}-"
              f"{ze.max():.2f} m; con la distancia: {c + b * ze.min():.4f} a {ze.min():.2f} m, {c + b * ze.max():.4f} "
              f"a {ze.max():.2f} m")
        extra["profundidad_marcador"] = {"mediana": round(float(np.median(r)), 4), "pendiente_por_m": round(float(b), 4),
                                         "muestras": [[n, l, round(z1, 4), round(z2, 4)] for n, l, z1, z2 in prof]}
    if a.consigna:
        _, mc, _ = leer(a.grabacion, cfg, usar="q_cmd")
        cal_c, inf_c = cb.ajustar(mc, cin, intr, cal0, lado_m)
        mm, deg = diferencia(cal.T_torso_color, cal_c.T_torso_color)
        print(f"con la consigna: residuo {cal_c.residuo_px:.2f} px / {cal_c.residuo_mm:.1f} mm; la camara cambia "
              f"{mm:.1f} mm y {deg:.2f} grados respecto de la calibrada con la q medida")
        extra["con_consigna"] = {"residuo_px": round(cal_c.residuo_px, 2), "residuo_mm": round(cal_c.residuo_mm, 1),
                                 "camara_mm": round(mm, 1), "camara_deg": round(deg, 2)}
    ruta_verdad = os.path.join(a.grabacion, "verdad.yaml")
    if os.path.exists(ruta_verdad):
        v = cb.cargar(ruta_verdad)
        mm, deg = diferencia(v.T_torso_color, cal.T_torso_color)
        print(f"VERDAD (sintetica): camara a {mm:.1f} mm y {deg:.2f} grados de la verdadera")
        for lado in inf["lados"]:
            mm, deg = diferencia(v.T_dorso[lado], cal.T_dorso[lado])
            print(f"VERDAD (sintetica): dorso {lado} a {mm:.1f} mm y {deg:.2f} grados")
        extra["frente_a_verdad"] = True
    if not a.no_guardar and not os.path.exists(ruta_verdad):
        print(f"-> {cb.guardar(cal, extra=extra)}")


if __name__ == "__main__":
    main()
