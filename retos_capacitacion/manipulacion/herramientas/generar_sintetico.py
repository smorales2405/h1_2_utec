#!/usr/bin/env python3
"""Datos sinteticos del gemelo, con la verdad conocida, en el mismo formato que las grabaciones reales.

    python3 herramientas/generar_sintetico.py calibracion            # las poses de config/poses_calibracion.yaml
    python3 herramientas/generar_sintetico.py objetos --n 100        # los objetos al azar en la zona de recogida

calibracion: el gemelo con una camara y unos marcadores del dorso "verdaderos" (los del URDF desviados unos
grados y milimetros, como se espera del robot) recorre las poses; la grabacion es la de grabar_calibracion.py
y verdad.yaml guarda lo que calibrar_camara_torso.py tiene que recuperar.
objetos: dataset de fotogramas (formato de manip/fuentes.py) y verdad.json con la clase y la pose de cada
objeto en el torso, para evaluar_percepcion.py.
Necesita OpenGL (MUJOCO_GL=glfw en la PC).
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
from manip import config, montaje as mt  # noqa: E402
from manip.cinematica import Cinematica  # noqa: E402
from manip.fuentes import EscritorDataset, guardar_meta  # noqa: E402
from manip.geometria import T, rpy_a_R  # noqa: E402
from manip.mensajes import LADOS  # noqa: E402
from manip.registro import carpeta_tirada, guardar_json  # noqa: E402
from sim.camara_virtual import CamaraVirtual  # noqa: E402
from sim.escena import Estado, construir  # noqa: E402

T_COLOR_PROF_TIPICO = np.array([[1, 0, 0, 0.015], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def desviar(M, rpy_deg, t_mm):
    """M seguida de un pequeno giro y desplazamiento en su propio marco."""
    return M @ T(rpy_a_R(*np.radians(rpy_deg)), np.asarray(t_mm) / 1000.0)


def calibracion(a, cfg):
    cin = Cinematica.desde_config(cfg)
    base = cb.desde_urdf(cin, T_COLOR_PROF_TIPICO)
    verdad = cb.desde_urdf(cin, T_COLOR_PROF_TIPICO)
    verdad.T_torso_color = desviar(base.T_torso_color, a.giro_camara, a.mover_camara)
    verdad.T_dorso = {"izq": desviar(base.T_dorso["izq"], (0, 0, 7), (4, -3, 1)),
                      "der": desviar(base.T_dorso["der"], (0, 0, -12), (-5, 2, 1))}
    m, info = construir(cal=verdad, con_bandejas=False)
    est = Estado(m, info)
    cam = CamaraVirtual(m, info, verdad, ruido_mm=1.0)
    poses = config._leer(os.path.join(config.CONFIG, "poses_calibracion.yaml"))["poses"]
    carpeta = carpeta_tirada(config.ruta_datos(cfg), "calibracion", "sintetica")
    os.makedirs(os.path.join(carpeta, "color"), exist_ok=True)
    rng = np.random.default_rng(a.semilla)
    reposo = {"izq": info["postura"][13:20], "der": info["postura"][20:27]}
    muestras = []
    for lado in LADOS:
        for k, q in enumerate(poses[lado]):
            est.poner_brazo("izq", reposo["izq"])
            est.poner_brazo("der", reposo["der"])
            est.poner_brazo(lado, q)
            est.adelante()
            f = cam.fotograma(est)
            nombre = f"color/{lado}_{k:02d}.png"
            cv2.imwrite(os.path.join(carpeta, nombre), f.color)
            # la q "medida": la verdadera con el ruido de los encoders (~1 mrad)
            q_med = np.asarray(q) + rng.normal(0, a.ruido_q, 7)
            muestras.append({"n": len(muestras), "lado": lado, "q": q_med.round(5).tolist(), "q_cmd": list(q),
                             "color": nombre})
            if len(muestras) == 1:
                guardar_meta(carpeta, f.meta)
    guardar_json(os.path.join(carpeta, "muestras.json"), muestras)
    cb.guardar(verdad, os.path.join(carpeta, "verdad.yaml"))
    print(f"{len(muestras)} muestras -> {carpeta}")


def objetos(a, cfg):
    cin = Cinematica.desde_config(cfg)
    try:
        cal = cb.cargar()
    except FileNotFoundError:
        cal = cb.desde_urdf(cin, T_COLOR_PROF_TIPICO)
    mon = config.cargar_montaje()
    obj = config.cargar_objetos()["objetos"]
    m, info = construir(montaje=mon, cal=cal)
    est = Estado(m, info)
    cam = CamaraVirtual(m, info, cal, ruido_mm=a.ruido_prof)
    carpeta = carpeta_tirada(config.ruta_datos(cfg), "dataset", "sintetico_objetos")
    esc = EscritorDataset(carpeta)
    rng = np.random.default_rng(a.semilla)
    z = mon["zona_recogida"]
    verdad = []
    for k in range(a.n):
        puestos = []
        for clave, o in obj.items():
            if rng.random() < 0.15:          # a veces falta alguno
                est.poner_objeto(clave)
                continue
            r = max(o["medidas_m"][:2]) / 2 if o["forma"] != "esfera" else o["medidas_m"][0] / 2
            for _ in range(50):
                xy = np.array([rng.uniform(*z["x"]), rng.uniform(*z["y"])])
                if all(np.linalg.norm(xy - p[0]) > r + p[1] + 0.02 for p in puestos):
                    break
            else:
                est.poner_objeto(clave)
                continue
            alto = o["medidas_m"][0] if o["forma"] == "esfera" else (o["medidas_m"][1] if o["forma"] == "cilindro"
                                                                     else o["medidas_m"][2])
            yaw = rng.uniform(-math.pi, math.pi)
            Tm = mt.T_torso_mesa(mon)
            p = mt.punto_en_mesa(mon, xy, alto / 2)
            R = Tm[:3, :3] @ rpy_a_R(0, 0, yaw)
            est.poner_objeto(clave, p, R)
            puestos.append((xy, r))
            verdad.append({"n": k + 1, "clase": clave, "p": p.round(4).tolist(),
                           "yaw": round(float(yaw + math.atan2(Tm[1, 0], Tm[0, 0])), 4)})
        est.adelante()
        f = cam.fotograma(est, t=float(k) / 30.0)
        esc.escribir(f)
    esc.cerrar()
    guardar_json(os.path.join(carpeta, "verdad.json"), verdad)
    print(f"{a.n} fotogramas, {len(verdad)} objetos -> {carpeta}")


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("tipo", choices=["calibracion", "objetos"])
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--semilla", type=int, default=0)
    ap.add_argument("--giro-camara", type=float, nargs=3, default=[0.6, -1.2, 0.4], help="grados")
    ap.add_argument("--mover-camara", type=float, nargs=3, default=[6.0, -4.0, 8.0], help="mm")
    ap.add_argument("--ruido-q", type=float, default=0.001, help="rad")
    ap.add_argument("--ruido-prof", type=float, default=2.0, help="mm")
    a = ap.parse_args()
    calibracion(a, cfg) if a.tipo == "calibracion" else objetos(a, cfg)


if __name__ == "__main__":
    main()
