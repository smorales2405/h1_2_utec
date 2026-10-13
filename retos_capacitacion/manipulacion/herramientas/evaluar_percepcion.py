#!/usr/bin/env python3
"""Evalua la percepcion sobre un dataset (6.2): deteccion y reconocimiento, error de posicion y de yaw contra
la verdad (verdad.json del gemelo, o la regla: --regla clase=x,y en m del torso), ms por fotograma, e
imagenes con los objetos dibujados.

    python3 herramientas/evaluar_percepcion.py datos/dataset_<fecha>_sintetico_objetos
    python3 herramientas/evaluar_percepcion.py datos/dataset_<fecha>_objetos_a --regla tenis=0.45,0.05 cubo=0.47,-0.08
"""

import argparse
import json
import math
import os
import sys
from collections import defaultdict

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import calibracion as cb  # noqa: E402
from manip import config  # noqa: E402
from manip.cinematica import Cinematica  # noqa: E402
from manip.fuentes import FuenteDataset, intrinsecos  # noqa: E402
from manip.geometria import envolver_simetria  # noqa: E402
from manip.percepcion import Percepcion, dibujar  # noqa: E402


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dataset")
    ap.add_argument("--regla", nargs="*", default=[], help="clase=x,y medidos con regla (torso, m)")
    ap.add_argument("--imagenes", type=int, default=8, help="cuantas imagenes dibujadas guardar")
    a = ap.parse_args()
    fuente = FuenteDataset(a.dataset)
    cin = Cinematica.desde_config(cfg)
    cal = cb.cargar_o_urdf(cin, fuente.meta)
    obj = config.cargar_objetos()
    per = Percepcion(cfg, config.cargar_montaje(), obj, cal, cin)
    verdad = defaultdict(dict)
    ruta_v = os.path.join(a.dataset, "verdad.json")
    if os.path.exists(ruta_v):
        for v in json.load(open(ruta_v)):
            verdad[v["n"]][v["clase"]] = v
    regla = {}
    for r in a.regla:
        k, xy = r.split("=")
        regla[k] = np.array([float(v) for v in xy.split(",")])
    salida = os.path.join(a.dataset, "percepcion")
    os.makedirs(salida, exist_ok=True)
    err_pos, err_yaw, ms = defaultdict(list), defaultdict(list), []
    aciertos = fallos = falsos = 0
    for k, f in enumerate(fuente):
        esc = per.procesar(f)
        ms.append(esc.ms)
        vistos = {o.clase: o for o in esc.objetos}
        ref = verdad.get(f.n) or verdad.get(k + 1) or {c: {"p": list(p) + [0]} for c, p in regla.items()}
        for clase, v in ref.items():
            o = vistos.get(clase)
            if o is None:
                fallos += 1
                continue
            aciertos += 1
            err_pos[clase].append(1000 * float(np.linalg.norm(o.p[:2] - np.array(v["p"][:2]))))
            sim = math.radians(obj["objetos"][clase].get("simetria_deg", 0))
            if "yaw" in v and sim > 0:
                err_yaw[clase].append(abs(math.degrees(envolver_simetria(o.yaw - v["yaw"], sim))))
        falsos += sum(1 for c in vistos if ref and c not in ref)
        if k < a.imagenes and f.color is not None:
            cv2.imwrite(os.path.join(salida, f"{k:03d}.png"), dibujar(f.color, esc, per.montaje, cal, intrinsecos(f, "color")))
        if not ref and k < 3:
            print(f"fotograma {f.n}: " + ", ".join(f"{o.clase} ({o.confianza:.2f}) en {np.round(o.p, 3).tolist()} "
                                                  f"yaw {math.degrees(o.yaw):.0f} medidas {np.round(o.medidas * 1000).astype(int).tolist()}"
                                                  for o in esc.objetos))
    print(f"{k + 1} fotogramas; {np.median(ms):.1f} ms de mediana por fotograma (max {max(ms):.1f})")
    if aciertos + fallos:
        print(f"reconocidos {aciertos} de {aciertos + fallos} ({100 * aciertos / (aciertos + fallos):.1f} %); "
              f"falsos o de mas: {falsos}")
        for c in sorted(err_pos):
            e = np.array(err_pos[c])
            y = np.array(err_yaw.get(c, []))
            print(f"  {c:9s} posicion {e.mean():5.1f} mm de media, {np.percentile(e, 95):5.1f} p95, {e.max():5.1f} max"
                  + (f" | yaw {y.mean():4.1f} grados de media, {y.max():4.1f} max" if len(y) else ""))
    print(f"imagenes en {salida}")


if __name__ == "__main__":
    main()
