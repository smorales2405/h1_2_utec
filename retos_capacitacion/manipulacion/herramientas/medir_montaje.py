#!/usr/bin/env python3
"""Mide la mesa (plano, ArUco 2-5, superficie util, labio y asas) y la escribe en config/montaje.yaml.

Solo lee la camara: no mueve nada. Con la mesa VACIA (sin bandejas ni objetos) para que el plano y el
labio salgan limpios. Conserva lo que el plan fija en el montaje (zona de recogida, bandejas, cruz).

    python3 herramientas/medir_montaje.py datos/dataset_<fecha>_mesa       # en la PC, sobre una grabacion
    ./ejecutar.sh herramientas/medir_montaje.py --vivo                     # en el robot, con la camara
Usa la calibracion mas reciente o, si no hay, la del URDF (sin calibrar: lo dice en el YAML).
"""

import argparse
import datetime
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import calibracion, config  # noqa: E402
from manip import montaje as mt  # noqa: E402
from manip.cinematica import Cinematica  # noqa: E402
from manip.fuentes import FuenteDataset, FuenteZmq  # noqa: E402
from manip.mensajes import Fotograma  # noqa: E402


def promediar(fotos):
    """Un fotograma con la mediana de la profundidad valida de varios (menos ruido) y la ultima imagen."""
    f = fotos[-1]
    P = np.stack([x.prof for x in fotos]).astype(np.float32)
    P[P == 0] = np.nan
    med = np.nan_to_num(np.nanmedian(P, 0), nan=0.0).astype(np.uint16)
    return Fotograma(n=f.n, t_cam=f.t_cam, t_rx=f.t_rx, ir=f.ir, color=f.color, prof=med, emisor=f.emisor, meta=f.meta)


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dataset", nargs="?", help="carpeta grabada con grabar_dataset.py")
    ap.add_argument("--vivo", action="store_true", help="con camara_servidor.py en marcha")
    ap.add_argument("--n", type=int, default=15, help="fotogramas a promediar")
    ap.add_argument("--salida", default=config.RUTA_MONTAJE)
    a = ap.parse_args()
    if a.vivo:
        fuente = FuenteZmq(cfg["red"]["zmq_fotogramas"], solo_ultimo=False, cola=30)
        if fuente.fresco(timeout_s=5.0) is None:
            sys.exit("No llegan fotogramas: ./camara_servidor.sh")
        fotos = [x for x in (fuente.siguiente(2.0) for _ in range(a.n)) if x is not None]
        origen = "camara en vivo"
    elif a.dataset:
        fotos = [x for x in FuenteDataset(a.dataset)][-a.n:]
        origen = os.path.basename(os.path.normpath(a.dataset))
    else:
        sys.exit("Falta el dataset o --vivo")
    fotos = [x for x in fotos if x.prof is not None]
    if not fotos:
        sys.exit("Sin profundidad: camara_servidor.sh con --profundidad")
    f = promediar(fotos)
    cin = Cinematica.desde_config(cfg)
    cal = calibracion.cargar_o_urdf(cin, f.meta)
    previo = config.cargar_montaje() if os.path.exists(a.salida) else {}
    mesa, informe = mt.medir(f, cal, previo, cfg["calibracion"]["diccionario"])
    nuevo = dict(previo)
    nuevo["fecha"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    nuevo["origen"] = f"{origen} ({len(fotos)} fotogramas); calibracion {cal.origen} {cal.fecha}"
    nuevo["mesa"] = mesa
    config.guardar_yaml(a.salida, nuevo, "Montaje del reto. mesa: la escribe herramientas/medir_montaje.py.\n"
                        "zona_recogida, bandejas y cruz: las fija el plan (marco del torso).")
    print(json.dumps({"yaw_deg": mesa["yaw_deg"], "superficie": mesa["superficie"], "labio_alto_m": mesa["labio_alto_m"],
                      "asas": mesa["asas"], "origen_mesa": [r[3] for r in mesa["T_torso_mesa"][:3]], **informe},
                     indent=1, ensure_ascii=False))
    print(f"-> {a.salida}")


if __name__ == "__main__":
    main()
