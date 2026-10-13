#!/usr/bin/env python3
"""Graba fotogramas de camara_servidor.py en datos/dataset_<fecha>_<nombre>/ (formato de manip/fuentes.py).

Solo lee la camara: no mueve nada. Para el dataset de percepcion (6.2), con los objetos colocados en
posiciones medidas con regla, y para las capturas del montaje.

    ./ejecutar.sh herramientas/grabar_dataset.py --nombre mesa_vacia --n 30
    ./ejecutar.sh herramientas/grabar_dataset.py --nombre objetos_a --segundos 5 --cada 3
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import config  # noqa: E402
from manip.fuentes import EscritorDataset, FuenteZmq  # noqa: E402
from manip.registro import carpeta_tirada  # noqa: E402


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--nombre", required=True)
    ap.add_argument("--n", type=int, default=None, help="fotogramas a guardar")
    ap.add_argument("--segundos", type=float, default=None)
    ap.add_argument("--cada", type=int, default=1, help="guardar 1 de cada N")
    a = ap.parse_args()
    if a.n is None and a.segundos is None:
        a.n = 30
    carpeta = carpeta_tirada(config.ruta_datos(cfg), "dataset", a.nombre)
    fuente = FuenteZmq(cfg["red"]["zmq_fotogramas"], solo_ultimo=False, cola=60)
    if fuente.fresco(timeout_s=5.0) is None:
        sys.exit("No llegan fotogramas: ./camara_servidor.sh")
    esc = EscritorDataset(carpeta)
    t0, k, guardados = time.monotonic(), 0, 0
    try:
        while True:
            if a.segundos is not None and time.monotonic() - t0 > a.segundos:
                break
            if a.n is not None and guardados >= a.n:
                break
            f = fuente.siguiente(timeout_s=2.0)
            if f is None:
                sys.exit("La camara dejo de dar fotogramas")
            k += 1
            if (k - 1) % a.cada:
                continue
            esc.escribir(f)
            guardados += 1
    finally:
        esc.cerrar()
        fuente.cerrar()
    flujos = [n for n in ("ir", "color", "prof") if os.path.isdir(os.path.join(carpeta, n))]
    print(f"{esc.escritos} fotogramas ({', '.join(flujos)}), {esc.descartados} descartados -> {carpeta}")


if __name__ == "__main__":
    main()
