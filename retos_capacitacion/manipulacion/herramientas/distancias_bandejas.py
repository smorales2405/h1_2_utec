#!/usr/bin/env python3
"""Donde colocar cada bandeja con regla, desde la mesa guardada con [g] en marcar_montaje.py: la distancia de sus
paredes a la cara interior de los labios del tablero, con la bandeja en su objetivo de montaje.yaml.

    ./ejecutar.sh herramientas/distancias_bandejas.py         # en el robot, despues del [g] de la mesa
    python3 herramientas/distancias_bandejas.py               # en la PC (config/montaje.yaml)
    python3 herramientas/distancias_bandejas.py --montaje datos/dataset_<fecha>_mesa_marcar/montaje.yaml

Solo lee el montaje: ni camara ni robot. marcar_montaje.py imprime la misma tabla al guardar la mesa. Las cuentas
(con el giro de la mesa y lo que la camara la ve de mas) estan en manip/montaje.py, distancias_bandejas().
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import config  # noqa: E402
from manip import montaje as mt  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--montaje", default=config.RUTA_MONTAJE, help="montaje.yaml (por defecto, el de config/)")
    a = ap.parse_args()
    montaje = config.cargar_montaje(a.montaje)
    if "mesa" not in montaje:
        sys.exit(f"{a.montaje} no tiene la mesa medida: [g] en marcar_montaje.py")
    print("\n".join(mt.texto_distancias(montaje)))


if __name__ == "__main__":
    main()
