"""Donde estan el codigo del reto y el regulador: se importan tal cual, sin copiarlos ni tocarlos.

El reto (retos_capacitacion/manipulacion) se busca, en orden:
  1. $H1_2_RETO
  2. ../retos_capacitacion/manipulacion (la PC, el repo)
  3. ~/utec/manipulacion (el robot: lo despliega retos_capacitacion/manipulacion/desplegar.sh)
En el robot se usa su codigo, su config (montaje, calibracion de la camara y de las manos) y su .venv: el brazo
"firmware" del banco es el reto tal cual.

El regulador es la copia fija de regulador/ (regulador/ORIGEN.md).
"""

import os
import sys

AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))          # h1_2_manipulation
REGULADOR = os.path.join(AQUI, "regulador")


def ruta_reto():
    candidatas = [os.environ.get("H1_2_RETO"),
                  os.path.join(os.path.dirname(AQUI), "retos_capacitacion", "manipulacion"),
                  os.path.expanduser("~/utec/manipulacion")]
    for c in candidatas:
        if c and os.path.isfile(os.path.join(c, "manip", "supervisor.py")):
            return os.path.abspath(c)
    raise RuntimeError("no encuentro el codigo del reto (manip/supervisor.py) en: "
                       + ", ".join(c for c in candidatas if c) + ". Pon H1_2_RETO.")


def preparar():
    """Pone el reto y el regulador en sys.path (una vez). Devuelve la ruta del reto."""
    reto = ruta_reto()
    for p in (REGULADOR, reto):
        if p not in sys.path:
            sys.path.insert(0, p)
    return reto
