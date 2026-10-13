"""Gemelo del montaje en MuJoCo (docs/PLAN.md, "Gemelo del montaje en MuJoCo").

Usa h1_2_mujoco/modelo_h1_2_manos.py del repo (en el PC2, la copia de modelo/ que hace desplegar.sh)."""

import sys

from manip import config

if config.ruta_repo() not in sys.path:
    sys.path.insert(0, config.ruta_repo())
