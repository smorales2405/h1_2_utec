#!/usr/bin/env bash
# Editor de la pose de observacion del reto de manipulacion (MuJoCo, sin fisica): sliders de los dos brazos en
# espejo, poses guardadas en poses_observacion.json, movimiento reposo <-> pose con sus colisiones y la
# comprobacion de manipulacion.py. Se ejecuta EN ESTE PC (no en el robot). Detalle: README.md
#   ./editor_observacion.sh
#   ./editor_observacion.sh --poses /otro/archivo.json
# Python con mujoco, PyQt5 y pinocchio: $PY o python3.
AQUI="$(cd "$(dirname "$0")" && pwd)"
PY=${PY:-python3}
"$PY" -c "import mujoco, pinocchio, PyQt5.QtWidgets" 2>/dev/null \
    || { echo "$PY no tiene mujoco, pinocchio y PyQt5 (sudo apt install python3-pyqt5, o PY=/ruta/python)"; exit 1; }
export MUJOCO_GL=${MUJOCO_GL:-glfw}
cd "$AQUI" && exec "$PY" "$AQUI/editor_observacion.py" "$@"
