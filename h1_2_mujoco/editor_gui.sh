#!/usr/bin/env bash
# Interfaz gráfica del editor de poses del H1-2 con manos (MuJoCo, sin física): vista 3D, rutinas y su tabla
# de pasos, gestos, sliders de brazos y manos en grados y colisiones, en una sola ventana. Se ejecuta EN ESTE PC.
#   ./editor_gui.sh
#   ./editor_gui.sh --poses /otra/carpeta --margen 0.01
# Python con mujoco y PyQt5: $PY, o el entorno h1_mujoco, o python3. Detalle: README.md
AQUI="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${PY:-}" ]; then
    for p in "$HOME/miniconda3/envs/h1_mujoco/bin/python" "$(command -v python3)"; do
        [ -x "$p" ] && "$p" -c "import mujoco, PyQt5.QtWidgets" 2>/dev/null && PY="$p" && break
    done
fi
[ -n "${PY:-}" ] || { echo "No encuentro un python con mujoco y PyQt5 (sudo apt install python3-pyqt5, o PY=/ruta/python)"; exit 1; }
export MUJOCO_GL=${MUJOCO_GL:-glfw}
exec "$PY" "$AQUI/editor_gui.py" "$@"
