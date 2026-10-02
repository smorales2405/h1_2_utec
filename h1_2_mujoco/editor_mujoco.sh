#!/usr/bin/env bash
# Editor de poses del H1-2 CON MANOS en MuJoCo: robot levantado y parado, juntas y dedos con los sliders
# del visor, colisiones en la terminal, y la rutina .json a la carpeta del selector con manos
# (h1_2_joint_control/scripts/selector_poses_manos/poses). Se ejecuta EN ESTE PC (no en el robot).
#   ./editor_mujoco.sh
#   ./editor_mujoco.sh --poses /otra/carpeta --margen 0.03
# Python con mujoco: $PY, o el entorno h1_mujoco, o python3. Detalle: README.md
AQUI="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${PY:-}" ]; then
    for p in "$HOME/miniconda3/envs/h1_mujoco/bin/python" "$(command -v python3)"; do
        [ -x "$p" ] && "$p" -c "import mujoco.viewer" 2>/dev/null && PY="$p" && break
    done
fi
[ -n "${PY:-}" ] || { echo "No encuentro un python con mujoco (pip install mujoco, o PY=/ruta/python)"; exit 1; }
exec "$PY" "$AQUI/editor_poses_mujoco_h1_2_manos.py" "$@"
