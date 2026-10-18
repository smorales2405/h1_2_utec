#!/usr/bin/env bash
# Copia la manipulacion al PC2 desde la PC, con lo que necesita del repo (que el PC2 no tiene):
# el URDF con manos y sus mallas, h1_2_mujoco/modelo_h1_2_manos.py y las ganancias de
# h1_2_joint_control/config/gains.yaml (herramientas/comparar_ganancias.py), en modelo/ con la misma
# estructura relativa que en el repo.
#   ./desplegar.sh            codigo, config y modelo -> ~/utec/manipulacion/
#   ./desplegar.sh --traer    datos/ del robot -> datos/ de la PC
# La configuracion (tambien la calibracion y el montaje) se escribe en la PC y se despliega; el
# robot solo graba en datos/. Por cable: ROBOT_SSH=unitree@192.168.123.164 ./desplegar.sh
set -e
ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
AQUI="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$AQUI/../.." && pwd)"
DEST=utec/manipulacion

if [ "${1:-}" = "--traer" ]; then
    rsync -av "$ROBOT_SSH:$DEST/datos/" "$AQUI/datos/"
    exit 0
fi
rsync -av --exclude datos/ --exclude __pycache__ --exclude .venv/ --exclude modelo/ \
      --exclude docs/ --exclude montajes/ --exclude .pytest_cache/ "$AQUI/" "$ROBOT_SSH:$DEST/"
ssh "$ROBOT_SSH" "mkdir -p $DEST/modelo/ros_h1_2_ws/src/h1_2_inspire_description $DEST/modelo/h1_2_mujoco $DEST/modelo/h1_2_joint_control/config"
rsync -av "$REPO/ros_h1_2_ws/src/h1_2_inspire_description/urdf" "$REPO/ros_h1_2_ws/src/h1_2_inspire_description/meshes" \
      "$ROBOT_SSH:$DEST/modelo/ros_h1_2_ws/src/h1_2_inspire_description/"
rsync -av "$REPO/h1_2_mujoco/modelo_h1_2_manos.py" "$ROBOT_SSH:$DEST/modelo/h1_2_mujoco/"
rsync -av "$REPO/h1_2_joint_control/config/gains.yaml" "$ROBOT_SSH:$DEST/modelo/h1_2_joint_control/config/"
