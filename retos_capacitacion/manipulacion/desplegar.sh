#!/usr/bin/env bash
# Copia la manipulacion al PC2 desde la PC, con lo que necesita del repo (que el PC2 no tiene):
# el URDF con manos y sus mallas, el acople de la mano (scripts/hand_coupling.py y sus curvas en config/),
# h1_2_mujoco/modelo_h1_2_manos.py y las ganancias de
# h1_2_joint_control/config/gains.yaml (herramientas/comparar_ganancias.py), en modelo/ con la misma
# estructura relativa que en el repo.
#   ./desplegar.sh            codigo, config y modelo -> ~/utec/manipulacion/
#   ./desplegar.sh --traer    datos/ del robot -> datos/ de la PC
# La configuracion (tambien la calibracion y el montaje) se escribe en la PC y se despliega; el
# robot solo graba en datos/. Por cable: ROBOT_SSH=unitree@192.168.123.164 ./desplegar.sh
# Salvo config/montaje.yaml cuando el del robot es mas nuevo (su `fecha:`): lo escribe el [g] de
# marcar_montaje.py en cada sesion (el robot se pone de pie cada vez un poco distinto) y un despliegue
# lo pisaba con el de la PC (2026-11-11). Entonces no se copia y se trae a datos/montaje_robot_<fecha>.yaml.
set -e
ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
AQUI="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$AQUI/../.." && pwd)"
DEST=utec/manipulacion

if [ "${1:-}" = "--traer" ]; then
    rsync -av "$ROBOT_SSH:$DEST/datos/" "$AQUI/datos/"
    exit 0
fi
fecha_pc=$(grep -m1 '^fecha:' "$AQUI/config/montaje.yaml" | cut -d' ' -f2-)
fecha_robot=$(ssh "$ROBOT_SSH" "grep -m1 '^fecha:' $DEST/config/montaje.yaml 2>/dev/null | cut -d' ' -f2-" || true)
SIN_MONTAJE=()
if [ -n "$fecha_robot" ] && [[ "$fecha_robot" > "$fecha_pc" ]]; then
    copia="$AQUI/datos/montaje_robot_$(echo "$fecha_robot" | tr -d ': -').yaml"
    mkdir -p "$AQUI/datos"
    scp -q "$ROBOT_SSH:$DEST/config/montaje.yaml" "$copia"
    echo "OJO: el montaje del robot ($fecha_robot) es mas nuevo que el de la PC ($fecha_pc): no se pisa."
    echo "     Copia en $copia"
    SIN_MONTAJE=(--exclude config/montaje.yaml)
fi
rsync -av --exclude datos/ --exclude __pycache__ --exclude .venv/ --exclude modelo/ \
      --exclude docs/ --exclude montajes/ --exclude .pytest_cache/ "${SIN_MONTAJE[@]}" "$AQUI/" "$ROBOT_SSH:$DEST/"
ssh "$ROBOT_SSH" "mkdir -p $DEST/modelo/ros_h1_2_ws/src/h1_2_inspire_description/scripts $DEST/modelo/h1_2_mujoco $DEST/modelo/h1_2_joint_control/config"
rsync -av "$REPO/ros_h1_2_ws/src/h1_2_inspire_description/urdf" "$REPO/ros_h1_2_ws/src/h1_2_inspire_description/meshes" \
      "$REPO/ros_h1_2_ws/src/h1_2_inspire_description/config" \
      "$ROBOT_SSH:$DEST/modelo/ros_h1_2_ws/src/h1_2_inspire_description/"
rsync -av "$REPO/ros_h1_2_ws/src/h1_2_inspire_description/scripts/hand_coupling.py" \
      "$ROBOT_SSH:$DEST/modelo/ros_h1_2_ws/src/h1_2_inspire_description/scripts/"
rsync -av "$REPO/h1_2_mujoco/modelo_h1_2_manos.py" "$ROBOT_SSH:$DEST/modelo/h1_2_mujoco/"
rsync -av "$REPO/h1_2_joint_control/config/gains.yaml" "$ROBOT_SSH:$DEST/modelo/h1_2_joint_control/config/"
