#!/usr/bin/env bash
# Entorno comun de los lanzadores del seguidor de linea. Se carga con:  source "$(dirname "$0")/comun.sh"
# Mismo patron que ~/robotics40/comun.sh, pero para ~/utec/seguidor_linea.
#
# FUERA DEL ROBOT el lanzador se reenvia por SSH y se ejecuta el codigo del robot
# (rsync antes si se cambio algo aqui):
#   ./camara_servidor.sh   ->   ssh -t unitree@192.168.0.143 'cd ~/utec/seguidor_linea && ./camara_servidor.sh'
#   ROBOT_SSH=unitree@192.168.123.164 ./ejecutar.sh herramientas/comprobar.py    # por el cable

ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
if [ "$(hostname)" != "unitree-h1-2-pc4" ]; then
    _sl_base="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    _sl_yo="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
    _sl_rel="${_sl_yo#"$_sl_base"/}"
    _sl_args=""
    for _a in "$@"; do _sl_args+=" $(printf '%q' "$_a")"; done
    echo "(en el robot $ROBOT_SSH) ~/utec/seguidor_linea/$_sl_rel$_sl_args" >&2
    exec ssh -t -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
         "$ROBOT_SSH" "cd ~/utec/seguidor_linea && ./$_sl_rel$_sl_args"
fi

SL=/home/unitree/utec/seguidor_linea
# teleop_venv tiene a la vez unitree_sdk2py.h1 (LocoClient), pyrealsense2, zmq, cv2 y yaml
PY=/home/unitree/teleop_venv/bin/python

# ~/.local se cuela en cualquier python 3.10 del PC2; el CYCLONEDDS_URI del .bashrc es el del SLAM de fabrica
export PYTHONNOUSERSITE=1
unset CYCLONEDDS_URI RMW_IMPLEMENTATION
