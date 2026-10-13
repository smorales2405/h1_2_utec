#!/usr/bin/env bash
# Entorno comun de los lanzadores de la manipulacion. Se carga con:  source "$(dirname "$0")/comun.sh"
# Mismo patron que seguidor_linea/comun.sh, pero con el entorno propio ~/utec/manipulacion/.venv
# (lo crea instalar_entorno.sh) en vez de teleop_venv o env_tv.
#
# FUERA DEL ROBOT el lanzador se reenvia por SSH y se ejecuta el codigo del robot
# (desplegar.sh antes si se cambio algo aqui):
#   ./camara_servidor.sh   ->   ssh -t unitree@192.168.0.143 'cd ~/utec/manipulacion && ./camara_servidor.sh'
#   ROBOT_SSH=unitree@192.168.123.164 ./ejecutar.sh herramientas/comprobar.py    # por el cable

ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
if [ "$(hostname)" != "unitree-h1-2-pc4" ]; then
    _mp_base="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    _mp_yo="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
    _mp_rel="${_mp_yo#"$_mp_base"/}"
    _mp_args=""
    for _a in "$@"; do _mp_args+=" $(printf '%q' "$_a")"; done
    echo "(en el robot $ROBOT_SSH) ~/utec/manipulacion/$_mp_rel$_mp_args" >&2
    exec ssh -t -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
         "$ROBOT_SSH" "cd ~/utec/manipulacion && ./$_mp_rel$_mp_args"
fi

MP=/home/unitree/utec/manipulacion
# entorno propio (instalar_entorno.sh): cyclonedds, pinocchio, cv2, pyrealsense2, pymodbus, zmq, mujoco
PY=$MP/.venv/bin/python
exigir_entorno() { [ -x "$PY" ] || { echo "Falta $PY: ./instalar_entorno.sh"; exit 1; }; }

# ~/.local se cuela en cualquier python 3.10 del PC2; el CYCLONEDDS_URI del .bashrc es el del SLAM de fabrica
export PYTHONNOUSERSITE=1
unset CYCLONEDDS_URI RMW_IMPLEMENTATION
# sin pantalla en el PC2: MuJoCo solo se usa para colisiones, sin render
export MUJOCO_GL=${MUJOCO_GL:-disable}
