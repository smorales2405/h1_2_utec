#!/usr/bin/env bash
# Entorno comun de los lanzadores de robotics40. Se carga con:  source "$(dirname "$0")/comun.sh"
# No lanza nada: solo fija rutas y variables.
#
# FUERA DEL ROBOT (p. ej. el PC de UTEC) el mismo lanzador se reenvia al robot por SSH:
#   ~/robotics40/cuadrado.sh --lado 2   ->   ssh -t unitree@192.168.0.143 'cd ~/robotics40 && ./cuadrado.sh --lado 2'
# El codigo que se ejecuta es SIEMPRE el del robot (~/robotics40 alli), que es el que tiene los SDK,
# los entornos y el DDS de Unitree (que no sale de eth0). Hace falta acceso por clave:  ssh-copy-id.
#   ROBOT_SSH=unitree@192.168.123.164 ./estado.sh    # por el cable interno, si el PC esta en esa red

ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
if [ "$(hostname)" != "unitree-h1-2-pc4" ]; then
    _r40_base="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    _r40_yo="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
    _r40_rel="${_r40_yo#"$_r40_base"/}"
    _r40_args=""
    for _a in "$@"; do _r40_args+=" $(printf '%q' "$_a")"; done
    _r40_env=""                                   # variables que usan los lanzadores, tambien viajan
    for _v in IFACE KP_BRAZO CAMARA_FPS; do
        [ -n "${!_v:-}" ] && _r40_env+="$_v=$(printf '%q' "${!_v}") "
    done
    echo "(en el robot $ROBOT_SSH) ${_r40_env}~/robotics40/$_r40_rel$_r40_args" >&2
    exec ssh -t -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
         "$ROBOT_SSH" "cd ~/robotics40 && ${_r40_env}./$_r40_rel$_r40_args"
fi

R40=/home/unitree/robotics40
PY_SDK=/home/unitree/unitree_sdk2_python/.venv/bin/python   # tiene unitree_sdk2py.h1 (LocoClient)
PY_TELEOP=/home/unitree/teleop_venv/bin/python               # unitree_sdk2py.h1 + pymodbus + pyrealsense2 + flask
IFACE=${IFACE:-eth0}                                         # DDS de Unitree: solo por el cable interno
MANO_IZQ=192.168.124.211                                     # al reves que en la doc de Unitree
MANO_DER=192.168.124.210

# ~/.local se cuela en cualquier python 3.10 del PC2; el CYCLONEDDS_URI del .bashrc es el del SLAM de fabrica
export PYTHONNOUSERSITE=1
unset CYCLONEDDS_URI RMW_IMPLEMENTATION

aviso_mueve() {
    # $1 = que se mueve. Recordatorio antes de cualquier cosa que mueva el robot.
    echo "=================================================================="
    echo "  ESTO MUEVE EL ROBOT: $1"
    echo "  L2 + B en el mando es la parada de emergencia. Tenlo en la mano."
    echo "=================================================================="
}

mano_ip() {
    # izq|der -> IP Modbus de esa mano
    case "${1:-}" in
        izq|izquierda|l) echo "$MANO_IZQ" ;;
        der|derecha|r)   echo "$MANO_DER" ;;
        *) echo "mano desconocida: '${1:-}' (usa izq o der)" >&2; return 1 ;;
    esac
}
