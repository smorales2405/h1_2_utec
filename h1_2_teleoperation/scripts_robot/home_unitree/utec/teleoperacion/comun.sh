#!/usr/bin/env bash
# Entorno comun de los lanzadores de la teleoperacion. Se carga con:  source "$(dirname "$0")/comun.sh"
# No lanza nada: solo fija rutas y variables.
#
# FUERA DEL ROBOT (el PC del operador) el mismo lanzador se reenvia al robot por SSH:
#   ./teleop_xr.sh   ->   ssh -t unitree@192.168.0.143 'cd ~/utec/teleoperacion && ./teleop_xr.sh'
# El codigo que se ejecuta es SIEMPRE el del robot (~/utec/teleoperacion alli), que es el que tiene los SDK,
# los entornos y el DDS de Unitree (que no sale de eth0). Hace falta acceso por clave:  ssh-copy-id.
#   ROBOT_SSH=unitree@192.168.123.164 ./estado.sh    # por el cable interno, si el PC esta en esa red

ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
if [ "$(hostname)" != "unitree-h1-2-pc4" ]; then
    _base="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    _yo="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
    _rel="${_yo#"$_base"/}"
    _args=""
    for _a in "$@"; do _args+=" $(printf '%q' "$_a")"; done
    _env=""                                       # IFACE la usa estado.sh, tambien viaja
    [ -n "${IFACE:-}" ] && _env="IFACE=$(printf '%q' "$IFACE") "
    echo "(en el robot $ROBOT_SSH) ${_env}~/utec/teleoperacion/$_rel$_args" >&2
    exec ssh -t -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
         "$ROBOT_SSH" "cd ~/utec/teleoperacion && ${_env}./$_rel$_args"
fi

TELEOP=/home/unitree/utec/teleoperacion
R40=/home/unitree/robotics40                                 # sus lectores_h1_2, para estado.sh
PY_SDK=/home/unitree/unitree_sdk2_python/.venv/bin/python    # para estado.sh
IFACE=${IFACE:-eth0}                                         # DDS de Unitree: solo por el cable interno

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
