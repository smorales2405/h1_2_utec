#!/usr/bin/env bash
# Entorno comun de los lanzadores (como retos_capacitacion/manipulacion/comun.sh). Se carga con:
#   source "$(dirname "$0")/comun.sh"
# En el robot: el codigo de ~/utec/h1_2_manipulation con el reto de ~/utec/manipulacion (su config, su calibracion y su
# .venv). FUERA DEL ROBOT el lanzador se reenvia por SSH (./desplegar.sh antes si se cambio algo aqui o en el reto) y,
# si la camara externa (BRIO) esta en esta PC, graba mientras dura y al acabar trae la carpeta del robot y saca las
# fotos y los clips de cada ensayo (con el camara_externa.py del reto). CAMARA_EXTERNA=0 lo apaga.

ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
if [ "$(hostname)" != "unitree-h1-2-pc4" ]; then
    _mr_base="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    _mr_reto="$_mr_base/../retos_capacitacion/manipulacion"
    _mr_yo="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
    _mr_rel="${_mr_yo#"$_mr_base"/}"
    _mr_args=""
    for _a in "$@"; do _mr_args+=" $(printf '%q' "$_a")"; done
    echo "(en el robot $ROBOT_SSH) ~/utec/h1_2_manipulation/$_mr_rel$_mr_args" >&2
    _mr_ssh=(ssh -t -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=3
             "$ROBOT_SSH" "cd ~/utec/h1_2_manipulation && ./$_mr_rel$_mr_args")
    if [ "$_mr_rel" != "ejecutar.sh" ] && [ "${CAMARA_EXTERNA:-1}" != 0 ] && [[ " $* " != *" --sim "* ]] \
       && python3 "$_mr_reto/herramientas/camara_externa.py" hay 2>/dev/null; then
        mkdir -p "$_mr_reto/datos/externa"
        python3 "$_mr_reto/herramientas/camara_externa.py" grabar < /dev/null >> "$_mr_reto/datos/externa/grabar.log" 2>&1 &
        _mr_cam=$!
        echo "(camara externa grabando: retos_capacitacion/manipulacion/datos/externa/)" >&2
        "${_mr_ssh[@]}"
        _mr_rc=$?
        kill -TERM "$_mr_cam" 2>/dev/null
        wait "$_mr_cam" 2>/dev/null
        _mr_ult=$(ssh -o ConnectTimeout=5 "$ROBOT_SSH" "ls -td utec/h1_2_manipulation/datos/*_20* 2>/dev/null | head -1")
        if [ -n "$_mr_ult" ]; then
            mkdir -p "$_mr_base/datos"
            rsync -a "$ROBOT_SSH:$_mr_ult" "$_mr_base/datos/" \
                && python3 "$_mr_reto/herramientas/camara_externa.py" fotos "$_mr_base/datos/$(basename "$_mr_ult")" || true
        fi
        exit $_mr_rc
    fi
    exec "${_mr_ssh[@]}"
fi

MR=/home/unitree/utec/h1_2_manipulation
MP=/home/unitree/utec/manipulacion
# el entorno del reto (su instalar_entorno.sh): cyclonedds, pinocchio, cv2, pyrealsense2, pymodbus, zmq, mujoco
PY=$MP/.venv/bin/python
exigir_entorno() {
    [ -x "$PY" ] || { echo "Falta $PY: el reto sin desplegar o sin ./instalar_entorno.sh"; exit 1; }
    [ -f "$MP/manip/supervisor.py" ] || { echo "Falta el reto en $MP: ./desplegar.sh desde la PC"; exit 1; }
}
export H1_2_RETO=$MP
export PYTHONNOUSERSITE=1
unset CYCLONEDDS_URI RMW_IMPLEMENTATION
export MUJOCO_GL=${MUJOCO_GL:-disable}
