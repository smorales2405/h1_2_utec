#!/usr/bin/env bash
# El reto con la mano regulada (manipulacion_regulada.py), en el robot, como usuario unitree (nunca con sudo):
#   ./manipulacion_regulada.sh --nivel 2 --arnes
#   ./manipulacion_regulada.sh --nivel 2 --arnes --brazo firmware      # la mano del reto (como manipulacion.sh)
# Las opciones del reto (manipulacion.sh) pasan tal cual. Desde la PC se reenvia solo al robot (comun.sh, ssh -t).
source "$(dirname "$0")/comun.sh"
[ "$(id -u)" -ne 0 ] || { echo "No lo lances como root: el DDS no arranca con sudo."; exit 1; }
exigir_entorno
cd "$MR" && exec "$PY" -u manipulacion_regulada.py "$@"
