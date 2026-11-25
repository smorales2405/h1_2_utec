#!/usr/bin/env bash
# El banco de agarre (agarre/banco.py), en el robot, como usuario unitree (nunca con sudo):
#   ./banco.sh --banco cubo --ensayos 20 --arnes                 # regulador y firmware intercalados, 10 y 10
#   ./banco.sh --banco esfera --ensayos 20 --arnes --sostener-s 5
# El objeto, solo y FUERA de la cruz al lanzarlo: el robot mira la cruz y pide ponerlo encima, y antes de cada ensayo
# recolocarlo si no esta en su sitio (Enter; q: acabar; ESPACIO: salida segura; --sin-recolocar: no). Desde la PC se
# reenvia solo al robot (comun.sh, ssh -t) y graba la camara externa si esta conectada.
source "$(dirname "$0")/comun.sh"
[ "$(id -u)" -ne 0 ] || { echo "No lo lances como root: el DDS no arranca con sudo."; exit 1; }
exigir_entorno
[[ " $* " == *" --banco "* ]] || { sed -n '2,7p' "$0"; exit 1; }
cd "$MR" && exec "$PY" -u manipulacion_regulada.py "$@"
