#!/usr/bin/env bash
# SOLO LECTURA: estado del robot en unos segundos (no publica nada, no puede mover el robot).
#   ./estado.sh        lowstate + bateria durante 5 s, la FSM y el CheckMode (Debug o no)
#   ./estado.sh 15     lowstate durante 15 s
# Detalle: lectores_h1_2/README.md
source "$(dirname "$0")/comun.sh"
B=$R40/lectores_h1_2/build
"$B/lector_lowstate" "$IFACE" "${1:-5}"
echo; "$B/lee_fsm" "$IFACE"
echo; "$PY_SDK" "$R40/lectores_h1_2/check_mode.py" "$IFACE"
