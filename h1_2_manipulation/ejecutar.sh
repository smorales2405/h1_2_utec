#!/usr/bin/env bash
# Ejecuta un programa de h1_2_manipulation en el robot con el python y el entorno del reto:
#   ./ejecutar.sh analisis/comparar.py --ultima
#   ./ejecutar.sh -m pytest tests/
# Desde la PC se reenvia solo al robot (comun.sh). Nunca con sudo.
source "$(dirname "$0")/comun.sh"
[ $# -gt 0 ] || { sed -n '2,5p' "$0"; exit 1; }
[ "$(id -u)" -ne 0 ] || { echo "No lo lances como root: el DDS no arranca con sudo."; exit 1; }
exigir_entorno
cd "$MR" && exec "$PY" "$@"
