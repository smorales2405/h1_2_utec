#!/usr/bin/env bash
# Ejecuta un programa del seguidor como usuario unitree, con el python y el entorno correctos.
#   ./ejecutar.sh herramientas/comprobar.py
#   ./ejecutar.sh herramientas/calibrar_camara.py --capturas 30
#   ./ejecutar.sh -m unittest discover -s tests
# Desde la PC se reenvia solo al robot (comun.sh). Nunca con sudo: el DDS no arranca como root.
source "$(dirname "$0")/comun.sh"
[ $# -gt 0 ] || { sed -n '2,6p' "$0"; exit 1; }
[ "$(id -u)" -ne 0 ] || { echo "No lo lances como root: el DDS no arranca con sudo."; exit 1; }
cd "$SL" && exec "$PY" "$@"
