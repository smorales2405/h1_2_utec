#!/usr/bin/env bash
# Seguidor de linea del H1-2. MUEVE EL ROBOT salvo con --simulacro.
#   ./seguidor_linea.sh --nivel 1 --simulacro     el lazo entero sin Move (Hito 3)
#   ./seguidor_linea.sh --nivel 1                 a media escala (primeras tiradas de cada nivel)
#   ./seguidor_linea.sh --nivel 1 --escala 1
# ESPACIO durante la tirada: parada (velocidad 0, el robot se queda de pie). L2+B sigue siendo la de emergencia.
# Antes, en otra terminal: ./camara_servidor.sh. Desde la PC se reenvia solo al robot con ssh -t
# (hace falta para escribir SEGUIR). Nunca con sudo: el DDS no arranca como root.
source "$(dirname "$0")/comun.sh"
[ $# -gt 0 ] || { sed -n '2,8p' "$0"; exit 1; }
[ "$(id -u)" -ne 0 ] || { echo "No lo lances como root: el DDS no arranca con sudo."; exit 1; }
cd "$SL" && exec "$PY" seguidor_linea.py "$@"
