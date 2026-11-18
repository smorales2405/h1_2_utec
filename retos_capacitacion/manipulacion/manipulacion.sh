#!/usr/bin/env bash
# Programa principal de la manipulacion, en el robot, como usuario unitree (nunca con sudo).
#   ./manipulacion.sh --nivel 1 --simulacro
#   ./manipulacion.sh --nivel 2 --asignacion esfera=A cubo=C cilindro=B prisma=D
#   ./manipulacion.sh --nivel 2 --arnes --verificar-planes   (en la observacion, todos con plan antes de coger nada)
# Desde la PC se reenvia solo al robot (comun.sh, ssh -t: hace falta terminal para MOVER y ESPACIO).
source "$(dirname "$0")/comun.sh"
[ "$(id -u)" -ne 0 ] || { echo "No lo lances como root: el DDS no arranca con sudo."; exit 1; }
exigir_entorno
cd "$MP" && exec "$PY" -u manipulacion.py "$@"
