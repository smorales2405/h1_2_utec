#!/usr/bin/env bash
# Copia h1_2_manipulation al PC2 (~/utec/h1_2_manipulation). Usa el reto de ~/utec/manipulacion (su codigo, su config,
# su calibracion y su .venv), asi que antes despliega el reto con su propio desplegar.sh: el brazo "firmware" del banco
# es el reto de esta PC tal cual.
#   ./desplegar.sh            el reto y esto
#   ./desplegar.sh --traer    datos/ del robot -> datos/ de la PC
# Por cable: ROBOT_SSH=unitree@192.168.123.164 ./desplegar.sh
set -e
ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
export ROBOT_SSH
AQUI="$(cd "$(dirname "$0")" && pwd)"
RETO="$AQUI/../retos_capacitacion/manipulacion"
DEST=utec/h1_2_manipulation

if [ "${1:-}" = "--traer" ]; then
    mkdir -p "$AQUI/datos"
    rsync -av "$ROBOT_SSH:$DEST/datos/" "$AQUI/datos/"
    exit 0
fi
"$RETO/desplegar.sh"
rsync -av --exclude datos/ --exclude __pycache__ --exclude .pytest_cache/ --exclude tests/ \
      "$AQUI/" "$ROBOT_SSH:$DEST/"
