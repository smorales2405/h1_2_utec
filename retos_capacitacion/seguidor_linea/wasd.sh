#!/usr/bin/env bash
# Teleop WASD del H1-2 para llevar el robot mientras se graba un dataset. MUEVE EL ROBOT.
# Copia de ~/robotics40/wasd.sh con el entorno del seguidor; herramientas/wasd.py es identico al de
# ~/robotics40 (y al de code_cap). Robot de pie en FSM 201 (L2+UP en el mando), L2+B en la mano.
#   ./wasd.sh --vx 0.2 --vyaw 0.3        las velocidades de los datasets del nivel 1
# Teclas: w/s adelante/atras, a/d lateral, q/e girar, espacio para, +/- escala, x sale.
# Sin tecla durante 0.4 s el robot para solo. Desde la PC se reenvia al robot con ssh -t. Nunca con sudo.
source "$(dirname "$0")/comun.sh"
[ $# -gt 0 ] || { sed -n '2,7p' "$0"; exit 1; }
[ "$(id -u)" -ne 0 ] || { echo "No lo lances como root: el DDS no arranca con sudo."; exit 1; }
echo "=================================================================="
echo "  ESTO MUEVE EL ROBOT: camina con el teclado (w/s a/d q/e, espacio para, x sale)"
echo "  L2 + B en el mando es la parada de emergencia. Tenlo en la mano."
echo "=================================================================="
cd "$SL" && exec "$PY" herramientas/wasd.py --iface eth0 "$@"
