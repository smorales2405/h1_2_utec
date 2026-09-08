#!/usr/bin/env bash
# Lanza la teleoperacion XR sobre el H1-2 fisico con manos Inspire RH56DFTP.
#
# Requiere, en este orden:
#   1) PC2 del robot ejecutando teleimager           (servicio de imagen)
#   2) PC2 del robot ejecutando el driver Inspire FTP (Modbus <-> DDS)
#   3) Robot en modo debug / amortiguacion, segun corresponda
#
# Variables sobreescribibles:  IMG_SERVER_IP, NET_IF, INPUT_MODE, DISPLAY_MODE, EXTRA
# Resolucion de conda: busca la instalacion en vez de cablearla.
source "$(dirname "${BASH_SOURCE[0]}")/_conda.sh"
set -euo pipefail

# ROOT = la carpeta h1_2_teleoperation (la que contiene scripts/). Se deduce de la
# ubicacion de este script, asi que el repo se puede clonar donde sea.
ROOT="${ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
IMG_SERVER_IP="${IMG_SERVER_IP:-192.168.123.164}"   # PC2 del robot
NET_IF="${NET_IF:-enp0s31f6}"                        # NIC hacia el robot (DDS)
INPUT_MODE="${INPUT_MODE:-hand}"                     # hand | controller
DISPLAY_MODE="${DISPLAY_MODE:-immersive}"            # immersive | ego | pass-through
EXTRA="${EXTRA:-}"                                   # p.ej. --record  --motion  --headless

source ${CONDA_BASE}/etc/profile.d/conda.sh
conda activate tv

cd "$ROOT/xr_teleoperate/teleop"

echo "──────────────────────────────────────────────────────────────"
echo " Robot          : H1-2 (brazo 7 DoF)          --arm=H1_2"
echo " Efector final  : Inspire RH56DFTP            --ee=inspire_ftp"
echo " Servidor imagen: $IMG_SERVER_IP"
echo " Interfaz DDS   : $NET_IF"
echo " Entrada XR     : $INPUT_MODE     Display: $DISPLAY_MODE"
echo " Extra          : ${EXTRA:-(ninguno)}"
echo "──────────────────────────────────────────────────────────────"
echo " En el Quest 3 abre:"
ip -4 -o addr show scope global | awk '{split($4,a,"/"); print "     https://" a[1] ":8012"}'
echo " ADVERTENCIA: manten distancia de seguridad del robot."
echo "──────────────────────────────────────────────────────────────"

exec python teleop_hand_and_arm.py \
    --arm=H1_2 \
    --ee=inspire_ftp \
    --input-mode="$INPUT_MODE" \
    --display-mode="$DISPLAY_MODE" \
    --img-server-ip="$IMG_SERVER_IP" \
    --network-interface="$NET_IF" \
    $EXTRA
