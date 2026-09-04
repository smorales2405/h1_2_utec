#!/usr/bin/env bash
# Lanza la teleoperacion XR contra el SIMULADOR (no contra el H1-2 fisico).
#
# Es el "Terminal 2": antes tiene que estar corriendo 11_launch_sim.sh y haber
# aparecido `controller started, start main loop...` (y haber hecho un clic en
# la ventana de Isaac Sim).
#
# Diferencias con 03_launch_teleop.sh (robot real):
#   --sim                     dominio DDS 1 en vez de 0, y sin limite de
#                             velocidad articular en el brazo
#   --img-server-ip <IP LAN>  el servidor de imagen lo levanta el propio
#                             simulador, en esta misma maquina. OJO: NO vale
#                             127.0.0.1 — ver el comentario de IMG_SERVER_IP.
#   --ee inspire_ftp          requiere lanzar el simulador con
#                             --enable_inspire_ftp_dds (por defecto en
#                             11_launch_sim.sh). Con --enable_inspire_dds
#                             hay que poner EE=inspire_dfx aqui.
#
# Variables sobreescribibles:  EE, INPUT_MODE, DISPLAY_MODE, IMG_SERVER_IP, NET_IF, EXTRA
set -euo pipefail

# ROOT = la carpeta h1_2_teleoperation (la que contiene scripts/). Se deduce de la
# ubicacion de este script, asi que el repo se puede clonar donde sea.
ROOT="${ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
CONDA_BASE="${CONDA_BASE:-/home/utec/miniconda3}"

EE="${EE:-inspire_ftp}"                              # inspire_ftp | inspire_dfx

# El simulador corre en esta maquina, pero 127.0.0.1 NO sirve: la configuracion
# de camaras del simulador trae enable_webrtc, y el plano de video WebRTC lo
# pide el NAVEGADOR DEL QUEST (televuer le pasa la URL tal cual como `src`).
# Con 127.0.0.1 el visor la pediria a su propio localhost y no llegaria imagen.
# Hay que darle la IP de esta laptop en la red del visor; sirve igual para el
# ZMQ del ImageClient, que es local.
IMG_SERVER_IP="${IMG_SERVER_IP:-$(ip -4 route get 1.1.1.1 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')}"
IMG_SERVER_IP="${IMG_SERVER_IP:-127.0.0.1}"
INPUT_MODE="${INPUT_MODE:-hand}"                     # hand | controller
DISPLAY_MODE="${DISPLAY_MODE:-immersive}"            # immersive | ego | pass-through
NET_IF="${NET_IF:-}"                                 # vacio = interfaz por defecto de CycloneDDS
EXTRA="${EXTRA:-}"                                   # p.ej. --record

source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate tv

cd "$ROOT/xr_teleoperate/teleop"

# La interfaz DDS se deja sin fijar a proposito: el simulador hace
# ChannelFactoryInitialize(1) sin interfaz, y los dos procesos tienen que
# coincidir para descubrirse. Solo se pasa si NET_IF viene definida.
NET_ARG=()
[ -n "$NET_IF" ] && NET_ARG=(--network-interface="$NET_IF")

echo "──────────────────────────────────────────────────────────────"
echo " Modo           : SIMULACION (--sim, dominio DDS 1)"
echo " Robot          : H1-2 (brazo 7 DoF)          --arm=H1_2"
echo " Efector final  : $EE"
echo " Servidor imagen: $IMG_SERVER_IP  (lo levanta el propio simulador; WebRTC :60001)"
echo " Interfaz DDS   : ${NET_IF:-(por defecto)}"
echo " Entrada XR     : $INPUT_MODE     Display: $DISPLAY_MODE"
echo " Extra          : ${EXTRA:-(ninguno)}"
echo "──────────────────────────────────────────────────────────────"
echo " En el Quest 3 abre:  https://$(ip -4 -o addr show scope global | awk '{split($4,a,"/"); print a[1]}' | paste -sd'  o  ' -):8012"
echo " Teclas: r = empezar a seguir | s = grabar (con --record) | q = salir"
echo "──────────────────────────────────────────────────────────────"

exec python teleop_hand_and_arm.py \
    --arm=H1_2 \
    --ee="$EE" \
    --sim \
    --input-mode="$INPUT_MODE" \
    --display-mode="$DISPLAY_MODE" \
    --img-server-ip="$IMG_SERVER_IP" \
    "${NET_ARG[@]}" \
    $EXTRA
