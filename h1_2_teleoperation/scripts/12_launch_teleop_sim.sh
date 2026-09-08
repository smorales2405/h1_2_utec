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
# Variables sobreescribibles:  XR_IP, EE, INPUT_MODE, DISPLAY_MODE, IMG_SERVER_IP,
#                              NET_IF, EXTRA
# Resolucion de conda: busca la instalacion en vez de cablearla.
source "$(dirname "${BASH_SOURCE[0]}")/_conda.sh"
set -euo pipefail

# ROOT = la carpeta h1_2_teleoperation (la que contiene scripts/). Se deduce de la
# ubicacion de este script, asi que el repo se puede clonar donde sea.
ROOT="${ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
CONDA_BASE="${CONDA_BASE}"

EE="${EE:-inspire_ftp}"                              # inspire_ftp | inspire_dfx

# El simulador corre en esta maquina, pero 127.0.0.1 NO sirve: la configuracion
# de camaras del simulador trae enable_webrtc, y el plano de video WebRTC lo
# pide el NAVEGADOR DEL QUEST (televuer le pasa la URL tal cual como `src`).
# Con 127.0.0.1 el visor la pediria a su propio localhost y no llegaria imagen.
# Hay que darle la IP de esta maquina EN LA RED DEL VISOR; sirve igual para el
# ZMQ del ImageClient, que es local.
#
# XR_IP = IP del visor. Con varias interfaces activas (p.ej. WiFi del campus +
# ethernet al router del laboratorio) la ruta por defecto NO tiene por que ser
# la del visor, asi que si se da XR_IP se deduce de la ruta hacia el, que es
# exacto. Si no, se cae a la ruta por defecto.
XR_IP="${XR_IP:-}"
if [ -z "${IMG_SERVER_IP:-}" ] && [ -n "$XR_IP" ]; then
    IMG_SERVER_IP=$(ip -4 route get "$XR_IP" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')
fi
IMG_SERVER_IP="${IMG_SERVER_IP:-$(ip -4 route get 1.1.1.1 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')}"
IMG_SERVER_IP="${IMG_SERVER_IP:-127.0.0.1}"

# Aviso si hay mas de una IP global y no se ha dicho cual es la del visor: es
# justo el caso en el que la autodeteccion se equivoca en silencio y el visor
# se queda sin video.
if [ -z "$XR_IP" ] && [ "$(ip -4 -o addr show scope global | wc -l)" -gt 1 ]; then
    echo "AVISO: esta maquina tiene varias IP; se usara $IMG_SERVER_IP para el servidor de imagen."
    echo "       Si el visor no esta en esa red, lanza con:  XR_IP=<ip-del-visor> $0"
fi
INPUT_MODE="${INPUT_MODE:-hand}"                     # hand | controller
DISPLAY_MODE="${DISPLAY_MODE:-immersive}"            # immersive | ego | pass-through
NET_IF="${NET_IF:-}"                                 # vacio = interfaz por defecto de CycloneDDS
EXTRA="${EXTRA:-}"                                   # p.ej. --record

# Fotogramas por segundo hacia el visor. Solo pinta en el camino ZMQ, donde cada
# fotograma es un JPEG entero por el websocket: 640x480 q80 son ~40 kB, o sea
# ~9,7 Mbps a 30 fps. Sobre WiFi 5 eso satura y la sesion se atasca, asi que por
# defecto se baja a 15 (~4,9 Mbps). Con buen enlace, DISPLAY_FPS=30.
DISPLAY_FPS="${DISPLAY_FPS:-15}"
export XR_DISPLAY_FPS="$DISPLAY_FPS"

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
echo " Entrada XR     : $INPUT_MODE     Display: $DISPLAY_MODE  ($DISPLAY_FPS fps al visor)"
echo " Extra          : ${EXTRA:-(ninguno)}"
echo "──────────────────────────────────────────────────────────────"
# OJO con el ?ws=... : el cliente de Vuer deduce mal la URI del websocket cuando
# la pagina va por HTTPS. En su bundle:
#     window.location.protocol == "https:" ? `wss://${window.location.hostname}` : ...
# o sea que SE COME EL PUERTO y acaba intentando wss://<host>:443, donde no hay
# nada. El websocket no conecta, y como toda la escena (imagen incluida) viaja
# por ahi, el visor solo ve la rejilla por defecto y parece congelado.
# Por HTTP no pasa, porque esa rama si concatena el puerto.
# El cliente acepta la URI explicita en el parametro `ws`: getSocketURI(query.ws).
echo " En el Quest 3 abre (la URL ENTERA, el ?ws= es imprescindible):"
ip -4 -o addr show scope global | awk '{split($4,a,"/"); print a[1]}' | while read -r _ip; do
    _url="https://$_ip:8012/?ws=wss://$_ip:8012"
    if [ "$_ip" = "$IMG_SERVER_IP" ]; then
        echo "     $_url   <-- esta, la de la red del visor"
    else
        echo "     $_url"
    fi
done
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
