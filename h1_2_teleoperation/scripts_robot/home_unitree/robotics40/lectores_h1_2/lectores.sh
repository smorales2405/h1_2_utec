#!/usr/bin/env bash
# Herramientas de lectura del H1-2. La interfaz es eth0 (IFACE=... para cambiarla). Detalle: README.md
#
#  SOLO LECTURA (no publican nada, no pueden mover el robot):
#   ./lectores.sh lowstate [s]      rt/lowstate + bateria: Hz, modos, temperaturas, fallos (10 s)
#   ./lectores.sh postura           las 27 juntas (q, tau, temp) y si alguien publica en rt/lowcmd
#   ./lectores.sh fsm               GetFsmId / GetFsmMode / GetBalanceMode
#   ./lectores.sh modo              CheckMode: 'ai' = servicio de Unitree activo, '' = Debug de verdad
#   ./lectores.sh mando             wireless_remote[40] y el kp/kd del control interno
#   ./lectores.sh mando-vivo [s]    botones del mando en vivo (10 s)
#   ./lectores.sh espia-armsdk [s]  consigna de rt/arm_sdk contra la medida (10 s)
#   ./lectores.sh log [s]           CSV de tobillos, munecas y rodilla a 250 Hz (10 s)
#   ./lectores.sh d435 [--frames N] [--guardar DIR]   info e imagenes sueltas de la D435i (sudo)
#   ./lectores.sh stream-d435 [--puerto 8080]         los 4 flujos de la D435i en http://<ip>:8080 (sudo)
#
#  MUEVEN EL ROBOT:
#   ./lectores.sh atras [vx] [s]    marcha atras (vx -0.3, 10 s) por LocoClient, FSM 201
source "$(dirname "$0")/../comun.sh"
L=$R40/lectores_h1_2; B=$L/build
ORDEN=${1:-}; shift || true
case "$ORDEN" in
    lowstate)     exec "$B/lector_lowstate" "$IFACE" "${1:-10}" ;;
    postura)      exec "$B/lector_postura" "$IFACE" ;;
    fsm)          exec "$B/lee_fsm" "$IFACE" ;;
    modo)         exec "$PY_SDK" "$L/check_mode.py" "$IFACE" ;;
    mando)        exec "$B/lee_mando" "$IFACE" ;;
    mando-vivo)   exec "$B/mando_vivo" "$IFACE" "${1:-10}" ;;
    espia-armsdk) exec "$B/espia_armsdk" "$IFACE" "${1:-10}" ;;
    log)          exec "$B/log_bajo_nivel" "$IFACE" "${1:-10}" ;;
    d435)         exec sudo PYTHONNOUSERSITE=1 "$PY_TELEOP" "$L/d435_info.py" "$@" ;;
    stream-d435)  exec sudo PYTHONNOUSERSITE=1 "$PY_TELEOP" "$L/stream_d435.py" "$@" ;;
    atras)        aviso_mueve "camina hacia atras ${2:-10} s"
                  read -r -p "  Escribe ATRAS para empezar: " ok; [ "$ok" = ATRAS ] || exit 1
                  exec "$PY_SDK" "$L/atras.py" "$IFACE" "${1:--0.3}" "${2:-10}" ;;
    *) sed -n '2,20p' "$0"; [ -z "$ORDEN" ] && exit 0 || exit 1 ;;
esac
