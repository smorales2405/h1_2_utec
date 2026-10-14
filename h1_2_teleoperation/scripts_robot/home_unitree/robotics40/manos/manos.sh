#!/usr/bin/env bash
# Manos Inspire RH56DFTP del H1-2 (Modbus TCP: izq 192.168.124.211, der .210). Detalle: README.md
#
#   ./manos.sh leer [izq|der|ambas]            estado de cada dedo (SOLO LECTURA)
#   ./manos.sh puno  [izq|der|ambas]            cerrar en puno
#   ./manos.sh abrir [izq|der|ambas]            abrir dedos y pulgar
#   ./manos.sh borrar-error [izq|der|ambas]     borrar un dedo en fallo (registro 1004)
#   ./manos.sh prueba                           las dos: abrir/cerrar 15 s con registro (datos_manos/)
#   ./manos.sh prueba-pulgar                    igual, con la oposicion del pulgar
#   ./manos.sh preforma|cerrar|soltar izq|der   agarre de caja parando cada dedo al tocar (coger_caja.py)
#   ./manos.sh pulgar-tactil izq|der            cierra solo el pulgar hasta que su tactil toca
#   ./manos.sh test <read|wiggle|sweep|open> izq|der [--dof 3 ...]   hand_test.py del repo UTEC
#   ./manos.sh sonda                            probe_hands.py de las dos (SOLO LECTURA)
#   ./manos.sh driver [--no-touch]              puente Modbus<->DDS (el que usa el teleop XR), primer plano
source "$(dirname "$0")/../comun.sh"
M=$R40/manos
ORDEN=${1:-}; shift || true
case "$ORDEN" in
    leer|sonda) ;;
    ""|-h|--help|ayuda) sed -n '2,16p' "$0"; exit 0 ;;
    *) aviso_mueve "los dedos de las manos" ;;
esac
case "$ORDEN" in
    leer)                         exec "$PY_TELEOP" "$M/mano.py" leer "${1:-ambas}" ;;
    puno|abrir|borrar-error)      exec "$PY_TELEOP" "$M/mano.py" "$ORDEN" "${1:-ambas}" ;;
    prueba)                       cd "$R40/datos_manos" && exec "$PY_TELEOP" "$M/prueba_manos.py" ;;
    prueba-pulgar)                cd "$R40/datos_manos" && exec "$PY_TELEOP" "$M/prueba_manos_pulgar.py" ;;
    preforma|cerrar|soltar)       exec "$PY_TELEOP" "$M/coger_caja.py" "$ORDEN" "$(mano_ip "${1:-}")" ;;
    pulgar-tactil)                exec "$PY_TELEOP" "$M/pulgar_tactil.py" "$(mano_ip "${1:-}")" ;;
    test)                         CMD=${1:?falta read|wiggle|sweep|open}; IPM=$(mano_ip "${2:-}") || exit 1; shift 2
                                  exec "$PY_TELEOP" "$M/h1_2_utec/hand_test.py" "$CMD" "$IPM" "$@" ;;
    sonda)                        exec "$PY_TELEOP" "$M/h1_2_utec/probe_hands.py" "$MANO_IZQ" "$MANO_DER" ;;
    driver)                       exec "$PY_TELEOP" "$M/h1_2_utec/inspire_ftp_dual_driver.py" --network "$IFACE" "${@:---no-touch}" ;;
    *) echo "orden desconocida: $ORDEN"; sed -n '2,16p' "$0"; exit 1 ;;
esac
