#!/usr/bin/env bash
# Selector de rutinas BRAZOS + MANOS por rt/lowcmd y Modbus TCP en el robot REAL.  *** SIN PROBAR EN EL ROBOT ***
#   ./selector_manos.sh                       eth0 y las rutinas de poses/
#   ./selector_manos.sh eth0 /otra/carpeta
#   ./selector_manos.sh --sin-manos           solo brazos (la primera prueba)
# Se ejecuta EN EL PC2 del robot. Requisitos: robot COLGADO con los pies en el aire, Debug (L2+R2) y alguien
# con el mando. Si el servicio de movimiento sigue activo (CheckMode 'ai'), el programa ofrece ReleaseMode()
# tras teclear SOLTAR: las piernas se sueltan. Detalle: README.md
AQUI="$(cd "$(dirname "$0")" && pwd)"
PY=${PY:-/home/unitree/teleop_venv/bin/python}     # tiene unitree_sdk2py y pymodbus
export PYTHONNOUSERSITE=1                          # ~/.local se cuela en cualquier python 3.10 del PC2
unset CYCLONEDDS_URI RMW_IMPLEMENTATION            # el del .bashrc (SLAM de fábrica) rompe el DDS del SDK
[ -x "$PY" ] || { echo "No encuentro $PY (PY=/ruta/python ./selector_manos.sh)"; exit 1; }

# Nadie más mandando a los brazos ni a las manos.
if systemctl is-active --quiet h1_2_six_seven 2>/dev/null; then
    echo "El servicio h1_2_six_seven está activo: publica en rt/arm_sdk al pulsar su combinación del mando."
    echo "Páralo antes:  sudo systemctl stop h1_2_six_seven"
    exit 1
fi
case " $* " in
    *" --sin-manos "*) ;;
    *) if pgrep -f inspire_ftp_dual_driver >/dev/null; then
           echo "Hay un driver de manos en marcha (inspire_ftp_dual_driver: teleop XR o manos.sh driver). Páralo antes."
           exit 1
       fi ;;
esac

echo "=================================================================="
echo "  ESTO MUEVE EL ROBOT: las 27 juntas por bajo nivel y las dos manos"
echo "  (Debug: las piernas NO se sostienen solas)"
echo "  L2 + B en el mando es la parada de emergencia. Tenlo en la mano."
echo "=================================================================="
echo "  Sin probar en el robot: primera vez, con el responsable del laboratorio delante."
read -r -p "  Escribe COLGADO si el robot esta colgado y en Debug: " ok; [ "$ok" = COLGADO ] || exit 1
exec "$PY" "$AQUI/h1_2_robot_selector_manos.py" "$@"
