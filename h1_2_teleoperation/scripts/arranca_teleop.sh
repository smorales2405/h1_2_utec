#!/usr/bin/env bash
# Prepara los brazos y arranca la teleoperación, en ese orden.
#
#   ./scripts/arranca_teleop.sh              brazos + manos Inspire
#   ./scripts/arranca_teleop.sh --sin-manos  solo brazos (primera prueba)
#   ./scripts/arranca_teleop.sh --yes        sin pedir confirmación al colocar
#   VEL=0.4 ./scripts/arranca_teleop.sh      otro límite de velocidad
#
# Hace dos cosas que no se pueden hacer desde dentro de `xr_teleoperate`:
#
#   1. Baja `arm_velocity_limit`. ES LA DEFENSA PRINCIPAL: el primer
#      movimiento lo hace el propio controlador al construirse —91 líneas
#      antes de pedirte que pulses [r]— y de fábrica va a 30 rad/s.
#
#   2. Deja los catorce motores de los brazos en 0° DESPACIO, con toda la
#      protección de `h1_2_joint_control` puesta. Ojo: MEDIDO que los codos
#      vuelven solos a ~80° al soltar, porque 0° es flexionado y no es el
#      mínimo de gravedad. Así que esto vale para las otras doce y para
#      partir de una postura conocida, no para evitar el recorrido del codo.
#
# Requiere modo debug; `15_postura_cero.py` se niega a arrancar si no lo está
# y dice qué ejecutar.
set -euo pipefail

AQUI="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
RAIZ="$(cd -- "$AQUI/../.." &> /dev/null && pwd)"
JC="$RAIZ/h1_2_joint_control"
XR="$RAIZ/h1_2_teleoperation/xr_teleoperate"

VEL="${VEL:-0.6}"                     # rad/s del movimiento inicial de xr_teleoperate
VEL_POSTURA="${VEL_POSTURA:-0.15}"    # rad/s de la colocación previa, más lento

EE="inspire_ftp"
POSTURA_ARGS=()
TELEOP_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --sin-manos) EE="" ;;
        --yes)       POSTURA_ARGS+=("--yes") ;;
        *)           TELEOP_ARGS+=("$arg") ;;
    esac
done

echo "══ 1/2 · llevando los brazos a 0° a ${VEL_POSTURA} rad/s ══"
(
    cd "$JC"
    source scripts/env.sh >/dev/null
    python3 scripts/15_postura_cero.py --speed "$VEL_POSTURA" "${POSTURA_ARGS[@]}"
)

IP_WIFI="$(ip -br -4 addr | awk '$1!="lo" && $3 !~ /^192\.168\.123\./ {print $3; exit}' | cut -d/ -f1)"
# La NIC del robot, no la de salida a Internet: por defecto xr_teleoperate usa
# la interfaz por defecto, que aquí es el WiFi, y DDS no llegaría al robot.
NIC="${H12_NIC:-$(ip -br -4 addr | awk '$3 ~ /^192\.168\.123\./ {print $1; exit}')}"
if [ -z "$NIC" ]; then
    echo "  ⚠ no encuentro ninguna interfaz en 192.168.123.0/24."
    echo "    ¿Está el cable del robot conectado? Fuerza una con H12_NIC=..."
    exit 1
fi

echo
echo "══ 2/2 · arrancando la teleoperación ══"
echo "  arm_velocity_limit = ${VEL} rad/s   (de fábrica: 30)"
echo "  ganancias tuned_gff + gravedad, desde $JC"
echo "  DDS por ${NIC}"
if [ -n "$EE" ]; then echo "  manos Inspire activas"; else echo "  SIN manos (solo brazos)"; fi
echo
echo "  En el Quest:  https://${IP_WIFI:-<ip-wifi>}:8012"
echo "  Pulsa [r] en esta terminal cuando quieras que empiece a seguirte."
echo "  Paro de emergencia del mando: L2 + B."
echo

source "$AQUI/_conda.sh"
export H12_ARM_VELOCITY_LIMIT="$VEL"
export H12_JOINT_CONTROL="$JC"
cd "$XR/teleop"
exec "$CONDA_ENV_TV/bin/python" teleop_hand_and_arm.py \
     --arm H1_2 --network-interface "$NIC" \
     ${EE:+--ee "$EE"} ${TELEOP_ARGS[@]+"${TELEOP_ARGS[@]}"}
