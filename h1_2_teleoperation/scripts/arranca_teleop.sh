#!/usr/bin/env bash
# Prepara los brazos y arranca la teleoperación, en ese orden.
#
#   ./scripts/arranca_teleop.sh              brazos + manos Inspire
#   ./scripts/arranca_teleop.sh --sin-manos  solo brazos (primera prueba)
#   RAMPA=8 ./scripts/arranca_teleop.sh      arranque inicial más lento
#
# Hace dos cosas que `xr_teleoperate` no hace por si mismo:
#
#   1. Pone `H12_STARTUP_RAMP_S`: el movimiento inicial a 0° pasa de ser un
#      salto a una rampa de coseno alzado de N segundos, con la autoridad de
#      par intacta. Ese movimiento lo hace el propio controlador al
#      construirse, 91 líneas antes de pedirte que pulses [r].
#
#      NO uses `H12_ARM_VELOCITY_LIMIT` para esto, aunque lo parezca:
#      `clip_arm_q_target` recorta contra la posición MEDIDA, así que bajarlo
#      limita el error de posición y con él el par del PD. Medido en el codo:
#      con 0.6 rad/s no se mueve en absoluto, con 2.0 recorre 1.2° en 3 s.
#      Bajarlo no frena el brazo, lo desactiva.
#
#   2. Al terminar —salga como salga, incluido Ctrl-C— comprueba que los
#      brazos quedaron en la postura de reposo. Con el parche aplicado,
#      `ctrl_dual_arm_go_home()` ya lo hace por dentro y esto no encuentra
#      nada que hacer; es la red por si la teleoperación muere sin llegar a
#      llamarlo.
#
# Requiere modo debug:  cd h1_2_joint_control && source scripts/env.sh
#                       python3 scripts/06_debug_mode.py enter

set -euo pipefail

AQUI="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
RAIZ="$(cd -- "$AQUI/../.." &> /dev/null && pwd)"
JC="$RAIZ/h1_2_joint_control"
XR="$RAIZ/h1_2_teleoperation/xr_teleoperate"

RAMPA="${RAMPA:-4.0}"                 # segundos del movimiento inicial a 0°

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

# Direcciones por las que el visor puede alcanzar a esta laptop. NO se elige
# una sola automáticamente: esta máquina tiene a la vez el cable del robot, el
# del router y el WiFi del campus, y la buena es la del segmento donde esté el
# visor —que no se puede adivinar sin escanear—. Se listan todas y se marca
# cuáles cubre el certificado. `HOST_IP=...` fuerza una.
CERT="$HOME/.config/xr_teleoperate/cert.pem"
candidatos=$(ip -br -4 addr | awk '$1!="lo" && $3 !~ /^192\.168\.123\./ {split($3,a,"/"); print $1" "a[1]}')
# La NIC del robot, no la de salida a Internet: por defecto xr_teleoperate usa
# la interfaz por defecto, que aquí es el WiFi, y DDS no llegaría al robot.
NIC="${H12_NIC:-$(ip -br -4 addr | awk '$3 ~ /^192\.168\.123\./ {print $1; exit}')}"
if [ -z "$NIC" ]; then
    echo "  ⚠ no encuentro ninguna interfaz en 192.168.123.0/24."
    echo "    ¿Está el cable del robot conectado? Fuerza una con H12_NIC=..."
    exit 1
fi

echo
echo "══ 1/2 · arrancando la teleoperación ══"
echo "  rampa del movimiento inicial a 0°: ${RAMPA} s"
echo "  ganancias tuned_gff + gravedad, desde $JC"
echo "  DDS por ${NIC}"
if [ -n "$EE" ]; then echo "  manos Inspire activas"; else echo "  SIN manos (solo brazos)"; fi
echo
if [ -n "${HOST_IP:-}" ]; then
    echo "  En el Quest:  https://${HOST_IP}:8012"
else
    echo "  En el Quest, abre UNA de estas (la del segmento donde esté el visor;"
    echo "  si no sabes cuál, ejecuta ./scripts/busca_quest.sh):"
    while read -r _if _ip; do
        if openssl x509 -in "$CERT" -noout -ext subjectAltName 2>/dev/null \
           | grep -q "IP Address:${_ip}\b"; then marca="✔ certificado"
        else marca="⚠ el certificado NO la cubre: ./scripts/02_gen_certs.sh"; fi
        printf "      https://%-16s:8012   (%s)  %s\n" "$_ip" "$_if" "$marca"
    done <<< "$candidatos"
fi
echo "  Pulsa [r] en esta terminal cuando quieras que empiece a seguirte."
echo "  Paro de emergencia del mando: L2 + B."
echo

reposo() {
    echo
    echo "══ 2/2 · devolviendo los brazos a la postura de reposo ══"
    (
        cd "$JC"
        source scripts/env.sh >/dev/null
        python3 scripts/16_postura_reposo.py --yes
    ) || echo "  ⚠ el retorno falló; comprueba los brazos ANTES de soltar el arnés."
}
trap reposo EXIT

source "$AQUI/_conda.sh"
export H12_STARTUP_RAMP_S="$RAMPA"
export H12_JOINT_CONTROL="$JC"
cd "$XR/teleop"
# sin `exec`: el trap tiene que poder correr después
"$CONDA_ENV_TV/bin/python" teleop_hand_and_arm.py \
    --arm H1_2 --network-interface "$NIC" \
    ${EE:+--ee "$EE"} ${TELEOP_ARGS[@]+"${TELEOP_ARGS[@]}"} || true
