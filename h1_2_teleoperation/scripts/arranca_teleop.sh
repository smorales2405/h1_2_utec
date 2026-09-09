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
# OJO con el ?ws= : el cliente de Vuer 0.0.60 se come el puerto del websocket
# cuando la pagina va por HTTPS. En su bundle:
#     window.location.protocol == "https:" ? `wss://${window.location.hostname}` : ...
# o sea que acaba intentando wss://<host>:443, donde no hay nada. El websocket
# no conecta, y como toda la escena viaja por ahi, el visor carga la pagina, el
# seguimiento de manos funciona, y no llega ninguna imagen: parece congelado.
# WebXR obliga a HTTPS, asi que el fallo se dispara SIEMPRE con el visor.
# El cliente acepta la URI explicita en `ws`: getSocketURI(query.ws).
# Detalle en README_SIM.md 3.7.
echo "  En el Quest, abre la URL ENTERA. El ?ws= NO es opcional: sin él la"
echo "  imagen no llega nunca y parece que se ha congelado."
echo
if [ -n "${HOST_IP:-}" ]; then
    echo "      https://${HOST_IP}:8012/?ws=wss://${HOST_IP}:8012"
else
    echo "  (si no sabes cuál es la del visor: ./scripts/busca_quest.sh)"
    while read -r _if _ip; do
        if openssl x509 -in "$CERT" -noout -ext subjectAltName 2>/dev/null \
           | grep -q "IP Address:${_ip}\b"; then marca="✔"
        else marca="⚠ sin certificado: ./scripts/02_gen_certs.sh"; fi
        echo "      https://${_ip}:8012/?ws=wss://${_ip}:8012   ($_if) $marca"
    done <<< "$candidatos"
fi
echo
echo "  Comprobación: en el panel derecho de la página, el campo Socket URI"
echo "  tiene que poner wss://<ip>:8012. Si pone wss://<ip> sin puerto, es esto."
echo "  Pulsa [r] en esta terminal cuando quieras que empiece a seguirte."
echo "  Paro de emergencia del mando: L2 + B."
echo

reposo() {
    echo
    echo "══ 2/2 · devolviendo los brazos a la postura de reposo ══"
    (
        # `set -u` fuera: los setup.bash de ROS leen variables sin definir
        # (AMENT_TRACE_SETUP_FILES) y abortarian el retorno justo cuando mas
        # falta hace. Paso medido: sin esto el trap fallaba entero.
        set +u
        cd "$JC"
        source scripts/env.sh >/dev/null
        python3 scripts/16_postura_reposo.py --yes
    ) || echo "  ⚠ el retorno falló; comprueba los brazos ANTES de soltar el arnés."
}
trap reposo EXIT

# El entorno de ROS y el de conda no se llevan: si en esta terminal se ha
# hecho `source scripts/env.sh` -que es lo normal, hace falta para el modo
# debug-, PYTHONPATH apunta a /opt/ros/humble/lib/python3.10/site-packages y
# su `pinocchio` TAPA al de conda. El de ROS no trae el modulo `casadi`, asi
# que robot_arm_ik.py revienta en el import y la teleoperacion ni arranca.
#
# No hace falta PYTHONPATH: el parche encuentra h1_2_joint_control por
# H12_JOINT_CONTROL y se lo inserta el solo en sys.path.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    LD_LIBRARY_PATH="$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' \
                       | grep -v '/opt/ros/' | paste -sd: - || true)"
    export LD_LIBRARY_PATH
fi

source "$AQUI/_conda.sh"
export H12_STARTUP_RAMP_S="$RAMPA"
export H12_JOINT_CONTROL="$JC"
cd "$XR/teleop"
# sin `exec`: el trap tiene que poder correr después
"$CONDA_ENV_TV/bin/python" teleop_hand_and_arm.py \
    --arm H1_2 --network-interface "$NIC" \
    ${EE:+--ee "$EE"} ${TELEOP_ARGS[@]+"${TELEOP_ARGS[@]}"} || true
