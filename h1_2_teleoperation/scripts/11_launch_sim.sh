#!/usr/bin/env bash
# Lanza el simulador unitree_sim_isaaclab con el H1-2 (27 DoF) y manos Inspire.
#
# Es el "Terminal 1" del despliegue en simulacion. Cuando en la consola aparezca
#
#     controller started, start main loop...
#
# hay que PULSAR UNA VEZ dentro de la ventana de Isaac Sim para activarla, y
# entonces ya se puede lanzar la teleoperacion (scripts/12_launch_teleop_sim.sh).
#
# La primera arrancada tarda: Isaac Sim compila shaders y carga los USD.
# Si la vista sale rara: PerspectiveCamera -> Cameras -> PerspectiveCamera.
#
# OJO: el simulador publica los MISMOS topicos DDS que el robot real, pero en el
# dominio 1 (el robot fisico usa el 0). Aun asi, no lo lances con el H1-2 real
# encendido en la misma red sin saber lo que haces.
#
# Variables sobreescribibles:  TASK, HAND_DDS, DEVICE, ENV_NAME, EXTRA
set -euo pipefail

# ROOT = la carpeta h1_2_teleoperation (la que contiene scripts/). Se deduce de la
# ubicacion de este script, asi que el repo se puede clonar donde sea.
ROOT="${ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ENV_NAME="${ENV_NAME:-unitree_sim_env}"
CONDA_BASE="${CONDA_BASE:-/home/utec/miniconda3}"

# Tareas disponibles para el H1-2 con manos Inspire:
#   Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint
#   Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint
#   Isaac-Stack-RgyBlock-H12-27dof-Inspire-Joint
TASK="${TASK:-Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint}"

# Protocolo DDS de las manos:
#   --enable_inspire_ftp_dds  RH56DFTP, rt/inspire_hand/{ctrl,state}/{l,r}
#                             -> teleoperar con --ee inspire_ftp (igual que el
#                                robot real de UTEC). Lo añade este repo.
#   --enable_inspire_dds      DFX, rt/inspire/{cmd,state}
#                             -> teleoperar con --ee inspire_dfx (upstream)
HAND_DDS="${HAND_DDS:---enable_inspire_ftp_dds}"

# El README de Unitree dice `--robot_type h12`, pero sim_main.py solo reconoce
# `h1_2`; con h12 no se crea ningun objeto DDS y el robot no se mueve.
ROBOT_TYPE=h1_2

DEVICE="${DEVICE:-cpu}"     # los ejemplos de Unitree usan la fisica en CPU
EXTRA="${EXTRA:-}"          # p.ej. --no_render  --replay --file_path ...

source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$ENV_NAME"

cd "$ROOT/unitree_sim_isaaclab"

[ -d assets ] || { echo "ERROR: faltan los assets USD. Ejecuta: cd $PWD && bash fetch_assets.sh"; exit 1; }

echo "──────────────────────────────────────────────────────────────"
echo " Simulador      : unitree_sim_isaaclab (Isaac Sim + Isaac Lab)"
echo " Robot          : H1-2 27 DoF                 --robot_type $ROBOT_TYPE"
echo " Tarea          : $TASK"
echo " Manos          : $HAND_DDS"
echo " Dispositivo    : $DEVICE     Dominio DDS: 1"
echo " Extra          : ${EXTRA:-(ninguno)}"
echo "──────────────────────────────────────────────────────────────"
echo " Cuando salga 'controller started, start main loop...' haz UN CLIC"
echo " dentro de la ventana de Isaac Sim y lanza 12_launch_teleop_sim.sh"
echo "──────────────────────────────────────────────────────────────"

exec python sim_main.py \
    --device "$DEVICE" \
    --enable_cameras \
    --task "$TASK" \
    --robot_type "$ROBOT_TYPE" \
    $HAND_DDS \
    $EXTRA
