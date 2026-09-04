#!/usr/bin/env bash
# Instalación de dependencias del Host (laptop) para xr_teleoperate — entorno conda "tv"
set -x
ROOT=/home/utec/Documents/h1_2_teleoperation
PIP=/home/utec/miniconda3/envs/tv/bin/pip
PY=/home/utec/miniconda3/envs/tv/bin/python

set -e
# 1) teleimager (cliente de imagen). --no-deps segun README para no romper numpy<2
cd "$ROOT/xr_teleoperate/teleop/teleimager" && $PIP install -e . --no-deps
$PIP install "logging_mp" "opencv-python" "numpy>=1.21,<2" "pyyaml" "pyzmq"

# 2) televuer (servidor Vuer / WebXR)
cd "$ROOT/xr_teleoperate/teleop/televuer" && $PIP install -e .

# 3) dex-retargeting. --no-deps a proposito: su dependencia "pin" (pinocchio de PyPI)
#    machacaria el pinocchio 3.1.0 instalado por conda. Instalamos el resto a mano.
cd "$ROOT/xr_teleoperate/teleop/robot_control/dex-retargeting" && $PIP install -e . --no-deps
$PIP install "torch==2.3.0" --index-url https://download.pytorch.org/whl/cpu
$PIP install "pytransform3d>=3.5.0" "nlopt>=2.6.1,<2.8.0" "trimesh>=4.4.0" "anytree>=2.12.0" "pyyaml>=6.0.0" "lxml>=5.2.2"

# 4) requirements del repo principal
cd "$ROOT/xr_teleoperate" && $PIP install -r requirements.txt

echo "=== INSTALL HOST OK ==="

# 5) FIX: vuer 0.0.60 usa params_proto.Flag, eliminado en params-proto 3.x.
#    pip resuelve a 3.3.0 y "from vuer import Vuer" falla silenciosamente.
$PIP install "params-proto==2.13.2"
