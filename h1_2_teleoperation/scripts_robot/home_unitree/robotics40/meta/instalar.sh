#!/usr/bin/env bash
# Instalacion de xr_teleoperate en el PC2 del H1-2 (x86_64, Ubuntu 22.04), entorno conda env_tv.
# El entorno se creo con: conda create -p env_tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
set -e
# ~/.local tiene paquetes del usuario que se cuelan en cualquier python 3.10: aislarlo
export PYTHONNOUSERSITE=1
META=$HOME/robotics40/meta
PIP="$META/env_tv/bin/python -m pip"
XR=$META/xr_teleoperate
echo "### 1. requirements base"
$PIP install -r $XR/requirements.txt
echo "### 2. teleimager (+ servidor: realsense, zmq, aiortc, uvc)"
$PIP install -e $XR/teleop/teleimager --no-deps
$PIP install logging_mp opencv-python pyyaml pyzmq aiohttp aiortc pupil-labs-uvc pyrealsense2
echo "### 3. televuer (vuer[all]==0.0.60) + params-proto 2.x (la 3.x rompe vuer)"
$PIP install -e $XR/teleop/televuer
$PIP install "params-proto==2.13.2"
echo "### 4. dex-retargeting SIN deps (pediria pin de pip y pisaria el pinocchio de conda)"
$PIP install -e $XR/teleop/robot_control/dex-retargeting --no-deps
$PIP install "torch==2.3.0" --index-url https://download.pytorch.org/whl/cpu
$PIP install "pytransform3d>=3.5.0" "nlopt>=2.6.1,<2.8.0" "trimesh>=4.4.0" "anytree>=2.12.0" "lxml>=5.2.2"
echo "### 5. unitree_sdk2py e inspire_sdkpy (las fuentes del pc2_offline, las mismas del teleop_venv)"
$PIP install "cyclonedds==0.10.2"
$PIP install -e $HOME/pc2_offline/src/unitree_sdk2_python --no-deps
$PIP install -e $HOME/pc2_offline/src/inspire_hand_sdk --no-deps
$PIP install "pymodbus==3.6.9" pyserial "PyQt5==5.15.11" pyqtgraph colorcet requests   # inspire_sdkpy importa Qt al cargarse
echo "### 6. comprobacion"
$META/env_tv/bin/python - <<"PY"
import pinocchio, casadi, numpy, torch, vuer, televuer, teleimager, dex_retargeting, unitree_sdk2py, inspire_sdkpy, meshcat
print("OK pinocchio", pinocchio.__version__, "numpy", numpy.__version__, "torch", torch.__version__)
PY
echo "### LISTO"
