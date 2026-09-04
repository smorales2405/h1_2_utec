#!/usr/bin/env bash
# Instalación de dependencias del Host (laptop) para xr_teleoperate — entorno conda "tv"
set -x
# ROOT = la carpeta h1_2_teleoperation (la que contiene scripts/). Se deduce de la
# ubicacion de este script, asi que el repo se puede clonar donde sea.
ROOT="${ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# `env -u PYTHONPATH`: llamar a pip por su ruta absoluta NO dispara los hooks de
# conda, y con el PYTHONPATH de ROS puesto pip ve /opt/ros/humble/.../python3.10
# y da por "already satisfied" paquetes que no estan en el entorno.
PIP="env -u PYTHONPATH /home/utec/miniconda3/envs/tv/bin/pip"
PY="env -u PYTHONPATH /home/utec/miniconda3/envs/tv/bin/python"

set -e
# 0) SDK de comunicacion con el robot y SDK de las manos FTP. No estan en PyPI y
#    el README de xr_teleoperate no los menciona, pero sin ellos no hay ni DDS
#    (unitree_sdk2py) ni `--ee inspire_ftp` (inspire_sdkpy).
#
#    unitree_sdk2py fija cyclonedds==0.10.2, que no tiene rueda: pip la compila
#    y necesita CYCLONEDDS_HOME apuntando a una instalacion de CycloneDDS. La
#    construye 10_install_sim.sh; si no esta, se construye aqui.
CYCLONEDDS_SRC="$ROOT/cyclonedds"
if [ ! -f "$CYCLONEDDS_SRC/install/lib/libddsc.so" ]; then
    echo "== Compilando CycloneDDS (necesario para cyclonedds==0.10.2) =="
    [ -d "$CYCLONEDDS_SRC" ] || git clone https://github.com/eclipse-cyclonedds/cyclonedds \
        -b releases/0.10.x "$CYCLONEDDS_SRC"
    mkdir -p "$CYCLONEDDS_SRC/build" "$CYCLONEDDS_SRC/install"
    # -DENABLE_SSL=NO: sin esto CycloneDDS enlaza contra el OpenSSL del SISTEMA,
    # y en cuanto se importa unitree_sdk2py el proceso se queda con ese libcrypto;
    # el _ssl del entorno conda (compilado contra otro OpenSSL) ya no carga:
    #   ImportError: libcrypto.so.3: version `OPENSSL_3.3.0' not found
    # Rompe teleimager, que necesita TLS para su servidor WebRTC. Aqui el DDS de
    # Unitree va en UDP plano, asi que TLS en DDS no hace ninguna falta.
    cmake -S "$CYCLONEDDS_SRC" -B "$CYCLONEDDS_SRC/build" \
        -DCMAKE_INSTALL_PREFIX="$CYCLONEDDS_SRC/install" -DBUILD_IDLC=ON -DBUILD_TESTING=OFF \
        -DENABLE_SSL=NO
    cmake --build "$CYCLONEDDS_SRC/build" --target install -j "$(nproc)"
fi
export CYCLONEDDS_HOME="$CYCLONEDDS_SRC/install"

cd "$ROOT/unitree_sdk2_python" && $PIP install -e .
# inspire_sdkpy con --no-deps + las dependencias de verdad a mano: su setup.py
# pide ademas PyQt5/pyqtgraph/colorcet, que solo usa su visor `qt_tabs`... que
# el __init__ del paquete importa siempre, asi que hacen falta igualmente.
cd "$ROOT/inspire_hand_ws/inspire_hand_sdk" && $PIP install -e . --no-deps
$PIP install "pymodbus==3.6.9" "pyserial" "PyQt5" "pyqtgraph" "colorcet"

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

# 5) Aislar el entorno del PYTHONPATH global (ROS Humble / robotpkg). Sin esto,
#    `import pinocchio` carga la 4.1.0 de /opt/openrobots en vez de la 3.1.0 de
#    conda que fija el README de xr_teleoperate.
bash "$ROOT/scripts/09_isolate_conda_env.sh" tv

echo "=== INSTALL HOST OK ==="

# 6) FIX: vuer 0.0.60 usa params_proto.Flag, eliminado en params-proto 3.x.
#    pip resuelve a 3.3.0 y "from vuer import Vuer" falla silenciosamente.
$PIP install "params-proto==2.13.2"
