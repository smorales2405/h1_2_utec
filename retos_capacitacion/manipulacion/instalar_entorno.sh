#!/usr/bin/env bash
# Crea (o completa) el entorno propio ~/utec/manipulacion/.venv en el PC2. Desde la PC se reenvia
# solo al robot (comun.sh); antes hay que haber desplegado (./desplegar.sh).
#   ./instalar_entorno.sh
# Necesita internet en el PC2 (pypi). No toca ~/robotics40 ni ~/pc2_offline: unitree_sdk2py se usa
# desde ~/pc2_offline/src/unitree_sdk2_python con un .pth, sin instalarlo ni modificarlo.
source "$(dirname "$0")/comun.sh"
set -e
[ "$(id -u)" -ne 0 ] || { echo "Como usuario unitree, no con sudo."; exit 1; }
cd "$MP"
SDK=/home/unitree/pc2_offline/src/unitree_sdk2_python
[ -d "$SDK/unitree_sdk2py" ] || { echo "No esta $SDK"; exit 1; }

# El PC2 no tiene python3-venv (sin ensurepip): el venv se crea sin pip y el pip propio lo instala el
# de teleop_venv con --python (pip >= 22.3), que solo se ejecuta, no se modifica.
if [ ! -x .venv/bin/pip ]; then
    rm -rf .venv
    python3 -m venv --without-pip .venv
    /home/unitree/teleop_venv/bin/python -m pip --python .venv/bin/python install pip
fi
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requisitos_robot.txt
SITE=$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
echo "$SDK" > "$SITE/unitree_sdk2py_pc2_offline.pth"

echo "--- comprobacion"
.venv/bin/python - <<'EOF'
import importlib
for n in ("numpy", "scipy", "cv2", "pyrealsense2", "pymodbus", "zmq", "yaml", "cyclonedds",
          "pinocchio", "mujoco", "unitree_sdk2py"):
    m = importlib.import_module(n)
    print(f"{n:15s} {getattr(m, '__version__', '?'):10s} {getattr(m, '__file__', '')}")
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # noqa: F401
from unitree_sdk2py.utils.crc import CRC  # noqa: F401
import cv2
cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
print("unitree_hg, CRC y aruco: bien")
EOF
