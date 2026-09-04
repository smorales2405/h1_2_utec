#!/usr/bin/env bash
# Instalacion del lado SIMULACION (unitree_sim_isaaclab) — entorno conda "unitree_sim_env"
#
# Da por hecho que Isaac Sim 5.1 e Isaac Lab ya estan instalados en ese entorno.
# Aqui solo se añade lo que unitree_sim_isaaclab necesita por encima: el SDK de
# comunicacion con el robot, teleimager y sus requirements.
#
# Es idempotente: se puede volver a lanzar sin romper nada.
#
# Variables sobreescribibles:  ROOT, ENV_NAME, CONDA_BASE
set -euo pipefail

# ROOT = la carpeta h1_2_teleoperation (la que contiene scripts/). Se deduce de la
# ubicacion de este script, asi que el repo se puede clonar donde sea.
ROOT="${ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ENV_NAME="${ENV_NAME:-unitree_sim_env}"
CONDA_BASE="${CONDA_BASE:-/home/utec/miniconda3}"

# `env -u PYTHONPATH`: llamar a pip/python por su ruta absoluta NO dispara los
# hooks de conda, y con el PYTHONPATH de ROS puesto pip ve
# /opt/ros/humble/.../python3.10 y da por "already satisfied" paquetes que no
# estan en el entorno.
PYBIN="$CONDA_BASE/envs/$ENV_NAME/bin/python"
PIP="env -u PYTHONPATH $CONDA_BASE/envs/$ENV_NAME/bin/pip"
PY="env -u PYTHONPATH $PYBIN"

[ -x "$PYBIN" ] || { echo "ERROR: no existe el entorno '$ENV_NAME' en $CONDA_BASE/envs"; exit 1; }

echo "== Entorno: $($PY --version) en $CONDA_BASE/envs/$ENV_NAME"
# Se comprueba por metadatos, no importando: `import isaacsim` arranca medio Kit
# y en la primera ejecucion pide aceptar la EULA.
$PY - <<'EOF' || { echo "ERROR: el entorno no tiene Isaac Sim / Isaac Lab"; exit 1; }
import sys, importlib.metadata as md
try:
    print(f"== Isaac Sim {md.version('isaacsim')} + Isaac Lab {md.version('isaaclab')} ya presentes")
except Exception as e:
    print(e); sys.exit(1)
EOF

# ---------------------------------------------------------------- CycloneDDS
# unitree_sdk2py fija cyclonedds==0.10.2, que no publica rueda: pip la compila y
# necesita CYCLONEDDS_HOME apuntando a una instalacion de CycloneDDS.
CYCLONEDDS_SRC="$ROOT/cyclonedds"
if [ ! -f "$CYCLONEDDS_SRC/install/lib/libddsc.so" ]; then
    echo "== Compilando CycloneDDS 0.10.x =="
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
echo "== CYCLONEDDS_HOME=$CYCLONEDDS_HOME"

# ------------------------------------------------------------------ paquetes
echo "== unitree_sdk2_python =="
cd "$ROOT/unitree_sdk2_python" && $PIP install -e .

echo "== requirements de unitree_sim_isaaclab =="
cd "$ROOT/unitree_sim_isaaclab" && $PIP install -r requirements.txt

echo "== teleimager (rama sim, submodulo de unitree_sim_isaaclab) =="
# Sin extras: el servidor de imagen del simulador no usa el driver uvc, y aiortc
# y aiohttp ya vienen en requirements.txt.
cd "$ROOT/unitree_sim_isaaclab/teleimager" && $PIP install -e .

# omni/libcarb.so necesita GLIBCXX_3.4.30, que las libstdc++ viejas de conda no
# traen (problema conocido, doc/isaacsim5.1_install.md). Solo se instala la de
# conda-forge si de verdad falta: meter conda-forge en un entorno de "defaults"
# sin necesidad se paga caro.
LIBSTDCXX="$CONDA_BASE/envs/$ENV_NAME/lib/libstdc++.so.6"
if [ -f "$LIBSTDCXX" ] && strings "$LIBSTDCXX" 2>/dev/null | grep -q "GLIBCXX_3.4.30"; then
    echo "== libstdc++ del entorno ya trae GLIBCXX_3.4.30, no se toca =="
else
    echo "== libstdcxx-ng (falta GLIBCXX_3.4.30) =="
    "$CONDA_BASE/bin/conda" install -y -n "$ENV_NAME" -c conda-forge libstdcxx-ng
fi

# Aislar el entorno del PYTHONPATH global (ROS Humble / robotpkg, python3.10).
# Sin esto `import pinocchio` de sim_main.py revienta en un entorno python3.11.
echo "== Aislamiento de PYTHONPATH =="
bash "$ROOT/scripts/09_isolate_conda_env.sh" "$ENV_NAME"

# -------------------------------------------------------------- comprobacion
echo
echo "== Comprobacion de la configuracion de camaras =="
CAMCFG="$ROOT/unitree_sim_isaaclab/teleimager/cam_config_server.yaml"
grep -q "type: isaacsim" "$CAMCFG" \
    && echo "  ✔ type: isaacsim" \
    || echo "  ✗ $CAMCFG no tiene 'type: isaacsim' — corrigelo (doc/isaacsim5.1_install.md)"
grep -q "image_shape: \[480, 640\]" "$CAMCFG" \
    && echo "  ✔ image_shape: [480, 640]" \
    || echo "  ✗ $CAMCFG no tiene 'image_shape: [480, 640]'"

echo
echo "=== INSTALL SIM OK ==="
echo "Falta, si no se ha hecho ya:"
echo "  - assets USD:   cd $ROOT/unitree_sim_isaaclab && bash fetch_assets.sh"
echo "  - certificados: bash $ROOT/scripts/02_gen_certs.sh"
echo "Diagnostico:      bash $ROOT/scripts/13_check_sim.sh"
