#!/usr/bin/env bash
# Diagnostico del lado SIMULACION: entorno, GPU, assets, parche FTP y certificados.
#
# Variables sobreescribibles:  ROOT, ENV_NAME, CONDA_BASE
# ROOT = la carpeta h1_2_teleoperation (la que contiene scripts/). Se deduce de la
# ubicacion de este script, asi que el repo se puede clonar donde sea.
ROOT="${ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ENV_NAME="${ENV_NAME:-unitree_sim_env}"
CONDA_BASE="${CONDA_BASE:-/home/utec/miniconda3}"
PYBIN="$CONDA_BASE/envs/$ENV_NAME/bin/python"
# `env -u PYTHONPATH` reproduce lo que ve el entorno activado: los hooks de
# 09_isolate_conda_env.sh quitan el PYTHONPATH global de ROS/robotpkg.
PY="env -u PYTHONPATH $PYBIN"
SIM="$ROOT/unitree_sim_isaaclab"

ok(){ printf "  \033[32m✔\033[0m %s\n" "$1"; }
no(){ printf "  \033[31m✗\033[0m %s\n" "$1"; }

echo "== 1. Entorno conda '$ENV_NAME' =="
if [ -x "$PYBIN" ]; then ok "$($PY --version 2>&1)"; else no "falta $CONDA_BASE/envs/$ENV_NAME"; exit 1; fi

echo "== 2. GPU y driver =="
if command -v nvidia-smi >/dev/null; then
  nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | sed 's/^/    /'
else
  no "nvidia-smi no encontrado"
fi

echo "== 3. Paquetes clave =="
$PY - <<'EOF'
import importlib.metadata as md
esperado = {
 "isaacsim":"5.1.0.0", "isaaclab":"", "torch":"2.7.0+cu128",
 "unitree_sdk2py":"1.0.1", "cyclonedds":"0.10.2", "teleimager":"",
 "rerun-sdk":"0.20.1", "pyzmq":"27.0.0", "onnxruntime":"1.22.1",
 "pynput":"1.8.1", "aiortc":"", "aiohttp":"", "numpy":"<2",
}
for p, exp in esperado.items():
    try:
        v = md.version(p)
        print(f"  \033[32m✔\033[0m {p:<16} {v:<14} {('esperado: '+exp) if exp else ''}")
    except Exception:
        print(f"  \033[31m✗\033[0m {p:<16} NO INSTALADO")
EOF

echo "== 4. Assets USD =="
if [ -d "$SIM/assets" ]; then
  ok "$SIM/assets ($(du -sh "$SIM/assets" 2>/dev/null | cut -f1))"
else
  no "faltan los assets — ejecuta: cd $SIM && bash fetch_assets.sh"
fi

echo "== 5. Configuracion de camaras (teleimager, rama sim) =="
CAMCFG="$SIM/teleimager/cam_config_server.yaml"
if [ -f "$CAMCFG" ]; then
  grep -q "type: isaacsim"            "$CAMCFG" && ok "type: isaacsim"            || no "type != isaacsim en $CAMCFG"
  grep -q "image_shape: \[480, 640\]" "$CAMCFG" && ok "image_shape: [480, 640]"   || no "image_shape != [480, 640] en $CAMCFG"
else
  no "falta $CAMCFG (¿submodulo sin inicializar?)"
fi

echo "== 6. Puente FTP de las manos (parche de este repo) =="
[ -f "$SIM/dds/inspire_ftp_dds.py" ] && ok "dds/inspire_ftp_dds.py" || no "falta dds/inspire_ftp_dds.py"
[ -f "$SIM/dds/inspire_ftp_idl.py" ] && ok "dds/inspire_ftp_idl.py" || no "falta dds/inspire_ftp_idl.py"
grep -q "enable_inspire_ftp_dds" "$SIM/sim_main.py"     && ok "--enable_inspire_ftp_dds en sim_main.py"  || no "sim_main.py sin el flag FTP"
grep -q "InspireFTPDDS"          "$SIM/dds/dds_create.py" && ok "InspireFTPDDS registrado en dds_create.py" || no "dds_create.py sin InspireFTPDDS"

echo "== 7. Aislamiento de PYTHONPATH =="
# Solo es un problema si el entorno global mete /opt/ros o /opt/openrobots.
if printf '%s:%s' "${PYTHONPATH:-}" "${LD_LIBRARY_PATH:-}" | grep -q "/opt/ros\|/opt/openrobots"; then
  if [ -f "$CONDA_BASE/envs/$ENV_NAME/etc/conda/activate.d/00_isolate_from_ros.sh" ]; then
    ok "hook activate.d instalado (quita ROS Humble / robotpkg del entorno)"
  else
    no "esta maquina exporta /opt/ros o /opt/openrobots: sim_main.py fallara en 'import pinocchio' — corre scripts/09_isolate_conda_env.sh"
  fi
else
  ok "el entorno global no mete /opt/ros ni /opt/openrobots: no hace falta aislar"
fi
PINO_SRC=$(bash -lc "source $CONDA_BASE/etc/profile.d/conda.sh && conda activate $ENV_NAME && python -c 'import pinocchio; print(pinocchio.__version__, pinocchio.__file__)'" 2>/dev/null | tail -1)
case "$PINO_SRC" in
  *"envs/$ENV_NAME"/*) ok "pinocchio resuelto en el entorno: $PINO_SRC" ;;
  "")                  no "no se pudo importar pinocchio tras 'conda activate $ENV_NAME'" ;;
  *)                   no "pinocchio se resuelve FUERA del entorno: $PINO_SRC" ;;
esac

echo "== 8. Imports del simulador (sin arrancar Isaac Sim) =="
cd "$SIM" && $PY - <<'EOF' 2>&1 | grep -v "^\["
import sys
def t(desc, fn):
    try: fn(); print(f"  \033[32m✔\033[0m {desc}")
    except Exception as e: print(f"  \033[31m✗\033[0m {desc}: {type(e).__name__}: {e}")
t("unitree_sdk2py DDS",  lambda: __import__("unitree_sdk2py.core.channel", fromlist=["ChannelPublisher"]).ChannelPublisher)
t("teleimager server",   lambda: __import__("teleimager.image_server", fromlist=["run_isaacsim_server"]).run_isaacsim_server)
t("IDL Inspire FTP",     lambda: __import__("dds.inspire_ftp_idl", fromlist=["inspire_hand_ctrl"]).inspire_hand_ctrl)
t("InspireFTPDDS",       lambda: __import__("dds.inspire_ftp_dds", fromlist=["InspireFTPDDS"]).InspireFTPDDS)
EOF

echo "== 9. Certificados TLS (televuer :8012 y WebRTC :60001) =="
for f in "$HOME/.config/xr_teleoperate/cert.pem" "$HOME/.config/xr_teleoperate/key.pem"; do
  [ -f "$f" ] && ok "$f" || no "FALTA $f  (corre scripts/02_gen_certs.sh)"
done

echo "== 10. Entorno 'tv' (cliente de teleoperacion) =="
TVPY="$CONDA_BASE/envs/tv/bin/python"
[ -x "$TVPY" ] && ok "$($TVPY --version 2>&1)" || no "falta el entorno tv — corre scripts/01_install_host.sh"
