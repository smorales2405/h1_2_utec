#!/usr/bin/env bash
# Diagnostico del Host (esta laptop): entorno, dependencias, certificados y red.
# ROOT = la carpeta h1_2_teleoperation (la que contiene scripts/). Se deduce de la
# ubicacion de este script, asi que el repo se puede clonar donde sea.
# Resolucion de conda: busca la instalacion en vez de cablearla.
source "$(dirname "${BASH_SOURCE[0]}")/_conda.sh"
ROOT="${ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PYBIN=${CONDA_ENV_TV}/bin/python
# `env -u PYTHONPATH` reproduce lo que ve el entorno una vez activado: los
# hooks de 09_isolate_conda_env.sh quitan el PYTHONPATH global de ROS/robotpkg.
PY="env -u PYTHONPATH $PYBIN"
ok(){ printf "  \033[32m✔\033[0m %s\n" "$1"; }
no(){ printf "  \033[31m\u2717\033[0m %s\n" "$1"; }

echo "== 1. Entorno conda 'tv' =="
if [ -x "$PYBIN" ]; then ok "$($PY --version 2>&1)"; else no "falta ${CONDA_ENV_TV}"; exit 1; fi

echo "== 2. Paquetes clave =="
$PY - <<'EOF'
import importlib.metadata as md
esperado = {
 "pinocchio":"3.2.0 (conda)","numpy":">=1.26","torch":"2.3.0+cpu","vuer":"0.0.60",
 "params_proto":"2.13.2 (>=3 rompe vuer)","cyclonedds":"0.10.2","televuer":"4.0.0",
 "teleimager":"1.6.0","dex_retargeting":"0.4.7","unitree_sdk2py":"1.0.1",
 "inspire_sdkpy":"1.0.0","pymodbus":"3.6.9","rerun-sdk":"0.20.1","meshcat":"0.3.2",
 "sshkeyboard":"2.3.1","opencv-python":"", "matplotlib":"",
}
import glob, os
def conda_pkg_version(name):
    # La ruta del entorno se lee de la VARIABLE, no se interpola en la
    # f-string: este heredoc esta entrecomillado y bash no expande dentro.
    base = os.environ.get("CONDA_ENV_TV", "")
    hits = glob.glob(f"{base}/conda-meta/{name}-*.json")
    return os.path.basename(hits[0]).rsplit("-", 2)[1] if hits else "?"
for p, exp in esperado.items():
    try:
        if p == "pinocchio":
            # conda-meta no siempre refleja la version tras un cambio en
            # caliente; lo que importa es la que se importa de verdad
            v = conda_pkg_version("pinocchio")
            if v in (None, "", "?"):
                try:
                    import pinocchio as _pin; v = _pin.__version__
                except Exception:
                    v = None
        else:
            v = md.version(p)
        print(f"  \033[32m✔\033[0m {p:<18} {v:<14} {('esperado: '+exp) if exp else ''}")
    except Exception:
        print(f"  \033[31m✗\033[0m {p:<18} NO INSTALADO")
EOF

echo "== 3. Aislamiento de PYTHONPATH =="
# Solo es un problema si el entorno global mete /opt/ros o /opt/openrobots.
if printf '%s:%s' "${PYTHONPATH:-}" "${LD_LIBRARY_PATH:-}" | grep -q "/opt/ros\|/opt/openrobots"; then
  if [ -f ${CONDA_ENV_TV}/etc/conda/activate.d/00_isolate_from_ros.sh ]; then
    ok "hook activate.d instalado (quita ROS Humble / robotpkg del entorno)"
  else
    no "esta maquina exporta /opt/ros o /opt/openrobots y el entorno no esta aislado — corre scripts/09_isolate_conda_env.sh"
  fi
else
  ok "el entorno global no mete /opt/ros ni /opt/openrobots: no hace falta aislar"
fi
PINO_SRC=$(bash -lc 'source ${CONDA_BASE}/etc/profile.d/conda.sh && conda activate tv && python -c "import pinocchio; print(pinocchio.__version__, pinocchio.__file__)"' 2>/dev/null | tail -1)
case "$PINO_SRC" in
  *envs/tv/*) ok "pinocchio resuelto en el entorno: $PINO_SRC" ;;
  "")         no "no se pudo importar pinocchio tras 'conda activate tv'" ;;
  *)          no "pinocchio se resuelve FUERA del entorno: $PINO_SRC" ;;
esac

echo "== 4. Imports funcionales =="
ROOT="$ROOT" $PY - <<'EOF' 2>/dev/null
import sys, os
ROOT = os.environ["ROOT"]
sys.path.insert(0, f"{ROOT}/xr_teleoperate")
os.chdir(f"{ROOT}/xr_teleoperate/teleop")
def t(desc, fn):
    try:
        fn(); print(f"  \033[32m✔\033[0m {desc}")
    except Exception as e:
        print(f"  \033[31m✗\033[0m {desc}: {type(e).__name__}: {e}")
        if "class version" in str(e):
            # pinocchio serializa el modelo con pickle: una cache escrita por
            # otra version de pinocchio no se puede leer.
            print("      -> cache de pinocchio de otra version: borra teleop/*_model_cache.pkl")
t("vuer / televuer",       lambda: __import__("televuer").TeleVuerWrapper)
t("teleimager ImageClient",lambda: __import__("teleimager.image_client", fromlist=["ImageClient"]).ImageClient)
t("unitree_sdk2py DDS",    lambda: __import__("unitree_sdk2py.core.channel", fromlist=["ChannelPublisher"]).ChannelPublisher)
t("inspire_sdkpy (FTP)",   lambda: __import__("inspire_sdkpy.inspire_dds", fromlist=["inspire_hand_ctrl"]).inspire_hand_ctrl)
t("H1_2_ArmIK (pinocchio)",lambda: __import__("teleop.robot_control.robot_arm_ik", fromlist=["H1_2_ArmIK"]).H1_2_ArmIK())
def hr():
    m = __import__("teleop.robot_control.hand_retargeting", fromlist=["HandRetargeting","HandType"])
    m.HandRetargeting(m.HandType.INSPIRE_HAND)
t("HandRetargeting Inspire", hr)
EOF

echo "== 5. Certificados TLS =="
for f in "$HOME/.config/xr_teleoperate/cert.pem" "$ROOT/xr_teleoperate/teleop/televuer/cert.pem"; do
  if [ -f "$f" ]; then ok "$f"; else no "FALTA $f  (corre scripts/02_gen_certs.sh)"; fi
done
[ -f "$HOME/.config/xr_teleoperate/cert.pem" ] && \
  openssl x509 -in "$HOME/.config/xr_teleoperate/cert.pem" -noout -dates -ext subjectAltName | sed 's/^/    /'

echo "== 6. Red (solo aplica al despliegue FISICO) =="
ip -4 -brief addr | grep -v '^lo' | sed 's/^/    /'
# La NIC hacia el robot NO se cablea: se busca la que tenga IP en 192.168.123.0/24.
# En la maquina original era enp0s31f6 con .222; en otras cambia.
H12_NIC_FOUND=$(ip -br -4 addr | awk '$3 ~ /^192\.168\.123\./ {print $1; exit}')
H12_IP_FOUND=$(ip -br -4 addr | awk '$3 ~ /^192\.168\.123\./ {split($3,a,"/"); print a[1]; exit}')
if [ -n "$H12_NIC_FOUND" ]; then
  ok "$H12_NIC_FOUND = $H12_IP_FOUND (red del robot activa)"
else
  no "ninguna interfaz con IP en 192.168.123.0/24 — cable desconectado o perfil inactivo"
  echo "      activar con: nmcli con up unitree-h1_2"
fi
echo "  Alcance del robot:"
for ip in 192.168.123.161 192.168.123.164; do
  if ping -c1 -W1 "$ip" >/dev/null 2>&1; then ok "ping $ip"; else no "ping $ip (sin respuesta)"; fi
done
