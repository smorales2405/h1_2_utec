#!/usr/bin/env bash
# Instala el demonio del gesto en el PC2 del robot.
#
#   ./install_pc2.sh [usuario@ip]
#
# Por defecto unitree@192.168.123.164. Copia el código, instala la unidad de
# systemd y la deja arrancada y habilitada para el siguiente encendido.
#
# Necesita poder entrar por SSH sin contraseña:
#   ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519     (si no tienes clave)
#   ssh-copy-id unitree@192.168.123.164
set -euo pipefail

DESTINO="${1:-unitree@192.168.123.164}"
AQUI="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REMOTO="/home/unitree/h1_2_six_seven"

echo "══ comprobando el PC2 ══"
ssh "$DESTINO" '
  set -e
  echo -n "  python del venv: "
  /home/unitree/teleop_venv/bin/python --version
  /home/unitree/teleop_venv/bin/python - <<PY
import unitree_sdk2py, numpy, yaml
print("  unitree_sdk2py", getattr(unitree_sdk2py, "__version__", "?"))
print("  numpy", numpy.__version__)
PY
'

echo
echo "══ copiando a $DESTINO:$REMOTO ══"
ssh "$DESTINO" "mkdir -p $REMOTO"
rsync -a --delete "$AQUI/src/" "$DESTINO:$REMOTO/src/"
rsync -a "$AQUI/gains.yaml" "$DESTINO:$REMOTO/gains.yaml"

echo
echo "══ instalando el servicio ══"
scp "$AQUI/h1_2_six_seven.service" "$DESTINO:/tmp/h1_2_six_seven.service"
ssh -t "$DESTINO" '
  sudo mv /tmp/h1_2_six_seven.service /etc/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable h1_2_six_seven.service
  sudo systemctl restart h1_2_six_seven.service
  sleep 2
  sudo systemctl --no-pager status h1_2_six_seven.service | head -12
'

echo
echo "  Listo. Para ver qué hace:"
echo "      ssh $DESTINO 'journalctl -u h1_2_six_seven -f'"
echo "  Para pararlo:"
echo "      ssh $DESTINO 'sudo systemctl stop h1_2_six_seven'"
