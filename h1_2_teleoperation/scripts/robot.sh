#!/usr/bin/env bash
# Acceso al H1-2 desde otro PC (p. ej. el de UTEC). En el propio robot no hace falta.
#
#   ./robot.sh               terminal en el robot, ya en ~/robotics40
#   ./robot.sh comprobar     red, SSH por clave y estado del robot (solo lectura)
#   ./robot.sh traer         copia ~/robotics40 del robot a este PC (sin meta/env_tv ni __pycache__)
#   ./robot.sh camara        abre la D435i (http://<robot>:5000) en el navegador; antes: ./camara.sh
#   ./robot.sh gafas         URL para las Meta Quest 3 del teleop XR
#
# Los demas lanzadores (./estado.sh, ./cuadrado.sh, manos/manos.sh...) ya se reenvian solos al robot:
# ver comun.sh. Otra direccion:  ROBOT_SSH=unitree@192.168.123.164 ./robot.sh
ROBOT_SSH=${ROBOT_SSH:-unitree@192.168.0.143}
IP=${ROBOT_SSH#*@}
AQUI="$(cd "$(dirname "$0")" && pwd)"
if [ "$(hostname)" = "unitree-h1-2-pc4" ]; then
    echo "Ya estas en el robot: usa los lanzadores directamente."; exit 0
fi
case "${1:-}" in
    "")
        exec ssh -t "$ROBOT_SSH" "cd ~/robotics40 && exec bash -l" ;;
    comprobar)
        ping -c 2 -W 1 "$IP" >/dev/null && echo "red: $IP responde" || { echo "red: $IP NO responde"; exit 1; }
        if ssh -o BatchMode=yes -o ConnectTimeout=8 "$ROBOT_SSH" true 2>/dev/null; then
            echo "ssh: acceso por clave OK"
        else
            echo "ssh: sin acceso por clave. Una sola vez:  ssh-copy-id $ROBOT_SSH"; exit 1
        fi
        exec "$AQUI/estado.sh" 2 ;;
    traer)
        echo "Copia $ROBOT_SSH:~/robotics40 -> $AQUI (se pisan los ficheros cambiados; no se borra nada)"
        exec rsync -a --info=progress2 --exclude=meta/env_tv --exclude=__pycache__ \
             "$ROBOT_SSH:robotics40/" "$AQUI/" ;;
    camara)
        echo "http://$IP:5000  (si no carga: ./camara.sh usb y luego ./camara.sh o ./camara.sh usb2)"
        xdg-open "http://$IP:5000" >/dev/null 2>&1 & ;;
    gafas)
        echo "En las Quest 3 (WiFi UTEC_H1_2):  https://$IP:8012/?ws=wss://$IP:8012"
        echo "Antes, en este PC:  ./teleop_xr.sh   (robot COLGADO; r empieza, q sale)" ;;
    *)
        sed -n '2,11p' "$0"; exit 1 ;;
esac
