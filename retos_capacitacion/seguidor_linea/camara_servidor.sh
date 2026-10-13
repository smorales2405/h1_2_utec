#!/usr/bin/env bash
# Servidor de la D435i para el seguidor: abre la camara como root y publica los fotogramas
# por ZMQ en 127.0.0.1. No toca el DDS (con sudo no arranca: fs.protected_regular y /tmp/cdds.LOG).
#   ./camara_servidor.sh                     IR izquierdo sin emisor (config/seguidor.yaml)
#   ./camara_servidor.sh --profundidad       anade la profundidad (calibracion)
#   ./camara_servidor.sh --color --emisor    otras opciones: ver camara_servidor.py --help
#   ./camara_servidor.sh parar | estado | log
# Queda en segundo plano (setsid): sobrevive a que se corte el SSH. Log en /tmp/seguidor_camara.log
# Sin terminal (lanzado por otro programa): SUDO_STDIN=1 y la contrasena de sudo por la entrada estandar,
#   printf '%s\n' "$PW" | ssh unitree@<robot> 'cd ~/utec/seguidor_linea && SUDO_STDIN=1 ./camara_servidor.sh'
# (sudo recuerda la contrasena por proceso: tiene que validarse aqui dentro, no en quien lo llama).
source "$(dirname "$0")/comun.sh"
LOG=/tmp/seguidor_camara.log

validar_sudo() { if [ -n "${SUDO_STDIN:-}" ]; then sudo -S -p "" -v; else sudo -v; fi; }
# patron con [e]: no se encuentra a si mismo en la linea de ordenes de quien lo lance
parar() { sudo pkill -f "seguidor_lin[e]a/camara_servidor[.]py"; }

case "${1:-}" in
    parar)
        validar_sudo || exit 1
        parar && echo "parado" || echo "no estaba corriendo" ;;
    estado)
        pgrep -af "python -u .*seguidor_lin[e]a/camara_servidor[.]py" || echo "no esta corriendo"
        tail -3 "$LOG" 2>/dev/null ;;
    log)
        tail -40 "$LOG" ;;
    *)
        validar_sudo || exit 1
        if pgrep -f "robotics40/camara(_usb2)?[.]py" > /dev/null; then
            echo "camara.py de robotics40 tiene la camara abierta: ~/robotics40/camara.sh parar"; exit 1
        fi
        parar 2>/dev/null && sleep 1
        sudo PYTHONNOUSERSITE=1 setsid nohup "$PY" -u "$SL/camara_servidor.py" "$@" > "$LOG" 2>&1 < /dev/null &
        for _ in $(seq 1 15); do
            sleep 1
            grep -q "\[CAMARA\] en marcha" "$LOG" && break
        done
        if ! pgrep -f "python -u .*seguidor_lin[e]a/camara_servidor[.]py" > /dev/null; then
            echo "ERROR: el servidor no ha arrancado. Log:"; tail -20 "$LOG"; exit 1
        fi
        if grep -q "\[CAMARA\] en marcha" "$LOG"; then
            echo "camara_servidor en marcha (log $LOG)"
        else
            echo "AVISO: el servidor corre pero la camara aun no da imagen (se reintenta sola):"
            tail -5 "$LOG"
        fi ;;
esac
