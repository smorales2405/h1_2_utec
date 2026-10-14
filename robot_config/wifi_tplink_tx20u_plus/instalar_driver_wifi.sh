#!/usr/bin/env bash
# Instala el driver de la WiFi USB TP-Link Archer TX20U Plus (AX1800, Realtek RTL8832AU, USB 2357:013f) en el
# PC2 de un H1-2. Se ejecuta DESDE EL PC: el robot no tiene Internet hasta que su WiFi funciona, asi que el
# codigo se descarga aqui y se le pasa por el cable interno.
#
#   ./instalar_driver_wifi.sh [unitree@192.168.123.164]       compila, instala y carga el modulo (pide sudo)
#   SOLO_COMPILAR=1 ./instalar_driver_wifi.sh [...]           solo compila: comprueba que el kernel lo admite
#
# Replica lo que se hizo en el H1-2 de UTEC el 2026-09-23: lwfinger/rtl8852au en el commit 865ab0f, compilado
# con `make` contra el kernel en marcha del PC2 (6.5.2-rt8, fuentes en /usr/src/linux-6.5.2), el modulo en
# /lib/modules/<kernel>/extra/ y cargado en cada arranque por /etc/modules-load.d/8852au.conf.
# Variables:  DIR_ROBOT (carpeta en el robot, relativa a su $HOME), CACHE (clon local del driver).
set -euo pipefail
ROBOT=${1:-unitree@192.168.123.164}
REPO=https://github.com/lwfinger/rtl8852au.git
COMMIT=865ab0fa91471d595c283d2f3db323f7f15455f5
CACHE=${CACHE:-$HOME/.cache/h1_2_utec/rtl8852au}
DIR_ROBOT=${DIR_ROBOT:-robotics40/rtl8852au}
SOLO_COMPILAR=${SOLO_COMPILAR:-0}

echo "==> 1. codigo del driver en este PC ($CACHE, commit ${COMMIT:0:7})"
[ -d "$CACHE/.git" ] || git clone -q "$REPO" "$CACHE"
git -C "$CACHE" cat-file -e "$COMMIT^{commit}" 2>/dev/null || git -C "$CACHE" fetch -q origin
git -C "$CACHE" checkout -q "$COMMIT"

echo "==> 2. copia al robot ($ROBOT:$DIR_ROBOT)"
ssh "$ROBOT" "mkdir -p $DIR_ROBOT"
rsync -a --exclude=.git "$CACHE/" "$ROBOT:$DIR_ROBOT/"

echo "==> 3. compilar en el robot (unos minutos)"
ssh "$ROBOT" "set -e; cd $DIR_ROBOT; K=\$(uname -r); echo \"kernel \$K\"
    [ -d /lib/modules/\$K/build ] || { echo 'falta /lib/modules/'\$K'/build: sin las fuentes del kernel no se puede compilar'; exit 1; }
    make -j\$(nproc) > build.log 2>&1 || { tail -20 build.log; exit 1; }
    ls -la 8852au.ko"

if [ "$SOLO_COMPILAR" = 1 ]; then echo "==> SOLO_COMPILAR=1: no se instala"; exit 0; fi

echo "==> 4. instalar y cargar el modulo (pide el sudo del robot)"
ssh -t "$ROBOT" "set -e; cd $DIR_ROBOT; K=\$(uname -r)
    sudo install -D -m 644 8852au.ko /lib/modules/\$K/extra/8852au.ko
    sudo depmod -a \$K
    echo 8852au | sudo tee /etc/modules-load.d/8852au.conf > /dev/null
    sudo modprobe 8852au
    sleep 3
    lsusb | grep -i 2357: || echo 'AVISO: lsusb no ve el adaptador TP-Link (2357:...): ¿esta enchufado?'
    ip -br link | grep wlx || echo 'AVISO: aun no aparece la interfaz wlx...'"
echo "==> LISTO. Siguiente paso: conectar la WiFi (README.md, apartado 3)"
