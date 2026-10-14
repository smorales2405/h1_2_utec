#!/usr/bin/env bash
# Teleoperacion XR del H1-2 con Meta Quest 3 y manos Inspire FTP, todo en el PC2.
#   1) servidor de imagen (teleimager, D435i por ZMQ)   -> fondo, con sudo (pyrealsense2 lo necesita)
#   2) driver de las manos (Modbus TCP <-> DDS)          -> fondo
#   3) teleop_hand_and_arm.py                            -> PRIMER PLANO: r empieza, q sale (abre manos y a casa)
# ANTES: robot COLGADO, modo Debug con L2+R2, L2+B a mano, nadie cerca de los brazos.
set -e
META=$HOME/robotics40/meta
export PYTHONNOUSERSITE=1
mkdir -p $META/logs
if ! pgrep -f "teleimager[.]image_server" >/dev/null; then
    pkill -f "robotics40/camara[.]py" 2>/dev/null || sudo pkill -f "robotics40/camara[.]py" 2>/dev/null || true
    echo "==> servidor de imagen (pide sudo)"
    sudo bash -c "cd $META/xr_teleoperate/teleop/teleimager && PYTHONNOUSERSITE=1 HOME=$HOME nohup $META/env_tv/bin/python -m teleimager.image_server --rs > $META/logs/image_server.log 2>&1 &"
    sleep 8
fi
if ! pgrep -f "inspire_ftp_dual_driver[.]py" >/dev/null; then
    echo "==> driver de las manos"
    nohup $HOME/teleop_venv/bin/python $HOME/inspire_ftp_dual_driver.py --network eth0 --no-touch > $META/logs/manos.log 2>&1 &
    sleep 3
fi
IP=$(ip -4 -o addr show wlxc03a5522df31 | awk "{print \$4}" | cut -d/ -f1)
echo
echo "==> En las Quest 3 (WiFi $(iwgetid -r)), navegador:  https://$IP:8012/?ws=wss://$IP:8012"
echo "    Aviso de certificado: Avanzado -> Continuar. Luego Virtual Reality y permitir todo."
echo "    Pon los brazos como el robot, pulsa r aqui para empezar; q para salir (abre las manos y los"
echo "    brazos vuelven a casa). Si se corta el SSH, el teleop sale solo como con q."
echo
cd $META
# El teleop escribe solo en su log; lo que se ve aqui es un tail. Con tee, un SSH cortado le romperia la
# salida o, con la WiFi caida, la bloquearia, y tiene que poder salir como con q (ver lanzar_teleop.py).
LOG=$META/logs/teleop.log
: > $LOG
tail -n +1 -f $LOG &
VISOR=$!
RC=0
PYTHONUNBUFFERED=1 $META/env_tv/bin/python lanzar_teleop.py --arm=H1_2 --ee=inspire_ftp --input-mode=hand --headless \
     --network-interface=eth0 --img-server-ip=127.0.0.1 "$@" > $LOG 2>&1 || RC=$?
sleep 1; kill $VISOR 2>/dev/null || true
exit $RC
