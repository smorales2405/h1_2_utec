#!/usr/bin/env bash
# Para el servidor de imagen y el driver de las manos (el teleop se sale con q).
pkill -f "lanzar_teleop[.]py" 2>/dev/null || true
pkill -f "inspire_ftp_dual_driver[.]py" 2>/dev/null || true
sudo pkill -f "teleimager[.]image_server" 2>/dev/null || true
sleep 1; pgrep -af "lanzar_teleop|inspire_ftp_dual|teleimager.image_server" | grep -v pgrep || echo "parado"
