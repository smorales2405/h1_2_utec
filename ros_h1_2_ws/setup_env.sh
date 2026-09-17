#!/usr/bin/env bash
# Entorno para hablar con el H1-2 desde este workspace.
#
#   source setup_env.sh
#
# La NIC hacia el robot se detecta sola: la que tenga IP en 192.168.123.0/24.
# Si no hay ninguna, se usa H12_NIC, y si tampoco, enp12s0.

source /opt/ros/humble/setup.bash
source "$HOME/humanoid_ws/src/unitree_ros2/install/setup.bash"

_aqui="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
[ -f "$_aqui/install/setup.bash" ] && source "$_aqui/install/setup.bash"

# Aísla este workspace de ~/.local/lib/python3.10/site-packages.
#
# Hace falta por un choque de ABI de NumPy. En esta máquina conviven:
#
#   /usr/lib/python3/dist-packages      numpy 1.21.5  (paquete Debian)
#   ~/.local/.../site-packages          numpy 2.2.6   (instalado con pip)
#
# y `~/.local` va ANTES en `sys.path`, así que gana el 2.2.6. Pero
# `ros-humble-pinocchio` —y en general todo módulo binario de ROS Humble— está
# compilado contra la ABI de numpy 1.x, así que al importarlo revienta:
#
#   AttributeError: _ARRAY_API not found
#   [ros2run]: Segmentation fault
#
# No es un fallo de este paquete: le pasa a cualquier extensión C de ROS.
#
# La salida NO es desinstalar numpy 2.2.6: `opencv-python`, que está en ese
# mismo ~/.local, lo exige. Lo que se hace es que SOLO los procesos lanzados
# desde este entorno ignoren ~/.local, con lo que ven el numpy 1.21.5 de Debian
# y la ABI cuadra. El resto del sistema se queda como estaba.
#
# Si algún día hace falta un paquete de ~/.local aquí dentro, se desactiva con
#   H12_USE_USER_SITE=1 source setup_env.sh
# y entonces hay que resolver el choque de otra forma (un venv, por ejemplo).
if [ -z "${H12_USE_USER_SITE:-}" ]; then
    export PYTHONNOUSERSITE=1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

_nic="${H12_NIC:-}"
if [ -z "$_nic" ]; then
    _nic=$(ip -br -4 addr | awk '$3 ~ /^192\.168\.123\./ {print $1; exit}')
fi
export H12_NIC="${_nic:-enp12s0}"

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"${H12_NIC}\"/>
</Interfaces></General></Domain></CycloneDDS>"

# Si la NIC está caída, CycloneDDS no puede enlazarla y rclpy falla al crear el
# nodo con «rcl node's rmw handle is invalid», que no dice nada de la causa.
if [ -r "/sys/class/net/${H12_NIC}/carrier" ] \
   && [ "$(cat /sys/class/net/${H12_NIC}/carrier 2>/dev/null)" != "1" ]; then
    echo "  ⚠ ${H12_NIC} no tiene portadora: cable desconectado o robot apagado."
fi

echo "H1-2: NIC=${H12_NIC}  RMW=${RMW_IMPLEMENTATION}  numpy=$(python3 -c 'import numpy;print(numpy.__version__)' 2>/dev/null)"
unset _aqui _nic
