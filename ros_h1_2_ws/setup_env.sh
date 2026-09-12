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

echo "H1-2: NIC=${H12_NIC}  RMW=${RMW_IMPLEMENTATION}"
unset _aqui _nic
