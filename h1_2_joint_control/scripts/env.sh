#!/usr/bin/env bash
# Entorno para hablar con el H1-2 por DDS desde este portátil.
#
#   source scripts/env.sh
#
# NIC hacia el robot: se detecta la que tiene IP en 192.168.123.0/24; si no
# hay ninguna, se usa H12_NIC o enp12s0.

source /opt/ros/humble/setup.bash
source "$HOME/humanoid_ws/src/unitree_ros2/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

_nic="${H12_NIC:-}"
if [ -z "$_nic" ]; then
    _nic=$(ip -br -4 addr | awk '$3 ~ /^192\.168\.123\./ {print $1; exit}')
fi
export H12_NIC="${_nic:-enp12s0}"

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"${H12_NIC}\"/>
</Interfaces></General></Domain></CycloneDDS>"

# Si la NIC está caída, CycloneDDS no puede enlazarla y rclpy falla al crear
# el nodo con "rcl node's rmw handle is invalid", que no dice nada de la causa.
# Mejor avisar aquí.
if [ -r "/sys/class/net/${H12_NIC}/carrier" ] \
   && [ "$(cat /sys/class/net/${H12_NIC}/carrier 2>/dev/null)" != "1" ]; then
    echo "  ⚠ ${H12_NIC} no tiene portadora: cable desconectado o robot apagado."
    echo "    Los scripts fallarán al crear el nodo ROS. Revisa el cable."
fi

# Para poder importar el paquete sin instalarlo
_here="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )"
export PYTHONPATH="${_here}:${PYTHONPATH}"

echo "H1-2: NIC=${H12_NIC}  RMW=${RMW_IMPLEMENTATION}  PYTHONPATH+=${_here}"
unset _nic _here
