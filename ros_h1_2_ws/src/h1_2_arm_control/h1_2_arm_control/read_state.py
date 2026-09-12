#!/usr/bin/env python3
"""Lee el estado de las articulaciones. SOLO LECTURA: no publica nada.

No hay ningún publicador en este nodo, así que el robot no se entera. Es lo
primero que conviene correr para saber dónde está antes de mandarle nada.

    ros2 run h1_2_arm_control read_state
    ros2 run h1_2_arm_control read_state --ros-args -p all:=true
    ros2 run h1_2_arm_control read_state --ros-args -p watch:=true
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_hg.msg import LowState

from .joints import ARM_INDICES, BY_INDEX, JOINTS, NUM_CMD_MOTOR


class ReadState(Node):
    def __init__(self, nombre="h1_2_read_state"):
        super().__init__(nombre)
        self.declare_parameter("all", False)
        self.declare_parameter("watch", False)
        self.declare_parameter("timeout", 15.0)
        self.last = None
        self.create_subscription(LowState, "/lowstate", self._cb,
                                 qos_profile_sensor_data)

    def _cb(self, msg):
        self.last = msg

    def run(self) -> int:
        t0 = time.time()
        lim = float(self.get_parameter("timeout").value)
        while self.last is None and time.time() - t0 < lim:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.last is None:
            print(f"  no llega /lowstate en {lim:.0f} s. ¿Robot encendido? "
                  f"¿cable conectado? ¿RMW y CYCLONEDDS_URI bien puestos?")
            return 1

        idx = (range(NUM_CMD_MOTOR) if self.get_parameter("all").value
               else ARM_INDICES)
        vigilar = bool(self.get_parameter("watch").value)
        try:
            while True:
                m = self.last
                print(f"\n  modo máquina: {m.mode_machine}")
                print(f"  {'#':>3} {'articulación':<20}{'rad':>9}{'grados':>9}"
                      f"{'rad/s':>8}{'Nm':>8}{'°C':>5}")
                for i in idx:
                    s = m.motor_state[i]
                    print(f"  {i:>3} {BY_INDEX[i].name:<20}{s.q:>9.3f}"
                          f"{math.degrees(s.q):>9.1f}{s.dq:>8.3f}"
                          f"{s.tau_est:>8.2f}{s.temperature[0]:>5}")
                if not vigilar:
                    break
                time.sleep(0.3)
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        print("\n  Solo lectura: no se ha publicado nada.")
        return 0


def main(args=None):
    rclpy.init(args=args)
    n = ReadState()
    try:
        sys.exit(n.run())
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
