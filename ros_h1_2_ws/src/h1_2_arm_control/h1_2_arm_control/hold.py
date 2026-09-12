#!/usr/bin/env python3
"""Sostiene los brazos donde estén, con las ganancias del conjunto activo.

Sirve para dos cosas: comprobar que el control funciona antes de mandar nada
—si el brazo se queda firme y sin temblar, el canal y las ganancias están
bien— y mantener los brazos rígidos mientras se trabaja en otra cosa.

    ros2 run h1_2_arm_control hold --ros-args -p seconds:=10.0

Al terminar devuelve el control en rampa. Ctrl-C hace lo mismo.
"""
import math
import sys

from ._node_base import ArmNode, ejecuta
from .joints import ARM_INDICES, BY_INDEX


class Hold(ArmNode):
    def __init__(self, nombre="h1_2_hold"):
        super().__init__(nombre, {"seconds": 10.0})

    def run(self) -> int:
        with self.cliente() as cli:
            cli.wait_for_state()
            q0 = {i: cli.q(i) for i in ARM_INDICES}
            s = float(self.p("seconds"))
            print(f"\n  sosteniendo {len(q0)} articulaciones durante {s:.0f} s "
                  f"con el conjunto '{cli.gains.set_name}'…")
            cli.engage()
            cli.sleep(s)

            print(f"\n  {'articulación':<20}{'deriva':>10}{'par':>9}{'°C':>6}")
            for i in ARM_INDICES:
                d = math.degrees(cli.q(i) - q0[i])
                print(f"    {BY_INDEX[i].name:<18}{d:>9.3f}°"
                      f"{cli.tau(i):>8.2f}N{cli.temp(i):>6.0f}")
            print(f"\n  {cli.loop_health()}")
            cli.release()
        return 0


def main(args=None):
    sys.exit(ejecuta(Hold, "h1_2_hold", args))


if __name__ == "__main__":
    main()
