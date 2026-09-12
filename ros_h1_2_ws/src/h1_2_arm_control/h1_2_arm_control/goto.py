#!/usr/bin/env python3
"""Lleva articulaciones a los ángulos que se le digan, con rampa suave.

Es el ladrillo que un algoritmo usa para colocar el brazo en un punto: una
rampa de coseno alzado desde donde esté hasta el objetivo, con velocidad nula
al principio y al final, sin tirones.

Los ángulos van en GRADOS, que es como se leen y se escriben en la práctica.

    # una articulación
    ros2 run h1_2_arm_control goto --ros-args -p joints:='[L_elbow]' \
                                              -p targets_deg:='[45.0]'

    # los dos brazos a la vez
    ros2 run h1_2_arm_control goto --ros-args \
        -p joints:='[L_shoulder_pitch, L_elbow, R_shoulder_pitch, R_elbow]' \
        -p targets_deg:='[-20.0, 60.0, -20.0, 60.0]' -p speed:=0.20

La protección de autocolisión sigue puesta: si el objetivo pide un hombro más
cerca del cuerpo de lo que permite el codo, se recorta y se avisa.
"""
import math
import sys

from ._node_base import ArmNode, ejecuta
from .gains import joint_or_die
from .joints import BY_INDEX


class Goto(ArmNode):
    def __init__(self, nombre="h1_2_goto"):
        super().__init__(nombre, {
            "joints": [""],
            "targets_deg": [0.0],
            "hold": 1.0,
        })

    def run(self) -> int:
        nombres = [n for n in self.p("joints") if n]
        objetivos = list(self.p("targets_deg"))
        if not nombres:
            print("  falta -p joints:='[L_elbow]'")
            return 1
        if len(nombres) != len(objetivos):
            print(f"  {len(nombres)} articulaciones y {len(objetivos)} ángulos: "
                  f"tienen que ser tantos de uno como de otro.")
            return 1
        destino = {joint_or_die(n): math.radians(float(v))
                   for n, v in zip(nombres, objetivos)}

        with self.cliente() as cli:
            cli.wait_for_state()
            q0 = {i: cli.q(i) for i in destino}
            v = float(self.p("speed"))
            recorrido = max(abs(destino[i] - q0[i]) for i in destino)
            print(f"\n  {'articulación':<20}{'de':>10}{'a':>10}")
            for i in destino:
                print(f"    {BY_INDEX[i].name:<18}{math.degrees(q0[i]):>9.2f}°"
                      f"{math.degrees(destino[i]):>9.2f}°")
            print(f"\n  Recorrido mayor {math.degrees(recorrido):.1f}°, "
                  f"unos {recorrido / max(v, 1e-3):.0f} s a {v} rad/s")

            cli.engage()
            cli.ramp_to(destino, speed=v)
            for i in destino:
                cli.wait_settled(i)
            cli.sleep(float(self.p("hold")))
            self.tabla(cli, q0)
            # Soltar DONDE SE HA PEDIDO, no donde estaba: colocar y deshacerlo
            # sería lo contrario de lo que pide este nodo.
            cli.release(home_to=destino)
        return 0


def main(args=None):
    sys.exit(ejecuta(Goto, "h1_2_goto", args))


if __name__ == "__main__":
    main()
