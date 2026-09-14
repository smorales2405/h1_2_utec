#!/usr/bin/env python3
"""Devuelve los brazos a la postura de reposo sin que la mano roce la pierna.

Es lo que hay que ejecutar al TERMINAR, antes de soltar el control.

El problema que resuelve, medido: si se sueltan los brazos con el codo
flexionado y el hombro cerca de 0°, el antebrazo cae solo hasta quedar
colgando —0° de codo NO es el mínimo de gravedad, los codos pasan de 1° a 79°
y 85° en segundos— y en esa caída la mano golpea la pierna.

La secuencia que lo evita, y el orden es lo único que importa:

    1. los hombros salen a ±10°   (aparta la mano ANTES de que el brazo
                                   recorra nada a lo largo del cuerpo)
    2. los codos se estiran a 80° (ya lejos de la pierna)
    3. todo a la postura de reposo, con el brazo ya estirado, que ES el
       mínimo de gravedad: al soltar no cae nada

Verificado en el robot: soltando en 0° la deriva es de 84°; soltando aquí,
de 1.7°.

Los ángulos salen de `rest_posture_deg` en `config/gains.yaml`.

    ros2 run h1_2_arm_control rest_pose
    ros2 run h1_2_arm_control rest_pose --ros-args -p speed:=0.20
"""
import math
import sys

from ._node_base import ArmNode, ejecuta
from .joints import ARM_INDICES, BY_INDEX
from .postures import to_rest


class RestPose(ArmNode):
    def __init__(self, nombre="h1_2_rest_pose"):
        super().__init__(nombre)

    def run(self) -> int:
        with self.cliente() as cli:
            cli.wait_for_state()
            q0 = {i: cli.q(i) for i in ARM_INDICES}
            v = float(self.p("speed"))
            print(f"\n  Secuencia, a {v} rad/s. El orden es lo que evita que\n"
                  f"  la mano roce la pierna:\n")
            reposo = to_rest(cli, speed=v)

            self.tabla(cli, q0)
            print("\n  Brazos estirados y pegados al cuerpo. Al bajar las\n"
                  "  ganancias no hay nada que caiga.")
            # Soltar EN REPOSO. Volver a q0 sería volver a la postura peligrosa.
            cli.release(home_to=reposo)
        return 0


def main(args=None):
    sys.exit(ejecuta(RestPose, "h1_2_rest_pose", args))


if __name__ == "__main__":
    main()
