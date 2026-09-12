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


class RestPose(ArmNode):
    def __init__(self, nombre="h1_2_rest_pose"):
        super().__init__(nombre)

    def run(self) -> int:
        with self.cliente() as cli:
            cli.wait_for_state()
            reposo = cli.gains.rest_posture()
            if not reposo:
                print("  no hay `rest_posture_deg` en config/gains.yaml")
                return 1

            rolls = {i: BY_INDEX[i].name for i in ARM_INDICES
                     if BY_INDEX[i].name.endswith("shoulder_roll")}
            codos = [i for i in ARM_INDICES if BY_INDEX[i].name.endswith("elbow")]
            salida = cli.gains.rest_roll_exit
            fuera = {i: math.copysign(salida, 1.0 if n.startswith("L_") else -1.0)
                     for i, n in rolls.items()}

            q0 = {i: cli.q(i) for i in ARM_INDICES}
            v = float(self.p("speed"))
            print(f"\n  Secuencia, a {v} rad/s:")
            print(f"    1. hombros   → ±{math.degrees(salida):.0f}°"
                  f"   (aparta la mano de la pierna)")
            print(f"    2. codos     → {math.degrees(reposo[codos[0]]):.0f}°"
                  f"   (estirar, ya lejos)")
            print(f"    3. reposo    → brazo estirado = mínimo de gravedad")

            cli.engage()
            print("\n  1/3 · apartando los hombros…")
            cli.ramp_to(fuera, speed=v)
            print("  2/3 · estirando los codos…")
            cli.ramp_to({i: reposo[i] for i in codos}, speed=v)
            print("  3/3 · a la postura de reposo…")
            cli.ramp_to(reposo, speed=v)
            for i in ARM_INDICES:
                cli.wait_settled(i)

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
