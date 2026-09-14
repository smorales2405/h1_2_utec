#!/usr/bin/env python3
"""Lleva los catorce motores de los brazos a 0°, despacio y en orden.

Es la inicialización antes de cualquier algoritmo: una postura de partida
conocida y repetible, en vez de «donde estuviera el robot al arrancar».

EL ORDEN IMPORTA, y no es un detalle de implementación. El tope de
autocolisión del hombro depende del codo —±10° con el brazo estirado, 0° con
el codo flexionado—, así que hay que flexionar PRIMERO. En un solo tramo, los
hombros se quedarían topando en ±10° sin llegar a cero.

    1. codos a 0°          (flexionar, que libera al hombro)
    2. muñecas y el resto
    3. hombros a 0°        (ya con el codo flexionado)

    ros2 run h1_2_arm_control init_pose
    ros2 run h1_2_arm_control init_pose --ros-args -p speed:=0.10
    ros2 run h1_2_arm_control init_pose --ros-args -p dry_run:=true

Requiere el canal libre:  ros2 run h1_2_arm_control debug_mode --ros-args -p action:=enter
"""
import math
import sys

from ._node_base import ArmNode, ejecuta
from .joints import ARM_INDICES, BY_INDEX
from .postures import to_zero


class InitPose(ArmNode):
    def __init__(self, nombre="h1_2_init_pose"):
        super().__init__(nombre, {
            "hold": 1.0,
            # `keep` deja el brazo SOSTENIDO hasta Ctrl-C en vez de soltarlo.
            # Hace falta saber por qué: al soltar, las ganancias bajan a cero y
            # la gravedad se lleva los codos —medido, de 1° a 79° y 85° en
            # segundos—. Si lo que sigue es otro proceso, para cuando ese
            # enganche el brazo ya se cayó. Con `keep:=true` se queda firme,
            # pero OJO: mientras esté sostenido nadie más puede publicar en
            # /lowcmd sin pelearse con este nodo. Para encadenar un algoritmo,
            # lo correcto es llamar a `postures.to_zero()` desde el algoritmo
            # con su propio cliente: ver `algorithm_template.py`.
            "keep": False,
        })

    def run(self) -> int:
        with self.cliente() as cli:
            cli.wait_for_state()
            q0 = {i: cli.q(i) for i in ARM_INDICES}
            print("\n  Postura de partida:")
            for i in ARM_INDICES:
                print(f"    {BY_INDEX[i].name:<20}{math.degrees(q0[i]):>8.2f}°")

            v = float(self.p("speed"))
            cli.engage()

            print()
            to_zero(cli, speed=v)
            cli.sleep(float(self.p("hold")))

            self.tabla(cli, q0)
            peor = max(abs(cli.q(i)) for i in ARM_INDICES)
            print(f"\n  Peor desviación de 0°: {math.degrees(peor):.2f}°")

            if self.p("keep"):
                print("\n  SOSTENIENDO la postura. Ctrl-C para soltar.\n"
                      "  Mientras tanto ningún otro proceso debe publicar en\n"
                      "  /lowcmd: se pelearían por el canal.")
                try:
                    while True:
                        cli.sleep(0.5)
                except KeyboardInterrupt:
                    print("\n  soltando…")
            else:
                print("\n  Se suelta el control: las ganancias bajan a 0 y los\n"
                      "  codos CAERÁN solos hasta quedar colgando. Es normal.\n"
                      "  Si lo que viene después necesita partir de 0°, tiene\n"
                      "  que llamar a `postures.to_zero()` él mismo:\n"
                      "  ver `algorithm_template.py`.")
            # Soltar EN CERO: volver a donde estaba sería deshacerlo.
            cli.release(home_to={i: 0.0 for i in ARM_INDICES})
        return 0


def main(args=None):
    sys.exit(ejecuta(InitPose, "h1_2_init_pose", args))


if __name__ == "__main__":
    main()
