#!/usr/bin/env python3
"""Plantilla: colocar, ejecutar tu algoritmo, y devolver. En UN proceso.

═══════════════════════════════════════════════════════════════════════════
POR QUÉ NO SIRVE ENCADENAR COMANDOS
═══════════════════════════════════════════════════════════════════════════

Lo natural sería:

    ros2 run h1_2_arm_control init_pose
    ros2 run mi_paquete mi_algoritmo          # <-- aquí el brazo YA se cayó
    ros2 run h1_2_arm_control rest_pose

y no funciona. Cuando `init_pose` termina, baja las ganancias a cero y el
proceso muere. Sin nadie publicando, la gravedad se lleva los codos: medido,
de 1° a 79° y 85° en unos segundos. El siguiente comando tarda varios segundos
en arrancar —crear el nodo, esperar el primer `/lowstate`, enganchar en
rampa— y para entonces el brazo ya no está donde lo dejaron.

No es un fallo que se pueda tapar: mientras nadie mande, el brazo cae. La
única forma de que no haya hueco es que el mismo proceso que coloca sea el que
ejecuta y el que devuelve. Eso es lo que hace esta plantilla.

═══════════════════════════════════════════════════════════════════════════
CÓMO USARLA
═══════════════════════════════════════════════════════════════════════════

Copia este fichero a tu paquete, cambia `mi_algoritmo()` por lo tuyo y añade
la entrada en tu `setup.py`. El `with` y las dos secuencias de postura se
quedan como están: son lo que evita que la mano golpee la pierna al terminar.

    ros2 run h1_2_arm_control algorithm_template
    ros2 run h1_2_arm_control algorithm_template --ros-args -p dry_run:=true

═══════════════════════════════════════════════════════════════════════════
"""
import math
import sys

from ._node_base import ArmNode, ejecuta
from .joints import ARM_INDICES, BY_INDEX, BY_NAME
from .postures import to_rest, to_zero
from . import trajectories as traj


def mi_algoritmo(cli, log) -> None:
    """AQUÍ VA LO TUYO. El brazo llega a esta función colocado en 0° y firme.

    `cli` es el `H12Client`, ya enganchado. Lo que tienes a mano:

        cli.q(i), cli.dq(i), cli.tau(i)      estado medido de la articulación i
        cli.q_all()                          los 27 en un array
        cli.set_target(i, q)                 consigna inmediata
        cli.set_targets({i: q, ...})         varias a la vez
        cli.ramp_to({i: q}, speed=0.15)      rampa suave, bloqueante
        cli.set_trajectory(i, f)             f(t) -> (q, dq), evaluada en el
                                             hilo de control contra reloj
        cli.clear_trajectory(i)
        cli.wait_settled(i)                  espera a que se quede quieta
        cli.sleep(s)                         duerme vigilando la seguridad

    Los índices salen de `BY_NAME["L_elbow"].idx` o de `ARM_INDICES`.

    No hace falta que atrapes excepciones: si salta la protección de par o de
    temperatura, el `with` de fuera suelta el brazo en rampa igualmente.
    """
    codo = BY_NAME["L_elbow"].idx

    log("ejemplo 1: llevar el codo a 30° y volver")
    cli.ramp_to({codo: math.radians(30.0)}, speed=0.20)
    cli.wait_settled(codo)
    log(f"   llegó a {math.degrees(cli.q(codo)):.2f}°, "
        f"par {cli.tau(codo):.2f} Nm")
    cli.ramp_to({codo: 0.0}, speed=0.20)
    cli.wait_settled(codo)

    log("ejemplo 2: un seno de 7° a 0.5 Hz, tres ciclos")
    cli.set_trajectory(codo, traj.sine(math.radians(7.0), 0.5))
    cli.sleep(3.0 / 0.5)
    cli.clear_trajectory(codo)
    cli.wait_settled(codo)
    log(f"   acabó en {math.degrees(cli.q(codo)):.2f}°")


class AlgorithmTemplate(ArmNode):
    def __init__(self, nombre="h1_2_algorithm"):
        super().__init__(nombre, {"hold": 0.5})

    def run(self) -> int:
        log = lambda m: print(f"  {m}")
        with self.cliente() as cli:
            cli.wait_for_state()
            v = float(self.p("speed"))
            cli.engage()

            print("\n── colocando en 0° ──────────────────────────────────")
            to_zero(cli, speed=v)
            cli.sleep(float(self.p("hold")))
            peor = max(abs(cli.q(i)) for i in ARM_INDICES)
            log(f"peor desviación de 0°: {math.degrees(peor):.2f}°")

            print("\n── algoritmo ────────────────────────────────────────")
            mi_algoritmo(cli, log)

            print("\n── devolviendo a reposo ─────────────────────────────")
            reposo = to_rest(cli, speed=v)
            if cli.collision_clamps:
                log(f"⚠ autocolisión: consigna recortada en "
                    f"{cli.collision_clamps} ciclos")
            deriva, idx = cli.leg_drift()
            if deriva > math.radians(2.0):
                log(f"⚠ las piernas se movieron: {BY_INDEX[idx].name} "
                    f"{math.degrees(deriva):.1f}°")
            log(cli.loop_health())
            # Soltar EN REPOSO: es el mínimo de gravedad, así que al bajar las
            # ganancias no cae nada.
            cli.release(home_to=reposo)
        return 0


def main(args=None):
    sys.exit(ejecuta(AlgorithmTemplate, "h1_2_algorithm", args))


if __name__ == "__main__":
    main()
