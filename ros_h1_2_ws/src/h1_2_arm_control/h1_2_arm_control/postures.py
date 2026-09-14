"""Las dos secuencias de postura, como funciones reutilizables.

Están aquí y no dentro de los nodos porque hacen falta en dos sitios: en
`init_pose`/`rest_pose`, que son un comando cada uno, y dentro de un algoritmo
que quiera colocar el brazo y devolverlo **sin soltar el control por medio**.

Y eso último no es una comodidad, es un requisito. Al soltar, las ganancias
bajan a cero y la gravedad se lleva los codos: medido, de 1° a 79° y 85° en
unos segundos. Entre `ros2 run ... init_pose` y el siguiente comando hay un
hueco de varios segundos —crear el nodo, esperar el primer `/lowstate`,
enganchar en rampa— y para cuando el segundo proceso manda, el brazo ya se ha
caído. Un algoritmo que necesite partir de 0° tiene que llamar a `to_zero()`
con su propio cliente, no confiar en que otro proceso lo dejó ahí.
"""
from __future__ import annotations

import math

from .joints import ARM_INDICES, BY_INDEX


def _grupos(indices):
    rolls = [i for i in indices if BY_INDEX[i].name.endswith("shoulder_roll")]
    codos = [i for i in indices if BY_INDEX[i].name.endswith("elbow")]
    resto = [i for i in indices if i not in rolls and i not in codos]
    return rolls, codos, resto


def to_zero(cli, speed: float = 0.15, verbose: bool = True) -> None:
    """Los catorce a 0°, en el único orden que no topa.

    El tope de autocolisión del hombro DEPENDE DEL CODO: ±10° con el brazo
    estirado, 0° con el codo flexionado. Así que hay que flexionar primero. En
    un solo tramo los hombros se quedan clavados en ±10° sin llegar a cero.

    El cliente ya tiene que estar enganchado.
    """
    rolls, codos, resto = _grupos(ARM_INDICES)
    if verbose:
        print("  1/3 · flexionando los codos…")
    cli.ramp_to({i: 0.0 for i in codos}, speed=speed)
    if verbose:
        print("  2/3 · muñecas y el resto…")
    cli.ramp_to({i: 0.0 for i in resto}, speed=speed)
    if verbose:
        print("  3/3 · hombros a 0°, ya con el codo flexionado…")
    cli.ramp_to({i: 0.0 for i in rolls}, speed=speed)
    for i in ARM_INDICES:
        cli.wait_settled(i)


def to_rest(cli, speed: float = 0.15, verbose: bool = True) -> dict[int, float]:
    """A la postura de reposo sin que la mano roce la pierna.

    Soltar los brazos en 0° es lo peor posible: 0° de codo es la posición
    FLEXIONADA y no es el mínimo de gravedad, así que sin ganancia el antebrazo
    cae solo hasta quedar colgando y en esa caída la mano recorre la pierna.

    Tres tramos, y el orden es lo único que importa:

      1. los hombros salen a ±10°, apartando la mano ANTES de que el brazo
         recorra nada a lo largo del cuerpo;
      2. los codos se estiran, ya lejos de la pierna;
      3. todo a la postura de reposo, con el brazo ya estirado, que ES el
         mínimo de gravedad: al soltar no cae nada.

    Medido: soltando en 0° la deriva es de 84°; soltando aquí, de 1.7°.

    Devuelve la postura de reposo, para pasarla a `release(home_to=...)`.
    """
    reposo = cli.gains.rest_posture()
    if not reposo:
        raise SystemExit("  no hay `rest_posture_deg` en config/gains.yaml")
    rolls, codos, _ = _grupos(ARM_INDICES)
    salida = cli.gains.rest_roll_exit
    fuera = {i: math.copysign(salida, 1.0 if BY_INDEX[i].name.startswith("L_") else -1.0)
             for i in rolls}
    if verbose:
        print(f"  1/3 · apartando los hombros a ±{math.degrees(salida):.0f}°…")
    cli.ramp_to(fuera, speed=speed)
    if verbose:
        print("  2/3 · estirando los codos…")
    cli.ramp_to({i: reposo[i] for i in codos}, speed=speed)
    if verbose:
        print("  3/3 · a la postura de reposo…")
    cli.ramp_to(reposo, speed=speed)
    for i in ARM_INDICES:
        cli.wait_settled(i)
    return reposo
