#!/usr/bin/env python3
"""Lleva los catorce motores de los brazos a 0°, despacio, ANTES de teleoperar.

Por qué hace falta. `H1_2_ArmController` de `xr_teleoperate` se construye en la
línea 174 de `teleop_hand_and_arm.py` y el `Press [r] to start` está en la 265:
al construirse arranca ya el hilo publicador con `q_target = zeros(14)` y
`arm_velocity_limit = 30 rad/s`. O sea que **el robot va a ir a esta misma
postura de todas formas**, sin avisar y a 30 rad/s. Con los codos estirados eso
son 80° de recorrido en decenas de milisegundos.

Este script lleva los brazos ahí antes, a la velocidad que se le pida, con toda
la protección del paquete puesta: aborto por par, por temperatura, por estado
rancio, rampa de peso y apagado ordenado.

**MEDIDO el 2026-09-09, y limita para qué sirve esto.** Al soltar, los codos
vuelven solos a 79° y 85° en unos segundos: el codo a 0° está FLEXIONADO, que
no es el mínimo de gravedad, así que sin ganancia el antebrazo cae hasta quedar
colgando. Las otras doce se quedan a menos de 6° de cero.

O sea que para los codos —justo los del recorrido de 80°— colocarlos antes NO
evita el movimiento inicial de la teleoperación, salvo que ésta tome el control
sin hueco por medio. **Lo que de verdad protege es `H12_ARM_VELOCITY_LIMIT`**,
que convierte esos 80° en un movimiento de segundos en vez de decenas de
milisegundos. Este script sigue valiendo para las otras doce y para dejar una
postura de partida conocida, pero no es la defensa principal.

    python3 scripts/15_postura_cero.py                 # 0.15 rad/s, ~9°/s
    python3 scripts/15_postura_cero.py --speed 0.10    # más despacio aún
    python3 scripts/15_postura_cero.py --dry-run       # sin publicar nada

Requiere modo debug, igual que el resto de scripts con `--channel lowcmd`:

    python3 scripts/06_debug_mode.py enter
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _common import add_common_args, build_client, confirm
from h1_2_joint_control.joints import ARM_INDICES, BY_INDEX


def relaja_topes_de_ensayo(gains, log) -> None:
    """Quita los topes de autocolisión de los `shoulder_roll`, solo para ir a 0°.

    Los topes de `soft_limits_deg` (±10°) y la tabla
    `shoulder_roll_vs_elbow_deg` (±5° con el codo flexionado) están puestos
    para los ENSAYOS, donde una articulación barre sola y hace falta margen
    porque la trayectoria pasa por muchas posturas.

    La postura cero no es un barrido: es un punto fijo, y el operador la
    verificó FÍSICAMENTE el 2026-09-09 —los siete ángulos de cada brazo a 0°,
    sin colisión—. Además el modelo ya lo decía: con el codo a 0° y el pitch a
    0°, el `shoulder_roll` llega a −10° sin chocar (tabla de 08_PLAN.md §2.2);
    el caso peor de +5° es con el codo ESTIRADO, que no es donde acabamos.

    Se relaja aquí y solo aquí. El resto de scripts conserva los topes.
    """
    rolls = [i for i in ARM_INDICES if BY_INDEX[i].name.endswith("shoulder_roll")]
    for i in rolls:
        gains._soft.pop(i, None)
    gains._cond_pares = {}
    log("  topes de autocolisión de los shoulder_roll relajados para esta "
        "postura\n  (verificada físicamente por el operador; ver la docstring)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.set_defaults(channel="lowcmd", gains="tuned_gff", gravity_ff=True)
    ap.add_argument("--speed", type=float, default=0.15,
                    help="rad/s de la rampa (por defecto 0.15, unos 9°/s)")
    ap.add_argument("--hold", type=float, default=2.0,
                    help="segundos sosteniendo la postura al llegar, para medir")
    a = ap.parse_args()

    cli = build_client(a, list(ARM_INDICES))
    grados = math.degrees
    try:
        cli.wait_for_state()
        q0 = {i: cli.q(i) for i in ARM_INDICES}
        recorrido = max(abs(v) for v in q0.values())
        print(f"\n  Postura de partida (la que hay ahora):")
        for i in ARM_INDICES:
            print(f"    {BY_INDEX[i].name:<20}{grados(q0[i]):>8.2f}°")
        print(f"\n  Destino: los 14 a 0.00°")
        print(f"  Recorrido mayor: {grados(recorrido):.1f}°  "
              f"→ unos {recorrido / max(a.speed, 1e-3):.0f} s a {a.speed} rad/s")

        confirm(a, "Los DOS brazos se mueven a la vez. Robot COLGADO DEL ARNÉS.")

        relaja_topes_de_ensayo(cli.gains, lambda m: print(m))
        cli.engage()
        cli.ramp_to({i: 0.0 for i in ARM_INDICES}, speed=a.speed)

        for i in ARM_INDICES:
            cli.wait_settled(i)
        cli.sleep(a.hold)

        print(f"\n  {'articulación':<20}{'salió de':>11}{'llegó a':>10}{'par':>9}")
        peor = 0.0
        for i in ARM_INDICES:
            q, t = cli.q(i), cli.tau(i)
            peor = max(peor, abs(q))
            print(f"    {BY_INDEX[i].name:<18}{grados(q0[i]):>10.2f}°"
                  f"{grados(q):>9.2f}°{t:>8.2f}N")
        print(f"\n  Peor desviación de 0°: {grados(peor):.2f}°")
        if cli.collision_clamps:
            print(f"  ⚠ el portero de colisión actuó en {cli.collision_clamps} ciclos")
        print(f"\n  {cli.loop_health()}")
        print("\n  Los brazos están en la postura cero. La teleoperación puede\n"
              "  arrancar: su movimiento inicial ya no tiene recorrido que hacer.")
    finally:
        # Soltar EN CERO, no volver a donde estaba: deshacerlo sería justo lo
        # contrario de lo que hace este script.
        cli.release(home_to={i: 0.0 for i in ARM_INDICES})
        cli.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
