#!/usr/bin/env python3
"""Devuelve los brazos a la postura de reposo SIN que la mano roce la pierna.

El problema, observado el 2026-09-09 al terminar una sesión: si se sueltan los
brazos con el codo flexionado y el hombro cerca de 0°, el antebrazo cae solo
hasta quedar colgando —0° de codo NO es el mínimo de gravedad, medido: de 1° a
79° y 85° en segundos— y en esa caída **la mano golpea la pierna**.

La secuencia que lo evita tiene tres tramos y el orden es lo único que importa:

  1. el `shoulder_roll` sale a ±10°, apartando la mano de la pierna ANTES de
     que el brazo recorra nada a lo largo del cuerpo;
  2. el codo se estira a 80°, ya lejos de la pierna;
  3. los hombros vuelven a la postura de reposo, con el brazo ya estirado, que
     es el mínimo de gravedad: al bajar las ganancias no cae nada.

Los ángulos salen de `rest_posture_deg` en `config/gains.yaml`.

    python3 scripts/16_postura_reposo.py
    python3 scripts/16_postura_reposo.py --speed 0.20
    python3 scripts/16_postura_reposo.py --dry-run

Es lo que hay que ejecutar DESPUÉS de la teleoperación: `xr_teleoperate` acaba
llamando a `ctrl_dual_arm_go_home()`, que lleva los catorce a 0° y ahí muere el
proceso; sin nadie publicando, los codos caen. Justo el caso que este script
existe para evitar.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _common import (add_common_args, build_client, confirm,
                     relaja_topes_de_ensayo)
from h1_2_joint_control.joints import ARM_INDICES, BY_INDEX


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.set_defaults(channel="lowcmd", gains="tuned_gff", gravity_ff=True)
    ap.add_argument("--speed", type=float, default=0.15,
                    help="rad/s de cada tramo (por defecto 0.15, unos 9°/s)")
    a = ap.parse_args()

    cli = build_client(a, list(ARM_INDICES))
    gr = math.degrees
    try:
        cli.wait_for_state()
        reposo = cli.gains.rest_posture()
        if not reposo:
            raise SystemExit("  no hay `rest_posture_deg` en config/gains.yaml")

        rolls = {i: BY_INDEX[i].name for i in ARM_INDICES
                 if BY_INDEX[i].name.endswith("shoulder_roll")}
        codos = [i for i in ARM_INDICES if BY_INDEX[i].name.endswith("elbow")]
        salida = cli.gains.rest_roll_exit
        # el signo lo pone el lado, igual que en la postura de ensayo
        fuera = {i: math.copysign(salida, 1.0 if n.startswith("L_") else -1.0)
                 for i, n in rolls.items()}

        q0 = {i: cli.q(i) for i in ARM_INDICES}
        print("\n  Postura de partida:")
        for i in ARM_INDICES:
            print(f"    {BY_INDEX[i].name:<20}{gr(q0[i]):>8.2f}°")
        print(f"\n  Secuencia, a {a.speed} rad/s:")
        print(f"    1. shoulder_roll  →  {gr(salida):+.0f}° / {-gr(salida):+.0f}°"
              f"   (aparta la mano de la pierna)")
        print(f"    2. codos          →  {gr(reposo[codos[0]]):.0f}°"
              f"   (estirar, ya lejos de la pierna)")
        print(f"    3. todo lo demás  →  postura de reposo"
              f"   (brazo estirado = mínimo de gravedad)")

        confirm(a, "Los DOS brazos se mueven a la vez. Robot COLGADO DEL ARNÉS.")

        relaja_topes_de_ensayo(cli.gains, lambda m: print(m))
        cli.engage()

        print("\n  1/3 · apartando los hombros…")
        cli.ramp_to(fuera, speed=a.speed)

        print("  2/3 · estirando los codos…")
        cli.ramp_to({i: reposo[i] for i in codos}, speed=a.speed)

        print("  3/3 · a la postura de reposo…")
        cli.ramp_to(reposo, speed=a.speed)
        for i in ARM_INDICES:
            cli.wait_settled(i)

        print(f"\n  {'articulación':<20}{'salió de':>11}{'llegó a':>10}"
              f"{'reposo':>9}{'par':>9}")
        for i in ARM_INDICES:
            print(f"    {BY_INDEX[i].name:<18}{gr(q0[i]):>10.2f}°"
                  f"{gr(cli.q(i)):>9.2f}°{gr(reposo[i]):>8.1f}°{cli.tau(i):>8.2f}N")
        if cli.collision_clamps:
            print(f"\n  ⚠ el portero de colisión actuó en {cli.collision_clamps} ciclos")
        print(f"\n  {cli.loop_health()}")
        print("\n  Brazos estirados y pegados al cuerpo. Al bajar las ganancias\n"
              "  no hay nada que caiga: es donde la gravedad los deja.")
    finally:
        # Soltar EN LA POSTURA DE REPOSO. Volver a q0 sería deshacerlo, y q0
        # aquí es justo la postura peligrosa de la que venimos.
        try:
            cli.release(home_to=cli.gains.rest_posture())
        except Exception:
            cli.release(home_to={})
        cli.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
