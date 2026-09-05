#!/usr/bin/env python3
"""Mide el par de gravedad de una articulación y comprueba que corregirlo
elimina el error permanente.

Por qué existe. El PD del motor no tiene término integral:

    tau = kp·(q_des − q) + kd·(dq_des − dq) + tau_ff

Sosteniendo una postura contra la gravedad, `dq = 0` y hace falta un par
`tau_g` constante. La única forma de que el PD lo genere es un error
permanente `e = tau_g / kp`, que **no se va nunca**. En el H1-2 esto se ve
sobre todo en `shoulder_roll`, que carga con el brazo entero: con kp = 140 y
15 Nm de gravedad, son 6.1° de retraso.

Subir kp no es la respuesta: haría falta kp ≈ 900 para bajar de 1°, y con eso
cualquier error de 0.045 rad saturaría el motor. La respuesta es `tau_ff`.

Qué hace:
    1. lleva la articulación al objetivo y mide el par que sostiene esa postura
    2. vuelve al inicio
    3. repite el mismo movimiento con ese par como `tau_ff`
    4. compara el error permanente de las dos pasadas

`xr_teleoperate` ya tiene el hueco: `H1_2_ArmController.ctrl_dual_arm(q, tauff)`
acepta el par por articulación; hoy la teleoperación le pasa ceros.

Uso:
    python3 scripts/07_gravity_ff.py --channel lowcmd --joint R_shoulder_roll
    python3 scripts/07_gravity_ff.py --channel lowcmd --joints arms --amp 0.12
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from _common import (add_common_args, apply_test_posture, build_client, confirm,
                     pick_amplitude)
from h1_2_joint_control import config as cfg
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX, resolve


def hold_and_measure(cli, idx, q_target, tau_ff, seconds, speed=0.3, ff_ramp=1.5):
    """Lleva la consigna a `q_target` y mide el régimen permanente.

    El `tau_ff` se mete DESPUÉS de llegar, y en rampa. Aplicarlo desde el
    principio lo sumaría al par que el PD ya está generando para recorrer el
    camino, y el total puede pasarse: en la primera versión de esto,
    R_shoulder_roll llegó a 28.7 Nm y saltó la protección.
    """
    cli.set_target(idx, cli.q_des(idx), tau_ff=0.0)
    cli.ramp_to({idx: q_target}, speed=speed)
    if abs(tau_ff) > 1e-6:
        steps = max(int(ff_ramp / 0.02), 1)
        for k in range(1, steps + 1):
            cli.set_target(idx, q_target, tau_ff=tau_ff * k / steps)
            cli.sleep(0.02)
    else:
        cli.set_target(idx, q_target, tau_ff=0.0)
    cli.sleep(seconds * 0.5)          # transitorio, se descarta
    taus, errs = [], []
    t_end = seconds * 0.5
    n = int(t_end / 0.02)
    for _ in range(max(n, 5)):
        taus.append(cli.tau(idx))
        errs.append(q_target - cli.q(idx))
        cli.sleep(0.02)
    return float(np.mean(taus)), float(np.mean(errs))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--joint", default=None, help="una articulación")
    ap.add_argument("--joints", default=None, help="varias: arms, shoulder, left_arm…")
    ap.add_argument("--amp", type=float, default=0.12,
                    help="desplazamiento desde la postura actual, en rad")
    ap.add_argument("--hold", type=float, default=2.0,
                    help="s de medida en cada postura")
    a = ap.parse_args()

    if not a.joint and not a.joints:
        raise SystemExit("hace falta --joint o --joints")
    targets = resolve(a.joints) if a.joints else resolve(a.joint)
    gains = cfg.load(a.gains)

    print(f"\n  Articulaciones : {', '.join(BY_INDEX[i].name for i in targets)}")
    print(f"  Ganancias      : '{gains.set_name}'")
    print(f"  Desplazamiento : {a.amp:+.3f} rad ({math.degrees(a.amp):+.1f}°)")
    confirm(a)

    cli = build_client(a, targets)
    rows = []
    stamp = rec.stamp()
    try:
        cli.wait_for_state()
        cli.engage()
        apply_test_posture(cli, a, gains)

        for n, idx in enumerate(targets, 1):
            j = BY_INDEX[idx]
            kp, kd = gains.for_index(idx)
            q0 = float(cli.q_base[idx])
            amp, _nota = pick_amplitude(idx, q0, a.amp, gains.limits(idx),
                                   gains.direction(idx, a.direction))
            if abs(amp) < 5e-3:
                print(f"  [{n}/{len(targets)}] {j.name}: sin recorrido, se salta.")
                continue
            q_target = q0 + amp
            print(f"\n  [{n}/{len(targets)}] {j.name}  kp={kp:.0f} kd={kd:.1f}  "
                  f"{q0:+.3f} -> {q_target:+.3f} rad")

            # pasada 1: sin compensar
            tau_g, err0 = hold_and_measure(cli, idx, q_target, 0.0, a.hold)
            print(f"        sin tau_ff : error {err0*1000:+7.2f} mrad "
                  f"({math.degrees(err0):+.2f}°)   par medido {tau_g:+6.2f} Nm")

            cli.set_target(idx, cli.q_des(idx), tau_ff=0.0)
            cli.ramp_to({idx: q0}, speed=0.3)
            cli.sleep(0.5)

            # pasada 2: con el par medido como feedforward
            _, err1 = hold_and_measure(cli, idx, q_target, tau_g, a.hold)
            print(f"        con tau_ff : error {err1*1000:+7.2f} mrad "
                  f"({math.degrees(err1):+.2f}°)   tau_ff = {tau_g:+.2f} Nm")

            mejora = (1 - abs(err1) / abs(err0)) * 100 if abs(err0) > 1e-6 else 0.0
            teorico = tau_g / kp
            print(f"        error previsto por el modelo tau_g/kp = "
                  f"{teorico*1000:+.2f} mrad   |   mejora {mejora:.0f} %")

            cli.set_target(idx, cli.q_des(idx), tau_ff=0.0)
            cli.ramp_to({idx: q0}, speed=0.3)
            cli.sleep(0.3)

            rows.append((j, kp, kd, q0, q_target, tau_g, err0, err1, teorico))
            rec.append_index({"stamp": stamp, "test": "gravity_ff", "channel": a.channel,
                              "gains": gains.set_name, "joint": j.name, "kp": kp, "kd": kd,
                              "amp": amp, "tau_g": tau_g, "err_sin_ff": err0,
                              "err_con_ff": err1, "err_teorico": teorico})

    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
    except KeyboardInterrupt:
        print("\n  interrumpido")
    finally:
        cli.__exit__(None, None, None)

    if not rows:
        return 1
    print("\n\n  ═══ Par de gravedad y efecto de compensarlo ══════════════════════")
    print(f"  {'articulación':<18} {'kp':>6} {'q':>8} {'tau_g':>8} "
          f"{'err sin ff':>11} {'err con ff':>11} {'tau_g/kp':>10} {'mejora':>8}")
    for j, kp, kd, q0, qt, tg, e0, e1, teo in rows:
        mej = (1 - abs(e1) / abs(e0)) * 100 if abs(e0) > 1e-6 else 0.0
        print(f"  {j.name:<18} {kp:>6.0f} {qt:>8.3f} {tg:>7.2f}N "
              f"{math.degrees(e0):>+10.2f}° {math.degrees(e1):>+10.2f}° "
              f"{math.degrees(teo):>+9.2f}° {mej:>7.0f} %")
    print("\n  `tau_g` es el par que hay que meter en `tauff_target` de")
    print("  H1_2_ArmController para que esa articulación deje de ir retrasada")
    print("  EN ESA POSTURA. Depende de la configuración del brazo: para")
    print("  teleoperar hace falta un modelo (pinocchio ya está en el entorno")
    print("  de xr_teleoperate), no una constante.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
