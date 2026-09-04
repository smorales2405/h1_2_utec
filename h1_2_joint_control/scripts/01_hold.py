#!/usr/bin/env python3
"""Primer paso con el robot: tomar el control de los brazos SIN moverlos.

No comanda ningún movimiento. Sube el peso de `arm_sdk` de 0 a 1 con la
consigna clavada en la postura actual y observa qué pasa. Es la prueba que hay
que superar antes de intentar mover nada:

  · Si el brazo NO se mueve y la deriva se queda en milirradianes, el canal
    funciona, las ganancias sostienen el brazo y podemos seguir.
  · Si el brazo se descuelga, kp es demasiado bajo para vencer la gravedad
    en esa postura.
  · Si el brazo tiembla, kd está mal o hay alguien más mandando (mira el
    diagnóstico: `python3 scripts/00_diagnose.py`).

Este script NO aplica la postura de ensayo de `config/gains.yaml`, ni siquiera
sin `--no-posture`: sostener la postura ACTUAL es justamente lo que se está
probando, y moverla antes lo desvirtuaría. Los que sí la aplican son
`02_move.py`, `03_tune.py`, `04_sweep_arms.py` y `07_gravity_ff.py`.

Uso:
    source scripts/env.sh
    python3 scripts/01_hold.py --seconds 10
    python3 scripts/01_hold.py --seconds 10 --gains xr_teleoperate
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from _common import add_common_args, build_client, confirm
from h1_2_joint_control import config as cfg
from h1_2_joint_control import metrics as mt
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import ARM_INDICES, BY_INDEX


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="cuánto mantener la postura")
    ap.add_argument("--joints", default="arms",
                    help="qué vigilar: arms, left_arm, wrist, L_elbow, ...")
    a = ap.parse_args()

    from h1_2_joint_control.joints import resolve
    watched = resolve(a.joints)

    g = cfg.load(a.gains)
    print(f"\n  Conjunto de ganancias: '{g.set_name}'")
    print(f"  Canal: {a.channel}   lazo a {a.rate:.0f} Hz")
    print(f"  Se vigilan: {', '.join(BY_INDEX[i].name for i in watched)}")
    confirm(a, "  Este script NO comanda movimiento: solo sostiene la postura actual.\n")

    cli = build_client(a, watched)
    try:
        cli.wait_for_state()
        q_before = cli.q_all()
        cli.engage()
        cli.record(True)

        t0 = time.monotonic()
        print(f"\n  Manteniendo {a.seconds:.0f} s. Ctrl-C para salir antes.\n")
        while time.monotonic() - t0 < a.seconds:
            q = cli.q_all()
            drift = np.abs(q[watched] - cli.q0[watched])
            worst = int(np.argmax(drift))
            print(f"    t={time.monotonic()-t0:5.1f} s   deriva máx "
                  f"{drift[worst]*1000:6.2f} mrad en "
                  f"{BY_INDEX[watched[worst]].name:<17}  "
                  f"|tau| máx {max(abs(cli.tau(i)) for i in watched):5.2f} Nm   ",
                  end="\r", flush=True)
            cli.sleep(0.25)
        print()

        samples = cli.record(False)
        print(f"\n  {'articulación':<18} {'deriva':>10} {'temblor dq':>13} "
              f"{'f pico':>8} {'tau rms':>9}")
        rows = []
        for i in watched:
            j = BY_INDEX[i]
            kp, kd = g.for_index(i)
            m = mt.tracking(samples, i, kp, kd, j.tau_max, skip=0.5)
            drift = abs(cli.q(i) - cli.q0[i])
            flag = "  ⚠" if m.chatter_dq > 0.05 else ""
            print(f"  {j.name:<18} {drift*1000:>7.2f} mrad {m.chatter_dq:>10.4f} rad/s "
                  f"{m.peak_freq:>6.1f} Hz {m.rms_tau:>7.2f} Nm{flag}")
            rows.append(m)

        stamp = rec.stamp()
        tag = f"_{a.tag}" if a.tag else ""
        csv = cfg.LOG_DIR / f"hold_{g.set_name}{tag}_{stamp}.csv"
        rec.save_samples(samples, watched, csv)
        for m in rows:
            rec.append_index({"stamp": stamp, "test": "hold", "channel": a.channel,
                              "gains": g.set_name, "rate_hz": a.rate,
                              "csv": csv.name, **m.as_row()})
        print(f"\n  registro -> {csv}")

        moved = float(np.max(np.abs(cli.q_all()[watched] - q_before[watched])))
        print(f"\n  Desplazamiento total respecto al inicio: {moved*1000:.2f} mrad "
              f"({math.degrees(moved):.2f}°)")
        if moved < 0.02:
            print("  ✔ El control sostiene la postura. Se puede pasar a mover.")
        else:
            print("  ⚠ El brazo se ha desplazado. Revisa kp antes de comandar nada.")
        return 0

    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
        return 2
    except KeyboardInterrupt:
        print("\n  interrumpido por el usuario")
        return 1
    finally:
        cli.__exit__(None, None, None)


if __name__ == "__main__":
    sys.exit(main())
