#!/usr/bin/env python3
"""Recorre las 14 articulaciones de los brazos, una a una, y las califica.

Es la respuesta a «¿cada articulación sigue su referencia?». Toma el control
una sola vez y mueve un motor cada vez, dejando los otros trece clavados;
al final saca una tabla con el veredicto por articulación.

Al terminar cada motor vuelve a su postura de partida, así que el brazo acaba
donde empezó.

Ejemplos:
    python3 scripts/04_sweep_arms.py                       # escalón de 0.15 rad
    python3 scripts/04_sweep_arms.py --joints left_arm
    python3 scripts/04_sweep_arms.py --gains xr_teleoperate --traj sine --freq 0.5
    python3 scripts/04_sweep_arms.py --amp 0.10 --settle 2.0
"""
from __future__ import annotations

import argparse
import importlib
import math
import sys
from pathlib import Path

from _common import add_common_args, build_client, confirm
from h1_2_joint_control import config as cfg
from h1_2_joint_control import metrics as mt
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX, resolve

sys.path.insert(0, str(Path(__file__).resolve().parent))
_move = importlib.import_module("02_move")

# Umbrales del veredicto. Deliberadamente flojos: aquí se busca detectar una
# articulación que NO sigue, no discutir el último milirradián.
MAX_STEADY_ERR = 0.030      # rad, error permanente aceptable (~1.7°)
MAX_OVERSHOOT = 0.25        # 25 % de sobreimpulso
MAX_CHATTER = 0.08          # rad/s de dq por encima de 8 Hz


def verdict(track, step) -> tuple[str, str]:
    problems = []
    if step is not None:
        if abs(step.steady_error) > MAX_STEADY_ERR:
            problems.append(f"error final {math.degrees(step.steady_error):+.1f}°")
        if step.overshoot > MAX_OVERSHOOT:
            problems.append(f"sobreimpulso {step.overshoot*100:.0f} %")
        if step.oscillations > 4:
            problems.append(f"{step.oscillations} oscilaciones")
    if track.chatter_dq > MAX_CHATTER:
        problems.append(f"temblor {track.chatter_dq:.3f} rad/s @ {track.peak_freq:.0f} Hz")
    if track.tau_headroom < 0.10:
        problems.append(f"par al {(1-track.tau_headroom)*100:.0f} % del límite")
    return ("✔ OK", "") if not problems else ("⚠", "; ".join(problems))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--joints", default="arms",
                    help="arms (por defecto), left_arm, right_arm, wrist, shoulder…")
    ap.add_argument("--traj", default="step",
                    choices=["step", "smooth_step", "sine", "chirp"])
    ap.add_argument("--amp", type=float, default=0.15)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--cycles", type=float, default=4.0)
    ap.add_argument("--f0", type=float, default=0.2)
    ap.add_argument("--f1", type=float, default=3.0)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--rise", type=float, default=0.3)
    ap.add_argument("--settle", type=float, default=2.5)
    ap.add_argument("--zero-dq", action="store_true")
    ap.add_argument("--pause", type=float, default=1.0)
    a = ap.parse_args()

    targets = resolve(a.joints)
    gains = cfg.load(a.gains)
    per = (a.settle + 1.5) if "step" in a.traj else (a.cycles / a.freq + 1.5)
    print(f"\n  Articulaciones : {len(targets)} -> "
          f"{', '.join(BY_INDEX[i].name for i in targets)}")
    print(f"  Ganancias      : '{gains.set_name}'")
    print(f"  Trayectoria    : {a.traj}, amplitud {a.amp:.3f} rad "
          f"({math.degrees(a.amp):.1f}°) en cada una, de una en una")
    print(f"  Canal          : {a.channel}   lazo {a.rate:.0f} Hz")
    print(f"  Duración aprox.: {len(targets)*(per+a.pause)/60:.1f} min")
    confirm(a)

    cli = build_client(a, targets, verbose=True)
    rows, stamp = [], rec.stamp()
    try:
        cli.wait_for_state()
        cli.engage()

        for n, idx in enumerate(targets, 1):
            j = BY_INDEX[idx]
            q0 = float(cli.q0[idx])
            margin = gains.safety.joint_limit_margin
            amp = a.amp
            note = ""
            if q0 + amp > j.q_max - margin or q0 + amp < j.q_min + margin:
                if j.q_min + margin <= q0 - amp <= j.q_max - margin:
                    amp, note = -amp, "(sentido invertido por el tope)"
                else:
                    room = min(j.q_max - margin - q0, q0 - j.q_min - margin)
                    amp = math.copysign(max(room * 0.8, 0.0), amp)
                    note = "(amplitud recortada por el tope)"
            print(f"\n  [{n:>2}/{len(targets)}] {j.name}  "
                  f"q0={q0:+.3f} rad  amp={amp:+.3f} rad {note}")
            if abs(amp) < 5e-3:
                print("        sin recorrido libre desde esta postura: se salta.")
                continue

            cli.ramp_to({idx: q0}, speed=0.3)
            cli.sleep(a.pause)
            samples, track, step = _move.run_once(cli, idx, a, amp, gains, quiet=True)
            cli.ramp_to({idx: q0}, speed=0.3)

            mark, why = verdict(track, step)
            print(f"        {track.summary()}")
            if step is not None:
                print(f"        {step.summary()}")
            print(f"        {mark} {why}")

            csv = cfg.LOG_DIR / f"sweep_{gains.set_name}_{j.name}_{stamp}.csv"
            rec.save_samples(samples, [idx], csv)
            row = {"stamp": stamp, "test": f"sweep_{a.traj}", "channel": a.channel,
                   "gains": gains.set_name, "rate_hz": a.rate, "amp": amp,
                   "zero_dq": int(a.zero_dq), "csv": csv.name, **track.as_row()}
            if step is not None:
                row.update({f"step_{k}": v for k, v in step.as_row().items()
                            if k not in ("joint", "kp", "kd")})
            rec.append_index(row)
            rows.append((j, track, step, mark, why))

    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
    except KeyboardInterrupt:
        print("\n  interrumpido por el usuario")
    finally:
        cli.__exit__(None, None, None)

    if not rows:
        return 1

    print("\n\n  ═══ Resumen ═══════════════════════════════════════════════════════")
    print(f"  {'articulación':<18} {'kp':>6} {'kd':>5} {'err final':>10} "
          f"{'sobreimp':>9} {'subida':>8} {'temblor':>9} {'tau máx':>9}  veredicto")
    ok = 0
    for j, t, st, mark, why in rows:
        se = f"{math.degrees(st.steady_error):+7.2f}°" if st else "      —"
        ov = f"{st.overshoot*100:7.1f} %" if st else "      —"
        ri = f"{st.rise_time*1000:6.0f}ms" if st and st.rise_time == st.rise_time else "     —"
        print(f"  {j.name:<18} {t.kp:>6.1f} {t.kd:>5.2f} {se:>10} {ov:>9} {ri:>8} "
              f"{t.chatter_dq:>7.4f}  {t.max_tau:>6.2f}/{j.tau_max:.0f} Nm  {mark} {why}")
        ok += mark.startswith("✔")
    print(f"\n  {ok}/{len(rows)} articulaciones dentro de tolerancia.")
    print(f"  Índice acumulado -> {rec.INDEX}")
    print(f"  Gráficas         -> python3 scripts/05_plot.py logs/sweep_*_{stamp}.csv")
    return 0 if ok == len(rows) else 3


if __name__ == "__main__":
    sys.exit(main())
