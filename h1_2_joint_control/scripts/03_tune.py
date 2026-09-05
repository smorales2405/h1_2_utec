#!/usr/bin/env python3
"""Barrido de kp/kd sobre UNA articulación, con criterio explícito.

Para cada par (kp, kd) del barrido ejecuta la misma trayectoria, mide y
ordena. Entre candidato y candidato vuelve a la postura de partida, así que
todos compiten en igualdad de condiciones.

El criterio por defecto (ver `metrics.cost`) mezcla tres cosas:
    error rms de seguimiento (mrad) + temblor (dq de alta frecuencia)
    + sobreimpulso del escalón.
Los pesos se ven y se cambian: si lo que molesta es la vibración,
`--w-chatter 3`.

Ejemplos:
    # rejilla alrededor de lo que trae el ejemplo oficial para el codo
    python3 scripts/03_tune.py --joint L_elbow --kp-list 50,80,110,140 --kd-list 1,2,3

    # solo kd, dejando kp en el valor de xr_teleoperate
    python3 scripts/03_tune.py --joint L_elbow --kp-list 140 --kd-list 1,2,3,4,5

    # sobre seguimiento sinusoidal en vez de escalón
    python3 scripts/03_tune.py --joint L_wrist_pitch --traj sine --freq 1.0 \
        --kp-list 30,50,70 --kd-list 0.5,1,2

Escribe el mejor par en el conjunto `tuned` de config/gains.yaml con `--write`.
"""
from __future__ import annotations

import argparse
import itertools
import math
import sys

from _common import (add_common_args, apply_test_posture, build_client, confirm,
                     describe, joint_index, pick_amplitude)
from h1_2_joint_control import config as cfg
from h1_2_joint_control import metrics as mt
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX

# `02_move` empieza por dígito, así que no vale un `import` normal.
import importlib  # noqa: E402
from pathlib import Path  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent))
_move = importlib.import_module("02_move")


def floats(spec: str) -> list[float]:
    return [float(x) for x in spec.replace(" ", "").split(",") if x]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--joint", required=True)
    ap.add_argument("--kp-list", required=True, help="p. ej. 50,80,110,140")
    ap.add_argument("--kd-list", required=True, help="p. ej. 1,2,3")
    ap.add_argument("--traj", default="step",
                    choices=["step", "smooth_step", "sine", "chirp"])
    ap.add_argument("--amp", type=float, default=0.15)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--cycles", type=float, default=5.0)
    ap.add_argument("--f0", type=float, default=0.2)
    ap.add_argument("--f1", type=float, default=3.0)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--rise", type=float, default=0.3)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--zero-dq", action="store_true")
    ap.add_argument("--repeats", type=int, default=1,
                    help="repeticiones por candidato; se promedian las métricas")
    ap.add_argument("--pause", type=float, default=1.0,
                    help="s de reposo entre candidatos")
    ap.add_argument("--w-err", type=float, default=1.0)
    ap.add_argument("--w-chatter", type=float, default=1.0)
    ap.add_argument("--w-overshoot", type=float, default=0.5)
    ap.add_argument("--write", metavar="CONJUNTO", default=None,
                    help="guardar el ganador en ese conjunto de gains.yaml "
                         "(normalmente: tuned)")
    a = ap.parse_args()

    idx = joint_index(a.joint)
    j = BY_INDEX[idx]
    kps, kds = floats(a.kp_list), floats(a.kd_list)
    grid = list(itertools.product(kps, kds))
    gains = cfg.load(a.gains)

    print("\n" + describe(idx, gains))
    print(f"\n    barrido        : {len(grid)} candidatos "
          f"(kp {kps} × kd {kds}), {a.repeats} repetición(es)")
    print(f"    trayectoria    : {a.traj}, amplitud {a.amp:.3f} rad")
    print(f"    criterio       : {a.w_err}·err_rms[mrad] + {a.w_chatter}·temblor[10⁻²rad/s]"
          f" + {a.w_overshoot}·sobreimpulso[%]")
    est = len(grid) * a.repeats * ((a.settle + 1.0) if "step" in a.traj
                                   else (a.cycles / a.freq + 1.0)) + len(grid) * a.pause
    print(f"    duración aprox.: {est/60:.1f} min")
    confirm(a)

    cli = build_client(a, [idx], verbose=False)
    results = []
    stamp = rec.stamp()
    try:
        cli.wait_for_state()
        cli.engage()
        apply_test_posture(cli, a, gains)
        q0 = float(cli.q_base[idx])
        amp, nota = pick_amplitude(idx, q0, a.amp, gains.limits(idx),
                                   gains.direction(idx, a.direction))
        print(f"  ensayo: {math.degrees(q0):+.1f}° -> "
              f"{math.degrees(q0 + amp):+.1f}°  {nota}\n")
        for n, (kp, kd) in enumerate(grid, 1):
            gains.set_index(idx, kp, kd)
            cli.set_gains(idx, kp, kd)
            # volver siempre al mismo punto: si no, cada candidato arrancaría
            # desde donde lo dejó el anterior y no serían comparables
            cli.ramp_to({idx: q0}, speed=0.3)
            cli.sleep(a.pause)

            tracks, steps = [], []
            for r in range(a.repeats):
                samples, track, step = _move.run_once(cli, idx, a, amp, gains,
                                                      quiet=True)
                tracks.append(track)
                if step is not None:
                    steps.append(step)
                csv = cfg.LOG_DIR / (f"tune_{j.name}_kp{kp:.0f}_kd{kd:.1f}"
                                     f"_r{r}_{stamp}.csv")
                rec.save_samples(samples, [idx], csv)
                if r + 1 < a.repeats:
                    cli.ramp_to({idx: q0}, speed=0.3)
                    cli.sleep(a.pause)

            track = tracks[0] if len(tracks) == 1 else _mean_track(tracks)
            step = steps[0] if len(steps) == 1 else (_mean_step(steps) if steps else None)
            J = mt.cost(track, step, a.w_err, a.w_chatter, a.w_overshoot)
            results.append((J, kp, kd, track, step))
            bar = "█" * int(min(J, 60))
            print(f"  [{n:>2}/{len(grid)}] kp={kp:6.1f} kd={kd:5.2f}  J={J:7.2f} {bar}")
            print(f"          {track.summary().split('│',1)[1].strip()}")
            if step is not None:
                print(f"          {step.summary().split('│',1)[1].strip()}")
            rec.append_index({"stamp": stamp, "test": f"tune_{a.traj}",
                              "channel": a.channel, "gains": "sweep",
                              "rate_hz": a.rate, "amp": amp, "cost": J,
                              "zero_dq": int(a.zero_dq), "csv": "",
                              **track.as_row()})

        cli.ramp_to({idx: q0}, speed=0.3)

    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
    except KeyboardInterrupt:
        print("\n  interrumpido por el usuario")
    finally:
        cli.__exit__(None, None, None)

    if not results:
        return 1

    results.sort(key=lambda r: r[0])
    print("\n  ── Clasificación ──────────────────────────────────────────")
    print(f"  {'#':>2} {'kp':>7} {'kd':>6} {'J':>8} {'err rms':>10} "
          f"{'temblor':>10} {'sobreimp':>9} {'tau máx':>9}")
    for n, (J, kp, kd, t, st) in enumerate(results, 1):
        ov = f"{st.overshoot*100:7.1f} %" if st else "        —"
        print(f"  {n:>2} {kp:>7.1f} {kd:>6.2f} {J:>8.2f} "
              f"{t.rms_error*1000:>7.2f} mrad {t.chatter_dq:>8.4f} {ov} "
              f"{t.max_tau:>7.2f} Nm")

    J, kp, kd, t, st = results[0]
    print(f"\n  Mejor: kp={kp:.1f}  kd={kd:.2f}   (J={J:.2f})")
    if kp in (min(floats(a.kp_list)), max(floats(a.kp_list))) and len(kps) > 1:
        print("  ⚠ el kp ganador está en el borde del barrido: amplía el rango.")
    if kd in (min(floats(a.kd_list)), max(floats(a.kd_list))) and len(kds) > 1:
        print("  ⚠ el kd ganador está en el borde del barrido: amplía el rango.")

    if a.write:
        target = cfg.load(a.write)
        target.set_index(idx, kp, kd)
        target.write_into(a.write, note=target.description)
        print(f"  escrito en config/gains.yaml -> sets.{a.write}.{j.name} "
              f"= {{kp: {kp}, kd: {kd}}}")
    else:
        print(f"  Para guardarlo: añade --write tuned")
    return 0


def _mean_track(ts):
    import statistics as st
    base = ts[0]
    fields = ("rms_error", "max_error", "mean_error", "chatter_dq", "chatter_tau",
              "peak_freq", "rms_tau", "max_tau", "tau_headroom")
    kw = {f: st.mean(getattr(t, f) for t in ts) for f in fields}
    return mt.TrackingMetrics(joint=base.joint, kp=base.kp, kd=base.kd,
                              n=base.n, fs=base.fs, **kw)


def _mean_step(ss):
    import statistics as st
    base = ss[0]
    fields = ("amplitude", "rise_time", "overshoot", "settling_time",
              "steady_error", "peak_tau")
    kw = {f: st.mean(getattr(s, f) for s in ss) for f in fields}
    kw["oscillations"] = round(st.mean(s.oscillations for s in ss))
    return mt.StepMetrics(joint=base.joint, kp=base.kp, kd=base.kd, **kw)


if __name__ == "__main__":
    sys.exit(main())
