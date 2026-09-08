#!/usr/bin/env python3
"""Repetibilidad y `delta_min` (fase F1), con aislamiento del contexto.

Por qué esta versión y no la del protocolo tal cual. F1 propone un diseño
frío/precalentado para contrastar la hipótesis de fricción estática de §6. Pero
lo que se observó el 2026-09-08 es mucho más grande que eso: el MISMO ensayo con
las MISMAS ganancias dio 0.8 %, 3.0 % y 29.5 % de sobreimpulso según se corriera
aislado, dentro de un barrido de una articulación, o dentro de uno de siete.
Un efecto de 30x no lo explica la temperatura.

Así que se mide primero lo que hay: cuánto varía el ensayo canónico repetido, y
qué parte de esa varianza la mete cada elemento del contexto.

Bloques
    base       N repeticiones seguidas, sin tocar nada más. Es el suelo de ruido.
    ganancias  entre repetición y repetición se cambian las ganancias y se
               vuelven a poner. Reproduce lo que hace un barrido de kp.
    vecinas    entre repetición y repetición se mueven las OTRAS articulaciones
               del brazo. Reproduce el contexto de un barrido de siete.

`delta_min = 1.5 x IQR` del bloque `base`: por debajo de eso, ninguna diferencia
entre candidatos es defendible.

Uso:
    python3 scripts/06_debug_mode.py enter
    python3 scripts/14_repeatability.py --channel lowcmd --joint L_shoulder_roll --n 8
"""
from __future__ import annotations

import argparse
import importlib
import math
import statistics as st
import sys
from pathlib import Path

import numpy as np

from _common import (add_common_args, apply_test_posture, build_client, confirm,
                     joint_index, pick_amplitude)
from h1_2_joint_control import config as cfg
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX, resolve

sys.path.insert(0, str(Path(__file__).resolve().parent))
_move = importlib.import_module("02_move")


class Args:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def iqr(v):
    if len(v) < 4:
        return float(max(v) - min(v)) if v else 0.0
    q1, q3 = np.percentile(v, [25, 75])
    return float(q3 - q1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--joint", default="L_shoulder_roll")
    ap.add_argument("--n", type=int, default=8, help="repeticiones por bloque")
    ap.add_argument("--blocks", default="base,ganancias,vecinas")
    ap.add_argument("--amp", type=float, default=0.12)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--pause", type=float, default=0.4)
    a = ap.parse_args()

    idx = joint_index(a.joint)
    j = BY_INDEX[idx]
    gains = cfg.load(a.gains)
    kp0, kd0 = gains.for_index(idx)
    bloques = [b.strip() for b in a.blocks.split(",") if b.strip()]
    brazo = "left_arm" if j.side == "left" else "right_arm"
    vecinas = [i for i in resolve(brazo) if i != idx]

    print(f"\n  Articulación   : {j.name}  (kp={kp0:.0f} kd={kd0:.2f})")
    print(f"  Bloques        : {', '.join(bloques)} × {a.n} repeticiones")
    print(f"  Ensayo canónico: escalón suave de {a.amp:.3f} rad")
    print(f"  Gravedad       : {'COMPENSADA' if a.gravity_ff else 'sin compensar'}")
    est = len(bloques) * a.n * (a.settle + 3.5)
    print(f"  Duración aprox.: {est/60:.1f} min")
    confirm(a)

    base = Args(traj="smooth_step", amp=a.amp, freq=0.5, cycles=3,
                settle=a.settle, rise=0.3, f0=0.2, f1=3.0, duration=10.0,
                zero_dq=False, pause=a.pause)

    cli = build_client(a, resolve(brazo))
    res: dict[str, list] = {b: [] for b in bloques}
    stamp = rec.stamp()
    try:
        cli.wait_for_state()
        cli.engage()
        apply_test_posture(cli, a, gains)
        q0 = float(cli.q_base[idx])
        amp, _ = pick_amplitude(idx, q0, a.amp, gains.limits(idx),
                                gains.direction(idx, a.direction))
        print(f"\n  {math.degrees(q0):+.1f}° -> {math.degrees(q0+amp):+.1f}°\n")

        for b in bloques:
            print(f"  ── bloque «{b}» ──")
            for k in range(a.n):
                if b == "ganancias":
                    # lo que hace un barrido: cambiar kp y volver
                    cli.set_target(idx, cli.q(idx))
                    cli.sleep(0.15)
                    cli.set_gains(idx, kp0 * 0.4, kd0)
                    cli.sleep(0.3)
                    cli.set_target(idx, cli.q(idx))
                    cli.sleep(0.15)
                    cli.set_gains(idx, kp0, kd0)
                    cli.sleep(0.3)
                elif b == "vecinas":
                    # mover las otras seis y devolverlas
                    d = {i: float(cli.q_base[i]) + 0.10 for i in vecinas}
                    cli.ramp_to(d, speed=0.6)
                    cli.ramp_to({i: float(cli.q_base[i]) for i in vecinas},
                                speed=0.6)
                cli.ramp_to({idx: q0}, speed=0.3)
                cli.sleep(a.pause)
                cli.wait_settled(idx)
                _, track, step = _move.run_once(cli, idx, base, amp, gains,
                                                quiet=True)
                if step is None:
                    continue
                res[b].append((track.rms_error * 1000, step.overshoot * 100,
                               step.steady_error * 1000))
                print(f"    {k+1:>2}/{a.n}  err rms {track.rms_error*1000:6.2f} mrad  "
                      f"sobreimp {step.overshoot*100:5.1f} %  "
                      f"err final {step.steady_error*1000:+7.2f} mrad")
            cli.ramp_to({idx: q0}, speed=0.3)

    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
    except KeyboardInterrupt:
        print("\n  interrumpido")
    finally:
        cli.__exit__(None, None, None)

    vivos = {b: v for b, v in res.items() if len(v) >= 3}
    if not vivos:
        return 1

    print("\n\n  ═══ Repetibilidad ═══════════════════════════════════════════════")
    print(f"  {'bloque':<12}{'n':>3}"
          f"{'err rms: mediana':>19}{'IQR':>8}"
          f"{'sobreimp: mediana':>20}{'IQR':>8}"
          f"{'err final: mediana':>21}{'IQR':>8}")
    for b, v in vivos.items():
        e, o, s = [x[0] for x in v], [x[1] for x in v], [x[2] for x in v]
        print(f"  {b:<12}{len(v):>3}"
              f"{st.median(e):>16.2f} mr{iqr(e):>7.2f}"
              f"{st.median(o):>18.1f} %{iqr(o):>7.1f}"
              f"{st.median(s):>18.2f} mr{iqr(s):>7.2f}")

    if "base" in vivos:
        e = [x[0] for x in vivos["base"]]
        o = [x[1] for x in vivos["base"]]
        s = [x[2] for x in vivos["base"]]
        print(f"\n  δ_min = 1.5 × IQR del bloque «base»:")
        print(f"    error rms       {1.5*iqr(e):6.2f} mrad")
        print(f"    sobreimpulso    {1.5*iqr(o):6.1f} puntos porcentuales")
        print(f"    error final     {1.5*iqr(s):6.2f} mrad")
        print(f"\n  Por debajo de esos valores ninguna diferencia entre "
              f"candidatos es defendible.")
        for b in ("ganancias", "vecinas"):
            if b in vivos:
                ob = [x[1] for x in vivos[b]]
                d = st.median(ob) - st.median(o)
                marca = "SIGNIFICATIVO" if abs(d) > 1.5 * iqr(o) else "dentro del ruido"
                print(f"  «{b}» desplaza el sobreimpulso {d:+.1f} pp -> {marca}")

    rec.append_index({"stamp": stamp, "test": "repeatability", "channel": a.channel,
                      "gains": gains.set_name, "joint": j.name, "kp": kp0, "kd": kd0,
                      "csv": "", **{f"{b}_n": len(v) for b, v in vivos.items()}})
    return 0


if __name__ == "__main__":
    sys.exit(main())
