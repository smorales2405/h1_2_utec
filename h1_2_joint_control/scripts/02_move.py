#!/usr/bin/env python3
"""Mueve UNA articulación con una consigna conocida y mide cómo la sigue.

Es la herramienta central: todo lo demás (barrido de ganancias, recorrido de
los 14 motores de los brazos) llama a lo mismo que hay aquí.

Trayectorias:

  step         escalón puro. Sobreimpulso, tiempo de subida, error final.
               Es la prueba más dura y la que mejor separa un kd bueno de uno malo.
  smooth_step  escalón con coseno alzado. Se parece más a la teleoperación.
  sine         seno a una frecuencia. Error de seguimiento y desfase.
  chirp        barrido de frecuencia. De un tirón se ve dónde deja de seguir
               y dónde resuena.
  hold         no mover: mide el ruido y el temblor en reposo.

Ejemplos:
    python3 scripts/02_move.py --joint L_elbow --traj step  --amp 0.15
    python3 scripts/02_move.py --joint L_elbow --traj sine  --amp 0.15 --freq 0.5
    python3 scripts/02_move.py --joint L_elbow --traj chirp --amp 0.08 --f1 3.0
    python3 scripts/02_move.py --joint L_elbow --traj sine  --amp 0.15 --freq 0.5 --zero-dq

`--zero-dq` anula la velocidad de referencia, que es lo que hace
`xr_teleoperate` (`msg.motor_cmd[id].dq = 0`). Comparar con y sin ella dice
cuánto retardo mete esa decisión.
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from _common import add_common_args, build_client, confirm, describe, joint_index
from h1_2_joint_control import config as cfg
from h1_2_joint_control import metrics as mt
from h1_2_joint_control import recorder as rec
from h1_2_joint_control import trajectories as tr
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX

T_STEP = 0.5          # cuándo salta el escalón, desde el inicio del registro


def build_traj(a, amp: float):
    """(func, duración total, descripción)."""
    if a.traj == "step":
        return (tr.step(amp, T_STEP), T_STEP + a.settle,
                f"escalón de {amp:+.3f} rad ({math.degrees(amp):+.1f}°) en t={T_STEP} s")
    if a.traj == "smooth_step":
        return (tr.smooth_step(amp, a.rise, T_STEP), T_STEP + a.rise + a.settle,
                f"escalón suave de {amp:+.3f} rad en {a.rise:.2f} s")
    if a.traj == "sine":
        dur = a.cycles / a.freq
        return (tr.sine(amp, a.freq), dur,
                f"seno de ±{amp:.3f} rad a {a.freq:.2f} Hz, {a.cycles:.0f} ciclos "
                f"({dur:.1f} s)")
    if a.traj == "chirp":
        return (tr.chirp(amp, a.f0, a.f1, a.duration), a.duration + 0.5,
                f"chirp de ±{amp:.3f} rad, {a.f0:.2f} -> {a.f1:.2f} Hz en {a.duration:.0f} s")
    return tr.hold(), a.settle, "reposo"


def run_once(cli, idx, a, amp, gains, quiet=False):
    """Ejecuta una trayectoria y devuelve (muestras, métricas de seguimiento,
    métricas de escalón o None). El cliente debe estar ya en `engage()`."""
    j = BY_INDEX[idx]
    kp, kd = gains.for_index(idx)
    func, duration, _ = build_traj(a, amp)
    if a.zero_dq:
        func = tr.zero_velocity(func)

    q_base = cli.q0[idx]
    cli.record(True)
    cli.set_trajectory(idx, func, q_base=q_base)
    t_end = duration
    tick = 0
    while cli.trajectory_time(idx) < t_end:
        # Refrescar la línea de estado cada ~0.2 s: a 20 Hz el terminal parpadea
        # y, si la salida se está redirigiendo a un fichero, lo llena de basura.
        if not quiet and tick % 4 == 0:
            t = cli.trajectory_time(idx)
            q = cli.q(idx)
            print(f"    t={t:5.2f}/{t_end:.1f} s  q={q:+.4f}  "
                  f"err={q - cli.q_des(idx):+.4f} rad  tau={cli.tau(idx):+6.2f} Nm   ",
                  end="\r", flush=True)
        tick += 1
        cli.sleep(0.05)
    cli.clear_trajectory(idx)
    if not quiet:
        print()
    # cola de asentamiento: la consigna se queda donde acabó la trayectoria
    cli.sleep(0.3)
    samples = cli.record(False)

    track = mt.tracking(samples, idx, kp, kd, j.tau_max,
                        skip=(T_STEP if a.traj in ("step", "smooth_step") else 0.5))
    step = None
    if a.traj in ("step", "smooth_step"):
        try:
            step = mt.step_response(samples, idx, kp, kd, T_STEP)
        except ValueError:
            step = None
    return samples, track, step


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--joint", required=True, help="p. ej. L_elbow, R_wrist_pitch, 16")
    ap.add_argument("--traj", default="step",
                    choices=["step", "smooth_step", "sine", "chirp", "hold"])
    ap.add_argument("--amp", type=float, default=0.15,
                    help="amplitud en rad (0.15 rad = 8.6°)")
    ap.add_argument("--freq", type=float, default=0.5, help="Hz, para sine")
    ap.add_argument("--cycles", type=float, default=6.0, help="ciclos, para sine")
    ap.add_argument("--f0", type=float, default=0.2, help="Hz inicial del chirp")
    ap.add_argument("--f1", type=float, default=3.0, help="Hz final del chirp")
    ap.add_argument("--duration", type=float, default=20.0, help="s, para chirp")
    ap.add_argument("--rise", type=float, default=0.3, help="s, para smooth_step")
    ap.add_argument("--settle", type=float, default=2.5,
                    help="s de observación tras el escalón")
    ap.add_argument("--zero-dq", action="store_true",
                    help="anular la velocidad de referencia, como xr_teleoperate")
    a = ap.parse_args()

    idx = joint_index(a.joint)
    j = BY_INDEX[idx]
    gains = cfg.load(a.gains)
    if a.kp is not None or a.kd is not None:
        kp0, kd0 = gains.for_index(idx)
        gains.set_index(idx, a.kp if a.kp is not None else kp0,
                        a.kd if a.kd is not None else kd0)

    cli = build_client(a, [idx])
    try:
        cli.wait_for_state()
        q0 = cli.q(idx)

        # Elegir el sentido del movimiento con el recorrido que quede libre.
        margin = gains.safety.joint_limit_margin
        amp = a.amp
        if q0 + amp > j.q_max - margin or q0 + amp < j.q_min + margin:
            if q0 - amp >= j.q_min + margin and q0 - amp <= j.q_max - margin:
                print(f"  ⚠ {q0:+.3f}{amp:+.3f} se sale del tope: se invierte el sentido.")
                amp = -amp
            else:
                room = min(j.q_max - margin - q0, q0 - j.q_min - margin)
                amp = math.copysign(max(room * 0.8, 0.0), amp)
                print(f"  ⚠ poco recorrido libre: amplitud recortada a {amp:+.3f} rad.")
        if abs(amp) < 1e-3 and a.traj != "hold":
            raise SystemExit("  no queda recorrido para moverse desde esta postura.")

        _, duration, desc = build_traj(a, amp)
        print("\n" + describe(idx, gains))
        print(f"\n    postura actual : {q0:+.3f} rad ({math.degrees(q0):+.1f}°)")
        print(f"    trayectoria    : {desc}")
        print(f"    canal          : {a.channel}   lazo {a.rate:.0f} Hz   "
              f"dq de referencia: {'ANULADA (como xr_teleoperate)' if a.zero_dq else 'activa'}")
        confirm(a)

        cli.engage()
        print(f"\n  Ejecutando…")
        samples, track, step = run_once(cli, idx, a, amp, gains)

        print("\n  ── Resultados ─────────────────────────────────────────────")
        if step is not None:
            print("  " + step.summary())
        print("  " + track.summary())
        if track.chatter_dq > 0.05:
            print("\n  ⚠ Temblor apreciable "
                  f"({track.chatter_dq:.3f} rad/s por encima de 8 Hz, pico a "
                  f"{track.peak_freq:.1f} Hz).")
            print("    Suele ser kd alto para ese kp, o alguien más publicando")
            print("    en el mismo canal: comprueba con 00_diagnose.py.")
        if abs(track.mean_error) > 0.01:
            print(f"\n  ⚠ Sesgo de {math.degrees(track.mean_error):+.2f}° en el error medio.")
            print("    Es la caída por gravedad: el PD no tiene término integral,")
            print(f"    así que el error permanente es tau_gravedad/kp. Con kp="
                  f"{track.kp:.0f} eso son {track.rms_tau/track.kp*1000:.1f} mrad "
                  "por cada Nm de par estático.")

        stamp = rec.stamp()
        tag = f"_{a.tag}" if a.tag else ""
        name = (f"{a.traj}_{j.name}_kp{track.kp:.0f}_kd{track.kd:.1f}"
                f"{'_zerodq' if a.zero_dq else ''}{tag}_{stamp}")
        csv = cfg.LOG_DIR / f"{name}.csv"
        rec.save_samples(samples, [idx], csv)
        row = {"stamp": stamp, "test": a.traj, "channel": a.channel,
               "gains": gains.set_name, "rate_hz": a.rate, "amp": amp,
               "freq": a.freq if a.traj == "sine" else "",
               "zero_dq": int(a.zero_dq), "csv": csv.name, **track.as_row()}
        if step is not None:
            row.update({f"step_{k}": v for k, v in step.as_row().items()
                        if k not in ("joint", "kp", "kd")})
        rec.append_index(row)
        print(f"\n  registro -> {csv}")
        print(f"  gráfica  -> python3 scripts/05_plot.py {csv.name}")
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
