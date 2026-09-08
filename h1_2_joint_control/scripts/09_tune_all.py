#!/usr/bin/env python3
"""Sintoniza varias articulaciones seguidas, con el método completo.

Para cada articulación, dos barridos encadenados:

  1. **kp**, con kd en su valor de referencia. kp domina el error de
     seguimiento, así que se fija primero.
  2. **kd**, con el kp ganador. kd domina el temblor y está poco acoplado a kp
     en este rango, de ahí que barrerlos por separado cueste 8 ensayos en vez
     de los 16 de una rejilla completa, sin perder gran cosa.

Los valores del barrido son **múltiplos de las ganancias de referencia** de
cada articulación, no una lista fija: el hombro (40 Nm) y la muñeca (19 Nm) no
admiten los mismos números, y así el barrido se adapta solo.

Se toma el control UNA vez para todas: la postura de ensayo se aplica al
principio y entre articulaciones no se suelta. Eso ahorra minutos y, sobre
todo, evita que cada articulación parta de una postura distinta.

La medida es seguimiento sinusoidal, no escalón: es lo que de verdad hace la
teleoperación, y un escalón premia ganancias agresivas que luego tiemblan.

Ejemplos:
    python3 scripts/09_tune_all.py --channel lowcmd --joints shoulder --gains xr_teleoperate
    python3 scripts/09_tune_all.py --channel lowcmd --joints L_wrist_roll,L_wrist_pitch,L_wrist_yaw
    python3 scripts/09_tune_all.py --channel lowcmd --joints wrist --write tuned
"""
from __future__ import annotations

import argparse
import importlib
import math
import sys
from pathlib import Path

from _common import (add_common_args, apply_test_posture, build_client, confirm,
                     pick_amplitude)
from h1_2_joint_control import config as cfg
from h1_2_joint_control import metrics as mt
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX, resolve

sys.path.insert(0, str(Path(__file__).resolve().parent))
_move = importlib.import_module("02_move")

# Múltiplos de la ganancia de referencia de cada articulación.
KP_MULT = (0.6, 1.0, 1.5, 2.0)
KD_MULT = (0.5, 1.0, 2.0, 3.0, 4.5)


def kp_maximo(joint, safety, escalon: float) -> float:
    """kp más alto admisible para esa articulación.

    Hace falta un tope, y no es una precaución teórica. En las articulaciones
    con carga estática grande —los hombros— el error de seguimiento es sobre
    todo caída por gravedad, `tau_g/kp`, así que **baja monótonamente con kp**:
    un criterio que solo mire el error elige siempre el kp más alto que se le
    ofrezca, y el barrido se va al infinito.

    Lo que pone el límite no es el seguimiento sino la SATURACIÓN. Un kp alto
    es inofensivo mientras la consigna se mueva despacio, y peligroso en cuanto
    da un salto: en teleoperación eso pasa cada vez que el seguimiento del
    mando parpadea. Así que se acota pidiendo que un salto de consigna de
    `escalon` radianes no pase del umbral de par con el que aborta la
    seguridad:

        kp_max = tau_abort_fraction · tau_max / escalon

    Con 0.10 rad (5.7°) y el 70 % por defecto: 280 en los hombros de 40 Nm,
    126 en el codo y el hombro-yaw de 18 Nm, 133 en las muñecas de 19 Nm.
    """
    return safety.tau_abort_fraction * joint.tau_max / max(escalon, 1e-3)


class Args:
    """Lo que `run_once` espera de un espacio de nombres de argparse."""
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _promedia(ts):
    """Mediana de las métricas. Mediana y no media: con fricción estática la
    distribución no es simétrica y un pegado estropea la media."""
    import statistics as st
    b = ts[0]
    campos = ("rms_error", "max_error", "mean_error", "chatter_dq", "chatter_tau",
              "peak_freq", "rms_tau", "max_tau", "tau_headroom")
    kw = {f: st.median([getattr(t, f) for t in ts]) for f in campos}
    return mt.TrackingMetrics(joint=b.joint, kp=b.kp, kd=b.kd, n=b.n, fs=b.fs, **kw)


def sweep(cli, idx, gains, base_args, q0, amp, values, fija, es_kp, log,
          repeats=1):
    """Un barrido de un solo parámetro. Devuelve [(valor, coste, track)]."""
    out = []
    for v in values:
        kp, kd = (v, fija) if es_kp else (fija, v)
        # Regla 1 del protocolo: nunca cambiar kp con `q_des` desactualizado.
        # Si la articulación está sosteniendo con un error `e`, pasar de kp a
        # kp' produce un salto de par `(kp'-kp)·e` en un solo ciclo, y el
        # ensayo siguiente empieza con la articulación todavía asentándose.
        # Medido lo que costaba no hacerlo: el barrido daba 52-84 % de
        # sobreimpulso en `L_shoulder_roll` donde el mismo kp aislado da
        # 0.8-17 %. Se iguala la consigna a la posición medida antes de tocar
        # las ganancias, y se deja asentar.
        cli.set_target(idx, cli.q(idx))
        cli.sleep(0.15)
        cli.set_gains(idx, kp, kd)
        gains.set_index(idx, kp, kd)
        cli.sleep(0.25)
        ts, sts = [], []
        sin_asentar = 0
        for _ in range(max(repeats, 1)):
            cli.ramp_to({idx: q0}, speed=0.3)
            cli.sleep(base_args.pause)
            # No basta una pausa fija: si la articulación sigue volviendo de la
            # repetición anterior, `step_response` toma como punto de partida
            # una posición en movimiento y el sobreimpulso sale inflado.
            if not cli.wait_settled(idx):
                sin_asentar += 1
            samples, track, step = _move.run_once(cli, idx, base_args, amp,
                                                  gains, quiet=True)
            ts.append(track)
            if step is not None:
                sts.append(step)
        if sin_asentar:
            print(f"      ⚠ {sin_asentar} repetición(es) empezaron sin asentar")
        track = ts[0] if len(ts) == 1 else _promedia(ts)
        # El sobreimpulso solo entra en el coste si la trayectoria es un
        # escalón: con seno no hay tal cosa y `cost` lo ignoraría igualmente.
        if not sts:
            step = None
        elif len(sts) == 1:
            step = sts[0]
        else:
            # la de sobreimpulso MEDIANO, no una arbitraria
            step = sorted(sts, key=lambda x: x.overshoot)[len(sts) // 2]
        J = mt.cost(track, step, base_args.w_err, base_args.w_chatter,
                    base_args.w_overshoot)
        out.append((v, J, track))
        extra = (f"  sobreimp {step.overshoot*100:5.1f}%  "
                 f"err final {step.steady_error*1000:+7.2f}" if step else "")
        print(f"      kp={kp:6.1f} kd={kd:5.2f}  J={J:6.2f}  "
              f"err {track.rms_error*1000:6.2f} mrad  "
              f"temblor {track.chatter_dq:.4f}  tau máx {track.max_tau:5.2f} Nm{extra}")
        log.append((BY_INDEX[idx].name, kp, kd, J, track))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--joints", required=True,
                    help="shoulder, wrist, left_arm, o una lista de nombres")
    ap.add_argument("--traj", default="sine", choices=["sine", "smooth_step"],
                    help="seno mide seguimiento; smooth_step añade sobreimpulso "
                         "y error final al criterio, que es lo que pide F2.3")
    ap.add_argument("--repeats", type=int, default=1,
                    help="repeticiones por candidato; se toma la mediana")
    ap.add_argument("--kp-mult", default=None,
                    help="múltiplos de kp, separados por comas. Con la gravedad "
                         "compensada interesa extender hacia abajo")
    ap.add_argument("--kd-mult", default=None)
    ap.add_argument("--w-overshoot", type=float, default=0.5)
    ap.add_argument("--amp", type=float, default=0.12)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--cycles", type=float, default=3.0)
    ap.add_argument("--pause", type=float, default=0.6)
    ap.add_argument("--w-err", type=float, default=1.0)
    ap.add_argument("--w-chatter", type=float, default=1.0)
    ap.add_argument("--zero-dq", action="store_true",
                    help="medir con dq_des = 0, como hace xr_teleoperate. "
                         "IMPORTANTE: un kd sintonizado CON velocidad de "
                         "referencia no vale sin ella. Con dq_des=0 el término "
                         "kd·(0-dq) frena el movimiento pedido, y con kd alto "
                         "eso solo satura el motor: kd=13.5 en el codo de 18 Nm "
                         "pide 27 Nm a 2 rad/s")
    ap.add_argument("--kp-step", type=float, default=0.10,
                    help="salto de consigna, en rad, que se exige que NO sature "
                         "el motor. Acota el kp del barrido: por encima de "
                         "tau_abort·tau_max/salto, un tirón de la teleoperación "
                         "pediría más par del que hay")
    ap.add_argument("--write", metavar="CONJUNTO", default=None,
                    help="guardar los ganadores en ese conjunto de gains.yaml")
    a = ap.parse_args()

    targets = resolve(a.joints)
    gains = cfg.load(a.gains)
    ref = {i: gains.for_index(i) for i in targets}
    kp_mult = (tuple(float(x) for x in a.kp_mult.split(",")) if a.kp_mult
               else KP_MULT)
    kd_mult = (tuple(float(x) for x in a.kd_mult.split(",")) if a.kd_mult
               else KD_MULT)

    por_art = (len(kp_mult) + len(kd_mult)) * max(a.repeats, 1)
    seg = por_art * ((a.cycles / a.freq if a.traj == "sine" else 2.8)
                     + a.pause + 1.6)
    print(f"\n  Articulaciones : {', '.join(BY_INDEX[i].name for i in targets)}")
    print(f"  Referencia     : '{gains.set_name}'")
    print(f"  Método         : {len(KP_MULT)} valores de kp (kd de referencia), "
          f"luego {len(KD_MULT)} de kd con el kp ganador")
    print(f"  Múltiplos      : kp {kp_mult}   kd {kd_mult}")
    print(f"  Trayectoria    : {a.traj}, {a.repeats} repetición(es) por candidato")
    print(f"  Gravedad       : {'COMPENSADA' if a.gravity_ff else 'sin compensar'}")
    print(f"  Medida         : amplitud {a.amp:.3f} rad, dq de referencia "
          f"{'ANULADA (como xr_teleoperate)' if a.zero_dq else 'activa'}")
    print(f"  Duración aprox.: {len(targets)*seg/60:.1f} min "
          f"({por_art} ensayos por articulación)")
    confirm(a)

    base = Args(traj=a.traj, amp=a.amp, freq=a.freq, cycles=a.cycles,
                settle=2.0, rise=0.3, f0=0.2, f1=3.0, duration=10.0,
                zero_dq=a.zero_dq, pause=a.pause, w_err=a.w_err,
                w_chatter=a.w_chatter, w_overshoot=a.w_overshoot)

    cli = build_client(a, targets, verbose=True)
    resultados, log = [], []
    stamp = rec.stamp()
    try:
        cli.wait_for_state()
        cli.engage()
        apply_test_posture(cli, a, gains)

        for n, idx in enumerate(targets, 1):
            j = BY_INDEX[idx]
            kp_ref, kd_ref = ref[idx]
            q0 = float(cli.q_base[idx])
            amp, nota = pick_amplitude(idx, q0, a.amp, gains.limits(idx),
                                       gains.direction(idx, a.direction))
            if abs(amp) < 5e-3:
                print(f"\n  [{n}/{len(targets)}] {j.name}: sin recorrido, se salta.")
                continue
            print(f"\n  [{n}/{len(targets)}] {j.name}   referencia kp={kp_ref:.0f} "
                  f"kd={kd_ref:.1f}   {math.degrees(q0):+.1f}° -> "
                  f"{math.degrees(q0+amp):+.1f}° {nota}")

            kp_max = kp_maximo(j, gains.safety, a.kp_step)
            lo, hi = kp_mult[0] * kp_ref, min(kp_mult[-1] * kp_ref, kp_max)
            if hi <= lo:
                # el tope cae por debajo del barrido: se explora lo que quepa
                lo, hi = 0.5 * kp_max, kp_max
            n_pts = len(kp_mult)
            # rejilla geométrica: kp actúa como 1/error, así que interesa
            # muestrear en proporción, no en diferencia
            razon = (hi / lo) ** (1.0 / (n_pts - 1))
            recortados = {round(lo * razon ** k, 1) for k in range(n_pts)}
            # El valor de referencia entra siempre en la rejilla, aunque no
            # caiga en ella: sin él no hay con qué comparar el ganador, y la
            # columna «mejora» del resumen sale vacía.
            if kp_ref <= hi:
                recortados.add(round(kp_ref, 1))
            recortados = sorted(recortados)
            if hi < kp_mult[-1] * kp_ref - 1e-6:
                print(f"    (kp acotado a {kp_max:.0f}: por encima, un salto de "
                      f"{a.kp_step:.2f} rad saturaría los {j.tau_max:.0f} Nm)")
            print(f"    barrido de kp (kd={kd_ref:.1f}):")
            kps = sweep(cli, idx, gains, base, q0, amp, recortados, kd_ref,
                        True, log, a.repeats)
            kp_best = min(kps, key=lambda r: r[1])[0]

            print(f"    barrido de kd (kp={kp_best:.0f}):")
            kds = sweep(cli, idx, gains, base, q0, amp,
                        [round(m * kd_ref, 2) for m in kd_mult], kp_best,
                        False, log, a.repeats)
            kd_best = min(kds, key=lambda r: r[1])[0]

            t_best = min(kds, key=lambda r: r[1])[2]
            t_ref = next((t for v, J, t in kps if abs(v - kp_ref) < 1e-6), None)
            print(f"    -> kp={kp_best:.0f} kd={kd_best:.2f}"
                  + (f"   (referencia daba {t_ref.rms_error*1000:.1f} mrad; "
                     f"ahora {t_best.rms_error*1000:.1f})" if t_ref else ""))
            borde = (kp_best in (kps[0][0], kps[-1][0])
                     or kd_best in (kds[0][0], kds[-1][0]))
            if borde:
                print("       ⚠ el ganador está en el borde del barrido")

            cli.set_gains(idx, kp_best, kd_best)
            gains.set_index(idx, kp_best, kd_best)
            cli.ramp_to({idx: q0}, speed=0.3)
            resultados.append((j, kp_ref, kd_ref, kp_best, kd_best, t_ref, t_best,
                               borde))

    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
    except KeyboardInterrupt:
        print("\n  interrumpido por el usuario")
    finally:
        cli.__exit__(None, None, None)

    for nombre, kp, kd, J, t in log:
        rec.append_index({"stamp": stamp, "test": "tune_all_sine", "channel": a.channel,
                          "gains": "sweep", "rate_hz": a.rate, "amp": a.amp,
                          "freq": a.freq, "cost": J, "csv": "", **t.as_row()})

    if not resultados:
        return 1
    print("\n\n  ═══ Ganancias sintonizadas ═══════════════════════════════════════")
    print(f"  {'articulación':<18} {'referencia':>13} {'sintonizado':>13} "
          f"{'err ref':>10} {'err nuevo':>10} {'mejora':>8}")
    for j, kp0, kd0, kp1, kd1, t0, t1, borde in resultados:
        e0 = f"{t0.rms_error*1000:7.2f} mr" if t0 else "         —"
        e1 = f"{t1.rms_error*1000:7.2f} mr"
        mej = (f"{(1 - t1.rms_error/t0.rms_error)*100:6.0f} %"
               if t0 and t0.rms_error > 0 else "      —")
        print(f"  {j.name:<18} {kp0:6.0f}/{kd0:<6.1f} {kp1:6.0f}/{kd1:<6.2f} "
              f"{e0:>10} {e1:>10} {mej:>8}" + ("  ⚠ borde" if borde else ""))

    if a.write:
        target = cfg.load(a.write)
        for j, _, _, kp1, kd1, _, _, _ in resultados:
            target.set_index(j.idx, kp1, kd1)
        target.write_into(a.write, note=target.description)
        print(f"\n  escrito en config/gains.yaml -> sets.{a.write}")
    else:
        print("\n  Para guardarlo: añade --write tuned")
    return 0


if __name__ == "__main__":
    sys.exit(main())
