#!/usr/bin/env python3
"""F3 — ¿cambian las ganancias óptimas con la postura del brazo?

Toda la sintonización de este repositorio se hizo en una sola postura: codo a
85° y brazo colgando. La matriz de inercia que ve el hombro y el brazo de
palanca de la gravedad cambian mucho con la configuración, así que la pregunta
es si un solo conjunto de ganancias sirve para todas.

Rejilla de `postures_f3` en `config/gains.yaml`:

    P1  pitch   0°  roll ±18°  codo 85°   reposo, la ya medida (referencia)
    P2  pitch −40°  roll ±25°  codo 60°   trabajo, alcance frontal a mesa
    P3  pitch −70°  roll ±35°  codo 20°   extendida, peor caso

Las tres están verificadas libres de colisión y dentro de los topes.

DECISIÓN, ESCRITA ANTES DE MIRAR LOS DATOS
──────────────────────────────────────────
    kp*_max / kp*_min  <  1.5  en todas   ->  un solo conjunto, elegido en P3
                                              (el peor caso), documentando lo
                                              que se degrada en P1 y P2
    razón >= 1.5 en alguna                ->  gain scheduling por postura,
                                              tabla de 3 puntos interpolada

Está aquí arriba a propósito: el criterio se fija antes de ver los números, no
después. El script la aplica solo y dice cuál sale.

    python3 scripts/17_f3_postura.py                    la rejilla entera
    python3 scripts/17_f3_postura.py --postures P1,P3   solo dos
    python3 scripts/17_f3_postura.py --joints L_elbow   una articulación
    python3 scripts/17_f3_postura.py --dry-run

Tiempo: 4 articulaciones × 3 posturas × 6 kp × 3 repeticiones ≈ 216 ensayos.
Contando las pausas y los cambios de postura, algo más de una hora de robot.
"""
from __future__ import annotations

import argparse
import importlib
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _common import (add_common_args, apply_test_posture, build_client,
                     confirm, joint_index)
from h1_2_joint_control import config as cfg
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX

_tune = importlib.import_module("03_tune")

RAZON_UMBRAL = 1.5          # de la tabla de decisión, fijada de antemano
PICO_P1 = 7.8               # Nm de transitorio medidos en P1 (§13), amp 0.12
JOINTS_F3 = ("L_shoulder_pitch", "L_shoulder_roll", "L_elbow", "L_wrist_pitch")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.set_defaults(channel="lowcmd", gains="tuned_gff", gravity_ff=True)
    ap.add_argument("--postures", default=None,
                    help="lista separada por comas (por defecto, todas)")
    ap.add_argument("--joints", default=",".join(JOINTS_F3))
    ap.add_argument("--kp-list", default="60,80,110,140,190,260")
    ap.add_argument("--kd-list", default="6")
    ap.add_argument("--traj", default="smooth_step",
                    help="smooth_step añade sobreimpulso y error final al "
                         "criterio, que es lo que pide F3")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--amp", type=float, default=0.12)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--cycles", type=float, default=3.0)
    ap.add_argument("--pause", type=float, default=0.6)
    ap.add_argument("--rise", type=float, default=0.3)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--f0", type=float, default=0.2)
    ap.add_argument("--f1", type=float, default=3.0)
    ap.add_argument("--w-err", type=float, default=1.0)
    ap.add_argument("--w-chatter", type=float, default=1.0)
    ap.add_argument("--w-overshoot", type=float, default=0.5)
    ap.add_argument("--zero-dq", action="store_true",
                    help="medir con dq_des = 0, como hacía xr_teleoperate sin "
                         "el parche. Por defecto se manda la velocidad de "
                         "referencia, que es el modo en que se sintonizó "
                         "`tuned_gff`.")
    a = ap.parse_args()

    gains = cfg.load(a.gains)
    posturas = ([p.strip() for p in a.postures.split(",")] if a.postures
                else gains.posture_names())
    for p in posturas:
        gains.posture(p)              # falla pronto si el nombre no existe
    articulaciones = [joint_index(n) for n in a.joints.split(",")]
    grid = [(kp, kd) for kp in _tune.floats(a.kp_list)
            for kd in _tune.floats(a.kd_list)]

    print(f"\n  F3 · {len(posturas)} posturas × {len(articulaciones)} "
          f"articulaciones × {len(grid)} candidatos × {a.repeats} repeticiones"
          f"  =  {len(posturas)*len(articulaciones)*len(grid)*a.repeats} ensayos")
    print(f"  criterio de decisión, fijado de antemano: "
          f"razón kp* < {RAZON_UMBRAL} → un solo conjunto")

    # Aviso de par: en P3 la gravedad ya consume buena parte del margen.
    if gains.posture_names() and a.gravity_ff:
        try:
            from h1_2_joint_control.gravity import GravityModel
            import numpy as np
            gm = GravityModel(verbose=False)
            print(f"\n  {'postura':<8}{'tau gravedad en shoulder_pitch':>34}"
                  f"{'margen al aborto':>19}")
            for p in posturas:
                q = np.zeros(27)
                for i, v in gains.posture(p).items():
                    q[i] = v
                t = abs(gm.tau(q).get(13, 0.0))
                tope = gains.safety.tau_abort_fraction * BY_INDEX[13].tau_max
                # El transitorio del escalón se SUMA a esto. Medido en P1
                # con `tuned_gff`: 7.8 Nm de pico en shoulder_pitch con
                # amplitud 0.12 rad. Si el margen no da para eso, el aborto
                # por par saltará a mitad del barrido y se perderá el tiempo
                # de robot; es mejor bajar la amplitud que tocar el umbral.
                # Factor 1.3 sobre el transitorio medido, y no es prudencia
                # genérica: la identificación de gravedad se hizo alrededor de
                # P1, así que en P2 y P3 este número EXTRAPOLA. Si el modelo se
                # queda corto, lo que falta sale del margen.
                margen = tope - t
                aviso = ""
                if margen < PICO_P1 * 1.3:
                    amp_ok = a.amp * max(margen / (PICO_P1 * 1.3), 0.3)
                    aviso = (f"   ⚠ transitorio ~{PICO_P1:.1f} N: "
                             f"usa --amp {amp_ok:.2f} aquí")
                elif margen < PICO_P1 * 2.0:
                    aviso = "   margen justo, vigila el aborto"
                print(f"  {p:<8}{t:>32.1f} N{margen:>17.1f} N" + aviso)
        except Exception as exc:
            print(f"  (no se pudo estimar el par de gravedad: {exc})")

    confirm(a, "Los brazos recorren posturas muy distintas. Robot COLGADO.")

    stamp = rec.stamp()
    ganadores: dict[tuple[str, int], tuple] = {}
    for postura in posturas:
        for idx in articulaciones:
            j = BY_INDEX[idx]
            print(f"\n  ══ {postura} · {j.name} "
                  f"{'═' * max(0, 46 - len(postura) - len(j.name))}")
            a.posture = postura
            g = cfg.load(a.gains)
            cli = build_client(a, [idx], verbose=False)
            resultados = []
            try:
                cli.wait_for_state()
                cli.engage()
                apply_test_posture(cli, a, g)
                _tune.barre_candidatos(cli, idx, a, g, grid, stamp, resultados)
            except SafetyAbort as e:
                print(f"    ⚠ abortado por seguridad: {e}")
            except KeyboardInterrupt:
                print("\n  interrumpido por el usuario")
                cli.__exit__(None, None, None)
                return 1
            finally:
                cli.__exit__(None, None, None)
            if resultados:
                resultados.sort(key=lambda r: r[0])
                ganadores[(postura, idx)] = resultados[0]
                J, kp, kd, t, st = resultados[0]
                print(f"    mejor: kp={kp:.1f} kd={kd:.2f}  J={J:.2f}  "
                      f"rms {t.rms_error*1000:.2f} mrad  tau máx {t.max_tau:.2f} Nm")

    return informe(ganadores, posturas, articulaciones)


def informe(ganadores, posturas, articulaciones) -> int:
    if not ganadores:
        print("\n  sin resultados.")
        return 1
    print("\n\n  ══ F3 · resultados ══════════════════════════════════════════")
    print(f"\n  {'articulación':<20}" + "".join(f"{p:>22}" for p in posturas)
          + f"{'razón':>9}")
    print(f"  {'':<20}" + "".join(f"{'kp*  kd*  rms  tau':>22}" for _ in posturas))
    razones = {}
    for idx in articulaciones:
        fila = f"  {BY_INDEX[idx].name:<20}"
        kps = []
        for p in posturas:
            g = ganadores.get((p, idx))
            if g is None:
                fila += f"{'—':>22}"
                continue
            J, kp, kd, t, st = g
            kps.append(kp)
            fila += (f"{kp:>6.0f}{kd:>5.1f}{t.rms_error*1000:>6.1f}"
                     f"{t.max_tau:>5.1f}")
        if len(kps) >= 2:
            r = max(kps) / min(kps)
            razones[idx] = r
            fila += f"{r:>8.2f}" + ("  ⚠" if r >= RAZON_UMBRAL else "")
        print(fila)
    print(f"\n  (kp*, kd*, error rms en mrad, par máximo en Nm)")

    if not razones:
        print("\n  Una sola postura: no hay razón que calcular. Para decidir "
              "hacen falta\n  al menos dos, y la comparación que importa es "
              "P1 contra P3.")
        return 0
    peor = max(razones.values())
    quien = BY_INDEX[max(razones, key=razones.get)].name
    print(f"\n  Razón mayor: {peor:.2f} en {quien}  (umbral {RAZON_UMBRAL})")
    if peor < RAZON_UMBRAL:
        print(f"""
  DECISIÓN: un solo conjunto de ganancias.

  Ninguna articulación cambia su kp óptimo en más de un factor {RAZON_UMBRAL}
  entre la postura de reposo y la extendida. Se elige el de P3, que es el peor
  caso de inercia y gravedad, y se documenta cuánto se degrada en P1 y P2.""")
    else:
        print(f"""
  DECISIÓN: gain scheduling por postura.

  {quien} cambia su kp óptimo por un factor {peor:.2f}, o sea que una
  sintonización única no preserva el margen de fase en todo el espacio de
  trabajo. Se implementa como tabla de tres puntos interpolada linealmente,
  con kp(q) proporcional a M_ii(q) del mismo modelo de `pinocchio`.""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
