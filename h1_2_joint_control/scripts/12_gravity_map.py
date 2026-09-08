#!/usr/bin/env python3
"""Muestrea el par de gravedad por el espacio de trabajo (fase F2, camino b).

Qué mejora sobre `11_gravity_id.py`. Aquel movía una articulación cada vez y
medía solo esa: 19 puntos agrupados alrededor de una postura, mal condicionados
para identificar nada. Éste sortea configuraciones **completas** del brazo y
mide **las siete articulaciones a la vez**, porque el par que sostiene cada una
se lee en la misma postura sin coste extra. Con la misma duración salen siete
veces más datos, y repartidos por el espacio de trabajo.

La separación de fricción es la misma de F2.1 y sigue siendo obligatoria: se
llega a cada configuración desde los dos sentidos, moviendo todas las
articulaciones a la vez en el sentido de su propia coordenada.

    tau_arriba = g + f        (llegando por valores crecientes)
    tau_abajo  = g - f
    g = (arriba + abajo)/2    f = (arriba - abajo)/2

Las configuraciones se sortean respetando los topes de autocolisión, incluido
el condicional hombro-codo, y el portero del cliente vuelve a comprobarlo en
cada ciclo.

Uso:
    python3 scripts/06_debug_mode.py enter
    python3 scripts/12_gravity_map.py --channel lowcmd --n 30
    python3 scripts/13_gravity_fit.py            # el ajuste, sin robot
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

from _common import add_common_args, build_client, confirm
from h1_2_joint_control import config as cfg
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX, BY_NAME, resolve

# Rangos de muestreo, en grados. Más estrechos que los topes: se busca cubrir
# el espacio de trabajo útil, no explorar los extremos con el brazo cargado.
RANGOS = {
    "shoulder_pitch": (-60.0, 25.0),
    "shoulder_roll": (8.0, 45.0),      # el mínimo real lo pone el codo
    "shoulder_yaw": (-40.0, 40.0),
    "elbow": (5.0, 100.0),
    "wrist_roll": (-45.0, 45.0),
    "wrist_pitch": (-20.0, 20.0),
    "wrist_yaw": (-45.0, 45.0),
}


def sortea(gains, lado, rng, intentos=200):
    """Una configuración admisible del brazo, o None."""
    pref = "L_" if lado == "left" else "R_"
    signo = 1.0 if lado == "left" else -1.0
    for _ in range(intentos):
        cfg_deg = {}
        # el codo primero: de él depende el mínimo del hombro
        lo, hi = RANGOS["elbow"]
        cfg_deg["elbow"] = rng.uniform(lo, hi)
        idx_roll = BY_NAME[pref + "shoulder_roll"].idx
        rmin = math.degrees(gains.roll_min_abs(
            idx_roll, math.radians(cfg_deg["elbow"]))) + 1.0
        lo, hi = RANGOS["shoulder_roll"]
        cfg_deg["shoulder_roll"] = rng.uniform(max(lo, rmin), hi)
        for k in ("shoulder_pitch", "shoulder_yaw", "wrist_roll",
                  "wrist_pitch", "wrist_yaw"):
            lo, hi = RANGOS[k]
            cfg_deg[k] = rng.uniform(lo, hi)

        q = {}
        ok = True
        for k, v in cfg_deg.items():
            idx = BY_NAME[pref + k].idx
            val = math.radians(v) * (signo if "roll" in k or "yaw" in k else 1.0)
            lo_r, hi_r = gains.limits(idx)
            if not (lo_r <= val <= hi_r):
                ok = False
                break
            q[idx] = val
        if not ok:
            continue
        if not gains.pair_ok(idx_roll, q[idx_roll],
                             q[BY_NAME[pref + "elbow"].idx]):
            continue
        return q
    return None


def mide(cli, q_obj, retro, asentar, medir_s, speed, vigilados):
    """Llega a `q_obj` desde los dos sentidos y devuelve (tau_up, tau_dn, q)."""
    lecturas = []
    for signo in (+1.0, -1.0):
        previo = {i: cli.gains.clamp(i, v - signo * retro)
                  for i, v in q_obj.items()}
        cli.ramp_to(previo, speed=speed)
        cli.sleep(0.25)
        cli.ramp_to(q_obj, speed=speed)
        cli.sleep(asentar)
        acc = {i: [] for i in vigilados}
        for _ in range(max(int(medir_s / 0.02), 5)):
            for i in vigilados:
                acc[i].append(cli.tau(i))
            cli.sleep(0.02)
        lecturas.append({i: float(np.mean(v)) for i, v in acc.items()})
    return lecturas[0], lecturas[1], cli.q_all()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--arm", choices=["left", "right"], default="left")
    ap.add_argument("--n", type=int, default=30, help="configuraciones a sortear")
    ap.add_argument("--back", type=float, default=0.10,
                    help="retroceso para llegar desde cada sentido, en rad")
    ap.add_argument("--settle", type=float, default=0.7)
    ap.add_argument("--measure", type=float, default=0.7)
    ap.add_argument("--speed", type=float, default=0.5, help="rad/s de las rampas")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    objetivos = resolve(f"{a.arm}_arm")
    gains = cfg.load(a.gains)
    rng = np.random.default_rng(a.seed)

    print(f"\n  Brazo          : {a.arm}, {len(objetivos)} articulaciones")
    print(f"  Configuraciones: {a.n} sorteadas, respetando topes y autocolisión")
    print(f"  Por cada una   : llegada desde los dos sentidos, se leen las 7")
    print(f"  Datos          : {a.n * len(objetivos)} medidas de (postura, par)")
    est = a.n * 2 * (a.settle + a.measure + 2.2)
    print(f"  Duración aprox.: {est/60:.1f} min")
    confirm(a)

    cli = build_client(a, objetivos)
    puntos, stamp = [], rec.stamp()
    try:
        cli.wait_for_state()
        cli.engage()
        q_inicio = {i: float(cli.q0[i]) for i in objetivos}

        n_ok = 0
        for k in range(a.n):
            q_obj = sortea(gains, a.arm, rng)
            if q_obj is None:
                print(f"  [{k+1}/{a.n}] sin configuración admisible, se salta")
                continue
            up, dn, qfull = mide(cli, q_obj, a.back, a.settle, a.measure,
                                 a.speed, objetivos)
            fila = {"q_full": [float(x) for x in qfull], "joints": {}}
            for i in objetivos:
                g = 0.5 * (up[i] + dn[i])
                f = 0.5 * (up[i] - dn[i])
                fila["joints"][BY_INDEX[i].name] = {
                    "idx": i, "q": float(qfull[i]), "tau_up": up[i],
                    "tau_dn": dn[i], "tau_g": g, "tau_f": f}
            puntos.append(fila)
            n_ok += 1
            res = "  ".join(f"{BY_INDEX[i].name.split('_',1)[1][:5]}"
                            f"={fila['joints'][BY_INDEX[i].name]['tau_g']:+5.2f}"
                            for i in objetivos[:4])
            print(f"  [{k+1}/{a.n}] pitch={math.degrees(qfull[objetivos[0]]):+6.1f}° "
                  f"roll={math.degrees(qfull[objetivos[1]]):+6.1f}° "
                  f"codo={math.degrees(qfull[objetivos[3]]):+6.1f}° | {res}")

        cli.ramp_to(q_inicio, speed=0.4)

    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
    except KeyboardInterrupt:
        print("\n  interrumpido")
    finally:
        cli.__exit__(None, None, None)

    if not puntos:
        return 1
    salida = cfg.LOG_DIR / f"gravity_map_{a.arm}_{stamp}.json"
    rec.save_meta({"stamp": stamp, "arm": a.arm, "gains": gains.set_name,
                   "n": len(puntos), "puntos": puntos}, salida)

    print(f"\n  {len(puntos)} configuraciones × {len(objetivos)} articulaciones "
          f"= {len(puntos)*len(objetivos)} medidas")
    print(f"\n  {'articulación':<18}{'|tau_g| máx':>13}{'|f| medio':>11}{'|f| máx':>10}")
    for i in objetivos:
        n = BY_INDEX[i].name
        gs = [abs(p["joints"][n]["tau_g"]) for p in puntos]
        fs = [abs(p["joints"][n]["tau_f"]) for p in puntos]
        print(f"  {n:<18}{max(gs):>12.2f}N{np.mean(fs):>10.2f}N{max(fs):>9.2f}N")
    print(f"\n  datos -> {salida}")
    print(f"  ajuste -> python3 scripts/13_gravity_fit.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
