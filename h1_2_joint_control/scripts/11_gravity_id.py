#!/usr/bin/env python3
"""Identificación de gravedad separando la fricción (fase F2.1).

El problema que resuelve. `07_gravity_ff.py` mide el par que sostiene una
postura, y eso NO es el par de gravedad: es gravedad **más fricción estática**.
La fricción se opone al último sentido de movimiento, así que en una medida de
una sola aproximación entra siempre con el mismo signo. Identificar masas con
esos números mete la fricción dentro de la masa: el modelo cuadra en estático y
sobre-compensa en movimiento, que es justo cuando `tau_ff` tiene que servir.

La separación es elemental. Sosteniendo la postura `q`, el motor equilibra la
gravedad `g` y la fricción, y la fricción apunta en contra de por dónde se
llegó:

    llegando desde abajo   tau_arriba = g + f
    llegando desde arriba   tau_abajo  = g - f

    g = (tau_arriba + tau_abajo) / 2       <- gravedad limpia
    f = (tau_arriba - tau_abajo) / 2       <- y el mapa de friccion, de regalo

El segundo número no es un subproducto menor: la fricción de Coulomb por
articulación explica la dispersión entre pasadas de `06_RESULTADOS.md` §6 y la
banda de pegado del codo. Sale del mismo ensayo sin coste extra.

Se registra la configuración COMPLETA en cada medida, porque el modelo de
gravedad la necesita entera como entrada. Los cuatro puntos que había en el
repositorio no la tenían, y por eso solo se podían contrastar de forma
aproximada.

Uso:
    source scripts/env.sh
    python3 scripts/06_debug_mode.py enter
    python3 scripts/11_gravity_id.py --channel lowcmd --joints L_shoulder_roll,L_elbow
    python3 scripts/11_gravity_id.py --channel lowcmd --joints left_arm --points 3
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from _common import (add_common_args, apply_test_posture, build_client, confirm,
                     pick_amplitude)
from h1_2_joint_control import config as cfg
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX, NUM_CMD_MOTOR, resolve

URDF = (Path.home() / "humanoid_ws/src/h1_2_utec/h1_2_description/urdf/h1_2.urdf")


def medir(cli, idx, q_obj, retroceso, asentar, medir_s, speed):
    """Sostiene `q_obj` llegando desde los dos sentidos.

    Devuelve (tau_arriba, tau_abajo, q_completo). `tau_arriba` es el par al
    llegar moviéndose en sentido positivo.
    """
    out = []
    for signo in (+1, -1):
        cli.ramp_to({idx: q_obj - signo * retroceso}, speed=speed)
        cli.sleep(0.3)
        cli.ramp_to({idx: q_obj}, speed=speed)
        cli.sleep(asentar)
        taus, n = [], max(int(medir_s / 0.02), 5)
        for _ in range(n):
            taus.append(cli.tau(idx))
            cli.sleep(0.02)
        out.append(float(np.mean(taus)))
    return out[0], out[1], cli.q_all()


def modelo_gravedad():
    """Devuelve f(q27) -> tau27 del URDF local, o None si no se puede."""
    try:
        import pinocchio as pin
    except ImportError:
        return None, "pinocchio no disponible"
    if not URDF.exists():
        return None, f"no está {URDF}"
    m = pin.buildModelFromUrdf(str(URDF))
    d = m.createData()
    # mapa de nuestros 27 motores a las juntas del modelo
    idx_q, idx_v = {}, {}
    for i in range(NUM_CMD_MOTOR):
        n = BY_INDEX[i].urdf
        if m.existJointName(n):
            j = m.joints[m.getJointId(n)]
            idx_q[i] = j.idx_q
            idx_v[i] = j.idx_v
    faltan = [BY_INDEX[i].name for i in range(NUM_CMD_MOTOR) if i not in idx_q]

    def g(q27):
        q = pin.neutral(m)
        for i, k in idx_q.items():
            q[k] = float(q27[i])
        t = pin.computeGeneralizedGravity(m, d, q)
        return {i: float(t[idx_v[i]]) for i in idx_v}

    return g, (f"faltan del URDF: {faltan}" if faltan else
               f"{len(idx_q)}/{NUM_CMD_MOTOR} motores mapeados")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--joints", required=True)
    ap.add_argument("--points", type=int, default=3,
                    help="ángulos por articulación")
    ap.add_argument("--span", type=float, default=0.45,
                    help="recorrido total explorado, en rad")
    ap.add_argument("--back", type=float, default=0.12,
                    help="retroceso para llegar desde cada sentido, en rad")
    ap.add_argument("--settle", type=float, default=0.8)
    ap.add_argument("--measure", type=float, default=0.8)
    a = ap.parse_args()

    objetivos = resolve(a.joints)
    gains = cfg.load(a.gains)
    g_mod, nota = modelo_gravedad()

    print(f"\n  Articulaciones : {', '.join(BY_INDEX[i].name for i in objetivos)}")
    print(f"  Ganancias      : '{gains.set_name}'")
    print(f"  Puntos         : {a.points} por articulación, recorrido {a.span:.2f} rad")
    print(f"  Método         : llegar a cada punto desde los DOS sentidos "
          f"(retroceso {a.back:.2f} rad)")
    print(f"  Modelo         : {'cargado — ' + nota if g_mod else 'NO — ' + nota}")
    est = len(objetivos) * a.points * 2 * (a.settle + a.measure + 2.0)
    print(f"  Duración aprox.: {est/60:.1f} min")
    confirm(a)

    cli = build_client(a, objetivos)
    filas, stamp = [], rec.stamp()
    try:
        cli.wait_for_state()
        cli.engage()
        apply_test_posture(cli, a, gains)

        for n, idx in enumerate(objetivos, 1):
            j = BY_INDEX[idx]
            q0 = float(cli.q_base[idx])
            lo, hi = gains.limits_dynamic(
                idx, float(cli.q_base[gains.elbow_of(idx)])
                if gains.elbow_of(idx) is not None else None)
            # puntos repartidos alrededor de q0, dentro de los topes y dejando
            # sitio para el retroceso en los dos sentidos
            margen = a.back + 0.02
            centros = np.linspace(q0 - a.span / 2, q0 + a.span / 2, a.points)
            centros = [c for c in centros if lo + margen <= c <= hi - margen]
            if not centros:
                centros = [min(max(q0, lo + margen), hi - margen)]
            print(f"\n  [{n}/{len(objetivos)}] {j.name}  "
                  f"{len(centros)} punto(s) entre {math.degrees(min(centros)):+.1f}° "
                  f"y {math.degrees(max(centros)):+.1f}°")

            for c in centros:
                t_up, t_dn, qfull = medir(cli, idx, float(c), a.back,
                                          a.settle, a.measure, speed=0.3)
                tau_g = 0.5 * (t_up + t_dn)
                tau_f = 0.5 * (t_up - t_dn)
                real = float(qfull[idx])
                mod = g_mod(qfull)[idx] if g_mod else float("nan")
                err = mod - tau_g
                print(f"      q={math.degrees(real):+7.2f}°  "
                      f"tau↑={t_up:+6.2f} tau↓={t_dn:+6.2f}  ->  "
                      f"g={tau_g:+6.2f} f=±{abs(tau_f):5.2f} Nm  |  "
                      f"modelo {mod:+6.2f}  err {err:+6.2f} Nm")
                filas.append(dict(joint=j.name, idx=idx, q=real, tau_up=t_up,
                                  tau_dn=t_dn, tau_g=tau_g, tau_f=tau_f,
                                  tau_modelo=mod, err=err,
                                  q_full=[float(x) for x in qfull]))
            cli.ramp_to({idx: q0}, speed=0.3)

    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
    except KeyboardInterrupt:
        print("\n  interrumpido")
    finally:
        cli.__exit__(None, None, None)

    if not filas:
        return 1

    salida = cfg.LOG_DIR / f"gravity_id_{stamp}.json"
    rec.save_meta({"stamp": stamp, "gains": gains.set_name, "urdf": str(URDF),
                   "puntos": filas}, salida)

    print("\n\n  ═══ Gravedad y fricción por articulación ══════════════════════")
    print(f"  {'articulación':<18}{'puntos':>7}{'|f| medio':>11}{'|f| máx':>9}"
          f"{'err modelo medio':>18}{'err máx':>9}")
    for nombre in dict.fromkeys(f["joint"] for f in filas):
        sub = [f for f in filas if f["joint"] == nombre]
        fs = [abs(f["tau_f"]) for f in sub]
        es = [abs(f["err"]) for f in sub if f["err"] == f["err"]]
        print(f"  {nombre:<18}{len(sub):>7}{np.mean(fs):>10.2f}N{max(fs):>8.2f}N"
              f"{(np.mean(es) if es else float('nan')):>17.2f}N"
              f"{(max(es) if es else float('nan')):>8.2f}N")

    todos_f = [abs(f["tau_f"]) for f in filas]
    todos_e = [abs(f["err"]) for f in filas if f["err"] == f["err"]]
    print(f"\n  Fricción estática: media {np.mean(todos_f):.2f} Nm, "
          f"máxima {max(todos_f):.2f} Nm")
    if todos_e:
        print(f"  Error del modelo contra la gravedad LIMPIA: "
              f"media {np.mean(todos_e):.2f} Nm, máxima {max(todos_e):.2f} Nm")
        print(f"  (criterio del protocolo: < max(1.0 Nm, 15 %))")
        print(f"\n  Para comparar, el mismo modelo contra `tau_est` sin separar "
              f"fricción\n  daba hasta 3.74 Nm de error en los cuatro puntos "
              f"antiguos.")
    print(f"\n  datos -> {salida}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
