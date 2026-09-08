#!/usr/bin/env python3
"""Identifica los parámetros de masa del brazo a partir del mapa. SIN robot.

El par de gravedad es **lineal en los parámetros de masa**:

    tau_g(q) = R(q) · pi        pi = [m, m·cx, m·cy, m·cz] por cuerpo

`R(q)` sale de `pinocchio` con velocidad y aceleración nulas y depende solo de
la CINEMÁTICA del URDF, que es geometría de eslabones y es lo que un URDF suele
tener bien. Lo que está mal son las masas, y eso es justo lo que se ajusta.

Es mejor que una regresión ciega: extrapola a posturas no medidas, respeta la
estructura del problema, necesita muchos menos puntos, y de regalo dice cuánto
pesa de verdad cada eslabón.

**Regularización de cresta hacia el URDF.** Muchas direcciones del espacio de
parámetros no son identificables con estos datos —el centro de masa de una
falange, por ejemplo, no afecta a nada medible—. Sin regularizar, el ajuste les
asigna valores arbitrarios que encajan el ruido. Con cresta hacia el prior del
URDF, las direcciones que los datos no ven se quedan donde estaban y solo se
mueven las que sí se observan.

Uso:
    python3 scripts/13_gravity_fit.py                    # el mapa más reciente
    python3 scripts/13_gravity_fit.py --lambda 0.05
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from h1_2_joint_control import config as cfg
from h1_2_joint_control.joints import BY_INDEX, BY_NAME, NUM_CMD_MOTOR

URDF = Path.home() / "humanoid_ws/src/h1_2_utec/h1_2_description/urdf/h1_2.urdf"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default=None, help="fichero del mapa (por defecto, el último)")
    ap.add_argument("--lam", type=float, default=0.03,
                    help="peso de la regularización hacia el URDF")
    ap.add_argument("--test-frac", type=float, default=0.3,
                    help="fracción de puntos reservada para validar")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    import pinocchio as pin

    ruta = Path(a.map) if a.map else Path(sorted(glob.glob(
        str(cfg.LOG_DIR / "gravity_map_*.json")))[-1])
    d = json.loads(ruta.read_text())
    pts = d["puntos"]
    print(f"\n  mapa: {ruta.name}  ({len(pts)} configuraciones)")

    m = pin.buildModelFromUrdf(str(URDF))
    dat = m.createData()
    nb = m.njoints - 1
    prior = np.concatenate([I.toDynamicParameters()[:4] for I in list(m.inertias)[1:]])

    # columnas de masa (4 de cada 10) y filas de las articulaciones medidas
    col_masa = np.concatenate([np.arange(10 * k, 10 * k + 4) for k in range(nb)])

    # solo los cuerpos que pueden influir: los distales al hombro del brazo medido
    brazo = d.get("arm", "left")
    raiz = m.getJointId(f"{brazo}_shoulder_pitch_joint")
    cuerpos = [k for k in range(nb) if raiz in list(m.supports[k + 1])]
    activos = np.concatenate([np.arange(4 * k, 4 * k + 4) for k in cuerpos])
    print(f"  cuerpos que influyen: {len(cuerpos)}  ->  {len(activos)} parámetros")

    filas, obj, etiquetas = [], [], []
    z = np.zeros(m.nv)
    for p in pts:
        q27 = p["q_full"]
        q = pin.neutral(m)
        for i in range(NUM_CMD_MOTOR):
            n = BY_INDEX[i].urdf
            if m.existJointName(n):
                q[m.joints[m.getJointId(n)].idx_q] = q27[i]
        R = pin.computeJointTorqueRegressor(m, dat, q, z, z)[:, col_masa]
        for nombre, v in p["joints"].items():
            fila_v = m.joints[m.getJointId(BY_NAME[nombre].urdf)].idx_v
            filas.append(R[fila_v])
            obj.append(v["tau_g"])
            etiquetas.append(nombre)
    A = np.array(filas)
    b = np.array(obj)
    etiquetas = np.array(etiquetas)
    print(f"  ecuaciones: {A.shape[0]}")

    rng = np.random.default_rng(a.seed)
    idx = rng.permutation(len(b))
    n_test = int(a.test_frac * len(b))
    te, tr = idx[:n_test], idx[n_test:]

    def ajusta(ii, lam):
        Aa = A[ii][:, activos]
        # residuo respecto al prior: se ajusta la CORRECCIÓN, no el valor
        r = b[ii] - A[ii] @ prior
        esc = np.maximum(np.abs(Aa).max(axis=0), 1e-9)
        An = Aa / esc
        M = An.T @ An + lam * np.trace(An.T @ An) / An.shape[1] * np.eye(An.shape[1])
        dx = np.linalg.solve(M, An.T @ r) / esc
        pi = prior.copy()
        pi[activos] += dx
        return pi

    print(f"\n  {'lambda':>8}{'rms train':>12}{'rms test':>11}{'max test':>10}"
          f"{'masa brazo':>12}{'masa mano':>11}")
    mano = [k for k in cuerpos
            if m.getJointId(f"{brazo}_wrist_yaw_joint") in list(m.supports[k + 1])]
    mejor = (1e9, None, None)
    for lam in (0.3, 0.1, 0.03, 0.01, 0.003):
        pi = ajusta(tr, lam)
        e_tr = b[tr] - A[tr] @ pi
        e_te = b[te] - A[te] @ pi
        m_brazo = sum(pi[4 * k] for k in cuerpos)
        m_mano = sum(pi[4 * k] for k in mano)
        rms_te = float(np.sqrt(np.mean(e_te ** 2)))
        print(f"  {lam:>8.3f}{np.sqrt(np.mean(e_tr**2)):>11.3f}N{rms_te:>10.3f}N"
              f"{np.max(np.abs(e_te)):>9.3f}N{m_brazo:>11.3f}kg{m_mano:>10.3f}kg")
        if rms_te < mejor[0]:
            mejor = (rms_te, lam, pi)

    rms_te, lam, pi = mejor
    e0 = b - A @ prior
    e1 = b - A @ pi
    print(f"\n  Mejor lambda: {lam}")
    print(f"\n  {'':<22}{'URDF de partida':>18}{'identificado':>15}")
    print(f"  {'rms sobre todo':<22}{np.sqrt(np.mean(e0**2)):>17.3f}N"
          f"{np.sqrt(np.mean(e1**2)):>14.3f}N")
    print(f"  {'error máximo':<22}{np.max(np.abs(e0)):>17.3f}N"
          f"{np.max(np.abs(e1)):>14.3f}N")
    print(f"  {'masa del brazo':<22}{sum(prior[4*k] for k in cuerpos):>17.3f}kg"
          f"{sum(pi[4*k] for k in cuerpos):>13.3f}kg")
    print(f"  {'masa de la mano':<22}{sum(prior[4*k] for k in mano):>17.3f}kg"
          f"{sum(pi[4*k] for k in mano):>13.3f}kg")

    print(f"\n  Error por articulación (rms, Nm):")
    print(f"  {'articulación':<20}{'URDF':>9}{'identificado':>15}{'mejora':>9}")
    for n in dict.fromkeys(etiquetas):
        s = etiquetas == n
        r0 = float(np.sqrt(np.mean(e0[s] ** 2)))
        r1 = float(np.sqrt(np.mean(e1[s] ** 2)))
        print(f"  {n:<20}{r0:>8.3f}N{r1:>14.3f}N{100*(1-r1/r0) if r0>0 else 0:>8.0f}%")

    neg = [BY_INDEX for k in cuerpos if pi[4 * k] < 0]
    print(f"\n  Masas negativas tras el ajuste: {len(neg)} "
          f"{'(ninguna: el resultado es físicamente admisible)' if not neg else '⚠'}")

    salida = cfg.LOG_DIR / f"gravity_params_{d['stamp']}.json"
    salida.write_text(json.dumps({
        "urdf": str(URDF), "map": ruta.name, "lambda": lam, "arm": brazo,
        "rms_prior": float(np.sqrt(np.mean(e0 ** 2))),
        "rms_fit": float(np.sqrt(np.mean(e1 ** 2))),
        "params": pi.tolist(), "bodies": cuerpos,
    }, indent=2), encoding="utf-8")
    print(f"\n  parámetros -> {salida}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
