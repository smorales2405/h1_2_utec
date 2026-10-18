#!/usr/bin/env python3
"""Escribe config/gravedad_brazos.yaml: las masas de los brazos identificadas en h1_2_joint_control (las que
acompanan a las ganancias tuned_gff), pasadas al URDF de este reto.

    python3 herramientas/gravedad_brazos.py --urdf-origen RUTA/h1_2_description/urdf/h1_2.urdf     # en la PC

El ajuste (h1_2_joint_control/scripts/13_gravity_fit.py, 2026-09-08) se hizo sobre otro URDF (el de
h1_2_description, con otras manos): sus cuerpos de brazo coinciden con los nuestros (mismas juntas y marcos),
los de la mano no. Por eso cada mano identificada se agrupa en el cuerpo de wrist_yaw (masa y centro de masas
con los dedos a 0) y los dedos de nuestro modelo quedan sin masa. Se comprueba contra los mapas medidos: el
error rms de nuestro modelo con estas masas tiene que salir como el del ajuste (~0.34 N m).
"""

import argparse
import glob
import json
import os
import sys

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from manip import config  # noqa: E402
from manip.dinamica import JUNTAS_URDF  # noqa: E402

RUTA = os.path.join(config.CONFIG, "gravedad_brazos.yaml")
LOGS = os.path.join(config.ruta_repo(), "h1_2_joint_control", "logs")
BRAZO = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
LADO = {"left": "izq", "right": "der"}


def aplicar_params(model, d):
    """Como h1_2_joint_control.gravity.GravityModel: los parametros pi = [m, m cx, m cy, m cz] por cuerpo."""
    import pinocchio as pin
    pi = np.asarray(d["params"], float)
    for k in d["bodies"]:
        m, mcx, mcy, mcz = pi[4 * k:4 * k + 4]
        if m <= 1e-9:
            continue
        model.inertias[k + 1] = pin.Inertia(float(m), np.array([mcx, mcy, mcz]) / m, model.inertias[k + 1].inertia)


def tau_g(model, q27):
    """Par de gravedad de las 27 juntas del cuerpo (base fija), con los dedos a 0."""
    import pinocchio as pin
    data = model.createData()
    q = pin.neutral(model)
    for i, n in enumerate(JUNTAS_URDF):
        q[model.joints[model.getJointId(n)].idx_q] = q27[i]
    t = pin.computeGeneralizedGravity(model, data, q)
    return np.array([t[model.joints[model.getJointId(n)].idx_v] for n in JUNTAS_URDF])


def rms(model, mapas):
    e = []
    for mp in mapas:
        for p in mp["puntos"]:
            t = tau_g(model, p["q_full"])
            for j in p["joints"].values():
                e.append(t[j["idx"]] - j["tau_g"])
    return float(np.sqrt(np.mean(np.square(e))))


def agrupar_mano(model, lado):
    """(masa, centro de masas en el marco de wrist_yaw) de wrist_yaw y todos sus descendientes, dedos a 0."""
    import pinocchio as pin
    data = model.createData()
    q = pin.neutral(model)
    pin.forwardKinematics(model, data, q)
    w = model.getJointId(f"{lado}_wrist_yaw_joint")
    oMw = data.oMi[w]
    m_tot, mc = 0.0, np.zeros(3)
    for k in range(1, model.njoints):
        j, dentro = k, False
        while j > 0:
            if j == w:
                dentro = True
                break
            j = model.parents[j]
        if not dentro:
            continue
        I = model.inertias[k]
        c = oMw.actInv(data.oMi[k].act(I.lever))
        m_tot += I.mass
        mc += I.mass * c
    return m_tot, mc / m_tot


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--urdf-origen", required=True, help="el URDF de h1_2_description con el que se hizo el ajuste")
    a.add_argument("--params", nargs="*", help="gravity_params_*.json (por defecto los de h1_2_joint_control/logs)")
    a = a.parse_args()
    import pinocchio as pin
    ficheros = a.params or sorted(glob.glob(os.path.join(LOGS, "gravity_params_*.json")))
    params = {}
    for f in ficheros:
        d = json.load(open(f))
        params[d["arm"]] = (f, d)                    # el mas reciente de cada brazo gana
    if set(params) != {"left", "right"}:
        sys.exit(f"faltan parametros de algun brazo: {sorted(params)}")
    mapas = {b: json.load(open(os.path.join(LOGS, d["map"]))) for b, (_, d) in params.items()}

    origen = pin.buildModelFromUrdf(a.urdf_origen)
    nuestro = pin.buildModelFromUrdf(config.ruta_urdf())
    sal = {}
    for b, (f, d) in sorted(params.items()):
        m_prior = rms(origen, [mapas[b]])
        ajustado = pin.buildModelFromUrdf(a.urdf_origen)
        aplicar_params(ajustado, d)
        m_fit = rms(ajustado, [mapas[b]])
        if abs(m_prior - d["rms_prior"]) > 0.01 or abs(m_fit - d["rms_fit"]) > 0.01:
            sys.exit(f"{b}: el URDF de origen no es el del ajuste (rms {m_prior:.3f}/{m_fit:.3f} frente a "
                     f"{d['rms_prior']:.3f}/{d['rms_fit']:.3f})")
        cuerpos = {}
        for n in BRAZO:
            jn = f"{b}_{n}_joint"
            ko, kn = ajustado.getJointId(jn), nuestro.getJointId(jn)
            dif = ajustado.jointPlacements[ko].inverse() * nuestro.jointPlacements[kn]
            if np.linalg.norm(dif.translation) > 1e-4 or np.linalg.norm(dif.rotation - np.eye(3)) > 1e-4:
                sys.exit(f"{jn}: el marco no coincide entre los dos URDF")
            if n == "wrist_yaw":
                m, c = agrupar_mano(ajustado, b)
            else:
                I = ajustado.inertias[ko]
                m, c = I.mass, I.lever
            cuerpos[n] = {"masa_kg": round(float(m), 4), "cdm_m": [round(float(x), 5) for x in c]}
        sal[LADO[b]] = {"cuerpos": cuerpos, "origen": os.path.basename(f), "rms_ajuste_nm": round(m_fit, 3),
                        "rms_urdf_nm": round(m_prior, 3)}
    # comprobacion: nuestro URDF con estas masas, contra los mismos mapas
    from manip.dinamica import aplicar_masas_brazos
    aplicar_masas_brazos(nuestro, sal)
    for b in params:
        sal[LADO[b]]["rms_este_urdf_nm"] = round(rms(nuestro, [mapas[b]]), 3)
        print(f"{LADO[b]}: rms URDF de origen {sal[LADO[b]]['rms_urdf_nm']:.3f} N m, ajuste {sal[LADO[b]]['rms_ajuste_nm']:.3f}, "
              f"este URDF con las masas agrupadas {sal[LADO[b]]['rms_este_urdf_nm']:.3f}; mano+muneca "
              f"{sal[LADO[b]]['cuerpos']['wrist_yaw']['masa_kg']:.3f} kg")
        if sal[LADO[b]]["rms_este_urdf_nm"] > sal[LADO[b]]["rms_ajuste_nm"] + 0.05:
            sys.exit("las masas agrupadas no reproducen el ajuste: no se escribe nada")
    cab = """Masas de los brazos para la compensacion de gravedad (ganancias tuned_gff). NO EDITAR A MANO: la escribe
herramientas/gravedad_brazos.py a partir del ajuste de h1_2_joint_control (13_gravity_fit.py, 2026-09-08).
Por cuerpo de junta (marco de la junta del URDF del reto): masa y centro de masas. wrist_yaw lleva la mano
entera agrupada (dedos a 0); los dedos del URDF quedan sin masa."""
    config.guardar_yaml(RUTA, sal, cab)
    print(f"escrito {RUTA}")


if __name__ == "__main__":
    main()
