#!/usr/bin/env python3
"""Abre en el visor de MuJoCo el H1-2 con las manos Inspire, quieto, para mirarlas (colores, conector y acople).

    python3 ver_manos.py              manos abiertas, la cámara sobre la mano izquierda
    python3 ver_manos.py 0.6          los 6 DOF al 60 % del comando: s = 0.6, ANGLE_SET = 400
    python3 ver_manos.py --angle-set=1000,1000,500,500,1000,0
                                      ANGLE_SET de las dos manos, en el orden del SDK: meñique,
                                      anular, medio, índice, flexión y rotación del pulgar
    python3 ver_manos.py der          la cámara empieza sobre la mano derecha (se puede combinar: der 0.6)
    python3 ver_manos.py --mimic 0.6  el modelo con <mimic> (acople lineal), para comparar: cada junta
                                      actuada al 60 % de su límite y las falanges con su multiplicador

Acople por software (por defecto): las 12 juntas de cada mano salen de las curvas medidas en la mano
real (config/hand_coupling.yaml del paquete, con hand_coupling.py), sin pasar por el modelo <mimic>.

Lee el URDF y las mallas de ros_h1_2_ws/src/h1_2_inspire_description con modelo_h1_2_manos.py. Ratón:
arrastrar con el botón izquierdo gira, con el derecho desplaza, la rueda acerca. Para diseñar rutinas, el
editor (editor_mujoco.sh).
"""
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco                       # noqa: E402
import mujoco.viewer                # noqa: E402
import numpy as np                  # noqa: E402

import modelo_h1_2_manos as M       # noqa: E402


def main():
    numeros = [a for a in sys.argv[1:] if not a.startswith("--") and a not in ("izq", "der")]
    cierre = min(max(float(numeros[0]), 0.0), 1.0) if numeros else 0.0
    lado, signo = ("right", -1.0) if "der" in sys.argv[1:] else ("left", 1.0)
    segundos = next((float(a.split("=")[1]) for a in sys.argv[1:] if a.startswith("--segundos=")), None)
    angle_set = next(([float(v) for v in a.split("=", 1)[1].split(",")]
                      for a in sys.argv[1:] if a.startswith("--angle-set=")), None)
    if angle_set is not None and len(angle_set) != 6:
        sys.exit("--angle-set necesita 6 valores: meñique, anular, medio, índice, flexión y rotación del pulgar")
    mimic = "--mimic" in sys.argv[1:]
    if mimic and angle_set is not None:
        sys.exit("--angle-set es del acople por software; con --mimic usa la fracción (0 a 1)")

    # el modelo del visor: quieto y con las mallas de colisión ocultas
    m, _ = M.modelos(M.URDF_MIMIC if mimic else M.URDF_ACOPLE)
    d = mujoco.MjData(m)
    if mimic:
        for mano in ("left", "right"):
            for dof in M.DOF_MANO:
                j = m.joint(f"{mano}_{dof}_joint")
                d.qpos[j.qposadr[0]] = cierre * j.range[1]
        M.Mimic(m).aplicar(d.qpos)
    else:
        acople = M.HandCoupling()
        for mano in ("left", "right"):
            q = (acople.joints_from_angle_set(angle_set, mano) if angle_set is not None
                 else acople.joints_from_s([cierre] * 6, mano))
            for nombre, valor in q.items():
                d.qpos[m.joint(nombre).qposadr[0]] = valor
    mujoco.mj_forward(m, d)

    # Cámara: frente al dorso de la mano, algo por delante y por el lado del pulgar. En el marco de la
    # palma los dedos van por +z, el meñique por +x y el dorso por +y (izquierda) o -y (derecha); en la
    # pose de arranque eso es dedos hacia delante, pulgar arriba y dorso hacia fuera.
    base = m.body(f"{lado}_hand_base_link").id
    giro = d.xmat[base].reshape(3, 3)
    centro = d.xpos[base] + giro @ [-0.008, 0.112 * signo, -0.02]
    hacia = -giro @ np.array([-0.3, 0.8 * signo, 0.25])           # hacia donde mira la cámara
    hacia /= np.linalg.norm(hacia)
    print(f"modelo: {M.URDF_MIMIC if mimic else M.URDF_ACOPLE}")
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.lookat[:] = centro
        v.cam.distance = 0.50
        v.cam.azimuth = float(np.degrees(np.arctan2(hacia[1], hacia[0])))
        v.cam.elevation = float(np.degrees(np.arcsin(hacia[2])))
        inicio = time.time()
        while v.is_running() and (segundos is None or time.time() - inicio < segundos):
            v.sync()
            time.sleep(0.02)
    # Como en h1_2_mujoco/simulador_fisica.py: la salida normal de Python choca con el cierre de GLFW.
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
