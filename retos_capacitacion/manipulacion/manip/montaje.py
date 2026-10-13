"""El montaje (6.1.2 del PDF): mesa, labio, asas y ArUco de las esquinas, medidos con la camara y una
calibracion; y lo que el plan fija sobre ella (zona de recogida, bandejas, cruz), que lee el resto.

Marco de la mesa: origen en el centro del borde cercano de la superficie util (entre los ArUco 4 y 5),
x hacia el borde lejano, z la normal del plano (arriba), y = z x x (a la izquierda del robot). La zona,
las bandejas y la cruz van en el marco del torso: lo que importa es el alcance del robot.
"""

import math

import numpy as np

from .geometria import Intrinsecos, Plano, T, aplicar, inv, nube
from .calibracion import detectar, detector

IDS_MESA = {"lejana_izq": 2, "lejana_der": 3, "cercana_izq": 4, "cercana_der": 5}
MARGEN_MARCADOR = 0.04      # del centro de un ArUco de las esquinas al borde de la superficie util (papel de 7 cm)


def plano_de_profundidad(prof_m, intr_prof, cal, plano_previo=None, paso=2):
    """Plano de la mesa en el torso ajustado a la profundidad, cerca del de la calibracion."""
    P, _ = nube(prof_m, intr_prof, paso)
    P = aplicar(cal.T_torso_prof, P)
    previo = Plano(*(plano_previo or cal.plano_mesa))
    h = previo.altura(P)
    cerca = np.abs(h) < 0.04
    plano, frac, rms = Plano.ajustar(P[cerca], umbral=0.006, cerca_de=previo)
    return plano, frac, rms


def marcadores_en_mesa(img, intr, T_torso_cam, plano, det, ids=None):
    """{id: (centro (3,), esquinas (4, 3))} de los ArUco detectados, cortando los rayos con el plano."""
    sal = {}
    o = T_torso_cam[:3, 3]
    for i, uv in detectar(img, det, ids).items():
        dirs = intr.rayos(uv) @ T_torso_cam[:3, :3].T
        pts = [plano.corte_rayo(o, dv) for dv in dirs]
        if any(p is None for p in pts):
            continue
        pts = np.array(pts)
        sal[i] = (pts.mean(0), pts)
    return sal


def marco_mesa(centros, plano):
    """T_torso_mesa desde los centros de los ArUco 2-5 (al menos 4 y 5 y uno de los lejanos)."""
    c = {k: plano.proyectar(np.asarray(v, float)) for k, v in centros.items()}
    cerca = (c[4] + c[5]) / 2
    lejos = [c[i] for i in (2, 3) if i in c]
    if not lejos:
        raise ValueError("hace falta un ArUco lejano (2 o 3)")
    # x: perpendicular al borde cercano (4 -> 5), hacia los lejanos
    borde = c[4] - c[5]
    x = np.cross(borde, plano.n)
    x /= np.linalg.norm(x)
    if x @ (np.mean(lejos, 0) - cerca) < 0:
        x = -x
    y = np.cross(plano.n, x)
    R = np.column_stack([x, y, plano.n])
    origen = cerca - MARGEN_MARCADOR * x
    return T(R, origen)


def medir(f, cal, montaje_previo=None, dicc="DICT_4X4_50"):
    """Mide la mesa en un Fotograma (con color, IR y profundidad). Devuelve (mesa, informe)."""
    from .fuentes import intrinsecos, prof_en_metros
    prof = prof_en_metros(f)
    i_prof = intrinsecos(f, "prof")
    plano_prev = None
    if montaje_previo and "mesa" in montaje_previo:
        p = montaje_previo["mesa"]["plano"]
        plano_prev = (np.array(p["n"]), p["d"])
    plano, frac, rms = plano_de_profundidad(prof, i_prof, cal, plano_prev)
    det = detector(dicc)
    ids = set(IDS_MESA.values())
    marcas = {}
    if f.ir is not None:
        marcas.update(marcadores_en_mesa(f.ir, intrinsecos(f, "ir"), cal.T_torso_prof, plano, det, ids))
    if f.color is not None:     # el color completa lo que el IR no vio (con el emisor, a veces falla)
        for i, v in marcadores_en_mesa(f.color, intrinsecos(f, "color"), cal.T_torso_color, plano, det, ids).items():
            marcas.setdefault(i, v)
    centros = {i: v[0] for i, v in marcas.items()}
    T_mesa = marco_mesa(centros, plano)
    en_mesa = {i: aplicar(inv(T_mesa), c) for i, c in centros.items()}
    xs = [p[0] for i, p in en_mesa.items()]
    ys_izq = [en_mesa[i][1] for i in (2, 4) if i in en_mesa]
    ys_der = [en_mesa[i][1] for i in (3, 5) if i in en_mesa]
    lejos = [en_mesa[i][0] for i in (2, 3) if i in en_mesa]
    sup = {"x": [0.0, float(np.mean(lejos) + MARGEN_MARCADOR)],
           "y": [float(np.mean(ys_der) - MARGEN_MARCADOR), float(np.mean(ys_izq) + MARGEN_MARCADOR)]}
    # lo que sobresale: labio (alrededor), asas (a los lados) y lo que haya encima
    P, _ = nube(prof, i_prof, 2)
    Pm = aplicar(inv(T_mesa) @ cal.T_torso_prof, P)
    x0, x1 = sup["x"]
    y0, y1 = sup["y"]
    dentro = (Pm[:, 0] > x0 + 0.03) & (Pm[:, 0] < x1 - 0.03) & (Pm[:, 1] > y0 + 0.03) & (Pm[:, 1] < y1 - 0.03)
    labio = ((Pm[:, 0] > x0 + 0.05) & (Pm[:, 0] < x1 - 0.05) & (np.abs(Pm[:, 2]) < 0.06)
             & (((Pm[:, 1] > y1 - 0.005) & (Pm[:, 1] < y1 + 0.03)) | ((Pm[:, 1] < y0 + 0.005) & (Pm[:, 1] > y0 - 0.03))))
    alto_labio = float(np.percentile(Pm[labio, 2], 90)) if labio.sum() > 50 else None
    asas = {}
    for nombre, signo, borde in (("izq", 1, y1), ("der", -1, y0)):
        sel = ((signo * (Pm[:, 1] - borde) > -0.02) & (signo * (Pm[:, 1] - borde) < 0.20)
               & (Pm[:, 2] > 0.06) & (Pm[:, 2] < 0.30) & (Pm[:, 0] > x0 - 0.10) & (Pm[:, 0] < x1 + 0.10))
        if sel.sum() < 50:
            continue
        A = Pm[sel]
        asas[nombre] = {"y": round(float(np.median(A[:, 1])), 3), "alto": round(float(np.percentile(A[:, 2], 97)), 3),
                        "x": [round(float(np.percentile(A[:, 0], 2)), 3), round(float(np.percentile(A[:, 0], 98)), 3)]}
    sobre = dentro & (Pm[:, 2] > 0.02) & (Pm[:, 2] < 0.30)
    mesa = {"plano": {"n": [round(float(v), 6) for v in plano.n], "d": round(plano.d, 5)},
            "T_torso_mesa": [[round(float(v), 6) for v in fila] for fila in T_mesa],
            "yaw_deg": round(math.degrees(math.atan2(T_mesa[1, 0], T_mesa[0, 0])), 2),
            "superficie": {k: [round(v, 3) for v in vals] for k, vals in sup.items()},
            "labio_alto_m": None if alto_labio is None else round(alto_labio, 3),
            "asas": asas,
            "aruco_torso": {int(i): [round(float(v), 4) for v in c] for i, c in sorted(centros.items())}}
    informe = {"plano_inliers": round(frac, 3), "plano_rms_mm": round(1000 * rms, 2),
               "aruco_vistos": sorted(int(i) for i in centros), "puntos_sobre_la_mesa": int(sobre.sum()),
               "lados_aruco_mm": {int(i): round(1000 * float(np.mean([np.linalg.norm(v[1][k] - v[1][(k + 1) % 4])
                                                                     for k in range(4)])), 1) for i, v in marcas.items()},
               "distancias_mm": {f"{a}-{b}": round(1000 * float(np.linalg.norm(centros[a] - centros[b])), 1)
                                 for a, b in ((4, 5), (2, 3), (4, 2), (5, 3)) if a in centros and b in centros}}
    return mesa, informe


def T_torso_mesa(montaje):
    return np.array(montaje["mesa"]["T_torso_mesa"], float)


def plano(montaje):
    p = montaje["mesa"]["plano"]
    return Plano(p["n"], p["d"])


def punto_en_mesa(montaje, xy, altura=0.0):
    """(x, y) del torso -> punto sobre la mesa (torso) a `altura` del plano."""
    pl = plano(montaje)
    p = np.array([xy[0], xy[1], 0.0])
    # en la vertical del torso (z): corte con el plano y subir por su normal
    z = -(pl.n[0] * p[0] + pl.n[1] * p[1] + pl.d) / pl.n[2]
    return np.array([p[0], p[1], z]) + altura * pl.n


def dentro_de_zona(montaje, p, margen=0.0):
    z = montaje["zona_recogida"]
    return (z["x"][0] - margen <= p[0] <= z["x"][1] + margen) and (z["y"][0] - margen <= p[1] <= z["y"][1] + margen)
