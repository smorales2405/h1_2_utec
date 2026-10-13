"""Calibracion camara-torso (6.1.3 del PDF): fichero con fecha, semilla del URDF y ajuste por minimos cuadrados.

Incognitas: T_torso_color (la pose del marco optico del color en el torso) y, por mano, T_dorso (el
marcador ArUco del dorso en *_wrist_yaw_link). Datos: el brazo en varias poses, con la q MEDIDA (6.1.4:
con la consigna se calibraria tambien la caida del brazo) y las cuatro esquinas del marcador detectadas en
el color. Residuo: la reproyeccion de las esquinas, en pixeles, con perdida robusta.

Arranque: con la camara del URDF, solvePnP de cada muestra da T_dorso por muestra; su media es la semilla
(incluido el giro del marcador sobre la mano, que no se conoce). La profundidad pasa al torso por el
extrinseco de fabrica profundidad -> color, que no se calibra.
"""

import datetime
import math

import cv2
import numpy as np
import yaml

from . import config
from .geometria import (R_LINK_OPTICO, Intrinsecos, Plano, T, T_a_vector, aplicar, inv, kabsch,
                        vector_a_T)
from .mensajes import LADOS, Calibracion

# Esquinas del marcador en su marco (OpenCV, SOLVEPNP_IPPE_SQUARE): sup-izq, sup-der, inf-der, inf-izq
def esquinas_marcador(lado_m):
    s = lado_m / 2
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], float)


# Marcador del dorso por defecto (semilla): centrado en la placa, normal hacia fuera del dorso, y con los
# dedos hacia abajo en la foto ArUcos_manos.jpeg (el "arriba" del marcador mira a la muneca).
T_DORSO_DEFECTO = {
    "izq": T(np.column_stack([[0, 0, -1], [-1, 0, 0], [0, 1, 0]]), [0.155, 0.036, 0.0]),
    "der": T(np.column_stack([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]), [0.155, -0.036, 0.0]),
}


def a_lista(M):
    return [[round(float(v), 7) for v in fila] for fila in np.asarray(M)]


def guardar(cal: Calibracion, ruta=None, extra=None):
    ruta = ruta or f"{config.CONFIG}/calibracion_{cal.fecha.replace('-', '').replace(':', '').replace(' ', '_')}.yaml"
    n, d = cal.plano_mesa
    datos = {"fecha": cal.fecha, "origen": cal.origen, "n_poses": cal.n_poses,
             "residuo_px": None if math.isnan(cal.residuo_px) else round(cal.residuo_px, 3),
             "residuo_mm": None if math.isnan(cal.residuo_mm) else round(cal.residuo_mm, 2),
             "T_torso_color": a_lista(cal.T_torso_color), "T_color_prof": a_lista(cal.T_color_prof),
             "T_dorso": {k: a_lista(v) for k, v in cal.T_dorso.items()},
             "plano_mesa": {"n": [round(float(v), 6) for v in n], "d": round(float(d), 5)}}
    if extra:
        datos["detalle"] = extra
    config.guardar_yaml(ruta, datos, "Escrito por herramientas/calibrar_camara_torso.py: no editar a mano.\n"
                        "T_a_b lleva puntos del marco b al a. Marcos opticos: z adelante, x derecha, y abajo.")
    return ruta


def cargar(ruta=None) -> Calibracion:
    ruta = config.ruta_calibracion(ruta)
    if ruta is None:
        raise FileNotFoundError("no hay config/calibracion_*.yaml: usar calibracion.desde_urdf()")
    with open(ruta) as f:
        d = yaml.safe_load(f)
    return Calibracion(fecha=str(d["fecha"]), T_torso_color=np.array(d["T_torso_color"]),
                       T_color_prof=np.array(d["T_color_prof"]),
                       T_dorso={k: np.array(v) for k, v in d["T_dorso"].items()},
                       plano_mesa=(np.array(d["plano_mesa"]["n"]), float(d["plano_mesa"]["d"])),
                       residuo_px=float(d.get("residuo_px") or "nan"), residuo_mm=float(d.get("residuo_mm") or "nan"),
                       n_poses=int(d.get("n_poses") or 0), origen=d.get("origen", ""))


def desde_urdf(cin, T_color_prof, plano_mesa=None) -> Calibracion:
    """Calibracion provisional: camera_link del URDF tomado como el IR izquierdo (la profundidad), sin
    calibrar. La mesa, a 0.118 m bajo la pelvis si no se da (medida del 2026-10-09)."""
    T_torso_prof = cin.T_camara_link() @ T(R_LINK_OPTICO)
    plano = plano_mesa or (np.array([0.0, 0.0, 1.0]), 0.118)
    return Calibracion(fecha=datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
                       T_torso_color=T_torso_prof @ inv(T_color_prof), T_color_prof=np.asarray(T_color_prof, float),
                       T_dorso={k: v.copy() for k, v in T_DORSO_DEFECTO.items()}, plano_mesa=plano, origen="urdf")


def cargar_o_urdf(cin, meta=None, ruta=None) -> Calibracion:
    """La calibracion mas reciente o, si no hay, la del URDF (con el extrinseco de la camara)."""
    try:
        return cargar(ruta)
    except FileNotFoundError:
        from .fuentes import T_color_prof
        return desde_urdf(cin, T_color_prof(meta or {}))


# --- deteccion -----------------------------------------------------------------------------------
def detector(diccionario="DICT_4X4_50"):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, diccionario))
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(d, p)


def detectar(img, det, ids_validos=None, escalas=(1, 2)):
    """{id: esquinas (4, 2)} en la imagen (gris o BGR). Lo que no sale a tamano natural se busca en la
    imagen ampliada (los ArUco lejanos de la mesa miden ~20 px en el IR: el 4 sale a x2, el 3 no)."""
    gris = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    sal = {}
    for e in escalas:
        g = gris if e == 1 else cv2.resize(gris, None, fx=e, fy=e, interpolation=cv2.INTER_CUBIC)
        esq, ids, _ = det.detectMarkers(g)
        if ids is None:
            continue
        for i, c in zip(ids.ravel(), esq):
            i = int(i)
            if i not in sal and (ids_validos is None or i in ids_validos):
                # centro de pixel: (u + 0.5) / e - 0.5
                sal[i] = (c.reshape(4, 2).astype(float) + 0.5) / e - 0.5
    return sal


def pose_marcador(esquinas, intr: Intrinsecos, lado_m):
    """T_cam_marcador por solvePnP (IPPE_SQUARE) o None."""
    ok, rvec, tvec = cv2.solvePnP(esquinas_marcador(lado_m), np.asarray(esquinas, float), intr.K, None,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return T(R, tvec.ravel())


# --- ajuste ----------------------------------------------------------------------------------------
def _media_T(Ms):
    R = sum(M[:3, :3] for M in Ms)
    U, _, Vt = np.linalg.svd(R)
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    return T(R, np.mean([M[:3, 3] for M in Ms], 0))


def ajustar(muestras, cin, intr: Intrinsecos, cal0: Calibracion, lado_m=0.05, perdida_px=2.0,
            fijar_camara=False):
    """muestras: [{"lado", "q" (7, medida), "esquinas" (4, 2) en el color}]. Devuelve (Calibracion, informe)."""
    from scipy.optimize import least_squares

    lados = [l for l in LADOS if any(m["lado"] == l for m in muestras)]
    K = esquinas_marcador(lado_m)
    T_mano = [cin.fk(m["lado"], m["q"], "mano") for m in muestras]
    uv = [np.asarray(m["esquinas"], float) for m in muestras]
    # semilla de cada dorso: media de solvePnP con la camara inicial
    T_dorso0 = dict(cal0.T_dorso)
    for l in lados:
        Ms = []
        for m, Tm, u in zip(muestras, T_mano, uv):
            if m["lado"] != l:
                continue
            Tcm = pose_marcador(u, intr, lado_m)
            if Tcm is not None:
                Ms.append(inv(Tm) @ cal0.T_torso_color @ Tcm)
        if Ms:
            T_dorso0[l] = _media_T(Ms)

    def desempaquetar(x):
        k = 0
        Tc = cal0.T_torso_color
        if not fijar_camara:
            Tc, k = vector_a_T(x[:6]), 6
        Td = {}
        for l in lados:
            Td[l] = vector_a_T(x[k:k + 6])
            k += 6
        return Tc, Td

    def residuos(x):
        Tc, Td = desempaquetar(x)
        Tc_inv = inv(Tc)
        r = []
        for m, Tm, u in zip(muestras, T_mano, uv):
            P = aplicar(Tc_inv @ Tm @ Td[m["lado"]], K)
            r.append((intr.proyectar(P) - u).ravel())
        return np.nan_to_num(np.concatenate(r), nan=1e3)

    x0 = ([] if fijar_camara else list(T_a_vector(cal0.T_torso_color))) + \
         [v for l in lados for v in T_a_vector(T_dorso0[l])]
    sol = least_squares(residuos, np.array(x0), loss="soft_l1", f_scale=perdida_px, x_scale="jac")
    Tc, Td = desempaquetar(sol.x)
    r = residuos(sol.x).reshape(-1, 4, 2)
    err_px = np.linalg.norm(r, axis=2).mean(1)
    # error 3D: distancia del centro del marcador segun la cadena del brazo al rayo de su centro detectado.
    # (No contra solvePnP: con un marcador de 5 cm a 0.6 m su distancia a lo largo del rayo yerra ~1 cm.)
    err_mm = []
    Tc_inv = inv(Tc)
    for m, Tm, u in zip(muestras, T_mano, uv):
        p = aplicar(Tc_inv @ Tm @ Td[m["lado"]], np.zeros(3))
        rayo = intr.rayos(u.mean(0))[0]
        rayo /= np.linalg.norm(rayo)
        err_mm.append(1000 * float(np.linalg.norm(p - (p @ rayo) * rayo)))
    cal = Calibracion(fecha=datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), T_torso_color=Tc,
                      T_color_prof=cal0.T_color_prof, T_dorso={**cal0.T_dorso, **Td}, plano_mesa=cal0.plano_mesa,
                      residuo_px=float(err_px.mean()), residuo_mm=float(np.mean(err_mm)) if err_mm else float("nan"),
                      n_poses=len(muestras), origen="calibrada")
    informe = {"error_px_por_muestra": err_px.round(2).tolist(), "error_mm_por_muestra": np.round(err_mm, 1).tolist(),
               "error_px_max": float(err_px.max()), "error_mm_max": float(max(err_mm)) if err_mm else None,
               "coste": float(sol.cost), "evaluaciones": int(sol.nfev), "lados": lados}
    return cal, informe


def kabsch_centros(muestras, cin, intr, cal: Calibracion, lado_m=0.05):
    """La alternativa de la pista del PDF: Kabsch entre los centros del marcador en el torso (por la
    cadena del brazo, con el dorso calibrado) y en la camara (solvePnP). Devuelve T_torso_color."""
    A, B = [], []
    for m in muestras:
        Tcm = pose_marcador(m["esquinas"], intr, lado_m)
        if Tcm is not None:
            A.append(Tcm[:3, 3])
            B.append((cin.fk(m["lado"], m["q"], "mano") @ cal.T_dorso[m["lado"]])[:3, 3])
    R, t = kabsch(np.array(A), np.array(B))
    return T(R, t)


def plano_en_torso(plano_prof: Plano, cal: Calibracion) -> Plano:
    return plano_prof.transformar(cal.T_torso_prof)
