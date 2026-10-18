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
MARGEN_MARCADOR = 0.031     # del centro de un ArUco de las esquinas a cada labio: 5 cm de negro y 6 mm de blanco
                            # contra el labio (wincha, 2026-10-18; antes se suponian 4 cm y el labio izquierdo
                            # quedaba 2 cm hacia fuera). montaje.yaml lo trae en mesa_fisica.


def fisica(montaje):
    """Las medidas de wincha de la mesa (mesa_fisica de montaje.yaml) o None: ancho y fondo entre las caras
    interiores del labio y del centro de un ArUco de las esquinas a cada labio."""
    m = (montaje or {}).get("mesa_fisica")
    if not m:
        return None
    return {"ancho": float(m["ancho_interior_m"]), "fondo": float(m["fondo_interior_m"]),
            "margen": float(m.get("aruco_al_labio_m", MARGEN_MARCADOR))}


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


def escala_mesa(centros, fis):
    """Lo que la camara ve de mas sobre la mesa: distancia entre los centros de los ArUco 2 y 3 (y 4 y 5) vista /
    de wincha; 1.0 sin mesa_fisica o sin una pareja. La profundidad mide ~3 % de mas a 1.1 m (RESULTADOS §22): el
    marco y la superficie se escalan con ella para quedar donde la camara ve los objetos y las bandejas, y vuelve
    a 1 sola cuando la camara este corregida."""
    if fis is None:
        return 1.0
    real = fis["ancho"] - 2 * fis["margen"]
    r = [np.linalg.norm(np.asarray(centros[a], float) - np.asarray(centros[b], float)) / real
         for a, b in ((2, 3), (4, 5)) if a in centros and b in centros]
    return float(np.mean(r)) if r else 1.0


def marco_mesa(centros, plano, fondo=None, margen=MARGEN_MARCADOR, escala=1.0):
    """T_torso_mesa desde los centros de los ArUco 2-5: 4 y 5 y uno de los lejanos, o solo 2 y 3 y `fondo`
    (del borde cercano de la superficie a los centros de los lejanos, de una medida anterior o de mesa_fisica):
    con la mesa a 0.27 m las esquinas cercanas quedan fuera de la vista (2026-10-17). `escala`: escala_mesa()."""
    c = {k: plano.proyectar(np.asarray(v, float)) for k, v in centros.items()}
    lejos = [c[i] for i in (2, 3) if i in c]
    if not lejos:
        raise ValueError("hace falta un ArUco lejano (2 o 3)")
    if 4 in c and 5 in c:
        # x: perpendicular al borde cercano (4 -> 5), hacia los lejanos
        cerca = (c[4] + c[5]) / 2
        x = np.cross(c[4] - c[5], plano.n)
        x /= np.linalg.norm(x)
        if x @ (np.mean(lejos, 0) - cerca) < 0:
            x = -x
        origen = cerca - escala * margen * x
    elif 2 in c and 3 in c and fondo is not None:
        # x: perpendicular al borde lejano (2 -> 3), hacia delante del torso
        x = np.cross(c[2] - c[3], plano.n)
        x /= np.linalg.norm(x)
        if x[0] < 0:
            x = -x
        origen = (c[2] + c[3]) / 2 - escala * fondo * x
    else:
        raise ValueError("hacen falta los ArUco 4 y 5, o el 2 y el 3 y el fondo de la mesa de una medida anterior")
    y = np.cross(plano.n, x)
    R = np.column_stack([x, y, plano.n])
    return T(R, origen)


def _plano_y_marcas(f, cal, montaje_previo, det, paso=2):
    """Plano de la profundidad (alrededor del de la medida anterior) y ArUco 2-5 cortados con el: lo comun a
    medir() y a pose_mesa(). Devuelve (plano, frac, rms, marcas, fondo)."""
    from .fuentes import intrinsecos, prof_en_metros
    plano_prev = None
    if montaje_previo and "mesa" in montaje_previo:
        p = montaje_previo["mesa"]["plano"]
        plano_prev = (np.array(p["n"]), p["d"])
    plano, frac, rms = plano_de_profundidad(prof_en_metros(f), intrinsecos(f, "prof"), cal, plano_prev, paso)
    if plano is None:
        raise ValueError("no encuentro el plano de la mesa en la profundidad")
    ids = set(IDS_MESA.values())
    marcas = {}
    if f.ir is not None:
        marcas.update(marcadores_en_mesa(f.ir, intrinsecos(f, "ir"), cal.T_torso_prof, plano, det, ids))
    if f.color is not None:     # el color completa lo que el IR no vio (con el emisor, a veces falla)
        for i, v in marcadores_en_mesa(f.color, intrinsecos(f, "color"), cal.T_torso_color, plano, det, ids).items():
            marcas.setdefault(i, v)
    fondo, fis = None, fisica(montaje_previo)
    if fis is not None:
        fondo = fis["fondo"] - fis["margen"]
    elif montaje_previo and "mesa" in montaje_previo:
        fondo = montaje_previo["mesa"]["superficie"]["x"][1] - MARGEN_MARCADOR
    return plano, frac, rms, marcas, fondo


def medir(f, cal, montaje_previo=None, dicc="DICT_4X4_50"):
    """Mide la mesa en un Fotograma (con color, IR y profundidad). Devuelve (mesa, informe)."""
    from .fuentes import intrinsecos, prof_en_metros
    prof = prof_en_metros(f)
    i_prof = intrinsecos(f, "prof")
    plano, frac, rms, marcas, fondo = _plano_y_marcas(f, cal, montaje_previo, detector(dicc))
    centros = {i: v[0] for i, v in marcas.items()}
    fis = fisica(montaje_previo)
    margen = fis["margen"] if fis else MARGEN_MARCADOR
    esc = escala_mesa(centros, fis)
    T_mesa = marco_mesa(centros, plano, fondo, margen, esc)
    en_mesa = {i: aplicar(inv(T_mesa), c) for i, c in centros.items()}
    xs = [p[0] for i, p in en_mesa.items()]
    ys_izq = [en_mesa[i][1] for i in (2, 4) if i in en_mesa]
    ys_der = [en_mesa[i][1] for i in (3, 5) if i in en_mesa]
    lejos = [en_mesa[i][0] for i in (2, 3) if i in en_mesa]
    if fis is not None:     # la wincha (con la escala de la camara): el marco esta centrado entre los ArUco
        sup = {"x": [0.0, esc * fis["fondo"]], "y": [-esc * fis["ancho"] / 2, esc * fis["ancho"] / 2]}
    else:
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
               "marco": "ArUco 4 y 5" if 4 in centros and 5 in centros else f"ArUco 2 y 3 y el fondo anterior ({fondo:.3f} m)",
               "aruco_vistos": sorted(int(i) for i in centros), "puntos_sobre_la_mesa": int(sobre.sum()),
               "lados_aruco_mm": {int(i): round(1000 * float(np.mean([np.linalg.norm(v[1][k] - v[1][(k + 1) % 4])
                                                                     for k in range(4)])), 1) for i, v in marcas.items()},
               "distancias_mm": {f"{a}-{b}": round(1000 * float(np.linalg.norm(centros[a] - centros[b])), 1)
                                 for a, b in ((4, 5), (2, 3), (4, 2), (5, 3)) if a in centros and b in centros}}
    if fis is not None:
        # camara / wincha entre centros de ArUco: la profundidad sobre la mesa mide ~3 % de mas (RESULTADOS §22)
        real = {"4-5": fis["ancho"] - 2 * margen, "2-3": fis["ancho"] - 2 * margen,
                "4-2": fis["fondo"] - 2 * margen, "5-3": fis["fondo"] - 2 * margen}
        informe["escala_aruco"] = {k: round(v / 1000 / real[k], 4) for k, v in informe["distancias_mm"].items()}
        informe["escala_mesa"] = round(esc, 4)
    return mesa, informe


# --- colocacion en vivo (herramientas/marcar_montaje.py) ----------------------------------------------------

def pose_mesa(f, cal, montaje_previo, det, paso=4):
    """La pose de la mesa en un fotograma, rapida (para colocar el carrito mirando la pantalla): dict con T
    (T_torso_mesa), plano, x0 (borde cercano, m), y (m, + a la izquierda del robot), yaw_deg y los ArUco vistos.
    ValueError si no hay plano o faltan ArUco."""
    plano, _, _, marcas, fondo = _plano_y_marcas(f, cal, montaje_previo, det, paso)
    fis = fisica(montaje_previo)
    centros = {i: v[0] for i, v in marcas.items()}
    esc = escala_mesa(centros, fis)
    T_mesa = marco_mesa(centros, plano, fondo, fis["margen"] if fis else MARGEN_MARCADOR, esc)
    return {"T": T_mesa, "plano": plano, "x0": float(T_mesa[0, 3]), "y": float(T_mesa[1, 3]),
            "yaw_deg": math.degrees(math.atan2(T_mesa[1, 0], T_mesa[0, 0])), "aruco": sorted(int(i) for i in marcas),
            "escala": esc}


def desvio_mesa(pose, objetivo, medio_ancho=0.43):
    """Lo que falta para llevar la mesa a `objetivo` (montaje.yaml, objetivo_mesa): dict con los errores, si
    cada uno esta dentro de su tolerancia y las indicaciones para quien empuja el carrito."""
    ex = pose["x0"] - objetivo["x0_m"]
    ey = pose["y"] - objetivo.get("y_m", 0.0)
    eyaw = pose["yaw_deg"] - objetivo.get("yaw_deg", 0.0)
    ok = {"x": abs(ex) <= objetivo["tol_x_m"], "y": abs(ey) <= objetivo["tol_y_m"],
          "yaw": abs(eyaw) <= objetivo["tol_yaw_deg"]}
    # yaw > 0: el eje x de la mesa gira hacia la izquierda del robot, el extremo izquierdo queda mas cerca
    extremo = medio_ancho * math.sin(math.radians(abs(eyaw)))
    pasos = []
    if not ok["x"]:
        pasos.append(f"{'acerca' if ex > 0 else 'aleja'} {abs(ex) * 100:.1f} cm")
    if not ok["y"]:
        pasos.append(f"{abs(ey) * 100:.1f} cm a la {'derecha' if ey > 0 else 'izquierda'} del robot")
    if not ok["yaw"]:
        pasos.append(f"aleja el extremo {'izquierdo' if eyaw > 0 else 'derecho'} {extremo * 1000:.0f} mm")
    return {"ex": ex, "ey": ey, "eyaw": eyaw, "ok": ok, "todo_ok": all(ok.values()), "pasos": pasos}


def _sdf_cuadrado(q, a):
    """Distancia con signo de los puntos q (N, 2) al cuadrado de semilado a centrado en el origen."""
    d = np.abs(q) - a
    fuera = np.hypot(np.maximum(d[:, 0], 0), np.maximum(d[:, 1], 0))
    return fuera + np.minimum(np.maximum(d[:, 0], d[:, 1]), 0)


def ajustar_bandeja(xy, h, hueco, pared, c0, yaw0=0.0, alto_pared=0.012):
    """Centro (x, y) y yaw (rad, modulo 90 grados) de una bandeja cuadrada de medidas conocidas a partir de sus
    puntos (planta xy y altura h sobre la mesa): las paredes (h > alto_pared) sobre el contorno y el fondo
    dentro del hueco. Vale con la bandeja vista a medias (tapada por la mano o fuera de la imagen) mientras se
    vean dos paredes que no sean paralelas; un tiron suave hacia (c0, yaw0) la sujeta si falta alguna.
    Devuelve (centro, yaw, rms de las paredes en m)."""
    from scipy.optimize import least_squares
    xy, h = np.asarray(xy, float), np.asarray(h, float)
    paredes, fondo = xy[h > alto_pared], xy[h <= alto_pared]
    a_pared, a_hueco = hueco / 2 + pared / 2, hueco / 2

    def residuos(v):
        c, t = v[:2], v[2]
        R = np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])      # del torso a la bandeja
        r = [_sdf_cuadrado((paredes - c) @ R.T, a_pared),
             0.2 * np.maximum(_sdf_cuadrado((fondo - c) @ R.T, a_hueco), 0.0),
             [0.05 * (c[0] - c0[0]) / 0.10, 0.05 * (c[1] - c0[1]) / 0.10, 0.05 * (t - yaw0) / math.radians(20)]]
        return np.concatenate([np.ravel(x) for x in r])
    v0 = np.array([c0[0], c0[1], yaw0], float)
    sol = least_squares(residuos, v0, loss="soft_l1", f_scale=0.004, max_nfev=200)
    c, t = sol.x[:2], float(sol.x[2])
    t = (t + math.pi / 4) % (math.pi / 2) - math.pi / 4
    rms = float(np.sqrt(np.mean(np.minimum(np.abs(residuos(sol.x)[:len(paredes)]), 0.02) ** 2))) if len(paredes) else float("nan")
    return c, t, rms


def lados_vistos(paredes, c, t, a, tol=0.01):
    """Puntos de pared (planta) junto a cada lado de una bandeja de centro c, yaw t y semilado a (sin las
    esquinas): {"+x": n, "-x": n, "+y": n, "-y": n} en el marco de la bandeja."""
    R = np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])
    q = (np.asarray(paredes, float).reshape(-1, 2) - c) @ R.T
    return {("+" if s > 0 else "-") + eje: int(((np.abs(q[:, i] - s * a) < tol) & (np.abs(q[:, 1 - i]) < a - tol)).sum())
            for i, eje in ((0, "x"), (1, "y")) for s in (1, -1)}


def medir_bandejas(f, cal, plano, T_mesa, montaje, hsv, ventana=0.12, paso=2, margen_borde=0.02, n_lado=15):
    """Las bandejas de montaje.yaml que se ven en un fotograma, por su color (hsv: {color: [[min, max], ...]} de
    OpenCV) y su altura: {nombre: dict(centro (torso), yaw_deg (respecto de la mesa), n_paredes, n_fondo, rms,
    lados, a_medias)} o {nombre: None} si no hay bastante. Solo mira cerca de donde tiene que ir cada una
    (`ventana` m) y dentro de la superficie util menos `margen_borde` (el labio y las pegatinas del borde no son
    bandeja). a_medias: menos de dos lados con `n_lado` puntos de pared; con una sola pared el centro no queda
    determinado (la bandeja blanca, a la derecha de la mano derecha: la sombra de la profundidad del D435i, a la
    izquierda de lo que esta cerca, se come su pared interior)."""
    import cv2
    from .fuentes import T_color_prof, intrinsecos, prof_en_metros
    P, _ = nube(prof_en_metros(f), intrinsecos(f, "prof"), paso)
    Pt = aplicar(cal.T_torso_prof, P)
    h = plano.altura(Pt)
    sel = (h > -0.01) & (h < 0.055)
    sup = (montaje.get("mesa") or {}).get("superficie")
    if sup is not None:
        Pm = aplicar(inv(T_mesa), Pt)
        sel &= ((Pm[:, 0] > sup["x"][0] + margen_borde) & (Pm[:, 0] < sup["x"][1] - margen_borde)
                & (Pm[:, 1] > sup["y"][0] + margen_borde) & (Pm[:, 1] < sup["y"][1] - margen_borde))
    P, Pt, h = P[sel], Pt[sel], h[sel]
    ic = intrinsecos(f, "color")
    uv = np.round(ic.proyectar(aplicar(T_color_prof(f.meta), P))).astype(float)
    dentro = np.isfinite(uv).all(1) & (uv[:, 0] >= 0) & (uv[:, 0] < ic.ancho) & (uv[:, 1] >= 0) & (uv[:, 1] < ic.alto)
    Pt, h, uv = Pt[dentro], h[dentro], uv[dentro].astype(int)
    color = cv2.cvtColor(f.color, cv2.COLOR_BGR2HSV)[uv[:, 1], uv[:, 0]]
    yaw_mesa = math.atan2(T_mesa[1, 0], T_mesa[0, 0])
    sal = {}
    for nombre, b in montaje.get("bandejas", {}).items():
        rangos = hsv.get(b.get("color") or "", [])
        if not rangos:
            sal[nombre] = None
            continue
        es = np.zeros(len(Pt), bool)
        for lo, hi in rangos:
            es |= np.all((color >= lo) & (color <= hi), axis=1)
        c0 = np.asarray(b["centro"], float)
        es &= np.all(np.abs(Pt[:, :2] - c0) < ventana, axis=1)
        n_par, n_fondo = int((es & (h > 0.012)).sum()), int((es & (h <= 0.012)).sum())
        if n_par < 40 or n_par + n_fondo < 150:
            sal[nombre] = None
            continue
        # empieza donde estan sus puntos (puede estar lejos del objetivo), con el yaw de la mesa
        inicio = np.median(Pt[es, :2], 0)
        c, t, rms = ajustar_bandeja(Pt[es, :2], h[es], b["hueco_m"], b["pared_m"], inicio, yaw_mesa)
        yaw_rel = (math.degrees(t - yaw_mesa) + 45) % 90 - 45
        lados = lados_vistos(Pt[es & (h > 0.012), :2], c, t, b["hueco_m"] / 2 + b["pared_m"] / 2)
        sal[nombre] = {"centro": c, "yaw_deg": yaw_rel, "n_paredes": n_par, "n_fondo": n_fondo, "rms": rms,
                       "lados": lados, "a_medias": sum(n >= n_lado for n in lados.values()) < 2}
    return sal


def desvio_bandeja(medida, objetivo_xy, tol_m, tol_yaw_deg=5.0):
    """Lo que falta para llevar una bandeja a su centro: errores, si esta dentro y la indicacion."""
    e = np.asarray(medida["centro"], float) - np.asarray(objetivo_xy, float)
    ok = bool(np.hypot(*e) <= tol_m and abs(medida["yaw_deg"]) <= tol_yaw_deg)
    pasos = []
    if abs(e[0]) > tol_m / 2:
        pasos.append(f"{abs(e[0]) * 100:.1f} cm {'hacia el robot' if e[0] > 0 else 'hacia el fondo'}")
    if abs(e[1]) > tol_m / 2:
        pasos.append(f"{abs(e[1]) * 100:.1f} cm a la {'derecha' if e[1] > 0 else 'izquierda'} del robot")
    if abs(medida["yaw_deg"]) > tol_yaw_deg:
        pasos.append(f"enderezala {abs(medida['yaw_deg']):.0f} grados")
    return {"ex": float(e[0]), "ey": float(e[1]), "ok": ok, "pasos": pasos}


def bandeja_real(b):
    """(centro en el torso, giro en grados respecto de la mesa) donde esta una bandeja de montaje.yaml: lo medido
    con el [g] de las bandejas de marcar_montaje.py (`medida`) si lo hay; si no, su objetivo (`centro`) y 0. El
    robot suelta ahi: con el objetivo, la tolerancia de colocacion (1.5 cm) se come el margen del prisma (10 cm
    en un hueco de 12: +-0.6 cm en la peor orientacion)."""
    m = b.get("medida")
    if m:
        return np.asarray(m["centro"], float), float(m.get("yaw_deg", 0.0))
    return np.asarray(b["centro"], float), 0.0


def comprobar(fotos, cal, montaje, det, hsv, tol_mesa_m=0.01, tol_mesa_deg=1.0, tol_bandeja_m=0.01):
    """Antes de una tirada: la mesa y las bandejas siguen donde dice montaje.yaml? Con varios fotogramas
    (medianas). La mesa, contra la del ultimo [g]; cada bandeja que se vea entera, contra su `medida` (o contra su
    objetivo con tol_bandejas_m si no se midio). Devuelve (problemas: [texto], informe: [texto])."""
    poses = []
    for f in fotos:
        try:
            poses.append(pose_mesa(f, cal, montaje, det))
        except ValueError:
            pass
    if len(poses) < max(1, len(fotos) // 2):
        return ["no veo la mesa (plano y ArUco 2 y 3) en la mitad de los fotogramas"], []
    x0, y, yaw = (float(np.median([p[k] for p in poses])) for k in ("x0", "y", "yaw_deg"))
    T = np.asarray(montaje["mesa"]["T_torso_mesa"], float)
    dx, dy = x0 - T[0, 3], y - T[1, 3]
    dyaw = yaw - math.degrees(math.atan2(T[1, 0], T[0, 0]))
    problemas = []
    informe = [f"mesa: {dx * 100:+.1f} cm hacia el fondo, {dy * 100:+.1f} cm a la izquierda, {dyaw:+.1f} grados desde el [g]"]
    if abs(dx) > tol_mesa_m or abs(dy) > tol_mesa_m or abs(dyaw) > tol_mesa_deg:
        problemas.append(f"la mesa se ha movido desde el [g]: {dx * 100:+.1f} cm hacia el fondo, {dy * 100:+.1f} cm a la "
                         f"izquierda del robot, {dyaw:+.1f} grados (tolerancia {tol_mesa_m * 100:.0f} cm y {tol_mesa_deg:g})")
    vistas = {k: [] for k in montaje.get("bandejas", {})}
    for f, p in zip(fotos, poses):
        for k, v in medir_bandejas(f, cal, p["plano"], p["T"], montaje, hsv).items():
            if v is not None and not v["a_medias"]:
                vistas[k].append(v["centro"])
    for k, b in montaje.get("bandejas", {}).items():
        etiqueta = f"bandeja {k} ({b.get('color')})"
        if len(vistas[k]) < max(1, len(poses) // 2):
            informe.append(f"{etiqueta}: no se ve entera (sin comprobar)")
            continue
        c_ref, _ = bandeja_real(b)
        tol = tol_bandeja_m if b.get("medida") else float(montaje.get("tol_bandejas_m", 0.015))
        e = np.median(vistas[k], 0) - c_ref
        donde = "de donde se midio" if b.get("medida") else "de su sitio"
        informe.append(f"{etiqueta}: {np.linalg.norm(e) * 100:.1f} cm {donde}")
        if np.linalg.norm(e) > tol:
            problemas.append(f"{etiqueta} a {np.linalg.norm(e) * 100:.1f} cm {donde} ({e[0] * 100:+.1f} hacia el fondo, "
                             f"{e[1] * 100:+.1f} a la izquierda; tolerancia {tol * 100:.1f} cm)")
    return problemas, informe


def resolver_asignacion(asignacion, montaje):
    """{clase: bandeja}, con la bandeja por su nombre (A-D) o por su color de montaje.yaml -> {clase: nombre}."""
    bandejas = montaje.get("bandejas", {})
    por_color = {b["color"]: k for k, b in bandejas.items() if b.get("color")}
    sal = {}
    for clase, b in (asignacion or {}).items():
        if b in bandejas:
            sal[clase] = b
        elif b in por_color:
            sal[clase] = por_color[b]
        else:
            raise ValueError(f"bandeja '{b}' de {clase}: no esta en montaje.yaml "
                             f"(nombres {sorted(bandejas)}, colores {sorted(por_color)})")
    return sal


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
