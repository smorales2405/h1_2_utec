"""Calibracion de la camara sobre el suelo: detectores minimos de la cinta y ajustes.

No son el bloque de percepcion: solo lo justo para medir la geometria con el robot quieto
(herramientas/calibrar_camara.py). Sin SDK, para poder probarlo en la PC.
"""

import math

import cv2
import numpy as np

from .geometria import ModeloSuelo


def estrecho(img, horizontal, ancho=31):
    """Lo que se aparta del fondo (claro u oscuro) y es mas estrecho que `ancho` pixeles en esa
    direccion: top-hat morfologico en las dos polaridades. La cinta de 5 cm pasa; las manchas
    anchas, como los reflejos de los focos en el suelo, no. Antes, mediana 3x3: quita los puntos
    del emisor (1-2 px), que con poca luz ambiente dominan la imagen, y deja la cinta (>= 2 px)."""
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (ancho, 1) if horizontal else (1, ancho))
    img = cv2.medianBlur(img, 3)
    return np.maximum(cv2.morphologyEx(img, cv2.MORPH_TOPHAT, k), cv2.morphologyEx(img, cv2.MORPH_BLACKHAT, k))


def puntos_de_linea(ir, modelo, x_min=0.3, x_max=3.0, contraste_min=25, y_max=0.6):
    """En cada fila, la columna con mas respuesta de `estrecho` en horizontal (la cinta a lo
    largo), a menos de `y_max` m del eje; solo para medir el yaw con el robot quieto y alineado."""
    resp = estrecho(ir, horizontal=True)
    puntos = []
    for v in range(0, resp.shape[0], 2):
        fila = resp[v].astype(np.float32)
        u = int(np.argmax(fila))
        if fila[u] < contraste_min:
            continue
        # centro del tramo por encima de media altura: la cinta es una meseta y argmax da su borde
        u0, u1 = u, u
        while u0 > 0 and fila[u0 - 1] >= fila[u] / 2:
            u0 -= 1
        while u1 < len(fila) - 1 and fila[u1 + 1] >= fila[u] / 2:
            u1 += 1
        u = (u0 + u1) / 2
        x, y = modelo.pixel_a_suelo(u, v)
        if np.isfinite(x) and x_min <= x <= x_max and abs(y) <= y_max:
            puntos.append((float(x), float(y), u, v))
    return puntos


def ajustar_recta(puntos, tolerancia=0.03):
    """y = a + b x. Empieza en la mediana de y (un punto suelto no la mueve) y reajusta con los
    puntos cada vez mas cerca de la recta: 30 cm, 10 cm y `tolerancia`."""
    p = np.array([(x, y) for x, y, _, _ in puntos])
    a, b = float(np.median(p[:, 1])), 0.0
    for tol in (0.3, 0.1, tolerancia):
        dentro = np.abs(p[:, 1] - (a + b * p[:, 0])) < tol
        if dentro.sum() < 3:
            break
        b, a = np.polyfit(p[dentro, 0], p[dentro, 1], 1)
    return float(a), float(b), int(dentro.sum())


def buscar_tramos(ir, modelo, largo=(0.45, 0.75), grosor=(0.02, 0.15), y_max=0.6):
    """Tramos de cinta transversales (la barra de fin y las marcas de calibracion), del mas
    cercano al mas lejano. Se marcan los pixeles de `estrecho` en vertical (la linea y los reflejos
    son altos y desaparecen) y se aceptan los tramos que en el suelo miden `largo` m de ancho y
    `grosor` m de fondo, con el centro a menos de `y_max` m del eje del robot (fuera el portico).
    Detector minimo para la calibracion, no el de la percepcion."""
    crudo = (estrecho(ir, horizontal=False) > 20).astype(np.uint8)
    # donde la linea cruza o toca el tramo, `estrecho` lo corta (la linea es alta): cerrar el hueco
    marca = cv2.morphologyEx(crudo, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (25, 1)))
    candidatos = []   # (v, u0, u1) de las filas con un tramo del largo de la cinta
    for v in range(0, marca.shape[0] - 4):
        fila = np.concatenate([[0], marca[v].astype(np.int8), [0]])
        cambios = np.flatnonzero(np.diff(fila))
        for u0, u1 in zip(cambios[0::2], cambios[1::2] - 1):
            # al cerrar huecos tambien se unen puntos sueltos del emisor: la cinta ya era casi entera
            # (en +-1 fila: la barra lejana tiene 1-2 px de grueso y con el roll cruza filas)
            if u1 - u0 < 30 or crudo[max(0, v - 1):v + 2, u0:u1 + 1].any(axis=0).mean() < 0.7:
                continue
            # aqui el filtro de largo es holgado: los puntos del emisor que quedan junto a los
            # extremos alargan el tramo; el largo de verdad se mira despues de afinar los extremos
            x0, y0 = modelo.pixel_a_suelo(u0, v)
            x1, y1 = modelo.pixel_a_suelo(u1, v)
            if np.isfinite(x0) and np.isfinite(x1) and 0.6 * largo[0] <= math.hypot(x1 - x0, y1 - y0) <= 1.6 * largo[1]:
                candidatos.append((v, int(u0), int(u1)))
    grupos = []   # filas seguidas que se solapan en u = un mismo tramo
    for v, u0, u1 in candidatos:
        for g in grupos:
            if v - g[-1][0] <= 2 and min(u1, g[-1][2]) - max(u0, g[-1][1]) > 0.5 * (u1 - u0):
                g.append((v, u0, u1))
                break
        else:
            grupos.append([(v, u0, u1)])
    tramos = []
    for g in grupos:
        v_lejos, v_cerca = g[0][0], g[-1][0]
        u0, u1 = _extremos(ir, v_lejos, v_cerca, int(np.median([c[1] for c in g])), int(np.median([c[2] for c in g])))
        uc = (u0 + u1) / 2
        x_lejos = float(modelo.pixel_a_suelo(uc, v_lejos - 0.5)[0])
        x_cerca = float(modelo.pixel_a_suelo(uc, v_cerca + 0.5)[0])
        y_centro = float(modelo.pixel_a_suelo(uc, v_cerca)[1])
        if not grosor[0] <= x_lejos - x_cerca <= grosor[1] or abs(y_centro) > y_max:
            continue
        xa, ya = modelo.pixel_a_suelo(u0, v_cerca)
        xb, yb = modelo.pixel_a_suelo(u1, v_cerca)
        largo_m = float(math.hypot(xb - xa, yb - ya))
        if not largo[0] <= largo_m <= largo[1]:
            continue
        tramos.append({"fila": v_cerca, "u0": u0, "u1": u1, "x_m": x_cerca,
                       "grosor_m": x_lejos - x_cerca, "largo_m": largo_m})
    return sorted(tramos, key=lambda t: t["x_m"])


def _extremos(ir, v0, v1, u0, u1, k=3, hueco=12):
    """Extremos del tramo: desde su centro, las columnas en las que las filas v0-1..v1+1 (con el
    roll, un tramo fino cruza filas) contrastan con el suelo de encima y de debajo al menos la
    mitad que la mediana del tramo. Se toleran huecos de `hueco` px (donde cruza la linea el
    contraste cae). Sobre la imagen sin filtrar, con una mediana de 5 columnas contra los puntos
    del emisor."""
    img = ir.astype(np.float32)
    encima = img[max(0, v0 - 2 - k):max(0, v0 - 2)]
    debajo = img[v1 + 3:v1 + 3 + k]
    if len(encima) == 0 or len(debajo) == 0:
        return u0, u1
    franja = img[max(0, v0 - 1):v1 + 2]
    suelo = (encima.mean(axis=0) + debajo.mean(axis=0)) / 2
    contraste = np.abs(franja - suelo).max(axis=0)
    contraste = np.median(np.lib.stride_tricks.sliding_window_view(np.pad(contraste, 2, mode="edge"), 5), axis=1)
    alto = contraste > 0.5 * np.median(contraste[u0:u1 + 1])
    centro = (u0 + u1) // 2
    extremos = []
    for paso in (-1, 1):
        u, ultimo = centro, centro
        while 0 <= u < len(alto) and abs(u - ultimo) <= hueco:
            if alto[u]:
                ultimo = u
            u += paso
        extremos.append(ultimo)
    return extremos[0], extremos[1]


def inclinacion_por_marcas(modelo, cerca, lejos, separacion_m):
    """Inclinacion (rad) con la que los bordes cercanos de dos tramos quedan a `separacion_m`
    en el suelo (medida con cinta). Biseccion: la separacion proyectada baja al inclinar mas."""
    def separacion(incl):
        m = ModeloSuelo(modelo.intr, modelo.altura, incl, modelo.roll, modelo.yaw)
        xc = m.pixel_a_suelo((cerca["u0"] + cerca["u1"]) / 2, cerca["fila"] + 0.5)[0]
        xl = m.pixel_a_suelo((lejos["u0"] + lejos["u1"]) / 2, lejos["fila"] + 0.5)[0]
        return float(xl - xc)
    a, b = math.radians(40.0), math.radians(60.0)
    if not separacion(b) < separacion_m < separacion(a):
        return None
    for _ in range(60):
        c = (a + b) / 2
        a, b = (c, b) if separacion(c) > separacion_m else (a, c)
    return (a + b) / 2
