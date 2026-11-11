"""Dianas del nivel 1: cruces claras al azar en la zona de recogida (pegatinas de 2 cm o, las del usuario, cruces de
cinta de carrocero de ~7.5 cm con brazos de 1.9 cm). Fotograma -> [(x, y)] en el torso, el centro de cada cruz sobre
el plano de la mesa.

Marcas claras sobre el tablero negro y las alfombrillas grises: contraste local en el color (top-hat), tamano en la
mesa entre lado_m, forma de cruz (rellenan 25-80 % de su caja) y planas en la profundidad (los objetos sobresalen).
El centro es el rayo del color por el centroide cortado con el plano. Se mira la zona ampliada `mascara_m` (la cruz
entera aunque este en el borde: con la mascara justa, el 2026-11-12, una cruz en el borde se corto y su centro salio
3 cm movido) y se quedan las de centro dentro de la zona ampliada `margen_m`.
"""

import math

import cv2
import numpy as np

from .fuentes import intrinsecos, prof_en_metros
from .geometria import Plano, aplicar, inv


def detectar(f, cal, montaje, margen_m=0.06, lado_m=(0.01, 0.10), contraste=40, n_max=6, mascara_m=0.12):
    if f.color is None:
        return []
    ic = intrinsecos(f, "color")
    pl = montaje["mesa"]["plano"]
    plano = Plano(pl["n"], pl["d"])
    z = montaje["zona_recogida"]
    mascara_m = max(float(mascara_m), float(margen_m) + float(lado_m[1]) / 2)

    def en_zona(p, m):
        return z["x"][0] - m <= p[0] <= z["x"][1] + m and z["y"][0] - m <= p[1] <= z["y"][1] + m
    # la zona ampliada en la imagen
    esq = []
    mg = mascara_m
    for x, y in ((z["x"][0] - mg, z["y"][0] - mg), (z["x"][1] + mg, z["y"][0] - mg),
                 (z["x"][1] + mg, z["y"][1] + mg), (z["x"][0] - mg, z["y"][1] + mg)):
        zz = -(plano.n[0] * x + plano.n[1] * y + plano.d) / plano.n[2]
        esq.append([x, y, zz])
    uv = ic.proyectar(aplicar(inv(cal.T_torso_color), np.array(esq)))
    mascara = np.zeros(f.color.shape[:2], np.uint8)
    cv2.fillPoly(mascara, [np.round(uv).astype(np.int32)], 255)
    gris = cv2.cvtColor(f.color, cv2.COLOR_BGR2GRAY)
    claro = cv2.morphologyEx(gris, cv2.MORPH_TOPHAT, np.ones((31, 31), np.uint8))
    bin_ = ((claro > contraste) & (mascara > 0)).astype(np.uint8)
    n, lab, st, cen = cv2.connectedComponentsWithStats(bin_, connectivity=8)
    o = cal.T_torso_color[:3, 3]
    alturas = _alturas(f, cal, plano) if f.prof is not None else None
    sal = []
    for k in range(1, n):
        x0, y0, w, h, area = st[k]
        if area < 15:
            continue
        p = plano.corte_rayo(o, cal.T_torso_color[:3, :3] @ ic.rayos(cen[k])[0])
        if p is None or not en_zona(p, margen_m):
            continue
        # tamano en la mesa: px -> m a esa distancia
        dist = float(np.linalg.norm(p - o))
        tam = max(w, h) * dist / ic.fx
        if not (lado_m[0] <= tam <= lado_m[1]):
            continue
        relleno = area / float(w * h)
        if not (0.25 <= relleno <= 0.8) or max(w, h) > 2.2 * min(w, h):
            continue
        if alturas is not None:
            v = alturas[y0:y0 + h, x0:x0 + w]
            v = v[np.isfinite(v)]
            if len(v) and np.percentile(v, 90) > 0.01:
                continue                       # sobresale: es un objeto, no una pegatina
        sal.append((relleno, p[:2]))
    sal.sort(key=lambda s: abs(s[0] - 0.5))
    return [p for _, p in sal[:n_max]]


def _alturas(f, cal, plano):
    """Altura sobre la mesa de cada pixel del color (por la profundidad reproyectada; nan sin dato)."""
    from .fuentes import T_color_prof
    ip, ic = intrinsecos(f, "prof"), intrinsecos(f, "color")
    prof = prof_en_metros(f)
    v, u = np.nonzero(prof > 0)
    z = prof[v, u]
    P = np.stack([(u - ip.cx) / ip.fx * z, (v - ip.cy) / ip.fy * z, z], 1)
    h = plano.altura(aplicar(cal.T_torso_prof, P))
    uv = np.round(ic.proyectar(aplicar(T_color_prof(f.meta), P))).astype(int)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < ic.ancho) & (uv[:, 1] >= 0) & (uv[:, 1] < ic.alto)
    A = np.full((ic.alto, ic.ancho), np.nan, np.float32)
    A[uv[ok, 1], uv[ok, 0]] = h[ok]
    return A


def cruz_en_mesa(spec_cuerpo, nombre, centro, R, lado=0.02, ancho=0.005):
    """Para el gemelo: una cruz blanca plana de `lado` con trazos de `ancho` (dos cajas finas)."""
    import mujoco
    from sim.escena import _quat
    for k, a in enumerate((0.0, math.pi / 2)):
        g = spec_cuerpo.add_geom()
        g.name, g.type = f"{nombre}_{k}", mujoco.mjtGeom.mjGEOM_BOX
        Rk = R @ np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        g.size = [lado / 2, ancho / 2, 0.0004]
        g.pos = list(np.asarray(centro) + 0.0008 * R[:, 2])
        g.quat = list(_quat(Rk))
        g.rgba = [0.95, 0.95, 0.95, 1.0]
        g.contype = g.conaffinity = 0
