"""Percepcion: ajuste de esfera, una captura real (2026-10-09) y escenas sinteticas del gemelo."""

import copy
import math
import os

import numpy as np
import pytest

from manip import calibracion as cb
from manip import config
from manip.cinematica import Cinematica
from manip.fuentes import FuenteDataset
from manip.geometria import Plano, envolver_simetria, rpy_a_R
from manip.percepcion import Percepcion, ajustar_esfera_apoyada
from test_gemelo import gl

AQUI = os.path.dirname(os.path.abspath(__file__))
T_COLOR_PROF = np.array([[1, 0, 0, 0.015], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


@pytest.fixture(scope="module")
def cin():
    return Cinematica.desde_config(config.cargar())


def test_esfera_vista_solo_por_delante():
    """Solo el casquete que ve la camara (y ~2 cm "aplastado", como la D435i): el centro sale igual."""
    rng = np.random.default_rng(0)
    plano = Plano([0.01, 0.02, 1.0], 0.118)
    o, ex, ey = plano.base()
    r = 0.035
    c = o + 0.45 * ex + 0.05 * ey + r * plano.n
    d = rng.normal(size=(3000, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    hacia_camara = np.array([-0.6, 0.0, 0.8])
    d = d[d @ hacia_camara > 0.2]
    P = c + r * d + rng.normal(0, 0.002, (len(d), 3))
    est, rms = ajustar_esfera_apoyada(P, plano, r, plano.proyectar(P.mean(0)) + r * plano.n)
    assert np.linalg.norm(est - c) < 0.002 and rms < 0.004


def test_captura_real(cin):
    """Mesa del 2026-10-09: tenis y cubo; la bola roja de espuma (fuera del reto) y el borde de una bandeja
    no son ningun objeto del reto."""
    cfg = config.cargar()
    f = list(FuenteDataset(os.path.join(AQUI, "datos", "captura_20261009_objetos")))[0]
    cal = cb.desde_urdf(cin, T_COLOR_PROF)
    mon = copy.deepcopy(config.cargar_montaje())
    mon["zona_recogida"] = {"x": [0.50, 0.66], "y": [-0.22, 0.12]}      # los objetos estaban mas lejos
    esc = Percepcion(cfg, mon, config.cargar_objetos(), cal, cin).procesar(f)
    clases = sorted(o.clase for o in esc.objetos)
    assert clases.count("tenis") == 1 and clases.count("cubo") == 1
    assert "prisma" not in clases and "cilindro" not in clases
    tenis = next(o for o in esc.objetos if o.clase == "tenis")
    cubo = next(o for o in esc.objetos if o.clase == "cubo")
    assert 0.05 < np.linalg.norm(tenis.p[:2] - cubo.p[:2]) < 0.15     # estaban casi en contacto


@gl
def test_escenas_sinteticas(cin):
    from sim.camara_virtual import CamaraVirtual
    from sim.escena import Estado, construir
    from manip import montaje as mt
    cfg = config.cargar()
    cal = cb.desde_urdf(cin, T_COLOR_PROF)
    mon = config.cargar_montaje()
    obj = config.cargar_objetos()
    m, info = construir(montaje=mon, cal=cal)
    est = Estado(m, info)
    cam = CamaraVirtual(m, info, cal, ruido_mm=2.0)
    per = Percepcion(cfg, mon, obj, cal, cin)
    Tm = mt.T_torso_mesa(mon)
    yaw_mesa = math.atan2(Tm[1, 0], Tm[0, 0])
    colocacion = {"tenis": (0.42, -0.11, 0.0), "cubo": (0.50, 0.06, 0.4), "cilindro": (0.42, 0.09, 0.0),
                  "prisma": (0.50, -0.05, 1.1)}         # sin tocarse (el prisma mide 10 cm de largo)
    for clave, (x, y, yaw) in colocacion.items():
        o = obj["objetos"][clave]
        alto = o["medidas_m"][0] if o["forma"] == "esfera" else o["medidas_m"][-1 if o["forma"] == "caja" else 1]
        est.poner_objeto(clave, mt.punto_en_mesa(mon, (x, y), alto / 2), Tm[:3, :3] @ rpy_a_R(0, 0, yaw))
    est.adelante()
    esc = per.procesar(cam.fotograma(est))
    vistos = {o.clase: o for o in esc.objetos}
    assert set(vistos) == set(colocacion)
    for clave, (x, y, yaw) in colocacion.items():
        o = vistos[clave]
        assert np.linalg.norm(o.p[:2] - [x, y]) < 0.005, clave
        sim = math.radians(obj["objetos"][clave]["simetria_deg"])
        if sim > 0:
            assert abs(math.degrees(envolver_simetria(o.yaw - (yaw + yaw_mesa), sim))) < 3.0, clave
