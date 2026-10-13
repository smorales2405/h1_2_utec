"""Cinematica: marcos del reto, espejo de los brazos e IK propia (sin robot)."""

import math

import numpy as np
import pytest

from manip import config
from manip.cinematica import (Cinematica, TareaIK, dedos_inclinados, ejes_mano, espejo, guinada,
                              palma_abajo, palma_vertical)
from manip.geometria import aplicar, inv

ZT = -0.118          # mesa del 2026-10-09 respecto del torso


@pytest.fixture(scope="module")
def cin():
    return Cinematica.desde_config(config.cargar())


def test_yema_en_la_mano(cin):
    q0 = np.zeros(7)
    for lado, s in (("izq", 1), ("der", -1)):
        M = cin.fk(lado, q0, "mano")
        Y = cin.fk(lado, q0, "yema")
        p = aplicar(inv(M), Y[:3, 3])
        assert np.allclose(p, [0.2954, -s * 0.0085, 0.0358], atol=1e-3)


def test_camara_del_urdf(cin):
    Tc = cin.T_camara_link()
    assert np.allclose(Tc[:3, 3], [0.11109, 0.0175, 0.68789], atol=1e-4)


def test_brazo_derecho_es_el_espejo(cin):
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = cin.lo["izq"] + (cin.hi["izq"] - cin.lo["izq"]) * rng.random(7)
        qd = espejo(q)
        if not cin.dentro_de_limites("der", qd):
            continue
        Mi, Md = cin.fk("izq", q, "mano"), cin.fk("der", qd, "mano")
        assert np.allclose(Md[:3, 3], Mi[:3, 3] * [1, -1, 1], atol=1e-3)
        di, ai, pi = ejes_mano(Mi, "izq")
        dd, ad, pd = ejes_mano(Md, "der")
        assert np.allclose(dd, di * [1, -1, 1], atol=1e-3)     # los dedos, en espejo
        assert np.allclose(pd, pi * [1, -1, 1], atol=1e-3)     # la palma tambien


def test_ik_ida_y_vuelta(cin):
    rng = np.random.default_rng(7)
    for lado in ("izq", "der"):
        for _ in range(8):
            q = cin.lo[lado] + (cin.hi[lado] - cin.lo[lado]) * rng.random(7)
            p = cin.fk(lado, q, "mano")[:3, 3]
            r = cin.ik(lado, TareaIK(posicion=p))
            assert r.ok, (lado, r)
            assert np.linalg.norm(cin.fk(lado, r.q, "mano")[:3, 3] - p) < 0.003


def test_agarre_desde_arriba_sobre_la_mesa(cin):
    punto = np.array([0.215, -0.070, 0.010])
    objetivo = np.array([0.45, 0.10, ZT + 0.035])
    r = cin.ik("izq", TareaIK(posicion=objetivo, punto=punto, bandas=[palma_abajo("izq", 35)]))
    assert r.ok
    M = cin.fk("izq", r.q, "mano")
    _, _, palma = ejes_mano(M, "izq")
    assert math.degrees(math.acos(-palma[2])) <= 35.5
    assert np.linalg.norm(aplicar(M, punto) - objetivo) < 0.003
    # y el espejo con la derecha
    rd = cin.ik("der", TareaIK(posicion=objetivo * [1, -1, 1], punto=punto * [1, -1, 1],
                               bandas=[palma_abajo("der", 35)]))
    assert rd.ok


def test_agarre_de_lado_con_los_dedos_inclinados(cin):
    punto = np.array([0.215, -0.065, 0.010])
    objetivo = np.array([0.45, 0.05, ZT + 0.04])
    r = cin.ik("izq", TareaIK(posicion=objetivo, punto=punto,
                              bandas=[palma_vertical("izq", 25), dedos_inclinados(15, 55)]))
    assert r.ok
    dedos, _, palma = ejes_mano(cin.fk("izq", r.q, "mano"), "izq")
    assert abs(palma[2]) <= math.sin(math.radians(25)) + 1e-3
    assert -math.sin(math.radians(55)) - 1e-3 <= dedos[2] <= -math.sin(math.radians(15)) + 1e-3


def test_fuera_de_alcance(cin):
    # el centro del tablero (x 0.6 m) no se alcanza con la palma abajo: lo dice el mapa del 2026-10-09
    r = cin.ik("izq", TareaIK(posicion=np.array([0.62, 0.0, ZT + 0.035]), punto=np.array([0.215, -0.07, 0.01]),
                              bandas=[palma_abajo("izq", 35)]))
    assert not r.ok


def test_guinada_de_la_mano(cin):
    punto = np.array([0.215, -0.070, 0.010])
    objetivo = np.array([0.44, 0.0, ZT + 0.035])
    r = cin.ik("izq", TareaIK(posicion=objetivo, punto=punto,
                              bandas=[palma_abajo("izq", 35), guinada([math.cos(-2.1), math.sin(-2.1)], 15)]))
    assert r.ok
    _, ancho, _ = ejes_mano(cin.fk("izq", r.q, "mano"), "izq")
    assert math.degrees(abs(math.atan2(ancho[1], ancho[0]) + 2.1)) < 16


def test_continuidad_desde_q0(cin):
    """Desde una q que ya cumple, la IK no salta a otra rama."""
    q = np.array([-0.5, 0.2, 0.0, 0.8, 0.0, 0.0, 0.0])
    p = cin.fk("izq", q, "mano")[:3, 3]
    r = cin.ik("izq", TareaIK(posicion=p + [0.01, 0.0, 0.0]), q0=q)
    assert r.ok and np.abs(r.q - q).max() < 0.15
