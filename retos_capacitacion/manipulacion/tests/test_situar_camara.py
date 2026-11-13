"""herramientas/situar_camara_externa.py (RESULTADOS §63): la pose de la camara externa con los ArUco 6-9 y lo que
hay que moverla, con una camara virtual."""

import os
import sys

import numpy as np
import pytest

from manip import config

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "herramientas"))
import situar_camara_externa as sc  # noqa: E402


@pytest.fixture(scope="module")
def escena():
    cfg = config.cargar()
    sit = cfg["camara_externa"]["situar"]
    esc = sc.Escena(config.cargar_montaje(), sit, cfg["percepcion"]["margen_zona_m"])
    return esc, sit, sc.intrinsecos(float(cfg["camara_externa"]["focal_4k_px"]))


def _asas(esc):
    """Los 8 y 9 de pie en los postes delanteros de las asas, mirando a la camara (+x), con el negro desde 3 cm."""
    h = esc.lado_asas / 2
    sal = {}
    for i, y in zip(esc.ids_asas, (-0.42, 0.42)):
        c = np.array([esc.fondo, y, 0.03 + h])
        der, arr = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
        sal[i] = np.array([c - h * der + h * arr, c + h * der + h * arr, c + h * der - h * arr, c - h * der - h * arr])
    return sal


def _vistos(R, C, K, esquinas):
    return {i: sc.proyectar(R, C, P, K)[0] for i, P in esquinas.items()}


def test_angulos_y_R_de_son_inversos():
    for incl, giro, ladeo in ((15.0, -3.5, 0.0), (30.0, 8.0, 2.5), (40.0, -12.0, -4.0)):
        R = sc.R_de(incl, giro, ladeo)
        C = np.array([1.4, 0.06, 0.6])
        _, i, g, l = sc.angulos(R, -R @ C)
        assert (i, g, l) == pytest.approx((incl, giro, ladeo), abs=1e-6)
    # sin ladeo, el eje x de la imagen (a la derecha de quien mira) es +y de la mesa
    assert sc.R_de(20.0, 0.0)[0] == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)


def test_pose_con_el_6_y_el_7_pegados_de_cualquier_manera(escena):
    esc, sit, K = escena
    C, incl, giro, ladeo = np.array([1.43, 0.063, 0.59]), 14.7, -3.5, -3.1
    R = sc.R_de(incl, giro, ladeo)
    giros = {6: 2, 7: 1}
    esquinas = {i: sc.esquinas_planas(c, esc.lado_planos, giros[i]) for i, c in esc.planos.items()}
    p = sc.pose_camara(_vistos(R, C, K, esquinas), esc, {}, K)
    assert p is not None and p["giros"] == giros and p["error_px"] < 0.01
    C2, i2, g2, l2 = sc.angulos(p["R"], p["t"])
    assert np.linalg.norm(C2 - C) < 1e-3
    assert (i2, g2, l2) == pytest.approx((incl, giro, ladeo), abs=0.05)


def test_las_asas_medidas_desde_el_6_y_el_7_valen_sin_ellos(escena):
    esc, sit, K = escena
    C, R = np.array([1.30, -0.05, 0.70]), sc.R_de(24.0, 2.0, 1.0)
    asas = _asas(esc)
    esquinas = {i: sc.esquinas_planas(c, esc.lado_planos, 0) for i, c in esc.planos.items()}
    esquinas.update(asas)
    vistos = _vistos(R, C, K, esquinas)
    p = sc.pose_camara(vistos, esc, {}, K)
    medidas = sc.asas_desde(vistos, p, K, esc)
    for i in esc.ids_asas:
        assert np.abs(medidas[i] - asas[i]).max() < 1e-3
    # sin el 6 ni el 7 (la mano encima, o fuera de la imagen): con las asas solas
    solo = {i: v for i, v in vistos.items() if i in esc.ids_asas}
    p2 = sc.pose_camara(solo, esc, medidas, K)
    C2, i2, g2, _ = sc.angulos(p2["R"], p2["t"])
    assert np.linalg.norm(C2 - C) < 5e-3 and (i2, g2) == pytest.approx((24.0, 2.0), abs=0.2)
    # con el 6 y el 7, las asas (aunque esten mal medidas) no cuentan
    mal = {i: v + [0.0, 0.02, 0.0] for i, v in medidas.items()}
    p3 = sc.pose_camara(vistos, esc, mal, K)
    assert p3["ids"] == sorted(esc.planos) and np.allclose(p3["t"], p["t"])
    # con un solo ArUco no hay pose
    assert sc.pose_camara({6: vistos[6]}, esc, {}, K) is None


def test_en_el_sitio_ideal_no_hay_que_mover_nada(escena):
    esc, sit, K = escena
    Co = sc.objetivo(esc, K, sit)
    el, _ = sc.elevacion(Co, esc.centro)
    assert el == pytest.approx(sit["elevacion_objetivo_deg"], abs=0.1)
    assert sc.zoom_apuntada(Co, esc, K, sit["margen"]) == pytest.approx(sit["zoom_objetivo"], abs=0.01)
    incl, giro = sc.apuntar(Co, esc.roi, K)
    pose = {"C": Co, "incl": incl, "giro": giro, "ladeo": 0.0, "error_px": 0.5, "ids": [6, 7]}
    ev = sc.evaluar(pose, esc, K, sit, 218)
    assert sc.propuesta(ev) == (0.0, 0.0)
    assert "La camara esta bien puesta." in sc.texto(ev, pose, esc, sit)
    # apuntada, lo que se quiere ver queda centrado en la imagen
    uv, _ = sc.proyectar(sc.R_de(incl, giro), Co, esc.roi, K)
    assert (uv.min(0) + uv.max(0)) / 2 == pytest.approx([K[0, 2], K[1, 2]], abs=2.0)


def test_baja_y_torcida_dice_subela_inclinala_y_girala(escena):
    esc, sit, K = escena
    Co = sc.objetivo(esc, K, sit)
    C = Co + [0.0, 0.0, -0.25]                      # 25 cm mas baja: la elevacion, por debajo del rango
    incl, giro = sc.apuntar(C, esc.roi, K)
    pose = {"C": C, "incl": incl - 8.0, "giro": giro + 5.0, "ladeo": 3.0, "error_px": 0.5, "ids": [6, 7]}
    ev = sc.evaluar(pose, esc, K, sit, 218)
    assert ev["elevacion"] < sit["elevacion_deg"][0]
    dh, dx = sc.propuesta(ev)
    assert dh > 0.10 and dx == 0.0                  # basta con subirla, al menos lo que falta para entrar
    t = sc.texto(ev, pose, esc, sit)
    assert "SUBELA" in t and "INCLINALA 8.0 grados hacia abajo" in t and "GIRALA 5.0 grados a tu izquierda" in t
    assert "baja el lado derecho" in t


def test_guardar_zoom_solo_toca_el_de_la_camara_externa(tmp_path, monkeypatch):
    ruta = tmp_path / "manipulacion.yaml"
    ruta.write_text("otra:\n    zoom_absolute: 7\ncamara_externa:\n  controles:\n    zoom_absolute: 218\n    pan_absolute: 0\n")
    monkeypatch.setattr(sc, "CONFIG", str(ruta))
    assert sc.guardar_zoom(185)
    assert ruta.read_text() == "otra:\n    zoom_absolute: 7\ncamara_externa:\n  controles:\n    zoom_absolute: 185\n    pan_absolute: 0\n"
    assert sc.zoom_absoluto(2.729) == 272 and sc.zoom_absoluto(0.8) == 100 and sc.zoom_absoluto(9) == 500
