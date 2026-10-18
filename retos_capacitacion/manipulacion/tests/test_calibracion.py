"""Calibracion camara-torso sobre el gemelo: con una camara y unos marcadores del dorso desviados del URDF,
el ajuste tiene que recuperarlos partiendo del URDF (se salta sin OpenGL)."""

import math
import os

import numpy as np
import pytest

from manip import calibracion as cb
from manip import config
from manip.cinematica import Cinematica
from manip.geometria import R_a_rotvec, T, inv, rpy_a_R
from test_gemelo import gl

T_COLOR_PROF = np.array([[1, 0, 0, 0.015], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def _dif(A, B):
    D = inv(A) @ B
    return 1000 * np.linalg.norm(A[:3, 3] - B[:3, 3]), math.degrees(np.linalg.norm(R_a_rotvec(D[:3, :3])))


@gl
def test_recupera_la_camara_y_los_dorsos():
    from sim.camara_virtual import CamaraVirtual
    from sim.escena import Estado, construir, postura_arnes
    cfg = config.cargar()
    cin = Cinematica.desde_config(cfg)
    base = cb.desde_urdf(cin, T_COLOR_PROF)
    verdad = cb.desde_urdf(cin, T_COLOR_PROF)
    verdad.T_torso_color = base.T_torso_color @ T(rpy_a_R(*np.radians([-0.8, 1.0, 0.5])), [-0.005, 0.007, 0.004])
    verdad.T_dorso = {"izq": base.T_dorso["izq"] @ T(rpy_a_R(0, 0, 0.1), [0.003, 0.0, 0.0]),
                      "der": base.T_dorso["der"] @ T(rpy_a_R(0, 0, -0.2), [0.0, 0.004, 0.0])}
    pc = config._leer(os.path.join(config.CONFIG, "poses_calibracion.yaml"))
    poses, inicio = pc["poses"], pc["inicio"]
    # de pie en el arnes con los brazos a 0, como las recorre arnes.py [k]
    m, info = construir(cal=verdad, con_bandejas=False, postura=list(postura_arnes(cfg)))
    est = Estado(m, info)
    cam = CamaraVirtual(m, info, verdad)
    det = cb.detector()
    muestras = []
    for lado in ("izq", "der"):
        for q in poses[lado][:10]:
            est.poner_brazo("izq", inicio["izq"])
            est.poner_brazo("der", inicio["der"])
            est.poner_brazo(lado, q)
            est.adelante()
            d = cb.detectar(cam.fotograma(est).color, det, {cfg["calibracion"]["ids"][lado]})
            if d:
                muestras.append({"lado": lado, "q": np.array(q), "esquinas": list(d.values())[0]})
    assert len(muestras) >= 18
    cal, inf = cb.ajustar(muestras, cin, cam.intr_color, base, cfg["calibracion"]["marcador_lado_m"])
    mm, deg = _dif(verdad.T_torso_color, cal.T_torso_color)
    assert mm < 3.0 and deg < 0.3
    for lado in ("izq", "der"):
        mm, deg = _dif(verdad.T_dorso[lado], cal.T_dorso[lado])
        assert mm < 3.0 and deg < 2.0
    assert cal.residuo_px < 1.5 and cal.residuo_mm < 2.0


def test_grabador_guarda_una_muestra_por_pose():
    """herramientas/grabar_calibracion.py: con el brazo yendo de 0 a dos poses y volviendo (minima sacudida de
    8 s y 3 s quieto en cada una, como arnes.py [k]) y ruido de encoder, una muestra por pose y ninguna en 0."""
    import importlib.util
    from manip.brazo import minima_sacudida
    ruta = os.path.join(config.RAIZ, "herramientas", "grabar_calibracion.py")
    spec = importlib.util.spec_from_file_location("grabar_calibracion", ruta)
    gc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gc)
    rng = np.random.default_rng(0)
    A = np.array([-0.3, 0.2, -0.5, 0.6, 0.9, 0.2, 0.6])
    B = A + [0.1, -0.05, 0.2, -0.1, 0.3, 0.0, -0.2]
    tramos = [(np.zeros(7), A), (A, B), (B, np.zeros(7))]
    g, t, guardadas = gc.Grabador(), 0.0, []
    for ini, fin in tramos:
        for fase, dur in (("mover", 8.0), ("quieto", 3.0)):
            for k in range(int(dur * 30)):
                s = minima_sacudida(min(1.0, k / (dur * 30))) if fase == "mover" else 1.0
                q27 = np.zeros(27)
                q27[13:20] = ini + s * (fin - ini) + rng.normal(0, 0.0005, 7)
                g.al_estado(t, q27)
                lado, q = g.lado_quieto(t)
                if lado:
                    guardadas.append((round(t, 2), lado, q))
                    g.guardada(lado, q)
                t += 1 / 30
    assert [l for _, l, _ in guardadas] == ["izq", "izq"]
    assert np.abs(guardadas[0][2] - A).max() < 0.003 and np.abs(guardadas[1][2] - B).max() < 0.003
