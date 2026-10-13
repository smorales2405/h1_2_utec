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
    from sim.escena import Estado, construir
    cfg = config.cargar()
    cin = Cinematica.desde_config(cfg)
    base = cb.desde_urdf(cin, T_COLOR_PROF)
    verdad = cb.desde_urdf(cin, T_COLOR_PROF)
    verdad.T_torso_color = base.T_torso_color @ T(rpy_a_R(*np.radians([-0.8, 1.0, 0.5])), [-0.005, 0.007, 0.004])
    verdad.T_dorso = {"izq": base.T_dorso["izq"] @ T(rpy_a_R(0, 0, 0.1), [0.003, 0.0, 0.0]),
                      "der": base.T_dorso["der"] @ T(rpy_a_R(0, 0, -0.2), [0.0, 0.004, 0.0])}
    m, info = construir(cal=verdad, con_bandejas=False)
    est = Estado(m, info)
    cam = CamaraVirtual(m, info, verdad)
    poses = config._leer(os.path.join(config.CONFIG, "poses_calibracion.yaml"))["poses"]
    det = cb.detector()
    muestras = []
    for lado in ("izq", "der"):
        for q in poses[lado][:10]:
            est.poner_brazo("izq", info["postura"][13:20])
            est.poner_brazo("der", info["postura"][20:27])
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
