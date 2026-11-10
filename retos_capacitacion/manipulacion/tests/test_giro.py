"""manip/giro.py: el giro del cuerpo para el vigilante, el yaw de la IMU sin su deriva medida con los ArUco 2-3 de la
mesa (en el robot, 2026-11-11, la IMU derivaba entre +0.4 y -0.6 grados/min con la camara quieta)."""

import importlib.util
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

from manip import calibracion as cb
from manip import config
from manip import giro as gm
from manip.geometria import Intrinsecos, Plano
from manip.vigilante import Vigilante
from sim.escena import postura_arnes
from sim.robot_simulado import RobotSimulado

CFG = config.cargar()
GRADO = math.radians(1.0)


def _hay_gl():
    try:
        import mujoco
        m = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
        mujoco.Renderer(m, 8, 8).close()
        return True
    except Exception:
        return False


@pytest.mark.skipif(not _hay_gl(), reason="sin OpenGL")
def test_yaw_de_la_mesa_en_el_gemelo():
    from manip.cinematica import Cinematica
    ruta = os.path.join(config.RAIZ, "herramientas", "marcar_montaje.py")
    spec = importlib.util.spec_from_file_location("marcar_montaje", ruta)
    mm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mm)
    mon = config.cargar_montaje()
    plano = Plano(**mm.montaje_en_objetivo(mon)["mesa"]["plano"])
    fs = mm.FuenteSim(CFG, mon, Cinematica.desde_config(CFG), [0.0, 0.0, 0.0])
    det = cb.detector(CFG["calibracion"]["diccionario"])

    def angulo():
        f = fs.siguiente()
        return gm.yaw_mesa(f.color, Intrinsecos.de_meta(f.meta["color"]), fs.cal.T_torso_color, plano, det)
    a0 = angulo()
    assert a0 is not None
    # en el gemelo los ArUco lejanos miden ~23 px y el render no suaviza: 0.5-2 mm de error por marcador, que
    # cambia al girar (en el robot, 0.12 px = 0.17 mm). Lo que se prueba aqui es el signo y la escala
    fs.mover(0.0, 0.0, 1.5)                     # la mesa gira 1.5 grados en el torso
    a1 = angulo()
    assert abs(math.degrees(gm.ang(a1 - a0)) - 1.5) < 0.2
    fs.mover(0.01, -0.01, 0.0)                  # desplazarla no es girar
    a2 = angulo()
    assert abs(math.degrees(gm.ang(a2 - a1))) < 0.15


class _Fot:
    def __init__(self):
        self.color, self.meta, self.n = np.zeros((2, 2, 3), np.uint8), {"color": {"fx": 1, "fy": 1, "cx": 0, "cy": 0}}, 0


def _giro(monkeypatch, mesa):
    """Un Giro cuyo angulo de la mesa vista es mesa[0] (None: tapada)."""
    monkeypatch.setattr(gm, "yaw_mesa", lambda *a, **k: mesa[0])
    cal = SimpleNamespace(T_torso_color=np.eye(4))
    return gm.Giro(None, None, cal, {"n": [0, 0, 1], "d": 0.1}, ventana_s=30.0, reloj=lambda: 0.0)


def test_quita_la_deriva_y_deja_el_giro_de_verdad(monkeypatch):
    mesa = [0.4]
    g = _giro(monkeypatch, mesa)
    ritmo = math.radians(-0.45) / 60.0           # lo medido en el robot
    g.iniciar(0.2)
    real = 0.0
    for k in range(0, 600 * 2 + 1):              # 10 min a 2 Hz
        t = k / 2.0
        if t == 300.0:
            real = 3 * GRADO                       # a los 5 min el cuerpo gira 3 grados de verdad
        mesa[0] = 0.4 - real                       # la mesa vista gira al reves
        if 420.0 <= t < 480.0:
            mesa[0] = None                         # un brazo tapa los marcadores un minuto
        imu = 0.2 + ritmo * t + real
        g.medir(_Fot(), imu, t)
        if t in (290.0, 330.0, 479.5, 600.0):
            esperado = 0.0 if t < 300 else 3.0
            assert abs(math.degrees(g.corregido(imu, t)) - esperado) < 0.05, (t, math.degrees(g.corregido(imu, t)))
            assert abs(math.degrees(g.deriva(t)) - math.degrees(ritmo * t)) < 0.05
    assert g.sin_ver == 120


def test_el_ritmo_extrapolado_tiene_tope(monkeypatch):
    mesa = [0.0]
    g = _giro(monkeypatch, mesa)
    g.iniciar(0.0)
    for k in range(20):                            # una deriva absurda de 6 grados/min, 10 s
        g.medir(_Fot(), math.radians(0.1) * k / 2.0, k / 2.0)
    # 60 s despues sin ver la mesa: como mucho 1 grado/min mas
    assert math.degrees(g.deriva(9.5 + 60.0)) <= math.degrees(g.deriva(9.5)) + 1.0 + 1e-6


def test_el_vigilante_no_para_por_la_deriva(monkeypatch):
    mesa = [0.0]
    g = _giro(monkeypatch, mesa)
    robot = RobotSimulado(postura=[float(v) for v in postura_arnes(CFG)], caida_rad=0.0, instantaneo=True)
    try:
        vig = Vigilante(robot, CFG, giro=g)
        vig.vigilar_camara = False
        reloj = [0.0]
        g.reloj = lambda: reloj[0]
        vig.iniciar()
        for k in range(0, 13 * 60 + 1, 5):         # 13 min de deriva a -0.45 grados/min: casi 6 grados
            reloj[0] = float(k)
            robot.yaw = math.radians(-0.45 / 60.0 * k)
            g.medir(_Fot(), robot.yaw, float(k))
        s = vig.salud()
        assert math.degrees(abs(s.yaw_imu)) > 5.5 and abs(math.degrees(s.yaw_desde_inicio)) < 0.1
        assert vig.motivo(s) is None
        robot.yaw += math.radians(6.0)              # y ahora un giro de verdad, que la camara tambien ve
        mesa[0] = -math.radians(6.0)
        g.medir(_Fot(), robot.yaw, reloj[0] + 0.5)
        reloj[0] += 0.5
        m = vig.motivo()
        assert m is not None and m.startswith("giro del cuerpo +6.0") and "deriva" in m
    finally:
        robot.parar()


@pytest.mark.skipif(not _hay_gl(), reason="sin OpenGL")
def test_el_hilo_mide_con_el_ultimo_fotograma():
    """Como en el robot: el hilo lee `ultimo` de la fuente (LectorContinuo) y el yaw de robot.estado()."""
    import time
    from manip.cinematica import Cinematica
    ruta = os.path.join(config.RAIZ, "herramientas", "marcar_montaje.py")
    spec = importlib.util.spec_from_file_location("marcar_montaje", ruta)
    mm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mm)
    mon = config.cargar_montaje()
    fs = mm.FuenteSim(CFG, mon, Cinematica.desde_config(CFG), [0.0, 0.0, 0.0])
    fuente = SimpleNamespace(ultimo=None)
    imu = SimpleNamespace(yaw=0.3)
    robot = SimpleNamespace(estado=lambda: SimpleNamespace(imu=imu))
    g = gm.Giro(robot, fuente, fs.cal, mm.montaje_en_objetivo(mon)["mesa"]["plano"], hz=50.0)
    g.iniciar(0.3)
    assert g.arrancar()
    try:
        for n in range(5):
            f = fs.siguiente()
            f.n = n
            fuente.ultimo = f
            imu.yaw = 0.3 + math.radians(0.1) * n          # la IMU deriva 0.1 grados por fotograma; la mesa, quieta
            time.sleep(0.1)
    finally:
        g.parar()
    assert g.medidas == 5 and g.sin_ver == 0
    assert abs(math.degrees(g.corregido(imu.yaw))) < 0.1
