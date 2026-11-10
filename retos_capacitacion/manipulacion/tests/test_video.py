"""herramientas/video_simulacion.py: el reloj de la tirada, la caida animada, las transiciones de camara y, con
OpenGL, la tirada de nivel 2 desde de pie en el arnes con los brazos a 0 (manipulacion.py --sim --arnes)."""

import importlib.util
import math
import os
import time

import numpy as np

from manip import config
from test_gemelo import gl


def _herramienta():
    ruta = os.path.join(config.RAIZ, "herramientas", "video_simulacion.py")
    spec = importlib.util.spec_from_file_location("video_simulacion", ruta)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_reloj_de_la_tirada():
    """Dentro de un movimiento del brazo cuenta solo lo que este espera; fuera, el reloj de pared."""
    vs = _herramienta()
    r = vs.Reloj()
    time.sleep(0.05)
    assert 0.04 < r.ahora() < 0.5
    t0 = r.ahora()

    def mover():
        time.sleep(0.1)             # el calculo de la simulacion acelerada no cuenta
        r.dormir(8.0)
    r.brazo(mover)()
    assert 8.0 <= r.ahora() - t0 < 8.05


def test_caida_con_la_gravedad():
    vs = _herramienta()
    M0, M1 = np.eye(4), np.eye(4)
    M0[:3, 3], M1[:3, 3] = [0.5, 0.2, 0.0], [0.5, 0.21, -0.06]
    T = math.sqrt(2 * 0.06 / vs.G)
    z = [vs.caida(M0, M1, tau)[0][2] for tau in np.linspace(0.0, T * 0.99, 10)]
    assert z[0] == 0.0 and all(np.diff(z) < 0) and z[-1] > -0.06
    assert vs.caida(M0, M1, T) is None


def test_camara_sin_saltos_al_cambiar_de_toma():
    """Una toma nueva antes de que acabe la transicion anterior sale de donde estaba la camara."""
    vs = _herramienta()
    a = (np.zeros(3), 2.0, 180.0, -30.0)
    b = (np.array([0.5, 0.2, 0.0]), 1.0, 215.0, -40.0)
    c = (np.array([0.4, -0.2, 0.0]), 1.0, 145.0, -35.0)
    tomas = [(0.0, a), (5.0, b), (5.6, c)]
    for v in (5.0, 5.6):
        antes, despues = vs.camara_en(tomas, v - 1e-6), vs.camara_en(tomas, v + 1e-6)
        assert np.allclose(antes[0], despues[0], atol=1e-4) and abs(antes[2] - despues[2]) < 1e-3
    assert np.allclose(vs.camara_en(tomas, 9.0)[0], c[0])


@gl
def test_video_de_la_tirada_desde_los_brazos_a_cero():
    from sim.escena import Estado, mm              # mm: el modelo del robot (h1_2_mujoco), lo importa escena
    vs = _herramienta()
    mon = config.cargar_montaje()
    grab = vs.grabar(config.cargar(), config.cargar_objetos(), mon)
    assert grab.motivo == "completado" and grab.finales == grab.asignacion and len(grab.asignacion) == 4
    est = Estado(grab.m, grab.info)
    brazos = [est.adr[n] for n in mm.JUNTAS_CUERPO[13:27]]
    for _, qpos, _, _ in (grab.muestras[0], grab.muestras[-1]):
        assert np.abs(qpos[brazos]).max() < 1e-3          # empieza y acaba con los brazos a 0
    assert len(grab.caidas) == 4
    assert all(0.0 < M0[2, 3] - M1[2, 3] < 0.12 for _, _, M0, M1 in grab.caidas)
    vid = vs.Video(grab, mon, 2.0, 320, 180)
    try:
        lin = vid.lin
        assert 100 < lin.duracion < 250
        assert vid.fotograma(1.0).shape == (180, 320, 3)
        assert lin.colocados(0.0) == 0 and lin.colocados(lin.duracion) == 4
        assert lin.rotulo_en(lin.duracion).startswith("4/4")
    finally:
        vid.cerrar()
