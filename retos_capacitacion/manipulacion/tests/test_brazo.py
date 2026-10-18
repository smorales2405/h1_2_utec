"""Brazo: los topes de la seccion 10 se cumplen aunque la configuracion pida otra cosa (sin robot)."""

import copy

import numpy as np
import pytest

from manip import config
from manip.brazo import Brazo
from sim.robot_simulado import RobotSimulado


@pytest.fixture
def robot():
    r = RobotSimulado()
    yield r
    r.parar()


def _brazo(robot, cambios=None, **kw):
    cfg = copy.deepcopy(config.cargar())
    for k, v in (cambios or {}).items():
        cfg["brazo"][k] = v
    recibido = []
    b = Brazo(robot, cfg, simulacro=True, destino=lambda o, p, kp: recibido.append((o, p, kp)),
              dormir=lambda s: None, **kw)
    return b, recibido


def test_topes_aunque_la_config_pida_mas(robot):
    b, recibido = _brazo(robot, {"kp": 300.0, "t_min_mov_s": 3.0, "vel_max_rad_s": 2.0, "t_rampa_salida_s": 1.0})
    assert max(v for j, v in b.kp.items() if j != 12) == 150.0 and b.kp[12] == 200.0
    ini = b.consigna()
    assert b.duracion(ini, dict(ini)) == 8.0                            # nunca por debajo de 8 s
    fin = dict(ini)
    fin[16] += 3.0
    assert b.duracion(ini, fin) == pytest.approx(1.875 * 3.0 / 0.5)     # pico de 0.5 rad/s
    assert b.rampa_peso(1.0) == 12.0 and b.peso == 1.0
    assert b.rampa_peso(0.0) == 5.0 and b.peso == 0.0                   # salida >= 5 s
    b.parar()


def test_escala_de_tiempo(robot):
    b, _ = _brazo(robot, escala_tiempo=2.0)
    ini = b.consigna()
    assert b.duracion(ini, dict(ini)) == 16.0
    b.parar()


def test_la_escala_se_aplica_una_sola_vez(robot):
    """El planificador da la duracion base y brazo.py la escala: con la escala 2, un tramo minimo dura 16 s, no 32."""
    from manip.planificacion import Planificador
    cfg = config.cargar()
    pl = Planificador(cfg, config.cargar_montaje(), config.cargar_objetos(), None, escala_tiempo=2.0)
    q = np.zeros(7)
    assert pl.duracion(q, q + 0.1) == 8.0
    b, _ = _brazo(robot, escala_tiempo=2.0)
    ini = b.consigna()
    fin = dict(ini)
    fin[16] += 0.1
    assert b.ir_a(fin, pl.duracion(q, q + 0.1)) == 16.0
    b.parar()


def test_ir_a_respeta_limites_y_no_mueve_la_cintura(robot):
    b, _ = _brazo(robot)
    c12 = b.consigna()[12]
    b.ir_a({12: 0.5, 16: 5.0, 18: -2.0})
    c = b.consigna()
    assert c[12] == c12
    assert c[16] == pytest.approx(1.60 - 0.05) and c[18] == pytest.approx(-0.4625 + 0.05)
    b.parar()


def test_congelar_y_parar(robot):
    b, recibido = _brazo(robot)
    b.peso = 1.0
    with pytest.raises(RuntimeError):
        b.parar()                                                       # con peso no se deja de publicar
    b.congelar()
    q = robot.q()
    assert all(abs(b.consigna()[j] - q[j]) < 0.02 for j in range(13, 27))
    b.peso = 0.0
    b.parar()
    assert recibido and recibido[-1][2][16] == 100.0


def test_el_simulado_sigue_la_consigna_con_caida(robot):
    """Con peso 1 el robot simulado va a la consigna pero se queda corto, como el real con kp 100."""
    import time
    b, _ = _brazo(robot)
    b._dormir = time.sleep
    b.hz = 200.0
    b.peso = 1.0
    robot.recibir_armsdk(b.consigna(), 1.0, b.kp)
    b.destino = robot.recibir_armsdk
    with b.cerrojo:
        b.obj[16] = b.obj[16] + 0.4
    time.sleep(1.5)
    err = abs(robot.q()[16] - b.consigna()[16])
    assert 0.05 < err < 0.12
    b.peso = 0.0
    b.parar()
