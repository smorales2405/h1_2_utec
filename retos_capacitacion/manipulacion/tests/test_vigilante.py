"""Vigilante: cada causa de salida segura de la seccion 10, provocada en el robot simulado."""

import copy
import time

import pytest

from manip import config
from manip.mensajes import Mando, Parada
from manip.vigilante import Vigilante
from sim.robot_simulado import RobotSimulado


@pytest.fixture
def robot():
    r = RobotSimulado()
    yield r
    r.parar()


class LectorFijo:
    def __init__(self, fsm):
        self.fsm = fsm


def _vig(robot, **kw):
    v = Vigilante(robot, copy.deepcopy(config.cargar()), **kw)
    time.sleep(0.05)
    v.iniciar()
    return v


def test_quieto_no_para(robot):
    v = _vig(robot, lector_fsm=LectorFijo(201), edad_fotograma=lambda: 0.05)
    assert v.motivo() is None
    v.comprobar()


@pytest.mark.parametrize("perturbar, texto", [
    (lambda r: r.girar(6.0), "giro"),
    (lambda r: r.dar_paso(0.2), "paso"),
    (lambda r: setattr(r, "roll", 0.2), "inclinacion"),
    (lambda r: setattr(r, "mando", Mando(0, 0.0, 0.3, 0.0, 0.0)), "joystick"),
    (lambda r: setattr(r, "fallos", [16]), "motor"),
])
def test_cada_causa(robot, perturbar, texto):
    v = _vig(robot)
    perturbar(robot)
    time.sleep(0.02)
    with pytest.raises(Parada, match=texto):
        v.comprobar()


def test_giro_bajo_el_umbral_no_para(robot):
    v = _vig(robot)
    robot.girar(4.0)
    time.sleep(0.02)
    assert v.motivo() is None


def test_fsm_camara_y_tecla(robot):
    assert "FSM" in _vig(robot, lector_fsm=LectorFijo(1)).motivo()
    assert "camara" in _vig(robot, edad_fotograma=lambda: 3.0).motivo()
    v = _vig(robot)
    v.pedir_parada("tecla ESPACIO")
    assert v.motivo() == "tecla ESPACIO"
