"""Mano: FORCE_SET antes de cerrar y con tope, nunca SAVE, comprobacion del agarre y apertura comprobada
(con el enclavamiento medido en banco y un dedo que no responde hasta borrar el error)."""

import copy

import numpy as np
import pytest

from manip import config
from manip import mano as m
from manip.mano import Mano, ManoSinRespuesta
from sim.mano_simulada import ManoSimulada


def _mano(sim=None, **cambios):
    cfg = copy.deepcopy(config.cargar())
    cfg["mano"].update(cambios)
    cfg["mano"]["t_base_s"] = 0.0
    sim = sim or ManoSimulada()
    return Mano("izq", cfg, cliente=sim), sim


def test_tope_de_force_set_y_nunca_save():
    mano, sim = _mano(force_set_max=2000, speed_cierre=500)
    mano.escribir_force_set([1500, 900, 300, -1, 700, -1])
    assert sim.fset.tolist() == [800, 800, 300, 800, 700, 800]
    assert mano.speed_cierre == 100
    with pytest.raises(ValueError):
        mano.escribir(m.SAVE, [1])
    with pytest.raises(ValueError):
        mano.escribir(1000, [0] * 10)                 # un bloque que pisaria 1005


def test_force_set_se_escribe_antes_de_cerrar():
    mano, sim = _mano()
    sim.escrituras.clear()
    mano.cerrar([-1, -1, 300, 300, 350, -1])
    regs = [a for a, _ in sim.escrituras]
    assert regs.index(m.FORCE_SET) < regs.index(m.ANGLE_SET)


def test_agarre_lleno_y_vacio():
    mano, sim = _mano()
    mano.linea_base(n=2)
    fs = [-1, -1, 300, 300, 350, -1]
    sim.poner_objeto({2: 650, 3: 650, 4: 700})
    e = mano.cerrar(fs)
    ok, det = mano.agarre(e, fs)
    assert ok and det[3] == "lleno" and det[4] == "lleno"
    mano.abrir()
    sim.poner_objeto({})
    e = mano.cerrar(fs)
    ok, det = mano.agarre(e, fs)
    assert not ok and det[3] == "vacio"


def test_abre_aunque_los_dedos_esten_enclavados():
    mano, sim = _mano()
    fs = [-1, -1, 300, 300, 350, -1]
    sim.poner_objeto({2: 650, 3: 650, 4: 700})
    mano.cerrar(fs)
    assert sim.enclavado[3]                           # llego a FORCE_SET: no acepta ANGLE_SET
    sim.poner_objeto({})
    e = mano.abrir()
    assert e.angulo[:5].min() >= 900


def test_dedo_atascado_se_recupera_borrando_el_error():
    mano, sim = _mano(t_abrir_s=0.8)
    mano.cerrar([-1, -1, 300, 300, 350, -1])
    sim.atascado = {3}
    e = mano.abrir()                                  # el primer intento no abre; CLEAR y reintento
    assert e.angulo[3] >= 900
    assert any(a == m.CLEAR for a, _ in sim.escrituras)


def test_mano_que_no_abre():
    mano, sim = _mano(t_abrir_s=0.5)
    mano.cerrar([-1, -1, 300, 300, 350, -1])

    class Terca(set):
        def clear(self):
            pass
    sim.atascado = Terca({3})
    with pytest.raises(ManoSinRespuesta):
        mano.abrir()


def test_error_y_estado_van_en_bytes():
    # como la mano real por Modbus TCP: 3 registros con dos DOF cada uno; los 3 siguientes dan 0xFFFF
    assert m.bytes_de([0x0102, 0x0300, 0x0005]) == [1, 2, 3, 0, 0, 5]
    mano, sim = _mano()
    sim.error[3] = 4
    assert sim.read_holding_registers(m.ERROR, 6).registers[3:] == [0xFFFF] * 3
    e = mano.leer()
    assert e.error.tolist() == [0, 0, 0, 4, 0, 0]
    assert e.estado.tolist() == [0] * 6
