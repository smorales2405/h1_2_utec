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


def test_una_lectura_corrupta_no_para():
    """inspire_hand_interface: lecturas sueltas de 3187 y 1873 gf. Una sola por encima del corte no para; tres
    seguidas, si."""
    from types import SimpleNamespace
    from manip.mensajes import Parada
    mano, sim = _mano()
    leer = sim.read_holding_registers
    n, picos = [0], set()

    def con_pico(a, k, slave=1):
        r = leer(a, k, slave)
        if a == m.FORCE_ACT:
            n[0] += 1
            if n[0] in picos:
                regs = list(r.registers)
                regs[3] = 3187
                return SimpleNamespace(registers=regs, isError=lambda: False)
        return r
    sim.read_holding_registers = con_pico
    picos.update({3})
    mano.cerrar([-1, -1, 300, 300, 350, -1])                   # un pico suelto: sigue
    mano.abrir()
    n[0] = 0
    picos.clear()
    picos.update({3, 4, 5})
    with pytest.raises(Parada, match="3 lecturas seguidas"):
        mano.cerrar([-1, -1, 300, 300, 350, -1])


def test_el_vigilante_cuenta_lecturas_distintas():
    from types import SimpleNamespace
    from manip.vigilante import Vigilante
    cfg = config.cargar()
    base = np.zeros(6)
    lectura = lambda t, f: SimpleNamespace(t=t, fuerza=np.array([0, 0, 0, f, 0, 0], float), base=base)
    mano = SimpleNamespace(ultimo=lectura(1.0, 3187.0))
    vig = Vigilante(SimpleNamespace(), cfg, manos={"izq": mano})
    for _ in range(5):                                          # la misma lectura vista 5 veces: cuenta una
        assert vig.fuerza_persistente()[1] == 1
    mano.ultimo = lectura(2.0, 300.0)
    assert vig.fuerza_persistente()[1] == 0
    for k in range(3):
        mano.ultimo = lectura(3.0 + k, 1500.0)
        f, n = vig.fuerza_persistente()
    assert n == 3 and f == 1500.0
