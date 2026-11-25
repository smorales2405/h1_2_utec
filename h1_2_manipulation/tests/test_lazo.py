"""El lazo de agarre (agarre/lazo.py) en la mano regulada, sobre la mano simulada con la planta del banco
(agarre/sim_mano.py): sin OpenGL ni mundo, la mano sola con el objeto puesto entre los dedos."""

import copy
import itertools
import time
from types import SimpleNamespace

import numpy as np
import pytest

from manip import config
from manip.mano import Mano
from manip.mensajes import Parada
from sim.mano_simulada import calibracion_simulada

from agarre.banco import calendario
from agarre.lazo import DEDOS_MODO
from agarre.mano_regulada import ManoRegulada, cargar_config
from agarre.sim_mano import CONTACTO, ManoFisica, planta
from analisis.comparar import fisher, mann_whitney

LADO = {"cubo": "der", "esfera": "der", "cilindro": "izq", "prisma": "izq", "prisma_de_pie": "izq"}


def _cfg():
    cfg = copy.deepcopy(config.cargar())
    cfg["mano"]["t_base_s"] = 0.0          # la mano simulada no tiene deriva del cero
    return cfg


def _mano(clase, regular=True, rc=None, sim=None, clase_mano=ManoRegulada, tmp_path=None, **kw_sim):
    cfg = _cfg()
    sim = sim or ManoFisica(**kw_sim)
    if clase_mano is ManoRegulada:
        m = ManoRegulada(LADO[clase], cfg, cliente=sim, calibracion=calibracion_simulada(cfg), regulador=rc or cargar_config(),
                         log=lambda *a: None)
        m.regular = regular
        m.carpeta = str(tmp_path) if tmp_path else None
    else:
        m = Mano(LADO[clase], cfg, cliente=sim, calibracion=calibracion_simulada(cfg))
    m.sup = SimpleNamespace(_plan_activo=SimpleNamespace(objeto=SimpleNamespace(clase=clase)), estado="CERRAR")
    return m, sim


def _coger(m, sim, clase, **kw_planta):
    o = config.cargar_objetos()
    ob = o["objetos"][clase]
    m.preforma(o["preformas"][ob["preforma"]])
    sim.poner_objeto(CONTACTO[clase], **(kw_planta or planta(clase)))
    e = m.cerrar(ob["force_set"], speed=ob.get("speed_cierre"))
    return e, ob


def _apriete(m, clase):
    e = m.leer()
    F = e.fuerza - getattr(m, "cero", np.zeros(6))
    dedos = DEDOS_MODO[cargar_config()["objetos"][clase]["modo"]]
    return (F[dedos[0]] + sum(F[d] for d in dedos[1:])) / 2.0, F


def _sostener(m, s, fase="TRANSPORTE"):
    m.sup.estado = fase
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        m.leer()
        time.sleep(0.05)


@pytest.mark.parametrize("clase", ["cubo", "esfera", "cilindro", "prisma", "prisma_de_pie"])
def test_el_agarre_regulado_de_cada_objeto(clase, tmp_path):
    """Cada dedo regulado toca y se congela, el apriete llega a su consigna sin pasarse y se establece."""
    rc = cargar_config()
    m, sim = _mano(clase, tmp_path=tmp_path)
    e, ob = _coger(m, sim, clase)
    lz = m.lazo
    assert lz is not None and not lz.observador
    ok, det = m.agarre(e, ob["force_set"])
    assert ok, det
    consigna = rc["objetos"][clase]["fuerza_gf"]
    assert lz.establecido.is_set(), lz.eventos
    assert set(lz.contacto) == set(DEDOS_MODO[rc["objetos"][clase]["modo"]]), lz.contacto
    a, F = _apriete(m, clase)
    assert abs(a - consigna) <= rc["control"]["banda_apriete_gf"] + 15, (a, consigna)
    _sostener(m, 1.0)
    m.abrir()
    r = m.agarres[-1]
    assert r["regulado"] and not r["perdido"] and r["fallo"] is None
    assert r["apriete_por_fase"]["CERRAR"]["max"] <= 1.25 * consigna, r["apriete_por_fase"]
    assert not [x for x in r["eventos"] if x["tipo"] in ("guarda", "resbalon", "escape")], r["eventos"]
    assert (tmp_path / f"regulador_01_{clase}_{LADO[clase]}.csv").exists()
    # la esfera: el anular cierra por FORCE_SET, como en el reto (no es del lazo)
    if clase == "esfera":
        assert e.angulo[1] < 700


def test_mantiene_el_apriete_cuando_la_fuerza_se_relaja():
    """E3.4: sin par activo, la fuerza cae y se agota. El firmware la deja caer; el lazo la repone."""
    m, sim = _mano("cilindro", relajar=0.20, tau=0.6)
    _coger(m, sim, "cilindro")
    a0, _ = _apriete(m, "cilindro")
    _sostener(m, 3.0)
    a1, _ = _apriete(m, "cilindro")
    m.abrir()
    f, sim_f = _mano("cilindro", clase_mano=Mano, relajar=0.20, tau=0.6)
    _coger(f, sim_f, "cilindro")
    b0, _ = _apriete(f, "cilindro")
    _sostener(f, 3.0)
    b1, _ = _apriete(f, "cilindro")
    f.abrir()
    consigna = cargar_config()["objetos"]["cilindro"]["fuerza_gf"]
    assert abs(a1 - consigna) <= cargar_config()["control"]["banda_apriete_gf"] + 15, (a0, a1)
    assert b1 <= 0.93 * b0, (b0, b1)          # la Mano del reto, en bruto (con el cero): cae ~10 %


def test_el_objeto_que_resbala_se_aprieta():
    """El objeto se escurre (menos penetracion en todos los dedos, POS quieto): el detector de escape sube la consigna
    y el lazo recupera el apriete sin darlo por perdido. Con la planta blanda (la del banco x 1.5): en la del cubo
    medido, que caiga un 35 % en todos los dedos es que el cubo ya se va (el indice lo pierde todo con 12 cuentas)."""
    rc = cargar_config()
    m, sim = _mano("cubo")
    _coger(m, sim, "cubo", rigidez=1.5)
    _sostener(m, 0.5)
    sim.resbalar(15)
    _sostener(m, 1.5)
    a, _ = _apriete(m, "cubo")
    lz = m.lazo
    assert any(x["tipo"] == "escape" for x in lz.eventos), lz.eventos
    assert lz.f_ref > rc["objetos"]["cubo"]["fuerza_gf"]
    assert not [x for x in lz.eventos if x["tipo"] == "guarda"], lz.eventos     # apretando, sin saltar la guarda
    assert not lz.perdido
    assert a >= 0.9 * rc["objetos"]["cubo"]["fuerza_gf"], a
    m.abrir()


def test_sostiene_el_cubo_sin_vaiven_de_los_dedos():
    """En el robot (2026-11-25), con el brazo moviendose, el reparto con banda de 40 no asentaba en el cubo de PLA (un
    escalon del indice, ~155 gf): sigma 78-115 gf y decenas de ordenes en el transporte. La planta medida del cubo con
    un vaiven lo reproduce; establecido el agarre, solo el apriete con banda de 50 lo quita."""
    res = {}
    for viejo in (True, False):
        rc = cargar_config()
        if viejo:
            rc["control"]["sostener_solo_apriete"] = False
        m, sim = _mano("cubo", rc=rc)
        _coger(m, sim, "cubo")
        sim.vaiven = (200.0, 1.2)
        n0 = m.lazo.n_acciones
        rep = []
        m.sup.estado = "TRANSPORTE"
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            F = m.leer().fuerza - m.cero
            rep.append(F[3] - F[2])
            time.sleep(0.02)
        res[viejo] = (m.lazo.n_acciones - n0, float(np.std(rep)))
        assert not m.lazo.perdido and not m.lazo.fallo
        m.abrir()
    assert res[True][0] >= 10, res                                # antes: los dedos van y vienen (21-26 ordenes)
    assert res[False][0] <= res[True][0] / 3, res                 # ahora: 0-5
    assert res[False][1] < min(15.0, res[True][1] / 2), res       # sigma del reparto: ~6 frente a 25-98


def test_un_tiron_del_pulgar_no_es_resbalon():
    """El pulgar echado atras 14 que vuelve solo (el vaiven del transporte): cede breve, sin aflojar ni bajar la
    consigna. Echado atras 30 y sin volver (el actuador cede de verdad): resbalon, afloja y baja la consigna."""
    m, sim = _mano("cubo")
    _coger(m, sim, "cubo")
    _sostener(m, 0.5)
    lz = m.lazo
    f0 = lz.f_ref
    sim.empujar(4, 14)
    _sostener(m, 2.0)                       # el lazo, si abre al volver el pulgar, deja 0.8 s sin historia al detector
    tipos = [x["tipo"] for x in lz.eventos]
    assert "cede_breve" in tipos and "resbalon" not in tipos, lz.eventos
    assert lz.f_ref == f0
    sim.empujar(4, 30, sostenido=True)
    _sostener(m, 1.0)
    assert "resbalon" in [x["tipo"] for x in lz.eventos] and lz.f_ref < f0, lz.eventos
    m.abrir()
    r = m.agarres[-1]
    assert r["eventos"] and not r["perdido"]


def test_el_objeto_que_se_va_es_un_escape():
    m, sim = _mano("prisma")
    e, ob = _coger(m, sim, "prisma")
    _sostener(m, 0.5)
    assert not m.escapado(m.leer(), ob["force_set"])
    sim.quitar_objeto()
    t0 = time.monotonic()
    while not m.escapado(m.leer(), ob["force_set"]) and time.monotonic() - t0 < 3.0:
        time.sleep(0.05)
    assert m.escapado(m.leer(), ob["force_set"]), m.lazo.eventos
    assert time.monotonic() - t0 < 1.5
    m.abrir()
    assert m.lazo is None and m.agarres[-1]["perdido"]


def test_la_guarda_afloja_por_encima_del_techo():
    """Una consigna por encima del techo de la guarda: la guarda afloja y baja la consigna, sin pasar del respaldo."""
    rc = cargar_config()
    rc["objetos"]["cilindro"]["fuerza_gf"] = 900
    m, sim = _mano("cilindro", rc=rc)
    _coger(m, sim, "cilindro")
    _sostener(m, 2.0)
    lz = m.lazo
    assert any(x["tipo"] == "guarda" for x in lz.eventos), lz.eventos
    assert lz.f_ref < 900
    a, F = _apriete(m, "cilindro")
    assert F[:5].max() < rc["seguridad"]["fset_respaldo_gf"], F
    m.abrir()


def test_el_brazo_firmware_es_la_mano_del_reto():
    """Con regular = False, la mano escribe en la mano exactamente lo mismo que la Mano del reto (el lazo, en modo
    observador, solo lee), y mide igual."""
    o = config.cargar_objetos()
    escrituras = []
    for clase_mano, regular in ((Mano, None), (ManoRegulada, False)):
        m, sim = _mano("cubo", regular=bool(regular), clase_mano=clase_mano)
        e, ob = _coger(m, sim, "cubo")
        if clase_mano is ManoRegulada:
            assert m.lazo is not None and m.lazo.observador
            assert not m.escapado(e, ob["force_set"])
        m.sujetar(ob["force_set"])
        m.abrir()
        escrituras.append([(a, list(v)) for a, v in sim.escrituras])
    assert escrituras[0] == escrituras[1]
    r = m.agarres[-1]
    assert not r["regulado"] and r["apriete_por_fase"]["CERRAR"]["n"] > 10


def test_un_fallo_del_lazo_es_una_parada():
    m, sim = _mano("cubo")
    _coger(m, sim, "cubo")
    m.lazo.fallo = "prueba"
    with pytest.raises(Parada):
        m.leer()
    assert m.lazo is None
    m.abrir()


def test_sin_config_del_objeto_es_el_agarre_del_reto():
    rc = cargar_config()
    del rc["objetos"]["cubo"]
    m, sim = _mano("cubo", rc=rc)
    _coger(m, sim, "cubo")
    assert m.lazo is None
    m.abrir()


def test_la_config_cubre_los_objetos_del_reto():
    """Cada objeto del reto tiene su modo, y sus dedos regulados cierran en objetos.yaml."""
    rc = cargar_config()
    o = config.cargar_objetos()["objetos"]
    for clase, ob in o.items():
        assert clase in rc["objetos"], clase
        for d in DEDOS_MODO[rc["objetos"][clase]["modo"]]:
            assert ob["force_set"][d] > 0, (clase, d)


def test_el_calendario_intercala_los_brazos():
    for semilla in range(20):
        c = calendario(20, semilla=semilla)
        assert c.count("regulador") == c.count("firmware") == 10
        for i in range(0, 20, 2):
            assert sorted(c[i:i + 2]) == ["firmware", "regulador"]
        assert max(len(list(g)) for _, g in itertools.groupby(c)) <= 2
    assert calendario(10, semilla=3) == calendario(10, semilla=3)


def test_las_pruebas_estadisticas():
    assert fisher(10, 0, 5, 5) == pytest.approx(0.0325, abs=1e-3)
    assert fisher(5, 5, 5, 5) == pytest.approx(1.0)
    assert mann_whitney([1, 2, 3, 4, 5], [6, 7, 8, 9, 10]) == pytest.approx(2 / 252, rel=1e-6)
    assert mann_whitney([1, 2, 3], [1, 2, 3]) == pytest.approx(1.0)
