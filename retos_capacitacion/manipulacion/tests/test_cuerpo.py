"""rt/lowcmd de cuerpo entero (Hito 0): dinamica, entrada a Debug y el controlador contra el robot con fisica."""

import math

import numpy as np
import pytest

from manip import config
from manip import modo
from manip.cuerpo import BRAZOS, Cuerpo
from manip.dinamica import Dinamica
from sim.cuerpo_simulado import ConmutadorSimulado, CuerpoSimulado

CFG = config.cargar()


@pytest.fixture(scope="module")
def din():
    return Dinamica()


def postura_pie():
    c = CFG["cuerpo"]
    return np.radians(np.r_[c["postura_pie_deg"], c["brazos_cero_deg"]])


# --- dinamica -------------------------------------------------------------------------------------------
def test_masas_identificadas(din):
    m = config._leer(config.CONFIG + "/gravedad_brazos.yaml")
    assert din.masas_identificadas
    for lado in ("izq", "der"):
        assert m[lado]["rms_este_urdf_nm"] <= m[lado]["rms_ajuste_nm"] + 0.01      # reproduce el ajuste
        assert 1.0 < m[lado]["cuerpos"]["wrist_yaw"]["masa_kg"] < 1.3                # mano entera (~1.1 kg)


def test_carga_de_los_pies_inversa(din):
    q = postura_pie()
    for c in (0.3, 1.0):
        tau, fz = din.de_pie(q, c)
        fz2, c2 = din.carga_pies(q, tau)
        assert c2 == pytest.approx(c, abs=1e-6)
        assert fz2["izq"] == pytest.approx(fz["izq"], abs=1e-3)


def test_sosten_colgado_es_la_gravedad(din):
    q = postura_pie()
    assert np.allclose(din.sosten(q, 0.0), din.gravedad(q))
    tau, _ = din.de_pie(q, 1.0)
    assert tau[3] < -30 and tau[9] < -30               # de pie, las rodillas sostienen el cuerpo


# --- entrada a Debug ------------------------------------------------------------------------------------
def test_entrar_debug():
    log = []
    c = ConmutadorSimulado()
    assert modo.entrar_debug(c, contar=lambda s: 0, preguntar=lambda t: "SOLTAR", log=log.append,
                             dormir=lambda s: None) == "ai"
    assert c.activo() == "" and c.llamadas == 1
    with pytest.raises(RuntimeError, match="cancelado"):
        modo.entrar_debug(ConmutadorSimulado(), contar=lambda s: 0, preguntar=lambda t: "si", log=log.append,
                          dormir=lambda s: None)
    c = ConmutadorSimulado(falla=9)
    with pytest.raises(RuntimeError, match="3 ReleaseMode"):
        modo.entrar_debug(c, contar=lambda s: 0, preguntar=lambda t: "SOLTAR", log=log.append, dormir=lambda s: None)
    assert c.llamadas == 3
    with pytest.raises(RuntimeError, match="alguien publica"):
        modo.entrar_debug(ConmutadorSimulado(""), contar=lambda s: 12, preguntar=lambda t: "", log=log.append)


# --- controlador contra el robot con fisica -----------------------------------------------------------
def montar(postura=None, reloj=None):
    sim = CuerpoSimulado(hilo=False, postura=postura)
    cu = Cuerpo(sim, CFG, publicar=False, destino=sim.recibir_lowcmd, reloj=reloj or sim.reloj, dormir=lambda s: None)
    cu.iniciar(hilo=False)
    return sim, cu


def correr(sim, cu, segundos, cada_paso=None):
    for _ in range(int(round(segundos / sim.dt))):
        sim.paso()
        cu.ciclo()
        if cada_paso:
            cada_paso()


def test_toma_sin_mover_y_postura_de_pie():
    sim, cu = montar()
    q0 = sim.q()
    cu.tomar()
    correr(sim, cu, 3.5)
    assert cu.f_kp.valor == 1.0
    assert np.abs(sim.q()[:13] - q0[:13]).max() < math.radians(3)
    q = cu.consigna()
    q[:13] = cu.postura_pie
    cu.mover(q, 10, juntas=range(13), esperar=False)
    while cu.moviendo():
        correr(sim, cu, 0.1)
    correr(sim, cu, 1.0)
    assert np.abs(sim.q()[:13] - cu.postura_pie).max() < math.radians(3)


def cuerda_parcial(sim, cu):
    """El procedimiento de trabajo, ya nivelado: polipasto hasta que los pies lleven ~75 %."""
    t0 = sim.t
    while not 0.72 <= cu.carga <= 0.78 and sim.t - t0 < 60:
        sim.polipasto(0.001 if cu.carga > 0.78 else -0.001)
        correr(sim, cu, 0.2)
    correr(sim, cu, 2.0)


def ponerse_de_pie(sim, cu, cuerda="floja"):
    """Lo que hace herramientas/arnes.py: tara colgado, bajar al 60 %, estabilizador, par de pie y la cuerda:
    "floja" (3 cm) o None (como la dejan las piernas al ponerse de pie: descargada)."""
    cu.tomar()
    correr(sim, cu, 4.0)
    cu.tarar()
    while cu.carga < 0.6 and sim.t < 60:
        sim.polipasto(-0.002)
        correr(sim, cu, 0.1)
    correr(sim, cu, 3.0)
    cu.estabilizador(True)
    correr(sim, cu, 3.5)
    cu.apoyar(True)
    correr(sim, cu, 3.5)
    if cuerda == "floja":
        while sim.tension > 0.5 and sim.t < 120:
            sim.polipasto(-0.002)
            correr(sim, cu, 0.1)
        sim.polipasto(-0.03)
    correr(sim, cu, 2.0)


def tobillo_max(sim):
    return float(np.abs(sim.d.actuator_force[sim.act[[4, 10]]]).max())


def test_de_pie_sin_cuerda_con_el_estabilizador():
    sim, cu = montar(postura=postura_pie())
    ponerse_de_pie(sim, cu)
    cu.nivelar(True)
    correr(sim, cu, 30.0)
    pies0 = sim.pies_xy()
    peor = [0.0]

    def mirar():
        e = cu.estado
        peor[0] = max(peor[0], abs(e["pitch"] - cu.ref[1]), abs(e["roll"] - cu.ref[0]))
    q = cu.consigna()
    q[13:20] = [-0.32, 0.18, -1.07, 0.48, 0.28, 0.30, 0.70]          # brazo izquierdo adelante, como un agarre
    cu.mover(q, juntas=BRAZOS, esperar=False)
    while cu.moviendo():
        correr(sim, cu, 0.1, mirar)
    correr(sim, cu, 2.0, mirar)
    assert sim.tension == 0.0 and sim.carga_real() > 0.95               # de pie sin la cuerda
    assert cu.f_estab.valor == 1.0 and not cu.avisos
    assert math.degrees(peor[0]) < 3.0
    assert np.linalg.norm(sim.pies_xy() - pies0, axis=1).max() < 0.005   # sin deslizar


def test_nivelar_con_la_cuerda_floja_sin_atascarse():
    """En el robot real, con la cuerda floja, [n] llevo los tobillos a 54 N m (de 60): el torso se atascaba
    inclinado y saltaba de +8 a -5 grados durante ~110 s (2026-10-16). La simulacion lleva el desfase URDF-robot
    y lo reproduce con lo de entonces (sin sesgo del tobillo ni seguimiento de la referencia)."""
    obj = CFG["cuerpo"]["estabilizador"]["ref_pitch_deg"]
    sim, cu = montar(postura=postura_pie())
    ponerse_de_pie(sim, cu)
    cu.nivelar(True)
    peor = {"tobillo": 0.0, "error": 0.0}

    def mirar():
        peor["tobillo"] = max(peor["tobillo"], tobillo_max(sim))
        peor["error"] = max(peor["error"], abs(cu.estado["pitch"] - cu.ref[1]))
    correr(sim, cu, 30.0, mirar)
    p = []
    correr(sim, cu, 5.0, lambda: p.append(cu.estado["pitch"]))
    assert math.degrees(cu.ref[1]) == pytest.approx(obj)
    assert abs(math.degrees(np.mean(p)) - obj) < 0.7
    assert peor["tobillo"] < 40 and math.degrees(peor["error"]) < 2.0
    assert sim.tension == 0.0 and not cu.avisos
    cu.nivelar(False)                                            # al colgar: todo quieto
    i0 = cu.integral.copy()
    correr(sim, cu, 1.0)
    assert np.allclose(cu.integral, i0)


def test_procedimiento_de_trabajo_con_la_cuerda_llevando_parte():
    """Desde el 2026-10-16: [n] con la cuerda descargada por las piernas, luego la cuerda llevando ~25 % del peso
    (2026-10-17: nivelar despues movia la cinta y la cuerda tiraba), y un brazo adelante."""
    obj = CFG["cuerpo"]["estabilizador"]["ref_pitch_deg"]
    sim, cu = montar(postura=postura_pie())
    ponerse_de_pie(sim, cu, cuerda=None)
    cu.nivelar(True)
    correr(sim, cu, 30.0)
    cuerda_parcial(sim, cu)
    correr(sim, cu, 15.0)
    assert abs(math.degrees(cu.estado["pitch"]) - obj) < 0.7
    assert sim.tension > 50 and 0.6 < sim.carga_real() < 0.9               # la cuerda sigue llevando parte
    peor = {"tobillo": 0.0, "error": 0.0}

    def mirar():
        peor["tobillo"] = max(peor["tobillo"], tobillo_max(sim))
        peor["error"] = max(peor["error"], abs(cu.estado["pitch"] - cu.ref[1]), abs(cu.estado["roll"] - cu.ref[0]))
    q = cu.consigna()
    q[13:20] = [-0.32, 0.18, -1.07, 0.48, 0.28, 0.30, 0.70]
    cu.mover(q, juntas=BRAZOS, esperar=False)
    while cu.moviendo():
        correr(sim, cu, 0.1, mirar)
    correr(sim, cu, 2.0, mirar)
    assert math.degrees(peor["error"]) < 1.0 and peor["tobillo"] < 30
    assert cu.f_estab.valor == 1.0 and not cu.avisos


def test_al_colgar_se_apaga_el_estabilizador():
    sim, cu = montar(postura=postura_pie())
    ponerse_de_pie(sim, cu)
    while cu.carga > 0.3 and sim.t < 200:
        sim.polipasto(0.002)
        correr(sim, cu, 0.1)
    correr(sim, cu, 1.5)
    assert cu.f_estab.valor == 0.0
    assert any("estabilizador fuera" in t for _, t in cu.avisos)


def test_fallo_de_motor_amortigua():
    sim, cu = montar()
    cu.tomar()
    correr(sim, cu, 3.5)
    sim.fallos = [3]
    correr(sim, cu, 0.6)
    assert cu.amortiguado and "fallo" in cu.motivo
    assert cu.f_kp.valor == 0.0 and np.all(sim.cmd["kp"] == 0.0)
    with pytest.raises(RuntimeError):
        cu.mover(cu.consigna())


def test_lowstate_viejo_amortigua():
    extra = [0.0]
    sim = CuerpoSimulado(hilo=False)
    cu = Cuerpo(sim, CFG, publicar=False, destino=sim.recibir_lowcmd, reloj=lambda: sim.t + extra[0],
                dormir=lambda s: None)
    cu.iniciar(hilo=False)
    cu.tomar()
    correr(sim, cu, 3.5)
    for _ in range(200):                    # 0.4 s sin lowstate: el reloj sigue y el robot no
        extra[0] += 0.002
        cu.ciclo()
    assert cu.amortiguado and "lowstate" in cu.motivo


def test_inclinacion_apaga_el_estabilizador():
    sim, cu = montar(postura=postura_pie())
    ponerse_de_pie(sim, cu)
    cu.ref = cu.ref + np.radians([0.0, 12.0])          # como si el robot se hubiera ido 12 grados
    correr(sim, cu, 0.5)
    assert cu.f_estab.meta == 0.0
    assert any("inclinacion" in t for _, t in cu.avisos)


def test_l2b_del_mando_amortigua():
    """En Debug el robot no hace nada con L2+B (2026-10-16): lo hace el hilo de control."""
    from manip.mando import BOTONES
    from manip.mensajes import Mando
    sim, cu = montar()
    cu.tomar()
    correr(sim, cu, 3.5)
    sim.mando = Mando(1 << BOTONES.index("L2"), 0.0, 0.0, 0.0, 0.0)          # solo L2: nada
    correr(sim, cu, 0.1)
    assert not cu.amortiguado
    sim.mando = Mando((1 << BOTONES.index("L2")) | (1 << BOTONES.index("B")), 0.0, 0.0, 0.0, 0.0)
    correr(sim, cu, 0.6)
    assert cu.amortiguado and "L2+B" in cu.motivo
    assert cu.f_kp.valor == 0.0 and np.all(sim.cmd["kp"] == 0.0) and np.any(sim.cmd["kd"] > 0.0)
    cu.soltar(3.0, "fin")                                                    # no frena una suelta mas rapida
    assert "L2+B" in cu.motivo


def test_tara_colgado():
    sim, cu = montar(postura=postura_pie())
    cu.tomar()
    correr(sim, cu, 4.0)
    tara = cu.tarar()
    correr(sim, cu, 1.0)
    assert abs(cu.carga) < 0.02                                              # colgado marca 0
    assert abs(tara["izq"] + tara["der"]) > 1.0                              # habia algo que tarar
    cu.tara = {"izq": 0.0, "der": 0.0}
    cu._fz_hist = [(0.02 * k, {"izq": 120.0, "der": 120.0}) for k in range(100)]   # pies apoyados: no se tara
    with pytest.raises(RuntimeError, match="apoyan"):
        cu.tarar()
