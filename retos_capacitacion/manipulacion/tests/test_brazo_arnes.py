"""manipulacion.py con el robot de pie en el arnes (opcion B): arnes.py presta los brazos por manip/servidor_brazos.py
y manipulacion.py los mueve con manip/brazo_arnes.py. Con el Cuerpo de verdad (sin publicar) sobre el robot
simulado y un reloj 40 veces mas rapido: los movimientos de 8 s duran 0.2 s."""

import socket
import time

import numpy as np
import pytest

from manip import config
from manip.brazo_arnes import BrazoArnes
from manip.cuerpo import BRAZOS, Cuerpo
from manip.mensajes import Parada
from manip.servidor_brazos import ServidorBrazos
from manip.vigilante import Vigilante
from sim.escena import postura_arnes
from sim.robot_simulado import RobotSimulado

CFG = config.cargar()
VEL = 40.0


class Reloj:
    def __init__(self, vel=VEL):
        self.vel, self.t0 = vel, time.monotonic()

    def __call__(self):
        return self.t0 + (time.monotonic() - self.t0) * self.vel


class RobotRapido:
    """El robot con el reloj de Cuerpo (si no, su lowstate pareceria viejo y amortiguaria)."""

    def __init__(self, robot, reloj):
        self.robot, self.reloj = robot, reloj

    def ultimo(self):
        return self.robot.ultimo()[0], self.reloj()

    def __getattr__(self, k):
        return getattr(self.robot, k)


def puerto_libre():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def montar(robot=None, listo=None, latido_max_s=2.0):
    """(robot, cuerpo, servidor, eventos) con el robot de pie y los brazos a 0."""
    robot = robot or RobotSimulado(postura=[float(v) for v in postura_arnes(CFG)], caida_rad=0.0, instantaneo=True)
    reloj = Reloj()
    cu = Cuerpo(RobotRapido(robot, reloj), CFG, publicar=False, reloj=reloj, dormir=lambda s: time.sleep(s / VEL),
                destino=lambda cmd: robot.recibir_armsdk({j: float(cmd["q"][j]) for j in range(12, 27)}, 1.0, {}))
    cu.iniciar()
    eventos = []
    estado = {"listo": (True, "")}
    srv = ServidorBrazos(cu, puerto_libre(), listo or (lambda: estado["listo"]), cu.brazos_cero,
                         log=eventos.append, latido_max_s=latido_max_s)
    srv.iniciar()
    srv.estado_listo = estado
    return robot, cu, srv, eventos


def cerrar(robot, cu, srv, *brazos):
    for b in brazos:
        b._vivo = False
        b._cerrar_socket()
    srv.parar()
    cu._vivo = False
    robot.parar()


@pytest.fixture
def montaje():
    m = montar()
    yield m
    cerrar(*m[:3])


def cliente(srv, **kw):
    return BrazoArnes(srv_robot(srv), CFG, escala_tiempo=1.0, puerto=srv.puerto, **kw)


def srv_robot(srv):
    return srv.cu.robot


ADELANTE_IZQ = [-0.32, 0.18, -1.07, 0.48, 0.28, 0.30, 0.70]     # el [m] de arnes.py


def test_toma_mueve_y_suelta(montaje):
    robot, cu, srv, eventos = montaje
    b = cliente(srv)
    srv.estado_listo["listo"] = (False, "sin nivelar ([n])")
    with pytest.raises(Parada, match="sin nivelar"):
        b.rampa_peso(1.0)
    srv.estado_listo["listo"] = (True, "")
    b.rampa_peso(1.0)
    assert b.peso == 1.0 and srv.controlado == b.id
    otro = cliente(srv)
    with pytest.raises(Parada, match="ya tiene los brazos"):
        otro.rampa_peso(1.0)
    T = b.ir_a(b.pose_brazo("izq", ADELANTE_IZQ))
    assert T >= 8.0
    assert np.allclose(robot.q()[13:20], ADELANTE_IZQ, atol=1e-3)          # el brazo llego
    assert np.allclose(robot.q()[20:27], 0.0, atol=1e-6)                   # el otro, quieto
    assert b.error() < 1e-3 and b.motivo_parada() is None
    b.ir_a(b.pose_brazo("izq", [0.0] * 7))
    b.rampa_peso(0.0)
    assert srv.controlado is None
    otro.rampa_peso(1.0)                                                    # ya puede el otro
    otro.rampa_peso(0.0)
    assert srv.movimientos == 2
    cerrar(robot, cu, srv, b, otro)


def test_brazos_fuera_de_cero_solo_para_recuperar(montaje):
    robot, cu, srv, _ = montaje
    q = cu.consigna()
    q[13:20] = ADELANTE_IZQ
    cu.mover(q, juntas=BRAZOS)                                              # como si se hubiera quedado ahi
    b = cliente(srv)
    with pytest.raises(Parada, match="manipulacion.py --arnes --recuperar"):
        b.rampa_peso(1.0)
    r = cliente(srv, recuperar=True)
    r.rampa_peso(1.0)
    r.ir_a(r.pose_brazo("izq", [0.0] * 7))
    r.rampa_peso(0.0)
    assert np.allclose(robot.q()[13:20], 0.0, atol=1e-3)
    cerrar(robot, cu, srv, b, r)


def test_comprobar_que_salta_congela_el_brazo(montaje):
    """El contacto de la yema (o una Parada) en mitad del movimiento: con arm_sdk el brazo se paraba porque los
    pasos los daba el propio bucle; con arnes.py hay que congelarlo antes de dejar salir la excepcion."""
    robot, cu, srv, _ = montaje
    b = cliente(srv)
    b.rampa_peso(1.0)
    t0 = time.monotonic()

    def contacto():
        if time.monotonic() - t0 > 0.08:                   # ~3 s del movimiento de 8 s
            raise RuntimeError("contacto")
    with pytest.raises(RuntimeError, match="contacto"):
        b.ir_a(b.pose_brazo("izq", ADELANTE_IZQ), comprobar=contacto)
    assert not cu.moviendo()
    q_parado = robot.q()[13:20].copy()
    time.sleep(0.3)                                        # mas de un movimiento entero: no sigue
    assert np.allclose(robot.q()[13:20], q_parado, atol=1e-6)
    assert 0.02 < np.abs(q_parado - ADELANTE_IZQ).max()    # se quedo a medio camino
    b.rampa_peso(0.0)
    cerrar(robot, cu, srv, b)


def test_sin_latido_congela_y_quita_el_control():
    robot, cu, srv, eventos = montar(latido_max_s=0.4)
    b = cliente(srv)
    b.rampa_peso(1.0)
    b._vivo = False                                        # manipulacion.py se cuelga: sin latido
    time.sleep(0.2)                                        # a mitad del plazo (a los 0.4 justos, una carrera con la
    assert srv.controlado == b.id                          # comprobacion del servidor, cada 0.1 s): aun no
    time.sleep(0.7)
    assert srv.controlado is None and any("sin noticias" in e for e in eventos)
    assert "quitado el control" in b.motivo_parada()
    with pytest.raises(Parada, match="sin el control|quitado"):
        b.ir_a(b.pose_brazo("izq", ADELANTE_IZQ))
    cerrar(robot, cu, srv, b)


def test_parada_de_arnes_deja_solo_la_salida_segura(montaje):
    robot, cu, srv, _ = montaje
    b = cliente(srv)
    b.rampa_peso(1.0)
    vig = Vigilante(robot, CFG, brazo=b)
    assert vig.motivo() is None
    t0 = time.monotonic()

    def parada_en_arnes():                                  # ESPACIO en arnes.py a mitad del movimiento
        if time.monotonic() - t0 > 0.05 and srv.parada is None:
            srv.parar_remoto("ESPACIO")
    with pytest.raises(Parada, match="parada en arnes.py: ESPACIO"):
        b.ir_a(b.pose_brazo("izq", ADELANTE_IZQ), comprobar=parada_en_arnes)
    assert not cu.moviendo()
    assert vig.motivo() == "parada en arnes.py: ESPACIO"    # el vigilante tambien la ve
    b.en_salida_segura()                                   # lo que hace el supervisor al empezar su salida segura
    assert vig.motivo() is None
    b.ir_a(b.pose_brazo("izq", [0.0] * 7))                 # la salida si se acepta
    assert np.allclose(robot.q()[13:20], 0.0, atol=1e-3)
    b.rampa_peso(0.0)
    assert srv.controlado is None
    cerrar(robot, cu, srv, b)


def test_amortiguado_no_mueve(montaje):
    robot, cu, srv, _ = montaje
    b = cliente(srv)
    b.rampa_peso(1.0)
    cu.soltar(0.5, "L2+B del mando")
    time.sleep(0.15)                                       # el estado guardado tiene <= 0.1 s
    assert "amortiguado" in b.motivo_parada()
    b.en_salida_segura()
    with pytest.raises(Parada, match="amortiguado"):       # ni la salida
        b.ir_a(b.pose_brazo("izq", [0.1] * 7))
    cerrar(robot, cu, srv, b)


def test_arnes_py_caido():
    robot, cu, srv, _ = montar()
    b = cliente(srv)
    b.rampa_peso(1.0)
    srv.parar()                                            # arnes.py termina (o el servidor muere)
    b.sin_respuesta = 0.2
    time.sleep(0.3)
    assert "no contesta" in b.motivo_parada()
    with pytest.raises(Parada, match="no contesta"):
        b.ir_a(b.pose_brazo("izq", ADELANTE_IZQ))
    with pytest.raises(RuntimeError, match="no contesta en"):
        BrazoArnes(robot, CFG, puerto=srv.puerto)
    b._vivo = False
    cu._vivo = False
    robot.parar()


def tirada_con_arnes(recuperar=False):
    """manipulacion.py --sim --arnes (nivel 2, las cuatro colocaciones de la simulacion) con los brazos movidos por
    el servidor de arnes.py sobre el robot del gemelo: (tirada, robot, cuerpo, servidor, eventos)."""
    import argparse
    import manipulacion as mp
    cfg = config.cargar()
    a = argparse.Namespace(nivel=2, simulacro=False, sim=True, escala_tiempo=1.0, dianas=None, asignacion=None,
                           colocacion=None, visor=False, velocidad=1.0, arnes=True, recuperar=recuperar, parada_en=None,
                           sin_comprobar_montaje=True, salida_directa=False)
    a.colocacion = {k: v for k, v in mp.COLOCACION_SIM.items()}
    a.asignacion = dict(mp.ASIGNACION_SIM)
    t = mp.armar(a, cfg)
    t.brazo.parar()                                        # el brazo simulado de arm_sdk, fuera
    robot, cu, srv, eventos = montar(robot=t.robot)
    b = BrazoArnes(robot, cfg, escala_tiempo=1.0, puerto=srv.puerto, recuperar=recuperar)
    t.brazo = t.sup.brazo = t.vig.brazo = b
    return t, robot, cu, srv, eventos


def test_tirada_de_nivel_2_con_arnes():
    """La tirada de --sim --arnes (4/4, RESULTADOS §26) con los brazos movidos por el servidor de arnes.py: el
    supervisor, el planificador, la percepcion y las manos del gemelo, y un Cuerpo de verdad."""
    import manipulacion as mp
    t, robot, cu, srv, eventos = tirada_con_arnes()
    try:
        motivo = mp.correr(t, confirmar=False)
    finally:
        srv.parar()
        cu._vivo = False
    assert motivo == "completado", (motivo, eventos)
    assert [r["resultado"] for r in t.sup.resultados] == ["colocado"] * 4
    assert srv.controlado is None and srv.movimientos > 20
    assert np.allclose(robot.q()[13:27], 0.0, atol=1e-3)   # acaba con los brazos a 0, como empezo


def test_parada_de_arnes_en_el_transporte():
    """ESPACIO en arnes.py con el cilindro en la mano: los brazos se congelan, el supervisor hace su salida segura
    (abrir, subir, por la observacion a 0) con movimientos de salida, y suelta."""
    import manipulacion as mp
    t, robot, cu, srv, eventos = tirada_con_arnes()
    t.sup.al_estado.append(lambda e: e == "TRANSPORTE" and srv.parada is None and srv.parar_remoto("ESPACIO"))
    try:
        motivo = mp.correr(t, confirmar=False)
    finally:
        srv.parar()
        cu._vivo = False
    assert motivo == "salida segura: parada en arnes.py: ESPACIO"
    assert srv.controlado is None
    assert np.allclose(robot.q()[13:27], 0.0, atol=1e-3)   # salio por la observacion hasta los brazos a 0
    import os
    texto = open(os.path.join(t.reg.carpeta, "eventos.csv")).read()
    assert "observacion -> reposo" in texto and "FIN" in texto


def test_recuperar_unos_brazos_que_se_quedaron_sobre_la_mesa():
    """manipulacion.py muere con un brazo sobre la mesa: arnes.py lo congela y le quita el control; con --recuperar
    se toma desde ahi y la salida segura lo trae a 0."""
    import manipulacion as mp
    t, robot, cu, srv, eventos = tirada_con_arnes(recuperar=True)
    # el brazo izquierdo sobre la zona de recogida, como a medio agarre
    plan = t.sup.pl.entre_reposo_y_observacion("izq", True)
    q = cu.consigna()
    q[13:20] = plan.puntos_de_paso[-1].q
    cu.mover(q, juntas=BRAZOS)
    assert np.abs(robot.q()[13:20]).max() > 0.3
    try:
        motivo = mp.correr(t, confirmar=False)
    finally:
        srv.parar()
        cu._vivo = False
    assert motivo == "recuperados", (motivo, eventos)
    assert srv.controlado is None and any("para recuperarlos" in e for e in eventos)
    assert np.allclose(robot.q()[13:27], 0.0, atol=1e-3)


def test_detener_deja_la_consigna_donde_iba():
    """El toque del nivel 1: el brazo va por encima de su consigna (aqui, un robot que se queda corto); al notar la mesa
    se detiene donde va la consigna (congelar la llevaria a lo medido y la yema subiria)."""
    robot = RobotSimulado(postura=[float(v) for v in postura_arnes(CFG)], caida_rad=0.05, tau_s=0.005, instantaneo=False)
    robot, cu, srv, _ = montar(robot=robot)
    b = cliente(srv)
    b.rampa_peso(1.0)
    parar = {"tras": 0.08}                                                 # el movimiento dura ~0.2 s reales (x40)
    t0 = time.monotonic()

    def a_mitad():
        if time.monotonic() - t0 > parar["tras"]:
            raise Parada("mesa")
    with pytest.raises(Parada):
        b.ir_a(b.pose_brazo("izq", ADELANTE_IZQ), comprobar=a_mitad, al_parar="detener")
    c = cu.consigna()[13:20]
    assert not cu.moviendo()
    assert 0.0 < abs(c[3]) < abs(ADELANTE_IZQ[3])                            # se quedo a medio camino
    assert np.abs(c - robot.q()[13:20]).max() > 0.01                          # y no en lo medido
    time.sleep(0.1)
    assert np.allclose(cu.consigna()[13:20], c)                               # quieta
    b.congelar()
    assert np.abs(cu.consigna()[13:20] - c).max() > 0.01                      # congelar: a lo medido (otro sitio)
    b.ir_a(b.pose_brazo("izq", [0.0] * 7))
    b.rampa_peso(0.0)
    cerrar(robot, cu, srv, b)


def test_fijar_la_consigna_con_saltos_pequenos(montaje):
    """El toque del nivel 1: la consigna pasa a lo medido mas el error estatico (un salto de milimetros); un salto
    grande no se acepta (seria un movimiento sin trayectoria)."""
    robot, cu, srv, _ = montaje
    b = cliente(srv)
    b.rampa_peso(1.0)
    b.ir_a(b.pose_brazo("izq", ADELANTE_IZQ))
    q = np.array([b.recortar(j, v + 0.02) for j, v in zip(range(13, 20), ADELANTE_IZQ)])   # dentro de los limites
    assert b.fijar(b.pose_brazo("izq", q))
    assert np.allclose(cu.consigna()[13:20], q, atol=1e-6) and not cu.moviendo()
    assert np.allclose(cu.consigna()[20:27], 0.0)                          # el otro brazo, donde estaba
    assert not b.fijar(b.pose_brazo("izq", q + 0.2))                       # 0.2 rad de golpe: no
    assert np.allclose(cu.consigna()[13:20], q, atol=1e-6)
    b.ir_a(b.pose_brazo("izq", [0.0] * 7))
    b.rampa_peso(0.0)
    cerrar(robot, cu, srv, b)
