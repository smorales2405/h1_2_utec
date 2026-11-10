"""herramientas/curva_pasos.py y analizar_pasos.py con arnes.py de verdad (Cuerpo sin publicar sobre el robot simulado,
reloj x40, el banco de test_brazo_arnes.py): el recorrido, lo que graba, la parada por la IMU y el analisis."""

import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "herramientas"))
import analizar_pasos as ap  # noqa: E402
import curva_pasos as cp  # noqa: E402
from test_brazo_arnes import CFG, VEL, Reloj, cerrar, montar  # noqa: E402

from manip.brazo_arnes import BrazoArnes  # noqa: E402
from manip.cuerpo import BRAZOS  # noqa: E402

MOVS = {k: v for k, v in cp.movimientos_definidos().items() if k in ("observacion", "alcance_izq")}


def curva(srv, carpeta, **kw):
    medido = cp._Medido()
    b = BrazoArnes(medido, CFG, escala_tiempo=1.0, puerto=srv.puerto)
    reloj = Reloj()
    c = cp.CurvaPasos(b, MOVS, kw.pop("duraciones", [8.0]), str(carpeta), quieto_s=kw.pop("quieto_s", 3.0), hz=4.0,
                      reloj=reloj, dormir=lambda s: time.sleep(s / VEL), log=lambda *x: None, **kw)
    medido.curva = c
    return b, c


def test_recorrido_completo_y_analisis(tmp_path):
    robot, cu, srv, _ = montar()
    b, c = curva(srv, tmp_path)
    motivo = c.correr()
    b.parar()
    assert motivo == "completado", motivo
    assert c.hechos == [("observacion", 8.0, 1), ("alcance_izq", 8.0, 1)]
    assert srv.controlado is None and srv.movimientos == 2 * sum(len(m) for m in MOVS.values())
    assert np.allclose(robot.q()[BRAZOS], 0.0, atol=1e-3)
    et, num = ap.cargar(str(tmp_path))
    for mov in MOVS:
        vistos = {tr for m, tr in zip(et["mov"], et["tramo"]) if m == mov}
        assert set(cp.TRAMOS) <= vistos, (mov, vistos)
    from manip.dinamica import Dinamica
    filas = ap.analizar(str(tmp_path), Dinamica())
    por = {(f["mov"], f["sentido"]): f for f in filas}
    assert set(por) == {(m, s) for m in MOVS for s in ("ida", "vuelta")}
    obs, alc = por[("observacion", "ida")], por[("alcance_izq", "ida")]
    assert abs(obs["dq"] - np.abs(cp.pose14(MOVS["observacion"][-1])).max()) < 0.02
    assert obs["dcom_x_cm"] < -2.0 and abs(alc["dcom_x_cm"]) < 1.0     # el c. de masas: atras / casi quieto
    assert abs(por[("observacion", "vuelta")]["dcom_x_cm"] + obs["dcom_x_cm"]) < 0.2
    assert obs["vq"] <= 0.5 + 1e-3                                     # el tope de la seccion 10
    assert abs(obs["pitch_pico"]) < 1e-6                               # el robot simulado no se inclina
    cerrar(robot, cu, srv, b)


def test_la_inclinacion_para_y_vuelve_a_cero(tmp_path):
    robot, cu, srv, _ = montar()
    b, c = curva(srv, tmp_path, max_incl_deg=2.0)

    def empujar():
        while c.etiqueta["tramo"] != "en_pose":
            time.sleep(0.005)
        robot.pitch = np.radians(4.0)                                    # el cuerpo se va 4 grados adelante
    threading.Thread(target=empujar, daemon=True).start()
    motivo = c.correr()
    b.parar()
    assert motivo.startswith("parada: inclinacion"), motivo
    assert c.hechos == []
    assert np.allclose(robot.q()[BRAZOS], 0.0, atol=1e-3)               # de vuelta a 0 por la misma recta
    assert srv.controlado is None
    texto = (tmp_path / "eventos.csv").read_text()
    assert "PARADA (parada: inclinacion" in texto and "brazos a 0" in texto
    cerrar(robot, cu, srv, b)


def test_caminos_en_el_gemelo():
    movs = cp.movimientos_definidos()
    malos = cp.caminos_con_choques(CFG, cp.config.cargar_montaje(), {"alcance_izq": movs["alcance_izq"],
                                                                     "alcance_der": movs["alcance_der"]})
    assert malos == {}
    # el antebrazo subiendo desde 0 barre la bandeja azul (A, delante a la izquierda)
    malos = cp.caminos_con_choques(CFG, cp.config.cargar_montaje(), {"codo": [{"izq": [0, 0, 0, 1.5, 0, 0, 0]}]})
    assert "codo" in malos and "bandeja_A" in " ".join(malos["codo"])


def test_explicacion_y_duracion_minima():
    filas = []
    for T in (30.0, 20.0, 10.0, 8.0):
        for mov, dcom in (("a", 4.0), ("b", 0.3)):
            acom = dcom * 5.77 / T ** 2
            filas.append({"mov": mov, "T": T, "sentido": "ida", "dcom_cm": dcom, "acom_cms2": acom, "vcom_cms": dcom / T,
                          "vq": 1.0 / T, "aq": 5.0 / T ** 2, "pitch_pico": 0.1 * dcom + 2.0 * acom,
                          "pitch_queda": 0.1 * dcom, "pitch_exceso": 2.0 * acom, "roll_pico": 0.0, "roll_exceso": 0.0,
                          "yaw_pico": 0.01 * dcom})
    exp = ap.explicacion(filas)
    assert exp["pitch_exceso"][0][0] == "acom_cms2" and exp["pitch_queda"][0][0] == "dcom_cm"
    # exceso 2 * 4 * 5.77 / T^2 grados: 0.2 a 15.2 s
    assert ap.duracion_minima(filas, 0.2, 0.5) == 20.0
    assert ap.duracion_minima(filas, 0.5, 0.5) == 10.0


def test_parada_en_el_segundo_tramo_deshace_el_camino(tmp_path):
    """Un movimiento con punto de paso: parado a mitad del segundo tramo, vuelve al punto de paso por la misma
    recta y de ahi a 0 (no en recta a 0, que podria no estar limpia)."""
    robot, cu, srv, _ = montar()
    medido = cp._Medido()
    b = BrazoArnes(medido, CFG, escala_tiempo=1.0, puerto=srv.puerto)
    mitad = list(np.array(cp.ALCANCE_IZQ) * 0.5)
    c = cp.CurvaPasos(b, {"dos": [{"izq": mitad}, {"izq": cp.ALCANCE_IZQ}]}, [8.0], str(tmp_path), quieto_s=3.0,
                      hz=4.0, reloj=Reloj(), dormir=lambda s: time.sleep(s / VEL), log=lambda *x: None)
    medido.curva = c
    vistos = []
    real_ir_a = b.ir_a

    def ir_a(pose, segundos=None, comprobar=None):
        vistos.append(np.array([pose[j] for j in range(13, 20)]))
        if len(vistos) == 2:                                   # el segundo tramo: parar a mitad
            threading.Timer(4.0 / VEL, lambda: c.pedir_parada("prueba")).start()
        return real_ir_a(pose, segundos, comprobar)
    b.ir_a = ir_a
    motivo = c.correr()
    b.parar()
    assert motivo == "parada: prueba", motivo
    assert len(vistos) == 4
    assert np.allclose(vistos[2], mitad) and np.allclose(vistos[3], 0.0)      # al punto de paso y a 0
    assert np.allclose(robot.q()[BRAZOS], 0.0, atol=1e-3)
    cerrar(robot, cu, srv, b)
