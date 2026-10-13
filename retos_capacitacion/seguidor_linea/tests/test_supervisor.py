"""Supervisor: transiciones, quien llama a Move/StopMove (nunca en simulacro) y paradas; y el lazo
cerrado simulado (seguidor/simulador.py) en los cuatro niveles."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config  # noqa: E402
from seguidor.control import Control  # noqa: E402
from seguidor.mensajes import EstadoLinea, Imu, Mando  # noqa: E402
from seguidor.simulador import simular  # noqa: E402
from seguidor.supervisor import (ESPERA, FIN, LINEA_PERDIDA, PARADA, SEGUIMIENTO,  # noqa: E402
                                 Supervisor)
from seguidor.vigilante import Vigilante  # noqa: E402

CFG = config.cargar()


class RobotFalso:
    def __init__(self):
        self.moves, self.stops = [], 0
        self.llamadas = []
        self.pitch = 0.0
        self.marcha_habilitada = True

    def mover(self, vx, vy, vyaw):
        self.moves.append((vx, vy, vyaw))
        self.llamadas.append("Move")

    def parar(self):
        self.stops += 1
        self.llamadas.append("StopMove")

    def edad_lowstate(self):
        return 0.002

    def motores_en_fallo(self):
        return []

    def imu(self):
        return Imu(t=0.0, tick=0, roll=0.0, pitch=self.pitch, yaw=0.0, gx=0.0, gy=0.0, gz=0.0)

    def mando(self):
        return Mando(botones=0, lx=0.0, ly=0.0, rx=0.0, ry=0.0)


class EstimadorFalso:
    """Devuelve el EstadoLinea que diga la prueba; anota las vx que le pasa el supervisor."""
    x_puntera, d_giro = -0.02, 0.14

    def __init__(self):
        self.edad, self.racha, self.conf, self.dist_fin, self.v = 0.0, 10, 0.9, None, 0.2
        self.vx = []

    def estado(self, t, L):
        return EstadoLinea(t=t, y=0.0, theta=0.0, kappa=0.0, confianza=self.conf, edad_s=self.edad, rumbo_ref=0.0,
                           dist_fin=self.dist_fin, objetivo=(0.5, 0.0), v=self.v, racha=self.racha, hay_linea=True)

    def orden(self, t, vx):
        self.vx.append(vx)

    def reiniciar(self):
        pass


def montar(simulacro=False):
    robot, est = RobotFalso(), EstimadorFalso()
    sup = Supervisor(CFG, robot, Vigilante(robot, CFG), Control(CFG, 0.5), est, nivel=1, simulacro=simulacro)
    sup.empezar(0.0)
    return sup, robot, est


def correr(sup, desde, hasta, paso=0.05):
    t = desde
    while t < hasta - 1e-9 and not sup.terminado:
        sup.paso(t, 0.03)
        t += paso
    return t


class TestSupervisor(unittest.TestCase):
    def test_espera_y_seguimiento(self):
        sup, robot, est = montar()
        est.racha = 2
        correr(sup, 0.0, 0.5)
        self.assertEqual(sup.estado, ESPERA)
        self.assertEqual(robot.moves, [])
        est.racha = 10
        correr(sup, 0.5, 1.5)
        self.assertEqual(sup.estado, SEGUIMIENTO)
        self.assertGreater(len(robot.moves), 10)
        self.assertGreater(robot.moves[-1][0], 0.0)

    def test_sin_linea_al_empezar(self):
        sup, robot, est = montar()
        est.racha = 0
        correr(sup, 0.0, CFG["supervisor"]["t_inicio_max_s"] + 1.0)
        self.assertEqual(sup.estado, PARADA)
        self.assertEqual(robot.moves, [])

    def test_perdida_recuperada_y_parada(self):
        sup, robot, est = montar()
        correr(sup, 0.0, 1.0)
        est.edad = 0.5
        correr(sup, 1.0, 1.2)
        self.assertEqual(sup.estado, LINEA_PERDIDA)
        n = len(robot.moves)
        est.edad = 0.0
        correr(sup, 1.2, 1.4)
        self.assertEqual(sup.estado, SEGUIMIENTO)
        self.assertGreater(len(robot.moves), n)      # en LINEA_PERDIDA sigue andando
        est.edad = CFG["supervisor"]["t_perdida_s"] + 0.1
        correr(sup, 1.4, 2.0)
        self.assertEqual(sup.estado, PARADA)
        self.assertIn("perdida", sup.motivo)
        self.assertEqual(robot.llamadas[-1], "StopMove")

    def test_final_a_ciegas_no_es_perdida(self):
        sup, robot, est = montar()
        correr(sup, 0.0, 1.0)
        est.edad, est.dist_fin, est.v = 1.0, 0.4, 0.05
        correr(sup, 1.0, 1.5)
        self.assertEqual(sup.estado, SEGUIMIENTO)

    def test_final_a_ciegas_con_tope(self):
        sup, robot, est = montar()
        correr(sup, 0.0, 1.0)
        est.dist_fin, est.v = 0.4, 0.05
        est.edad = CFG["supervisor"]["t_final_max_s"] + 0.1
        correr(sup, 1.0, 1.2)
        self.assertEqual(sup.estado, PARADA)
        self.assertIn("a ciegas", sup.motivo)

    def test_fin_en_la_barra(self):
        sup, robot, est = montar()
        correr(sup, 0.0, 1.0)
        s = CFG["supervisor"]
        est.dist_fin = est.v * s["t_frenado_s"] - s["fin_objetivo_m"] + 0.05
        correr(sup, 1.0, 1.2)
        self.assertEqual(sup.estado, SEGUIMIENTO)
        est.dist_fin -= 0.06
        correr(sup, 1.2, 1.4)
        self.assertEqual(sup.estado, FIN)
        self.assertEqual(robot.llamadas[-1], "StopMove")
        n = len(robot.moves)
        correr(sup, 1.4, 2.0)
        self.assertEqual(len(robot.moves), n)        # despues de FIN no se manda nada

    def test_vigilante_para(self):
        sup, robot, est = montar()
        correr(sup, 0.0, 1.0)
        robot.pitch = math.radians(25)
        sup.paso(1.0, 0.03)
        self.assertEqual(sup.estado, PARADA)
        self.assertIn("inclinacion", sup.motivo)

    def test_aviso_externo_y_detener(self):
        sup, robot, est = montar()
        correr(sup, 0.0, 1.0)
        sup.aviso_externo = "FSM 200"
        sup.paso(1.0, 0.03)
        self.assertEqual((sup.estado, sup.motivo), (PARADA, "FSM 200"))
        sup, robot, est = montar()
        correr(sup, 0.0, 1.0)
        sup.detener(1.0, "Ctrl+C")
        self.assertEqual(sup.estado, PARADA)
        self.assertEqual(robot.llamadas[-1], "StopMove")
        sup.detener(1.1, "otra vez")
        self.assertEqual(robot.stops, 1)             # StopMove una sola vez

    def test_simulacro_no_mueve(self):
        sup, robot, est = montar(simulacro=True)
        correr(sup, 0.0, 2.0)
        self.assertEqual(sup.estado, SEGUIMIENTO)
        est.edad = 10.0
        correr(sup, 2.0, 3.0)
        sup.detener(3.0, "fin")
        self.assertEqual(robot.moves, [])
        self.assertEqual(robot.stops, 0)
        self.assertGreater(max(est.vx), 0.0)         # pero si calcula la orden que mandaria


class TestLazoSimulado(unittest.TestCase):
    """Tiradas simuladas a media escala: llegar a la barra (0-30 cm mas alla) con los pies a menos de
    20 cm de la linea, saliendo girado 10 grados a cada lado."""

    def comprobar(self, nivel, sentido=+1, **kw):
        for des in (10.0, -10.0):
            r = simular(CFG, nivel, 0.5, des, 0.0, sentido, **kw)
            self.assertTrue(r["exito"], f"nivel {nivel} sentido {sentido} salida {des}: {r['estado_final']} "
                            f"{r['motivo']}, error max {r['error_max_m']:.2f}, pasada {r['pasada_m']:.2f}")

    def test_nivel_1(self):
        self.comprobar(1)

    def test_nivel_2(self):
        self.comprobar(2, +1)
        self.comprobar(2, -1)

    def test_nivel_3(self):
        self.comprobar(3, +1)
        self.comprobar(3, -1)

    def test_nivel_4(self):
        self.comprobar(4, +1)
        self.comprobar(4, -1)

    def test_nivel_1_con_la_marcha_peor(self):
        self.comprobar(1, marcha={"retardo_s": 0.30, "tau_giro_s": 0.45, "deriva": 0.052, "ganancia_giro": 0.8,
                                  "direccion_avance_deg": 11.0, "balanceo_yaw_deg": 2.8})

    def test_camara_tapada_para(self):
        r = simular(CFG, 1, 0.5, 10.0, tapar=[(5.0, 30.0)])
        self.assertEqual(r["estado_final"], PARADA)
        self.assertIn("perdida", r["motivo"])
        t_parada = [t for t, _, a, _ in r["transiciones"] if a == PARADA][0]
        self.assertAlmostEqual(t_parada, 5.0 + CFG["supervisor"]["t_perdida_s"], delta=0.2)


if __name__ == "__main__":
    unittest.main()
