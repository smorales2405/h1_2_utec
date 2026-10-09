"""Estimacion (historial de la IMU, linea en el marco del suelo, estima de avance, barra) y control
(signos, saturacion, escala, confianza, rampa)."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config  # noqa: E402
from seguidor.control import Control  # noqa: E402
from seguidor.estimacion import Estimador, HistorialImu  # noqa: E402
from seguidor.mensajes import EstadoLinea, MedidaLinea  # noqa: E402

CFG = config.cargar()
X_PUNTERA = -0.02


def medida(t, y=0.0, theta_deg=0.0, kappa=0.0, conf=0.9, alcance=(0.2, 2.5), barra=None, esquina=None):
    th = math.radians(theta_deg)
    return MedidaLinea(t=t, y=y, theta=th, kappa=kappa, objetivo=(1.0, y + math.tan(th)), confianza=conf,
                       n_franjas=4, barra_fin=barra, esquina=esquina, alcance=alcance)


def con_imu(yaw_de_t=lambda t: 0.0, hasta=20.0):
    imu = HistorialImu()
    t = 0.0
    while t <= hasta:
        imu.agregar(t, 0.01, 0.02, yaw_de_t(t))
        t += 0.002
    return imu


class TestHistorialImu(unittest.TestCase):
    def test_desenrolla_e_interpola(self):
        imu = HistorialImu()
        for i in range(100):     # de 170 a 190 grados: pasa por +-180
            yaw = math.radians(170 + 0.2 * i)
            imu.agregar(0.01 * i, 0.0, 0.0, math.atan2(math.sin(yaw), math.cos(yaw)))
        _, _, yaw = imu.en(0.505)
        self.assertAlmostEqual(math.degrees(yaw), 180.1, places=3)

    def test_media_y_capacidad(self):
        imu = HistorialImu(capacidad=1000)
        for i in range(5000):
            imu.agregar(0.002 * i, 0.1 if i % 2 else -0.1, 0.05, 0.0)
        roll, pitch = imu.media(8.0, 9.5)
        self.assertAlmostEqual(roll, 0.0, places=2)
        self.assertAlmostEqual(pitch, 0.05)
        self.assertLessEqual(len(imu), 1000)


class TestEstimador(unittest.TestCase):
    def test_quieto_y_objetivo(self):
        est = Estimador(CFG, con_imu(), X_PUNTERA)
        est.orden(0.0, 0.0)
        self.assertTrue(est.medida(medida(1.0, y=0.10)))
        e = est.estado(1.05, 0.6)
        self.assertAlmostEqual(e.y, 0.10, delta=0.002)
        self.assertAlmostEqual(e.theta, 0.0, delta=1e-3)
        # objetivo a 0.6 m del centro de giro (14 cm por detras de la camara), sobre la linea
        ox, oy = e.objetivo
        self.assertAlmostEqual(oy, 0.10, delta=0.002)
        self.assertAlmostEqual(math.hypot(ox + 0.14, oy), 0.6, delta=0.002)
        self.assertLess(e.edad_s, 0.1)

    def test_el_yaw_gira_la_linea(self):
        # el robot gira 10 grados a la izquierda sin medir: la linea se ve 10 grados a la derecha
        est = Estimador(CFG, con_imu(lambda t: 0.0 if t < 2.0 else math.radians(10.0)), X_PUNTERA)
        est.orden(0.0, 0.0)
        est.medida(medida(1.0, y=0.0, theta_deg=0.0))
        e = est.estado(3.0, 0.6)
        self.assertAlmostEqual(math.degrees(e.theta), -10.0, delta=0.1)
        self.assertAlmostEqual(math.degrees(e.rumbo_ref), 0.0, delta=0.1)

    def test_avance_a_ciegas(self):
        # vx 0.2 durante 4 s con rumbo fijo: avanza ~1.14 * 0.2 m/s y 7.5 grados a la izquierda
        est = Estimador(CFG, con_imu(), X_PUNTERA)
        est.orden(0.0, 0.0)
        est.medida(medida(0.5, y=0.0, alcance=(0.2, 2.5), barra=2.0))
        for k in range(4):
            est.medida(medida(0.6 + 0.03 * k, y=0.0, alcance=(0.2, 2.0), barra=2.0))
        e0 = est.estado(0.8, 0.6)
        self.assertAlmostEqual(e0.dist_fin, 2.0 - X_PUNTERA, delta=0.01)
        est.orden(1.0, 0.2)
        e = est.estado(5.0, 0.6)
        avance = 2.0 - X_PUNTERA - e.dist_fin
        esperado = 1.14 * 0.2 * (4.0 - 0.5 * (1 - math.exp(-4.0 / 0.5))) * math.cos(math.radians(7.5))
        self.assertAlmostEqual(avance, esperado, delta=0.02)
        # se ha ido a la izquierda: la linea queda a la derecha
        self.assertAlmostEqual(e.y, -esperado * math.tan(math.radians(7.5)), delta=0.01)
        self.assertGreater(e.edad_s, 4.0)

    def test_final_con_la_velocidad_medida_en_la_barra(self):
        # el robot se acerca a 0.15 m/s aunque se mande vx 0.4 (simulacro, o factor equivocado)
        est = Estimador(CFG, con_imu(), X_PUNTERA)
        est.orden(0.0, 0.4)
        t = 1.0
        while t < 2.5:
            xb = 0.6 - 0.15 * (t - 1.0)
            est.medida(medida(t, alcance=(0.2, xb), barra=xb))
            t += 1 / 30
        self.assertAlmostEqual(est.v_barra(), 0.15, delta=0.01)
        e = est.estado(3.5, 0.6)                  # 1 s sin ver la barra
        self.assertAlmostEqual(e.dist_fin, 0.6 - 0.15 * 2.5 - X_PUNTERA, delta=0.01)
        self.assertAlmostEqual(e.v, 0.15, delta=0.01)

    def test_barra_falsa_se_olvida(self):
        est = Estimador(CFG, con_imu(), X_PUNTERA)
        est.orden(0.0, 0.0)
        for k in range(3):
            est.medida(medida(1.0 + 0.03 * k, alcance=(0.2, 1.5), barra=1.5))
        self.assertIsNotNone(est.barra)
        self.assertIsNone(est.estado(1.2, 0.6).dist_fin)   # no confirmada todavia
        est.medida(medida(1.2, alcance=(0.2, 2.5)))         # la linea sigue mas alla: no era la barra
        self.assertIsNone(est.barra)

    def test_puerta_rechaza_un_salto(self):
        est = Estimador(CFG, con_imu(), X_PUNTERA)
        est.orden(0.0, 0.0)
        for k in range(3):
            est.medida(medida(0.9 + 0.03 * k, y=0.0))
        self.assertFalse(est.medida(medida(1.1, y=0.8, alcance=(0.3, 0.9))))     # canto de un mueble
        self.assertFalse(est.medida(medida(1.2, y=0.0, theta_deg=40.0)))
        self.assertTrue(est.medida(medida(1.3, y=0.05, theta_deg=3.0)))
        self.assertEqual(est.rechazadas, 2)
        # sin medida buena mas de puerta_edad_s, se readquiere otra linea, pero con 3 medidas coherentes
        t = 1.3 + CFG["estimacion"]["puerta_edad_s"] + 0.1
        self.assertFalse(est.medida(medida(t, y=0.5)))
        self.assertFalse(est.medida(medida(t + 0.03, y=0.5)))
        self.assertTrue(est.medida(medida(t + 0.06, y=0.5)))

    def test_falsos_sueltos_no_refrescan_la_edad(self):
        # camara tapada: la linea se pierde y llegan falsos sueltos en sitios distintos
        est = Estimador(CFG, con_imu(), X_PUNTERA)
        est.orden(0.0, 0.0)
        for k in range(5):
            est.medida(medida(1.0 + 0.03 * k, y=0.0))
        for k, y in enumerate((0.9, -0.9, 0.9, 0.4, -0.6, 0.9)):
            self.assertFalse(est.medida(medida(3.0 + 0.5 * k, y=y, alcance=(0.3, 0.8))))
        self.assertGreater(est.estado(6.0, 0.6).edad_s, 4.5)

    def test_un_falso_al_empezar_no_bloquea(self):
        est = Estimador(CFG, con_imu(), X_PUNTERA)
        est.orden(0.0, 0.0)
        est.medida(medida(1.0, y=0.77, alcance=(0.3, 0.9)))   # falso en el primer fotograma
        self.assertFalse(est.medida(medida(1.03, y=0.03)))     # la buena empieza una candidata...
        self.assertFalse(est.medida(medida(1.06, y=0.03)))
        self.assertTrue(est.medida(medida(1.09, y=0.03)))      # ...que se adopta a la tercera
        for k in range(3):
            self.assertTrue(est.medida(medida(1.12 + 0.03 * k, y=0.03)))
        self.assertFalse(est.medida(medida(1.3, y=0.77, alcance=(0.3, 0.9))))

    def test_medida_mala_no_cuenta(self):
        est = Estimador(CFG, con_imu(), X_PUNTERA)
        est.orden(0.0, 0.0)
        est.medida(medida(1.0, y=0.05))
        self.assertFalse(est.medida(medida(1.5, y=0.30, conf=0.1)))
        e = est.estado(1.6, 0.6)
        self.assertAlmostEqual(e.y, 0.05, delta=0.002)
        self.assertAlmostEqual(e.edad_s, 0.6, delta=1e-6)
        self.assertEqual(e.racha, 0)


def estado(objetivo, conf=0.9, kappa=0.0, dist_fin=None):
    return EstadoLinea(t=0.0, y=objetivo[1], theta=0.0, kappa=kappa, confianza=conf, edad_s=0.0, rumbo_ref=0.0,
                       dist_fin=dist_fin, objetivo=objetivo, hay_linea=True)


def en_regimen(ctl, est, n=60):
    for k in range(n):
        orden, info = ctl.calcular(0.05 * k, est)
    return orden, info


class TestControl(unittest.TestCase):
    def setUp(self):
        self.d = CFG["estimacion"]["centro_giro_m"]
        self.phi = math.radians(CFG["estimacion"]["direccion_avance_deg"])

    def objetivo(self, angulo_deg, L=0.6):
        # punto a distancia L del centro de giro, a `angulo_deg` de la direccion de avance
        a = self.phi + math.radians(angulo_deg)
        return (L * math.cos(a) - self.d, L * math.sin(a))

    def test_recto_por_la_direccion_de_avance(self):
        orden, info = en_regimen(Control(CFG, 1.0), estado(self.objetivo(0.0)))
        self.assertAlmostEqual(info["alfa"], 0.0, places=6)
        self.assertAlmostEqual(orden.vyaw, CFG["control"]["vyaw_sesgo"], places=6)
        self.assertAlmostEqual(orden.vx, min(CFG["control"]["vx"], 0.4), places=6)

    def test_linea_justo_delante_de_la_camara_gira_a_la_derecha(self):
        # el robot avanza 7.5 grados a la izquierda del eje: para ir por el eje tiene que girar a la derecha
        orden, _ = en_regimen(Control(CFG, 1.0), estado((0.6, 0.0)))
        self.assertLess(orden.vyaw, CFG["control"]["vyaw_sesgo"])

    def test_signos(self):
        izq, _ = en_regimen(Control(CFG, 1.0), estado(self.objetivo(+15.0)))
        der, _ = en_regimen(Control(CFG, 1.0), estado(self.objetivo(-15.0)))
        sesgo = CFG["control"]["vyaw_sesgo"]
        self.assertGreater(izq.vyaw, sesgo)
        self.assertLess(der.vyaw, sesgo)
        self.assertAlmostEqual(izq.vyaw - sesgo, -(der.vyaw - sesgo), places=6)

    def test_saturacion_y_escala(self):
        for escala in (1.0, 0.5):
            orden, info = en_regimen(Control(CFG, escala), estado(self.objetivo(80.0)))
            self.assertLessEqual(abs(orden.vyaw), 0.5 * escala + 1e-9)
            self.assertLessEqual(orden.vx, 0.4 * escala + 1e-9)
            self.assertGreaterEqual(orden.vx, 0.0)
        orden, _ = en_regimen(Control(CFG, 0.5), estado(self.objetivo(0.0)))
        self.assertAlmostEqual(orden.vx, 0.5 * min(CFG["control"]["vx"], 0.4), places=6)

    def test_confianza_baja_para(self):
        orden, _ = en_regimen(Control(CFG, 1.0), estado(self.objetivo(0.0), conf=0.2))
        self.assertEqual(orden.vx, 0.0)
        self.assertEqual(orden.vyaw, 0.0)

    def test_rampa(self):
        ctl = Control(CFG, 1.0)
        orden, _ = ctl.calcular(0.0, estado(self.objetivo(0.0)))
        orden, _ = ctl.calcular(0.05, estado(self.objetivo(0.0)))
        self.assertLessEqual(orden.vx, 2 * CFG["control"]["dvx_max"] * 0.05 + 1e-9)

    def test_mas_despacio_cerca_de_la_barra_y_en_curva(self):
        lejos, _ = en_regimen(Control(CFG, 1.0), estado(self.objetivo(0.0), dist_fin=3.0))
        cerca, _ = en_regimen(Control(CFG, 1.0), estado(self.objetivo(0.0), dist_fin=0.5))
        curva, _ = en_regimen(Control(CFG, 1.0), estado(self.objetivo(20.0)))
        self.assertLess(cerca.vx, lejos.vx)
        self.assertLess(curva.vx, lejos.vx)

    def test_la_confianza_no_frena_junto_a_la_barra(self):
        lejos, _ = en_regimen(Control(CFG, 1.0), estado(self.objetivo(0.0), conf=0.4, dist_fin=3.0))
        cerca, _ = en_regimen(Control(CFG, 1.0), estado(self.objetivo(0.0), conf=0.4, dist_fin=0.3))
        lleno, _ = en_regimen(Control(CFG, 1.0), estado(self.objetivo(0.0), conf=0.9, dist_fin=0.3))
        self.assertLess(lejos.vx, lleno.vx / CFG["control"]["vx_cerca_barra"])
        self.assertAlmostEqual(cerca.vx, lleno.vx, places=6)

    def test_girar_primero_en_el_sitio(self):
        import copy
        cfg = copy.deepcopy(CFG)
        cfg["control"]["girar_primero_deg"] = 10.0
        ctl = Control(cfg, 0.5)
        orden, info = ctl.calcular(0.0, estado(self.objetivo(-25.0)))
        self.assertEqual(orden.vx, 0.0)
        self.assertLess(orden.vyaw, 0.0)
        self.assertTrue(info.get("girando"))
        orden, info = ctl.calcular(0.05, estado(self.objetivo(-3.0)))     # ya alineado: anda
        self.assertFalse(info.get("girando", False))
        orden, info = ctl.calcular(0.10, estado(self.objetivo(-25.0)))    # solo al empezar
        self.assertFalse(info.get("girando", False))
        self.assertGreater(orden.vx, 0.0)

    def test_velocidad_minima(self):
        import copy
        cfg = copy.deepcopy(CFG)
        cfg["estimacion"]["v_minima"] = 0.18
        ctl = Control(cfg, 0.5)
        self.assertAlmostEqual(ctl.v_real(0.05), 0.18)
        self.assertAlmostEqual(ctl.v_real(0.3), 0.3 * cfg["estimacion"]["factor_velocidad"])
        self.assertEqual(ctl.v_real(0.0), 0.0)

    def test_sin_linea_no_anda(self):
        ctl = Control(CFG, 1.0)
        e = EstadoLinea(t=0.0, y=math.nan, theta=math.nan, kappa=math.nan, confianza=0.0, edad_s=math.inf,
                        rumbo_ref=math.nan)
        orden, _ = ctl.calcular(0.0, e)
        self.assertEqual((orden.vx, orden.vyaw), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
