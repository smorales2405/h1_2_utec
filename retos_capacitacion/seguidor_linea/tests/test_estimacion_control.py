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
CFG["control"]["girar_primero_deg"] = 0.0     # la ley de control, sin el giro previo ni la pausa para girar
CFG["control"]["pausa_giro_deg"] = 0.0        # (cada uno tiene su propia prueba)
CFG["control"]["pausa_perdida_deg"] = 0.0
CFG["control"]["pausa_perdida_rumbo_deg"] = 0.0
CFG["control"]["vx_tras_pausa"] = 0.0
CFG["control"]["vyaw_busqueda"] = 0.0
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

    def test_linea_corta_se_prolonga_hacia_delante(self):
        # alcance multiplo del paso (0.22-0.52 con 0.05): el ultimo punto salia repetido 1e-16 por detras y la
        # prolongacion iba hacia atras; andando a ciegas el objetivo acababa detras (2026-11-07)
        for x0, x1 in ((0.22, 0.52), (0.26, 0.61), (0.22, 0.57), (0.2, 0.2005)):
            est = Estimador(CFG, con_imu(), X_PUNTERA)
            est.orden(0.0, 0.0)
            self.assertTrue(est.medida(medida(1.0, y=0.0, theta_deg=30.0, alcance=(x0, x1))))
            est.orden(1.0, 0.2)
            for t in (1.05, 2.0, 3.0, 4.0):
                e = est.estado(t, 0.6)
                ox, oy = e.objetivo
                self.assertGreater(ox, 0.2, (x0, x1, t))
                self.assertGreater(oy, 0.0, (x0, x1, t))
                self.assertAlmostEqual(math.hypot(ox + 0.14, oy), 0.6, delta=0.01)

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

    def test_velocidad_de_la_barra_tras_cambiar_la_orden(self):
        # se manda frenar y la barra deja de verse enseguida (un reflejo): la velocidad medida es la de
        # antes de frenar y no vale; cuenta la estima de avance con la vx nueva (2026-11-06)
        est = Estimador(CFG, con_imu(), X_PUNTERA)
        est.orden(0.0, 0.2)
        t = 1.0
        while t < 3.0:
            est.medida(medida(t, alcance=(0.2, 2.0 - 0.23 * (t - 1.0)), barra=2.0 - 0.23 * (t - 1.0)))
            t += 1 / 30
        self.assertAlmostEqual(est.v_barra(), 0.23, delta=0.01)
        est.orden(3.0, 0.11)                      # frenar
        while t < 3.8:
            est.medida(medida(t, alcance=(0.2, 2.0 - 0.23 * (t - 1.0)), barra=2.0 - 0.23 * (t - 1.0)))
            t += 1 / 30
        self.assertIsNone(est.v_barra())          # menos de asentar_s tras la orden
        e = est.estado(5.0, 0.6)
        self.assertLess(e.v, 0.2)                 # la estima de avance con vx 0.11 (y la velocidad minima)
        self.assertGreater(e.v, 0.15)

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
        orden, info = ctl.calcular(0.05, estado(self.objetivo(-8.0)))     # aun por encima de la mitad
        self.assertTrue(info.get("girando"))
        orden, info = ctl.calcular(0.10, estado(self.objetivo(-3.0)))     # ya alineado: anda
        self.assertFalse(info.get("girando", False))
        orden, info = ctl.calcular(0.15, estado(self.objetivo(-25.0)))    # solo al empezar
        self.assertFalse(info.get("girando", False))
        self.assertGreater(orden.vx, 0.0)
        # saliendo poco torcido no gira en el sitio
        ctl = Control(cfg, 0.5)
        orden, info = ctl.calcular(0.0, estado(self.objetivo(-8.0)))
        self.assertFalse(info.get("girando", False))

    def test_pausa_para_girar(self):
        # a media escala el giro no cabe ni a la velocidad minima: para y gira en el sitio hasta la mitad
        import copy
        cfg = copy.deepcopy(CFG)
        cfg["control"]["pausa_giro_deg"] = 25.0
        cfg["estimacion"]["v_minima"] = 0.18
        ctl = Control(cfg, 0.5)
        orden, info = ctl.calcular(0.0, estado(self.objetivo(-15.0)))     # cabe: anda
        self.assertFalse(info.get("pausa", False))
        orden, info = ctl.calcular(0.05, estado(self.objetivo(-35.0)))    # no cabe: pausa
        self.assertTrue(info.get("pausa"))
        self.assertLess(orden.vyaw, 0.0)
        orden, info = ctl.calcular(0.10, estado(self.objetivo(-20.0)))    # aun por encima de la mitad
        self.assertTrue(info.get("pausa"))
        orden, info = ctl.calcular(0.15, estado(self.objetivo(-10.0)))    # alineado: vuelve a andar
        self.assertFalse(info.get("pausa", False))
        # a escala 1 el mismo angulo cabe y no se para
        ctl = Control(cfg, 1.0)
        orden, info = ctl.calcular(0.0, estado(self.objetivo(-35.0)))
        self.assertFalse(info.get("pausa", False))

    def test_pausa_con_la_linea_perdida(self):
        # con la linea perdida y la estimada hacia un lado, andar la aleja: para y gira en el sitio hacia ella
        import copy
        cfg = copy.deepcopy(CFG)
        cfg["control"]["pausa_perdida_deg"] = 15.0
        cfg["estimacion"]["v_minima"] = 0.18
        ctl = Control(cfg, 1.0)
        orden, info = ctl.calcular(0.0, estado(self.objetivo(20.0)))               # vista: anda aunque se tuerza
        self.assertFalse(info.get("pausa", False))
        orden, info = ctl.calcular(0.05, estado(self.objetivo(10.0)), perdida=True)  # perdida pero casi recta: anda
        self.assertFalse(info.get("pausa", False))
        orden, info = ctl.calcular(0.10, estado(self.objetivo(20.0)), perdida=True)  # perdida y hacia un lado
        self.assertTrue(info.get("pausa"))
        self.assertEqual(orden.vx, 0.0)
        self.assertGreater(orden.vyaw, 0.0)
        orden, info = ctl.calcular(0.15, estado(self.objetivo(10.0)))              # recuperada: sigue hasta la mitad
        self.assertTrue(info.get("pausa"))
        orden, info = ctl.calcular(0.20, estado(self.objetivo(5.0)))
        self.assertFalse(info.get("pausa", False))
        self.assertGreater(orden.vx, 0.0)

    def test_pausa_con_la_linea_perdida_de_lado(self):
        # perdida, el objetivo casi delante (alfa 5) pero la cinta estimada a +35 de la marcha: para y gira hasta
        # alinearse tambien con ella; luego anda despacio un rato
        import copy
        cfg = copy.deepcopy(CFG)
        cfg["control"]["pausa_perdida_deg"] = 15.0
        cfg["control"]["pausa_perdida_rumbo_deg"] = 20.0
        cfg["control"]["pausa_perdida_rumbo_edad_s"] = 0.8
        cfg["control"]["vx_tras_pausa"] = 0.16
        cfg["control"]["t_tras_pausa_s"] = 2.0
        ctl = Control(cfg, 1.0)

        def est_(alfa_deg, rumbo_deg, edad=1.0):
            e = estado(self.objetivo(alfa_deg))
            return EstadoLinea(**dict(e.__dict__, theta=self.phi + math.radians(rumbo_deg), edad_s=edad))
        orden, info = ctl.calcular(0.0, est_(5.0, 35.0))                     # vista: anda
        self.assertFalse(info.get("pausa", False))
        orden, info = ctl.calcular(0.05, est_(5.0, 15.0), perdida=True)       # perdida y casi recta: anda
        self.assertFalse(info.get("pausa", False))
        orden, info = ctl.calcular(0.10, est_(-5.0, 35.0), perdida=True)      # signos opuestos: alfa manda, anda
        self.assertFalse(info.get("pausa", False))
        orden, info = ctl.calcular(0.12, est_(5.0, 35.0, edad=0.4), perdida=True)  # recien perdida: aun anda
        self.assertFalse(info.get("pausa", False))
        orden, info = ctl.calcular(0.15, est_(5.0, 35.0), perdida=True)       # de lado: pausa hacia la izquierda
        self.assertTrue(info.get("pausa"))
        self.assertEqual(info["vx_deseada"], 0.0)
        self.assertGreater(orden.vyaw, 0.0)
        orden, info = ctl.calcular(0.20, est_(3.0, 12.0), perdida=True)       # alfa ya pequeno, la cinta aun no
        self.assertTrue(info.get("pausa"))
        orden, info = ctl.calcular(0.25, est_(2.0, 6.0))                     # alineado: anda, despacio
        self.assertFalse(info.get("pausa", False))
        for k in range(30):
            orden, info = ctl.calcular(0.30 + 0.05 * k, est_(0.0, 0.0))
        self.assertLessEqual(info["vx_deseada"], 0.16 + 1e-9)
        for k in range(30):
            orden, info = ctl.calcular(2.5 + 0.05 * k, est_(0.0, 0.0))
        self.assertGreater(info["vx_deseada"], 0.3)

    def test_busqueda_si_sigue_perdida(self):
        # la pausa con la linea perdida acaba alineada con la estimada pero sigue sin verse: gira despacio hacia el
        # mismo lado (no anda) hasta verla
        import copy
        cfg = copy.deepcopy(CFG)
        cfg["control"]["pausa_perdida_deg"] = 15.0
        cfg["control"]["vyaw_busqueda"] = 0.25
        cfg["control"]["vx_busqueda"] = 0.0
        ctl = Control(cfg, 1.0)
        orden, info = ctl.calcular(0.0, estado(self.objetivo(20.0)), perdida=True)
        self.assertTrue(info.get("pausa"))
        self.assertFalse(info.get("busqueda"))
        orden, info = ctl.calcular(0.05, estado(self.objetivo(3.0)), perdida=True)    # alineado, aun perdida
        self.assertTrue(info.get("busqueda"))
        self.assertEqual(info["vx_deseada"], 0.0)
        orden, info = ctl.calcular(0.10, estado(self.objetivo(-6.0)), perdida=True)   # pasado: sigue al mismo lado
        self.assertTrue(info.get("busqueda"))
        for k in range(20):
            orden, info = ctl.calcular(0.15 + 0.05 * k, estado(self.objetivo(-6.0)), perdida=True)
        self.assertAlmostEqual(orden.vyaw, 0.25, places=6)
        orden, info = ctl.calcular(1.5, estado(self.objetivo(2.0)))                   # vista otra vez: anda
        self.assertFalse(info.get("pausa", False))
        # andando en arco hacia el mismo lado
        cfg["control"]["vx_busqueda"] = 0.16
        ctl = Control(cfg, 1.0)
        ctl.calcular(0.0, estado(self.objetivo(20.0)), perdida=True)
        for k in range(40):
            orden, info = ctl.calcular(0.05 * (k + 1), estado(self.objetivo(3.0)), perdida=True)
        self.assertTrue(info.get("busqueda"))
        self.assertAlmostEqual(orden.vx, 0.16, places=6)
        self.assertAlmostEqual(orden.vyaw, 0.25 + cfg["control"]["vyaw_sesgo"], places=6)
        # sin vyaw_busqueda, como antes: anda al alinearse
        cfg["control"]["vyaw_busqueda"] = 0.0
        ctl = Control(cfg, 1.0)
        ctl.calcular(0.0, estado(self.objetivo(20.0)), perdida=True)
        orden, info = ctl.calcular(0.05, estado(self.objetivo(3.0)), perdida=True)
        self.assertFalse(info.get("pausa", False))

    def test_objetivo_detras_usa_el_rumbo(self):
        # linea estimada agotada: el objetivo detras del centro de giro no debe hacer girar 170 grados
        import copy
        cfg = copy.deepcopy(CFG)
        cfg["control"]["pausa_perdida_deg"] = 15.0
        cfg["control"]["pausa_perdida_rumbo_deg"] = 20.0
        ctl = Control(cfg, 1.0)
        e = EstadoLinea(t=0.0, y=0.0, theta=self.phi + math.radians(25.0), kappa=0.0, confianza=0.5, edad_s=2.0,
                        rumbo_ref=0.0, objetivo=(-0.2, -0.01), hay_linea=True)
        orden, info = ctl.calcular(0.0, e, perdida=True)
        self.assertAlmostEqual(math.degrees(info["alfa"]), 25.0, delta=0.5)
        self.assertGreater(orden.vyaw, 0.0)

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
