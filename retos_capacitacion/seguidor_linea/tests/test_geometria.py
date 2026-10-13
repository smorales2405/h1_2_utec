"""Geometria camara-suelo: el ajuste del plano recupera la pose y la proyeccion es coherente."""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import geometria  # noqa: E402
from seguidor.geometria import Intrinsecos, ModeloSuelo  # noqa: E402

INTR_IR = Intrinsecos(fx=378.8, fy=378.8, cx=321.5, cy=236.0)   # los de la D435i del robot
ALTURA, INCL, ROLL = 1.68, math.radians(51.7), math.radians(0.6)


def profundidad_sintetica(intr, altura, incl, roll, ruido_m=0.0, semilla=0):
    """Imagen de profundidad (m) de un suelo plano visto desde la pose dada."""
    n = geometria.normal_de_pose(incl, roll)
    v, u = np.mgrid[0:intr.alto, 0:intr.ancho]
    rx, ry = (u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy
    den = n[0] * rx + n[1] * ry + n[2]          # n . (rx, ry, 1)
    with np.errstate(divide="ignore", invalid="ignore"):
        z = np.where(den < -1e-6, -altura / den, 0.0)
    z[z > 8.0] = 0.0
    if ruido_m:
        z = np.where(z > 0, z + np.random.default_rng(semilla).normal(0, ruido_m, z.shape), 0.0)
    return z


class TestPlano(unittest.TestCase):
    def test_recupera_la_pose(self):
        z = profundidad_sintetica(INTR_IR, ALTURA, INCL, ROLL, ruido_m=0.005)
        n, d, frac = geometria.ajustar_plano(geometria.nube(z, INTR_IR))
        altura, incl, roll = geometria.pose_de_plano(n, d)
        self.assertAlmostEqual(altura, ALTURA, delta=0.01)
        self.assertAlmostEqual(math.degrees(incl), math.degrees(INCL), delta=0.3)
        self.assertAlmostEqual(math.degrees(roll), math.degrees(ROLL), delta=0.3)
        self.assertGreater(frac, 0.9)

    def test_normal_coincide_con_la_rotacion(self):
        m = ModeloSuelo(INTR_IR, ALTURA, INCL, ROLL)
        arriba_en_camara = m.rotacion().T @ np.array([0.0, 0.0, 1.0])
        np.testing.assert_allclose(arriba_en_camara, geometria.normal_de_pose(INCL, ROLL), atol=1e-12)

    def test_medida_real_del_robot(self):
        # normal ajustada en la captura del 2026-10-02: 51.7 grados de inclinacion
        _, incl, roll = geometria.pose_de_plano(np.array([0.0024, -0.6196, -0.7849]), 1.68)
        self.assertAlmostEqual(math.degrees(incl), 51.71, delta=0.05)
        self.assertLess(abs(math.degrees(roll)), 0.5)


class TestProyeccion(unittest.TestCase):
    def setUp(self):
        self.m = ModeloSuelo(INTR_IR, ALTURA, INCL, 0.0)

    def test_eje_optico(self):
        x, y = self.m.pixel_a_suelo(INTR_IR.cx, INTR_IR.cy)
        self.assertAlmostEqual(float(x), ALTURA / math.tan(INCL), places=6)
        self.assertAlmostEqual(float(y), 0.0, places=6)

    def test_izquierda_de_la_imagen_es_y_positiva(self):
        _, y = self.m.pixel_a_suelo(100, 300)
        self.assertGreater(float(y), 0.0)

    def test_ida_y_vuelta(self):
        rng = np.random.default_rng(1)
        u, v = rng.uniform(0, 639, 200), rng.uniform(150, 479, 200)
        x, y = self.m.pixel_a_suelo(u, v)
        u2, v2 = self.m.suelo_a_pixel(x, y)
        np.testing.assert_allclose(u2, u, atol=1e-6)
        np.testing.assert_allclose(v2, v, atol=1e-6)

    def test_horizonte(self):
        x, _ = self.m.pixel_a_suelo(INTR_IR.cx, -500)
        self.assertTrue(np.isnan(x))

    def test_alcance_de_la_captura(self):
        cerca, lejos = self.m.alcance()
        self.assertAlmostEqual(cerca, 0.17, delta=0.02)   # medido el 2026-10-02
        self.assertAlmostEqual(lejos, 4.78, delta=0.15)

    def test_pitch_de_la_imu_equivale_a_mas_inclinacion(self):
        mas = ModeloSuelo(INTR_IR, ALTURA, INCL + math.radians(2.0), 0.0)
        x1, y1 = self.m.pixel_a_suelo(300, 300, imu_roll=0.0, imu_pitch=math.radians(2.0))
        x2, y2 = mas.pixel_a_suelo(300, 300)
        self.assertAlmostEqual(float(x1), float(x2), places=9)
        self.assertAlmostEqual(float(y1), float(y2), places=9)

    def test_imu_igual_a_la_referencia_no_cambia_nada(self):
        m = ModeloSuelo(INTR_IR, ALTURA, INCL, 0.0, imu_roll_ref=0.01, imu_pitch_ref=-0.02)
        np.testing.assert_allclose(m.rotacion(0.01, -0.02), m.rotacion(), atol=1e-12)

    def test_yaw_a_la_izquierda_mira_a_la_izquierda(self):
        m = ModeloSuelo(INTR_IR, ALTURA, INCL, 0.0, yaw=math.radians(5.0))
        _, y = m.pixel_a_suelo(INTR_IR.cx, INTR_IR.cy)
        self.assertGreater(float(y), 0.0)

    def test_vista_superior(self):
        mu, mv = self.m.mapa_vista_superior(0.3, 2.5, 0.5, 0.01)
        self.assertEqual(mu.shape, (220, 100))
        # abajo en el centro: cerca y delante -> parte baja y central de la imagen
        self.assertTrue(250 < mu[-1, 50] < 390 and mv[-1, 50] > 300)


if __name__ == "__main__":
    unittest.main()
