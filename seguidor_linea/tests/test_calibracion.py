"""Calibracion con marcas: sobre una imagen sintetica con la geometria verdadera, los tramos de
cinta se encuentran (y el reflejo no) y la inclinacion se recupera desde una equivocada."""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import calibracion  # noqa: E402
from seguidor.geometria import Intrinsecos, ModeloSuelo  # noqa: E402

INTR = Intrinsecos(fx=378.8, fy=378.8, cx=321.5, cy=236.0)
ALTURA, INCL_REAL, ROLL = 1.675, math.radians(51.2), math.radians(-1.0)
X_MARCA, X_BARRA = 1.15, 3.73       # bordes cercanos, desde la vertical de la camara


def imagen_sintetica(modelo):
    """Suelo oscuro, linea clara de 5 cm en y = -0.05, marca y barra transversales de 60 x 5 cm
    y un reflejo de foco (mancha clara de 25 cm de radio), como en el IR del robot."""
    v, u = np.mgrid[0:INTR.alto, 0:INTR.ancho]
    x, y = modelo.pixel_a_suelo(u.ravel(), v.ravel())
    x, y = x.reshape(v.shape), y.reshape(v.shape)
    img = np.full(v.shape, 60, dtype=np.uint8)
    cinta = np.abs(y + 0.05) < 0.025
    for x0 in (X_MARCA, X_BARRA):
        cinta |= (x >= x0) & (x < x0 + 0.05) & (np.abs(y + 0.05) < 0.30)
    img[cinta & (x < X_BARRA + 0.05)] = 200
    img[(x - 0.9) ** 2 + (y - 0.5) ** 2 < 0.25 ** 2] = 220
    return img


class TestMarcas(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.real = ModeloSuelo(INTR, ALTURA, INCL_REAL, ROLL)
        cls.img = imagen_sintetica(cls.real)

    def test_encuentra_marca_y_barra_y_no_el_reflejo(self):
        tramos = calibracion.buscar_tramos(self.img, self.real)
        self.assertEqual(len(tramos), 2)
        marca, barra = tramos
        self.assertAlmostEqual(marca["x_m"], X_MARCA, delta=0.02)
        self.assertAlmostEqual(barra["x_m"], X_BARRA, delta=0.04)
        self.assertAlmostEqual(barra["largo_m"], 0.60, delta=0.04)

    def test_recupera_la_inclinacion(self):
        for error_deg in (-1.0, +0.8):
            plano = ModeloSuelo(INTR, ALTURA, INCL_REAL + math.radians(error_deg), ROLL)
            marca, barra = calibracion.buscar_tramos(self.img, plano)
            incl = calibracion.inclinacion_por_marcas(plano, marca, barra, X_BARRA - X_MARCA)
            self.assertAlmostEqual(math.degrees(incl), math.degrees(INCL_REAL), delta=0.15)

    def test_yaw_de_una_linea_recta(self):
        puntos = calibracion.puntos_de_linea(self.img, self.real)
        a, b, _ = calibracion.ajustar_recta(puntos)
        self.assertAlmostEqual(a, -0.05, delta=0.01)
        self.assertAlmostEqual(math.degrees(math.atan(b)), 0.0, delta=0.5)


if __name__ == "__main__":
    unittest.main()
