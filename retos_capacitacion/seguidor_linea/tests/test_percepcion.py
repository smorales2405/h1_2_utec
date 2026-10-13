"""Percepcion sobre imagenes sinteticas renderizadas con la geometria real del robot: recta, curva,
cinta oscura y clara, barra de fin, cruce, esquina, sin linea, puntos del emisor, sombra e
interrupcion de 40 cm."""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config  # noqa: E402
from seguidor.geometria import Intrinsecos, ModeloSuelo  # noqa: E402
from seguidor.mensajes import Fotograma  # noqa: E402
from seguidor.percepcion import Percepcion  # noqa: E402

INTR = Intrinsecos(fx=378.8, fy=378.8, cx=321.5, cy=236.0)
GEO = {"altura_m": 1.66, "inclinacion_deg": 54.1, "roll_deg": 0.0, "imu_roll_ref_deg": 0.0, "imu_pitch_ref_deg": 0.0}
MODELO = ModeloSuelo(INTR, GEO["altura_m"], math.radians(GEO["inclinacion_deg"]))
_v, _u = np.mgrid[0:INTR.alto, 0:INTR.ancho]
X, Y = (z.reshape(_v.shape) for z in MODELO.pixel_a_suelo(_u.ravel(), _v.ravel()))
SUELO, CINTA_OSCURA, CINTA_CLARA = 120.0, 60.0, 200.0


def imagen(cinta=None, valor=CINTA_OSCURA, ruido=3.0, semilla=0, extra=None):
    """Suelo de `SUELO` con ruido; `cinta` = mascara booleana (en X, Y) de la cinta."""
    rng = np.random.default_rng(semilla)
    img = np.full(X.shape, SUELO) + rng.normal(0, ruido, X.shape)
    if cinta is not None:
        img[cinta & np.isfinite(X)] = valor + rng.normal(0, ruido, int((cinta & np.isfinite(X)).sum()))
    if extra is not None:
        img = extra(img)
    img[~np.isfinite(X)] = 90.0          # por encima del horizonte
    return np.clip(img, 0, 255).astype(np.uint8)


def linea(f, x0=0.0, x1=9.0, ancho=0.05):
    return (np.abs(Y - f(X)) < ancho / 2) & (X >= x0) & (X <= x1)


def tramo(x_cerca, y0=0.0, largo=0.60, ancho=0.05, lado=0):
    """Tramo transversal con el borde cercano en x_cerca; lado 0 centrado, +1 a la izquierda, -1 a la derecha."""
    y_lo, y_hi = (y0 - largo / 2, y0 + largo / 2) if lado == 0 else ((y0, y0 + largo) if lado > 0 else (y0 - largo, y0))
    return (X >= x_cerca) & (X <= x_cerca + ancho) & (Y >= y_lo) & (Y <= y_hi)


def medir(img, veces=1, **cambios):
    p = dict(config.cargar()["percepcion"], **cambios)
    per = Percepcion(p, INTR, GEO)
    for k in range(veces):
        m = per.procesar(Fotograma(n=k, t_cam=0.0, t_rx=float(k), ir=img))
    return m, per


class TestRecta(unittest.TestCase):
    def test_cinta_oscura(self):
        m, per = medir(imagen(linea(lambda x: -0.08 + 0.05 * x)))
        self.assertAlmostEqual(m.y, -0.08, delta=0.01)
        self.assertAlmostEqual(m.theta, math.atan(0.05), delta=math.radians(0.5))
        self.assertGreater(m.confianza, 0.8)
        self.assertEqual(per.detalle.polaridad, -1)
        self.assertGreaterEqual(m.n_franjas, 3)
        self.assertLess(m.ms, 60)

    def test_cinta_clara_polaridad_automatica(self):
        m, per = medir(imagen(linea(lambda x: 0.10 - 0.08 * x), valor=CINTA_CLARA), veces=3)
        self.assertEqual(per.detalle.polaridad, +1)
        self.assertAlmostEqual(m.y, 0.10, delta=0.01)
        self.assertGreater(m.confianza, 0.8)

    def test_objetivo_sobre_la_linea(self):
        m, _ = medir(imagen(linea(lambda x: -0.08 + 0.05 * x)))
        x, y = m.objetivo
        self.assertAlmostEqual(x, 1.0, delta=0.01)
        self.assertAlmostEqual(y, -0.03, delta=0.015)


class TestCurva(unittest.TestCase):
    def test_curvatura(self):
        m, _ = medir(imagen(linea(lambda x: 0.05 + 0.15 * x ** 2)))
        self.assertAlmostEqual(m.kappa, 0.30, delta=0.06)
        self.assertGreater(m.confianza, 0.7)


    def test_curva_cerrada_del_nivel_3(self):
        # radio 1.2 m, tangente al eje en la camara (2026-10-05: con el ajuste anterior salia una recta
        # de 0.3-0.9 m con confianza 0.2, y se daba la linea por perdida en plena curva)
        R = 1.2
        cinta = (np.abs(np.hypot(X, Y - R) - R) < 0.025) & (X > 0) & (Y < R)
        m, per = medir(imagen(cinta))
        self.assertGreater(m.confianza, 0.5)
        self.assertGreater(per.detalle.x_fin, 0.9)
        a, b, c = per.detalle.coef
        for x in (0.4, 0.8):
            self.assertAlmostEqual(a + b * x + c * x * x, R - math.sqrt(R * R - x * x), delta=0.02)


class TestBarraYEsquina(unittest.TestCase):
    def test_barra_de_fin(self):
        img = imagen(linea(lambda x: 0.02 + 0 * x, x1=1.62) | tramo(1.60, y0=0.02))
        m, _ = medir(img)
        self.assertIsNotNone(m.barra_fin)
        self.assertAlmostEqual(m.barra_fin, 1.60, delta=0.03)
        self.assertIsNone(m.esquina)
        self.assertGreater(m.confianza, 0.7)

    def test_barra_con_el_robot_girado(self):
        # la linea va a 12 grados: la barra, perpendicular a ella, sale inclinada en la vista desde arriba
        b = math.tan(math.radians(12))
        cinta = linea(lambda x: b * x, x1=1.3 * math.cos(math.radians(12)) + 0.02)
        xb = 1.3 * math.cos(math.radians(12))
        yb = b * xb
        u, w = (X - xb) * math.cos(math.radians(12)) + (Y - yb) * math.sin(math.radians(12)), \
            -(X - xb) * math.sin(math.radians(12)) + (Y - yb) * math.cos(math.radians(12))
        cinta |= (u >= 0) & (u <= 0.05) & (np.abs(w) <= 0.30)
        m, _ = medir(imagen(cinta))
        self.assertIsNotNone(m.barra_fin)
        self.assertAlmostEqual(m.barra_fin, xb, delta=0.05)

    def test_un_cruce_no_es_la_barra(self):
        m, _ = medir(imagen(linea(lambda x: 0.0 * x) | tramo(1.0)))
        self.assertIsNone(m.barra_fin)

    def test_esquina_a_la_izquierda(self):
        img = imagen(linea(lambda x: 0.0 * x, x1=1.52) | tramo(1.50, lado=+1, largo=0.6))
        m, _ = medir(img)
        self.assertIsNotNone(m.esquina)
        self.assertAlmostEqual(m.esquina[0], 1.50, delta=0.04)
        self.assertEqual(m.esquina[1], +1)
        self.assertIsNone(m.barra_fin)


class TestRobustez(unittest.TestCase):
    def test_sin_linea(self):
        m, _ = medir(imagen())
        self.assertLess(m.confianza, 0.2)

    def test_un_borde_no_es_una_linea(self):
        # medio suelo oscuro (el canto de una puerta, 2026-10-05): contraste a un solo lado
        def borde(img):
            img = img.copy()
            img[np.isfinite(X) & (Y < -0.30 + 0.10 * X)] = 40.0
            return img
        m, _ = medir(imagen(extra=borde))
        self.assertLess(m.confianza, 0.3)
        m, _ = medir(imagen(linea(lambda x: 0.15 + 0 * x), extra=borde))
        self.assertAlmostEqual(m.y, 0.15, delta=0.01)

    def test_puntos_del_emisor(self):
        def puntos(img):
            rng = np.random.default_rng(7)
            vs, us = rng.integers(0, 480, 4000), rng.integers(0, 640, 4000)
            img[vs, us] = 230
            return img
        m, _ = medir(imagen(linea(lambda x: -0.05 + 0 * x), extra=puntos))
        self.assertAlmostEqual(m.y, -0.05, delta=0.01)
        self.assertGreater(m.confianza, 0.7)

    def test_sombra(self):
        # de 1.2 m en adelante, todo (suelo y cinta) a la mitad de luz
        def sombra(img):
            img = img.copy()
            img[np.isfinite(X) & (X > 1.2)] *= 0.5
            return img
        m, per = medir(imagen(linea(lambda x: 0.03 + 0 * x), extra=sombra))
        self.assertGreater(m.confianza, 0.6)
        self.assertGreater(per.detalle.x_fin, 2.0)

    def test_interrupcion_de_40_cm(self):
        cinta = linea(lambda x: 0.04 + 0 * x, x1=1.0) | linea(lambda x: 0.04 + 0 * x, x0=1.4)
        m, per = medir(imagen(cinta))
        self.assertAlmostEqual(m.y, 0.04, delta=0.01)
        self.assertGreater(per.detalle.x_fin, 2.0)      # sigue al otro lado del hueco



DATASET = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "datos",
                       "dataset_20261006_051142_nivel1_negra_a")


@unittest.skipUnless(os.path.isdir(DATASET), "dataset de la cinta negra no disponible (datos/ no va en git)")
class TestDatasetReal(unittest.TestCase):
    """Regresion sobre el recorrido alineado de la cinta negra: de pie al principio, la linea a
    -7.6 cm y -0.4 grados (analizar_dataset.py y calibracion lo confirman)."""

    def test_de_pie_al_principio(self):
        import json
        from seguidor.fuentes import FuenteDataset
        with open(os.path.join(DATASET, "resumen.json")) as f:
            geo = json.load(f)["geometria"]
        fuente = FuenteDataset(DATASET)
        per = Percepcion(config.cargar()["percepcion"], Intrinsecos.desde_dict(fuente.meta["ir"]), geo)
        for _ in range(30):
            m = per.procesar(fuente.siguiente())
            self.assertGreater(m.confianza, 0.8)
            self.assertAlmostEqual(m.y, -0.076, delta=0.01)
            self.assertAlmostEqual(math.degrees(m.theta), -0.4, delta=0.5)
            self.assertEqual(per.detalle.polaridad, -1)


if __name__ == "__main__":
    unittest.main()
