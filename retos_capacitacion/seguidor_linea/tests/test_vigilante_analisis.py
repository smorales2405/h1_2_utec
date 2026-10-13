"""Vigilante (con un robot falso), analisis de un escalon de vyaw e IMU de un dataset."""

import csv
import math
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config  # noqa: E402
from seguidor.analisis import respuesta_escalon  # noqa: E402
from seguidor.fuentes import ImuDataset  # noqa: E402
from seguidor.mensajes import Imu, Mando  # noqa: E402
from seguidor.vigilante import Vigilante  # noqa: E402


class RobotFalso:
    def __init__(self):
        self.edad = 0.002
        self.fallos = []
        self.roll = self.pitch = 0.0
        self.m = Mando(botones=0, lx=0.0, ly=0.0, rx=0.0, ry=0.0)

    def edad_lowstate(self):
        return self.edad

    def motores_en_fallo(self):
        return self.fallos

    def imu(self):
        return Imu(t=0.0, tick=0, roll=self.roll, pitch=self.pitch, yaw=0.0, gx=0.0, gy=0.0, gz=0.0)

    def mando(self):
        return self.m


class TestVigilante(unittest.TestCase):
    def setUp(self):
        self.cfg = config.cargar()
        self.cfg["supervisor"]["boton_parada"] = "R1"
        self.r = RobotFalso()
        self.v = Vigilante(self.r, self.cfg)

    def test_todo_bien(self):
        self.assertIsNone(self.v.motivo_parada(edad_fotograma=0.03))

    def test_cada_motivo(self):
        casos = [("edad", 0.8, "lowstate"), ("fallos", [3], "motor"), ("pitch", math.radians(25), "inclinacion"),
                 ("m", Mando(botones=0, lx=0.0, ly=0.4, rx=0.0, ry=0.0), "joystick"),
                 ("m", Mando(botones=1, lx=0.0, ly=0.0, rx=0.0, ry=0.0), "R1")]
        for campo, valor, texto in casos:
            r = RobotFalso()
            setattr(r, campo, valor)
            motivo = Vigilante(r, self.cfg).motivo_parada()
            self.assertIsNotNone(motivo, campo)
            self.assertIn(texto, motivo)

    def test_camara_parada(self):
        self.assertIn("camara", self.v.motivo_parada(edad_fotograma=2.0))

    def test_inclinacion_normal_de_pie(self):
        self.r.roll = math.radians(1.5)   # como esta el robot de pie (2026-10-03)
        self.assertIsNone(self.v.motivo_parada())


class TestEscalon(unittest.TestCase):
    def sintetico(self, retardo=0.5, ganancia=0.88, amplitud=0.3, deriva=math.radians(2.0)):
        """Yaw a 500 Hz: deriva + rampa retrasada que sube en 0.3 s + balanceo de la marcha (1.43 Hz)."""
        t = np.arange(0.0, 6.0, 0.002)
        te = np.clip(t - 3.0 - retardo + 0.15, 0.0, None)
        rampa = ganancia * amplitud * np.where(te < 0.3, te ** 2 / 0.6, te - 0.15)
        yaw = 2.9 + deriva * t + rampa + math.radians(1.5) * np.sin(2 * math.pi * 1.43 * t)
        return t, np.angle(np.exp(1j * yaw))   # envuelto, como lo da la IMU

    def test_recupera_retardo_y_ganancia(self):
        t, yaw = self.sintetico()
        r = respuesta_escalon(t, yaw, 0.0, 3.0, 6.0, 0.3)
        self.assertAlmostEqual(r["ganancia"], 0.88, delta=0.05)
        self.assertAlmostEqual(r["retardo"], 0.5, delta=0.12)
        self.assertAlmostEqual(math.degrees(r["deriva"]), 2.0, delta=0.6)
        self.assertGreater(r["t_umbral"], 0.4)

    def test_escalon_negativo(self):
        t, yaw = self.sintetico(amplitud=-0.3)
        r = respuesta_escalon(t, yaw, 0.0, 3.0, 6.0, -0.3)
        self.assertAlmostEqual(r["ganancia"], 0.88, delta=0.05)
        self.assertLess(r["giro"], 0.0)


class TestAvance(unittest.TestCase):
    def test_recupera_velocidad_y_yaw_de_la_camara(self):
        """Robot que avanza a 0.18 m/s con el rumbo oscilando +-8 grados y la camara girada +3 grados
        respecto de la direccion de avance; la linea del suelo es y = 0 en el mundo."""
        from seguidor.analisis import direccion_de_avance
        v, psi = 0.18, math.radians(3.0)
        t = np.arange(0.0, 30.0, 1 / 30)
        h = math.radians(8.0) * np.sin(2 * math.pi * t / 12.0)           # direccion de avance
        y_mundo = np.concatenate([[0.0], np.cumsum(v * np.sin(h[:-1]) / 30)]) - 0.10
        c = h + psi                                                       # eje de la camara
        rng = np.random.default_rng(3)
        a = -y_mundo / np.cos(c) + rng.normal(0, 0.003, len(t))
        theta = -c + math.radians(1.0) * np.sin(2 * math.pi * 1.4 * t)   # + guinada de cada paso
        r = direccion_de_avance(t, a, theta, np.ones(len(t), bool))
        self.assertAlmostEqual(r["v"], v, delta=0.01)
        self.assertAlmostEqual(math.degrees(-r["phi"]), 3.0, delta=0.3)

    def test_con_giro_alrededor_de_un_punto_por_detras(self):
        """Zigzag: el robot gira hasta 0.3 rad/s alrededor de un punto 0.15 m por detras de la camara.
        Sin omega en la regresion, phi sale sesgado; con omega se recuperan v, phi y d."""
        from seguidor.analisis import direccion_de_avance
        v, psi, d = 0.24, math.radians(-6.0), 0.15
        t = np.arange(0.0, 40.0, 1 / 30)
        h = math.radians(15.0) * np.sin(2 * math.pi * t / 6.0) + math.radians(1.4) * t   # zigzag + deriva
        omega = np.gradient(h, t)
        y_centro = np.concatenate([[0.0], np.cumsum(v * np.sin(h[:-1]) / 30)])
        y_cam = y_centro + d * np.sin(h) - 0.30
        c = h + psi
        rng = np.random.default_rng(4)
        a = -y_cam / np.cos(c) + rng.normal(0, 0.003, len(t))
        theta = -c + math.radians(1.0) * np.sin(2 * math.pi * 1.4 * t)
        r = direccion_de_avance(t, a, theta, np.abs(c) < math.radians(25), omega=omega)
        self.assertAlmostEqual(r["v"], v, delta=0.015)
        self.assertAlmostEqual(math.degrees(-r["phi"]), -6.0, delta=0.5)
        # d sale algo sesgado (suavizado de 1 s y angulos de +-15 grados); lo que importa es phi
        self.assertAlmostEqual(r["d"], d, delta=0.06)


class TestCsv(unittest.TestCase):
    def test_instantes_con_precision_de_ms(self):
        from seguidor.registro import CsvSimple
        with tempfile.TemporaryDirectory() as d:
            ruta = os.path.join(d, "ordenes.csv")
            c = CsvSimple(ruta, ["t_envio", "vx"])
            c.escribir({"t_envio": 123456.7891, "vx": 0.2})
            c.cerrar()
            with open(ruta) as f:
                fila = list(csv.DictReader(f))[0]
            self.assertAlmostEqual(float(fila["t_envio"]), 123456.7891, places=4)


class TestImuDataset(unittest.TestCase):
    def test_interpola_y_cruza_180(self):
        with tempfile.TemporaryDirectory() as d:
            ruta = os.path.join(d, "lowstate.csv")
            with open(ruta, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "tick", "roll_deg", "pitch_deg", "yaw_deg"])
                w.writerow([10.00, 1, 1.0, -1.0, 179.0])
                w.writerow([10.01, 2, 2.0, -2.0, -179.0])   # 2 grados mas alla de 180
            imu = ImuDataset(ruta)
            roll, pitch, yaw = imu.en(10.005)
            self.assertAlmostEqual(math.degrees(roll), 1.5, places=6)
            self.assertAlmostEqual(math.degrees(pitch), -1.5, places=6)
            self.assertAlmostEqual(math.degrees(yaw), 180.0, places=6)   # no 0


if __name__ == "__main__":
    unittest.main()
