"""Tecla ESPACIO de parada (seguidor/teclado.py) sobre un pseudoterminal, y la velocidad 0 del supervisor."""

import os
import pty
import sys
import termios
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor.teclado import Teclado  # noqa: E402
from test_supervisor import montar, correr  # noqa: E402
from seguidor.supervisor import PARADA  # noqa: E402


class TestTeclado(unittest.TestCase):
    def test_espacio_y_terminal_restaurado(self):
        maestro, esclavo = pty.openpty()
        try:
            antes = termios.tcgetattr(esclavo)
            pulsada = threading.Event()
            with Teclado(pulsada.set, fd=esclavo) as t:
                self.assertTrue(t.activo)
                self.assertFalse(termios.tcgetattr(esclavo)[3] & termios.ICANON)   # tecla a tecla, sin Enter
                os.write(maestro, b"xq\n")
                time.sleep(0.3)
                self.assertFalse(pulsada.is_set())        # otras teclas no paran
                os.write(maestro, b" ")
                self.assertTrue(pulsada.wait(1.0))
                self.assertTrue(t.pulsada)
            self.assertEqual(termios.tcgetattr(esclavo), antes)
        finally:
            os.close(maestro)
            os.close(esclavo)

    def test_sin_terminal(self):
        r, w = os.pipe()
        try:
            t = Teclado(lambda: None, fd=r).abrir()
            self.assertFalse(t.activo)
            t.cerrar()
        finally:
            os.close(r)
            os.close(w)


class TestParadaOperador(unittest.TestCase):
    def test_espacio_manda_velocidad_cero_y_la_repite(self):
        sup, robot, est = montar()
        correr(sup, 0.0, 1.0)
        self.assertGreater(robot.moves[-1][0], 0.0)
        sup.detener(1.0, "tecla ESPACIO (parada del operador)")
        self.assertEqual(sup.estado, PARADA)
        self.assertEqual(robot.llamadas[-1], "StopMove")
        n = len(robot.moves)
        for _ in range(5):
            sup.quieto()
        self.assertEqual(robot.llamadas[-6:], ["StopMove"] * 6)
        self.assertEqual(len(robot.moves), n)        # ningun Move con velocidad despues de parar

    def test_en_simulacro_no_manda_nada(self):
        sup, robot, est = montar(simulacro=True)
        correr(sup, 0.0, 1.0)
        sup.detener(1.0, "tecla ESPACIO (parada del operador)")
        sup.quieto()
        self.assertEqual(robot.llamadas, [])


if __name__ == "__main__":
    unittest.main()
