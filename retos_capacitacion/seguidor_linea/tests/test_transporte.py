"""Fotogramas por ZMQ y por dataset, mando y configuracion: lo que cruza entre procesos y ficheros."""

import os
import sys
import tempfile
import unittest

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config, fotograma_zmq, mando  # noqa: E402
from seguidor.fuentes import EscritorDataset, FuenteDataset  # noqa: E402
from seguidor.mensajes import Fotograma, Mando  # noqa: E402

META = {"ir": {"fx": 378.8, "fy": 378.8, "cx": 321.5, "cy": 236.0, "ancho": 640, "alto": 480},
        "escala_prof": 0.001}


def fotograma(n, color=True, prof=True):
    rng = np.random.default_rng(n)
    return Fotograma(n=n, t_cam=1000.0 + n / 30, t_rx=50.0 + n / 30, emisor=False, meta=META,
                     ir=rng.integers(0, 255, (480, 640), dtype=np.uint8),
                     color=rng.integers(0, 255, (480, 640, 3), dtype=np.uint8) if color else None,
                     prof=rng.integers(0, 6000, (480, 640), dtype=np.uint16) if prof else None)


class TestZmq(unittest.TestCase):
    def test_ida_y_vuelta(self):
        f = fotograma(7)
        g = fotograma_zmq.decodificar(fotograma_zmq.codificar(f))
        self.assertEqual((g.n, g.t_cam, g.t_rx, g.emisor, g.meta), (f.n, f.t_cam, f.t_rx, f.emisor, f.meta))
        for k in ("ir", "color", "prof"):
            np.testing.assert_array_equal(getattr(g, k), getattr(f, k))

    def test_solo_ir(self):
        g = fotograma_zmq.decodificar(fotograma_zmq.codificar(fotograma(1, color=False, prof=False)))
        self.assertIsNone(g.color)
        self.assertIsNone(g.prof)
        self.assertEqual(g.ir.shape, (480, 640))


try:
    import zmq  # noqa: F401  (solo en el robot, teleop_venv)
    HAY_ZMQ = True
except ImportError:
    HAY_ZMQ = False


@unittest.skipUnless(HAY_ZMQ, "sin zmq (solo en el robot)")
class TestColaVieja(unittest.TestCase):
    def test_fresco_descarta_los_viejos(self):
        """Con la cola llena de fotogramas viejos (como tras escribir SEGUIR), fresco() devuelve uno reciente."""
        import time
        from seguidor.fuentes import FuenteZmq
        ctx = zmq.Context.instance()
        pub = ctx.socket(zmq.PUB)
        pub.setsockopt(zmq.SNDHWM, 4)
        pub.setsockopt(zmq.LINGER, 0)
        puerto = pub.bind_to_random_port("tcp://127.0.0.1")
        fuente = FuenteZmq(f"tcp://127.0.0.1:{puerto}", solo_ultimo=True)
        time.sleep(0.3)                                  # que el suscriptor conecte
        try:
            ahora = time.monotonic()
            for k in range(12):                          # viejos: de hace 3 s
                f = fotograma(k, color=False, prof=False)
                f.t_rx = ahora - 3.0
                pub.send_multipart(fotograma_zmq.codificar(f))
            time.sleep(0.2)

            def publicar():
                for k in range(12, 40):
                    f = fotograma(k, color=False, prof=False)
                    f.t_rx = time.monotonic()
                    pub.send_multipart(fotograma_zmq.codificar(f))
                    time.sleep(1 / 30)
            import threading
            threading.Thread(target=publicar, daemon=True).start()
            f = fuente.fresco(edad_max_s=0.2, timeout_s=2.0)
            self.assertIsNotNone(f)
            self.assertGreaterEqual(f.n, 12)
            self.assertLess(time.monotonic() - f.t_rx, 0.2)
        finally:
            fuente.cerrar()
            pub.close()


class TestDataset(unittest.TestCase):
    def test_ida_y_vuelta(self):
        with tempfile.TemporaryDirectory() as d:
            esc = EscritorDataset(d)
            originales = [fotograma(i, color=False) for i in range(3)]
            for f in originales:
                esc.escribir(f)
            esc.cerrar()
            self.assertEqual((esc.escritos, esc.descartados), (3, 0))
            fuente = FuenteDataset(d)
            self.assertEqual(len(fuente), 3)
            self.assertEqual(fuente.meta, META)
            for f, g in zip(originales, fuente):
                self.assertEqual((g.n, g.t_rx), (f.n, round(f.t_rx, 6)))
                np.testing.assert_array_equal(g.ir, f.ir)       # PNG sin perdidas
                np.testing.assert_array_equal(g.prof, f.prof)
            self.assertIsNone(fuente.siguiente())


class TestMando(unittest.TestCase):
    def test_ida_y_vuelta(self):
        m = Mando(botones=(1 << 0) | (1 << 9), lx=0.25, ly=-0.5, rx=0.0, ry=1.0)
        g = mando.decodificar(list(mando.codificar(m)))
        self.assertEqual(mando.pulsados(g), ["R1", "B"])
        self.assertTrue(g.pulsado("R1") and not g.pulsado("L1"))
        self.assertAlmostEqual(g.ly, -0.5)
        self.assertTrue(g.ejes_activos())

    def test_reposo(self):
        g = mando.decodificar([0] * 40)
        self.assertFalse(g.ejes_activos())
        self.assertEqual(mando.pulsados(g), [])


class TestConfig(unittest.TestCase):
    def test_la_calibracion_pisa_la_geometria(self):
        with tempfile.TemporaryDirectory() as d:
            ruta = os.path.join(d, "geometria.yaml")
            config.guardar_geometria({"altura_m": 1.7, "calibrada": True}, ruta)
            cfg = config.cargar(ruta_geometria=ruta)
            self.assertEqual(cfg["geometria"]["altura_m"], 1.7)
            self.assertTrue(cfg["geometria"]["calibrada"])
            self.assertIn("inclinacion_deg", cfg["geometria"])   # lo demas sigue de seguidor.yaml

    def test_limites_dentro_del_reto(self):
        with open(config.RUTA_CONFIG) as f:
            lim = yaml.safe_load(f)["limites"]
        self.assertLessEqual(lim["vx_max"], 0.4)
        self.assertLessEqual(lim["vy_max"], 0.2)
        self.assertLessEqual(lim["vyaw_max"], 0.5)


if __name__ == "__main__":
    unittest.main()
