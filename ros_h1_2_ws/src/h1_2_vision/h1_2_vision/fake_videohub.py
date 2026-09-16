"""Un `videohub` de mentira, para trabajar sin el robot delante.

    ros2 run h1_2_vision fake_videohub

Imita el servicio del robot —mismos tópicos, mismo `api_id`, misma imagen JPEG
en el campo `binary`, mismo caché de ~15 Hz que hace que las peticiones más
rápidas devuelvan repeticiones— sobre una imagen sintética con un contador.

No es un juguete: es lo que permite probar `head_camera` y `camera_test`
enteros cuando el robot está apagado o desconectado, y comprobar que un fallo
es nuestro y no suyo. **Nunca debe correr con el robot conectado**: habría dos
servicios contestando en el mismo dominio DDS y las respuestas se mezclarían.
"""
from __future__ import annotations

import array
import time

import numpy as np
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

from .videohub_client import API_GET_IMAGE_SAMPLE, REQ_TOPIC, RES_TOPIC

PARAMS = {
    "width": 1920,
    "height": 1080,
    "fps": 15.0,        # el ritmo al que se renueva el fotograma en caché
    "quality": 80,
}


class FakeVideoHub(Node):

    def __init__(self):
        super().__init__("h1_2_fake_videohub")
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)

        import cv2
        self.cv2 = cv2
        self.ancho = int(self.get_parameter("width").value)
        self.alto = int(self.get_parameter("height").value)
        self.calidad = int(self.get_parameter("quality").value)

        self._cache = b""
        self._caducidad = 0.0
        self._periodo = 1.0 / max(1e-3, float(self.get_parameter("fps").value))
        self._n = 0
        self._servidas = 0

        self._pub = self.create_publisher(Response, RES_TOPIC, 10)
        self.create_subscription(Request, REQ_TOPIC, self._on_request, 10)
        self.create_timer(5.0, lambda: self.get_logger().info(
            f"{self._servidas} peticiones servidas, {self._n} fotogramas generados"))

        self.get_logger().warn(
            f"videohub SIMULADO en {RES_TOPIC} ({self.ancho}x{self.alto} a "
            f"{1 / self._periodo:.0f} fps). No lo uses con el robot conectado.")

    def _fotograma(self) -> bytes:
        """Devuelve el caché mientras no caduque, como hace el robot."""
        ahora = time.monotonic()
        if ahora < self._caducidad and self._cache:
            return self._cache

        self._n += 1
        img = np.zeros((self.alto, self.ancho, 3), np.uint8)
        # Un degradado fijo para ver el tamaño, y un contador para ver el
        # movimiento: si la imagen no avanza, el fallo está en el camino.
        img[:, :, 0] = np.linspace(0, 255, self.ancho, dtype=np.uint8)
        img[:, :, 1] = np.linspace(0, 255, self.alto, dtype=np.uint8)[:, None]
        self.cv2.putText(img, f"FAKE videohub #{self._n}",
                         (self.ancho // 20, self.alto // 2),
                         self.cv2.FONT_HERSHEY_SIMPLEX, self.alto / 300.0,
                         (255, 255, 255), 3)
        ok, jpeg = self.cv2.imencode(
            ".jpg", img, [int(self.cv2.IMWRITE_JPEG_QUALITY), self.calidad])
        if ok:
            self._cache = jpeg.tobytes()
            self._caducidad = ahora + self._periodo
        return self._cache

    def _on_request(self, msg: Request):
        if int(msg.header.identity.api_id) != API_GET_IMAGE_SAMPLE:
            return
        res = Response()
        res.header.identity.id = msg.header.identity.id
        res.header.identity.api_id = msg.header.identity.api_id
        res.header.status.code = 0
        res.data = ""
        # `binary` es int8[]: se reinterpretan los bytes, no se convierten.
        b = array.array("b")
        b.frombytes(self._fotograma())
        res.binary = b
        self._pub.publish(res)
        self._servidas += 1


def main(args=None):
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = FakeVideoHub()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        print("\n  cerrando el videohub simulado...")
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
