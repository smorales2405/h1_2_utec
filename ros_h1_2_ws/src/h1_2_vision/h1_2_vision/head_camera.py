"""Nodo de la cámara de cabeza: `videohub` (DDS) → tópicos ROS estándar.

    ros2 run h1_2_vision head_camera

Publica lo que publicaría cualquier cámara ROS, para que valgan las
herramientas de siempre (`rqt_image_view`, `image_transport`, `ros2 bag`):

    /head_camera/image_raw/compressed   sensor_msgs/CompressedImage  (JPEG)
    /head_camera/image_raw              sensor_msgs/Image  (bgr8, solo con raw:=true)
    /head_camera/camera_info            sensor_msgs/CameraInfo

El tópico comprimido es el camino natural: el `videohub` YA entrega JPEG, así
que se republica tal cual, sin decodificar ni recomprimir. La imagen en crudo
cuesta una decodificación por fotograma y 6 MB/s en el bus, y por eso va
apagada por defecto: se enciende con `-p raw:=true` cuando algo la necesite de
verdad (visión por computador, RViz).

**Esta cámara no da profundidad.** Es una D435i, pero llega por el `videohub`,
que solo expone el flujo de color ya comprimido. La profundidad exigiría la
cámara conectada a un computador nuestro — ver README_DEPLOY.md §7.

Sobre el rendimiento: los campos `uint8[]` de rclpy se rellenan con
`array.array('B')`, NUNCA con `bytes`. Asignar `bytes` hace que rclpy valide
el rango elemento a elemento en Python: medido aquí, **852 ms** para una imagen
de 6,2 MB, que hunde la cámara de 15 Hz a 1 Hz. Con `array.array` entra por el
camino rápido y no cuesta nada medible.

Sobre `cv_bridge`: no se usa. El que trae Humble está compilado contra numpy
1.x y en esta máquina hay numpy 2.x, así que importarlo revienta con
`_ARRAY_API not found`. Rellenar un `sensor_msgs/Image` a mano son cinco
líneas y quita la dependencia rota de en medio.
"""
from __future__ import annotations

import array
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from .videohub_client import VideoHub

# Campo de visión nominal del módulo RGB de la D435i (hoja de datos Intel).
# De aquí salen unos intrínsecos APROXIMADOS: sirven para dimensionar, no para
# medir. Si algún día se calibra de verdad, se pasan por parámetro.
HFOV_DEG = 69.4
VFOV_DEG = 42.5

PARAMS = {
    "poll_hz": 30.0,        # el doble de los ~15 Hz de fotogramas nuevos
    "repeats": False,       # publicar también las repeticiones del caché
    "raw": False,           # publicar además sensor_msgs/Image (bgr8)
    "camera_info": True,
    "width": 0,             # 0 = el tamaño nativo (1920x1080)
    "height": 0,
    "quality": 80,          # calidad JPEG al recomprimir (solo si se reescala)
    "frame_id": "camera_color_optical_frame",
    "optical_tf": True,     # TF estática camera_link -> frame_id
    "topic_ns": "/head_camera",
    "best_effort": False,   # QoS sensor_data en vez de la fiable por defecto
    "timeout": 2.0,
    "report_every": 5.0,    # segundos entre informes; 0 los silencia
}


class HeadCamera(Node):

    def __init__(self):
        super().__init__("h1_2_head_camera")
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)

        ns = self.p("topic_ns").rstrip("/")
        qos = qos_profile_sensor_data if self.p("best_effort") else 10

        self.pub_jpeg = self.create_publisher(CompressedImage,
                                              f"{ns}/image_raw/compressed", qos)
        self.pub_raw = (self.create_publisher(Image, f"{ns}/image_raw", qos)
                        if self.p("raw") else None)
        self.pub_info = (self.create_publisher(CameraInfo, f"{ns}/camera_info", qos)
                         if self.p("camera_info") else None)

        self._cv2 = None            # se importa solo si hace falta decodificar
        self._ultimo = None         # último JPEG, para detectar repeticiones
        self._forma = None          # (ancho, alto) de lo que se publica
        self._t0 = time.monotonic()
        self._publicados = 0
        self._nuevos = 0
        self._marca = (self._t0, 0, 0)
        self._quejado = False

        self.vh = VideoHub(self, on_frame=self._on_frame,
                           timeout=float(self.p("timeout")))

        if self.p("optical_tf"):
            self._publicar_tf_optica()

        periodo = 1.0 / max(1e-3, float(self.p("poll_hz")))
        self.create_timer(periodo, self._pedir)
        if float(self.p("report_every")) > 0:
            self.create_timer(float(self.p("report_every")), self._informe)

        destino = "nativo" if not self._reescala() else \
            f"{self.p('width')}x{self.p('height')} (recomprimido q={self.p('quality')})"
        self.get_logger().info(
            f"videohub → {ns}/image_raw/compressed"
            + (f" + {ns}/image_raw (bgr8)" if self.pub_raw else "")
            + f" · sondeo {self.p('poll_hz'):.0f} Hz · {destino}")

    def p(self, nombre):
        return self.get_parameter(nombre).value

    def _reescala(self) -> bool:
        return int(self.p("width")) > 0 and int(self.p("height")) > 0

    def cv2(self):
        if self._cv2 is None:
            import cv2
            self._cv2 = cv2
        return self._cv2

    # -- TF ---------------------------------------------------------------
    def _publicar_tf_optica(self):
        """`camera_link` (x adelante, URDF) → marco óptico (z adelante, REP-103).

        El URDF del robot solo trae `camera_link`, con la convención de los
        cuerpos: x hacia adelante. Las imágenes usan la convención óptica: z
        hacia adelante, x a la derecha, y hacia abajo. Sin esta rotación, todo
        lo que se proyecte con los intrínsecos sale girado 90°.
        """
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link"
        t.child_frame_id = self.p("frame_id")
        # rpy = (-90°, 0, -90°), la rotación óptica de toda la vida.
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        self._tf = StaticTransformBroadcaster(self)
        self._tf.sendTransform(t)

    # -- bucle ------------------------------------------------------------
    def _pedir(self):
        self.vh.request()
        if (not self._quejado and self.vh.received == 0
                and time.monotonic() - self._t0 > 3.0):
            self._quejado = True
            self.get_logger().warn(
                "el servicio `videohub` no contesta. Comprueba que el robot está "
                "encendido, que el cable va a la NIC de 192.168.123.0/24 y que se "
                "hizo `source setup_env.sh` (fija CYCLONEDDS_URI y el dominio 0).")

    def _on_frame(self, jpeg: bytes, latencia: float):
        nuevo = jpeg != self._ultimo
        if nuevo:
            self._ultimo = jpeg
            self._nuevos += 1
        elif not self.p("repeats"):
            return                   # el caché del robot repitiéndose

        stamp = self.get_clock().now().to_msg()
        imagen = None

        if self._reescala() or self.pub_raw is not None:
            cv2 = self.cv2()
            imagen = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if imagen is None:
                self.get_logger().warn("fotograma JPEG ilegible, descartado")
                return
            if self._reescala():
                destino = (int(self.p("width")), int(self.p("height")))
                if (imagen.shape[1], imagen.shape[0]) != destino:
                    imagen = cv2.resize(imagen, destino, interpolation=cv2.INTER_AREA)
                ok, recomprimido = cv2.imencode(
                    ".jpg", imagen,
                    [int(cv2.IMWRITE_JPEG_QUALITY), int(self.p("quality"))])
                if not ok:
                    self.get_logger().warn("no se pudo recomprimir, fotograma descartado")
                    return
                jpeg = recomprimido.tobytes()

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.p("frame_id")
        msg.format = "jpeg"
        msg.data = array.array("B", jpeg)     # ver la nota de arriba: NO bytes
        self.pub_jpeg.publish(msg)

        if self.pub_raw is not None:
            self.pub_raw.publish(self._mensaje_raw(imagen, stamp))

        if self._forma is None:
            self._forma = self._medir(imagen, jpeg)
            self.get_logger().info(
                f"✔ videohub responde: {self._forma[0]}x{self._forma[1]}, "
                f"{len(jpeg) / 1024:.0f} KB por fotograma, "
                f"latencia {latencia * 1e3:.1f} ms")
        if self.pub_info is not None:
            self.pub_info.publish(self._mensaje_info(stamp))

        self._publicados += 1

    def _medir(self, imagen, jpeg):
        if imagen is not None:
            return imagen.shape[1], imagen.shape[0]
        # Sin decodificar entera: la cabecera SOF del JPEG basta para el tamaño.
        cv2 = self.cv2()
        cab = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_REDUCED_COLOR_8)
        return (cab.shape[1] * 8, cab.shape[0] * 8) if cab is not None else (0, 0)

    def _mensaje_raw(self, imagen, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.p("frame_id")
        msg.height, msg.width = imagen.shape[0], imagen.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = imagen.shape[1] * 3
        msg.data = array.array("B", imagen.tobytes())
        return msg

    def _mensaje_info(self, stamp) -> CameraInfo:
        ancho, alto = self._forma
        # Intrínsecos DEL CAMPO DE VISIÓN NOMINAL, no de una calibración.
        # Valen para dimensionar un objeto o apuntar; no para medir.
        fx = (ancho / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
        fy = (alto / 2.0) / math.tan(math.radians(VFOV_DEG) / 2.0)
        cx, cy = ancho / 2.0, alto / 2.0

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.p("frame_id")
        info.width, info.height = ancho, alto
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _informe(self):
        t, pub0, nue0 = self._marca
        dt = time.monotonic() - t
        if dt <= 0:
            return
        self.get_logger().info(
            f"publicados {(self._publicados - pub0) / dt:5.1f} Hz · "
            f"nuevos {(self._nuevos - nue0) / dt:5.1f} Hz · {self.vh.stats()}")
        self._marca = (time.monotonic(), self._publicados, self._nuevos)


def main(args=None):
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = HeadCamera()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        print("\n  cerrando la cámara...")
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
