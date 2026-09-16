"""Cliente del servicio `videohub` del robot, hablado en ROS 2 puro.

La cámara de cabeza es una **Intel RealSense D435i cableada al PC1**, el
computador de locomoción, al que no tenemos acceso. En el PC2 no hay ningún
dispositivo de vídeo en el bus USB, así que no se puede abrir con `pyrealsense2`
ni con `realsense2_camera`. El camino que SÍ existe es el servicio `videohub`
del propio robot, que publica en el dominio DDS 0 y responde a cualquiera **sin
credenciales**. Todo el análisis está en
`h1_2_teleoperation/README_DEPLOY.md` §7.

`unitree_sdk2py` lo envuelve en `go2/video/video_client.py`, pero ese SDK habla
DDS por su cuenta y no se lleva bien con un proceso rclpy. Como `videohub` es un
servicio de la API de Unitree como cualquier otro —dos tópicos,
`unitree_api/Request` y `unitree_api/Response`—, aquí se habla directamente
desde el nodo ROS, igual que `h1_2_arm_control/motion_switcher.py` hace con
`motion_switcher`. Sin SDK, sin segundo participante DDS.

El detalle que lo distingue de los demás servicios: la respuesta **no viene en
`data` sino en `binary`**, y son los bytes de un JPEG. Es lo que hace
`_CallBinary` en el SDK (`rpc/client_base.py`).

Medido sobre este robot:

    resolución           1920x1080
    tamaño por fotograma ~77-82 KB (JPEG)
    latencia por petición media 6.4 ms, máx 7.3
    fotogramas NUEVOS    ~15 Hz

Los 15 Hz son el techo real: el servicio devuelve el último fotograma en caché,
así que pedir más rápido solo trae repeticiones.
"""
from __future__ import annotations

import random
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

SERVICE = "videohub"
REQ_TOPIC = f"/api/{SERVICE}/request"
RES_TOPIC = f"/api/{SERVICE}/response"

API_GET_IMAGE_SAMPLE = 1001


def jpeg_bytes(binary) -> bytes:
    """`Response.binary` es `int8[]`, que rclpy entrega como `array('b')`.

    Los bytes de un JPEG son sin signo, así que hay que reinterpretarlos, no
    convertirlos elemento a elemento: `array('b').tobytes()` devuelve el mismo
    buffer sin tocar, que es justo lo que queremos.
    """
    if isinstance(binary, (bytes, bytearray)):
        return bytes(binary)
    tobytes = getattr(binary, "tobytes", None)
    if tobytes is not None:
        return tobytes()
    return bytes(b & 0xFF for b in binary)


class VideoHub:
    """Peticiones de imagen al `videohub`, sobre un nodo ROS ya existente.

    Se usa de dos maneras, y el nodo y el script de prueba usan una cada uno:

      * **asíncrona** — `request()` manda una petición y vuelve enseguida; las
        respuestas llegan a `on_frame(jpeg, latencia)`. Es la que quiere un
        nodo que ya está girando: pedir a 30 Hz sin bloquear el ejecutor.
      * **síncrona** — `get_image_sample()` pide y espera, girando el nodo él
        mismo. Para scripts de una sola pasada.
    """

    def __init__(self, node: Node, on_frame=None, timeout: float = 2.0,
                 max_pending: int = 3):
        self.node = node
        self.on_frame = on_frame
        self.timeout = timeout
        self.max_pending = max_pending

        self._pub = node.create_publisher(Request, REQ_TOPIC, 10)
        node.create_subscription(Response, RES_TOPIC, self._on_response, 10)

        # Las respuestas del videohub las oye TODO el dominio, no solo quien
        # preguntó: si el puente del PC2 está corriendo, aquí entran también
        # sus respuestas. Se emparejan por `identity.id`, así que el nuestro
        # tiene que ser distinto del de cualquier otro cliente: se arranca de
        # un número al azar en vez de de cero.
        self._next_id = random.getrandbits(30)
        self._pending: dict[int, float] = {}

        # Estadísticas, que es la mitad de lo que se le pide a una cámara.
        self.sent = 0
        self.received = 0
        self.timed_out = 0
        self.errors = 0
        self.latency_sum = 0.0
        self.latency_max = 0.0

    # -- asíncrono --------------------------------------------------------
    def request(self) -> bool:
        """Manda una petición. False si hay demasiadas sin contestar.

        El tope existe para que un robot que no responde no deje una cola de
        peticiones creciendo en memoria y en el bus.
        """
        self._prune()
        if len(self._pending) >= self.max_pending:
            return False

        self._next_id += 1
        req = Request()
        req.header.identity.id = self._next_id
        req.header.identity.api_id = API_GET_IMAGE_SAMPLE
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = ""          # GetImageSample no lleva parámetros

        self._pending[self._next_id] = time.monotonic()
        self._pub.publish(req)
        self.sent += 1
        return True

    def _prune(self):
        """Las peticiones que ya no van a contestarse cuentan como perdidas."""
        limite = time.monotonic() - self.timeout
        for ident in [i for i, t in self._pending.items() if t < limite]:
            del self._pending[ident]
            self.timed_out += 1

    def _on_response(self, msg: Response):
        ident = int(msg.header.identity.id)
        enviada = self._pending.pop(ident, None)
        if enviada is None:
            return                   # de otro cliente, o ya caducada
        if int(msg.header.identity.api_id) != API_GET_IMAGE_SAMPLE:
            return

        latencia = time.monotonic() - enviada
        code = int(msg.header.status.code)
        if code != 0:
            self.errors += 1
            return

        datos = jpeg_bytes(msg.binary)
        if not datos:
            self.errors += 1
            return

        self.received += 1
        self.latency_sum += latencia
        self.latency_max = max(self.latency_max, latencia)
        if self.on_frame is not None:
            self.on_frame(datos, latencia)

    # -- síncrono ---------------------------------------------------------
    def wait_for_service(self, timeout: float = 2.0) -> bool:
        """Espera a que DDS empareje al servicio con nuestro publicador.

        Hace falta: si se publica antes del emparejamiento el mensaje se pierde
        sin más, y el servicio parece no existir cuando lo que pasa es que aún
        no nos oía.
        """
        limite = time.monotonic() + timeout
        while time.monotonic() < limite:
            if self._pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.02)
        return self._pub.get_subscription_count() > 0

    def get_image_sample(self, timeout: float | None = None):
        """(jpeg, latencia) del último fotograma, o (None, None) si no responde.

        Gira el nodo mientras espera, así que NO se puede llamar desde un
        proceso que ya tenga un ejecutor girando por su cuenta: mezclar
        `spin_once` con un ejecutor propio deja al proceso sordo. Para ese caso
        está la vía asíncrona.
        """
        timeout = self.timeout if timeout is None else timeout
        recibido: dict[str, object] = {}

        anterior = self.on_frame
        self.on_frame = lambda jpeg, lat: recibido.setdefault("f", (jpeg, lat))
        try:
            if not self.request():
                self._pending.clear()
                self.request()
            limite = time.monotonic() + timeout
            while time.monotonic() < limite and "f" not in recibido:
                rclpy.spin_once(self.node, timeout_sec=0.01)
        finally:
            self.on_frame = anterior

        if "f" in recibido:
            return recibido["f"]
        return None, None

    # -- informe ----------------------------------------------------------
    def stats(self) -> str:
        media = (self.latency_sum / self.received * 1e3) if self.received else 0.0
        return (f"{self.received}/{self.sent} respuestas · "
                f"latencia media {media:.1f} ms (máx {self.latency_max * 1e3:.1f}) · "
                f"perdidas {self.timed_out} · errores {self.errors}")
