"""Cliente mínimo del servicio `motion_switcher` del robot, por ROS 2.

El servicio dice qué controlador de alto nivel está activo (`CheckMode`) y
permite soltarlo (`ReleaseMode`, que es lo que hace «entrar en modo debug»).

Aquí solo se necesita de verdad `CheckMode`: es de solo lectura y responde a la
pregunta que importa —¿hay alguien más mandando en `/lowcmd`?—.

`ReleaseMode` está implementado porque el canal `lowcmd` lo exige, pero es una
operación **peligrosa**: si el robot está de pie sosteniéndose solo, deja de
equilibrarse y se cae. Ver docs/03_SEGURIDAD.md.
"""
from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

SERVICE = "motion_switcher"
REQ_TOPIC = f"/api/{SERVICE}/request"
RES_TOPIC = f"/api/{SERVICE}/response"

API_CHECK_MODE = 1001
API_SELECT_MODE = 1002
API_RELEASE_MODE = 1003


class MotionSwitcher:
    """Se puede usar con un nodo ya existente o creando uno propio."""

    def __init__(self, node: Node | None = None, timeout: float = 3.0,
                 executor=None):
        """`executor`: si el que llama ya tiene un ejecutor girando, se le añade
        el nodo y aquí no se hace `spin_once`. Mezclar `spin_once` con un
        ejecutor propio deja al proceso sin recibir nada por otros tópicos."""
        self._owns_rclpy = False
        self._owns_node = node is None
        if self._owns_node and not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True
        self.node = node or Node("h1_2_motion_switcher_client")
        self.timeout = timeout
        self._executor = executor
        self._responses: dict[int, Response] = {}
        self._event = threading.Event()
        self._pub = self.node.create_publisher(Request, REQ_TOPIC, 10)
        self.node.create_subscription(Response, RES_TOPIC, self._on_response, 10)
        self._next_id = int(time.time() * 1000) & 0x7FFFFFFF
        if self._executor is not None and self._owns_node:
            self._executor.add_node(self.node)

    def _on_response(self, msg: Response):
        self._responses[int(msg.header.identity.id)] = msg
        self._event.set()

    def _call(self, api_id: int, params: dict | None = None):
        self._next_id += 1
        req = Request()
        req.header.identity.id = self._next_id
        req.header.identity.api_id = api_id
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = json.dumps(params or {})

        self._event.clear()
        # DDS necesita emparejar publicador y suscriptor antes de que la primera
        # petición llegue a alguna parte. Si se publica de inmediato, el mensaje
        # se pierde y parece que el servicio no existe. Se espera al
        # emparejamiento y, aun así, se reintenta: el servicio del robot no
        # reencola nada.
        def pump(dt: float):
            if self._executor is None:
                rclpy.spin_once(self.node, timeout_sec=dt)
            else:
                time.sleep(dt)

        t_match = time.monotonic() + 1.0
        while (self._pub.get_subscription_count() == 0
               and time.monotonic() < t_match):
            pump(0.02)

        deadline = time.monotonic() + self.timeout
        next_retry = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_retry:
                self._pub.publish(req)
                next_retry = now + 0.5
            pump(0.02)
            if self._next_id in self._responses:
                r = self._responses.pop(self._next_id)
                return int(r.header.status.code), r.data
        return None, None

    def check_mode(self):
        """(code, dict) con el modo activo. code None = el servicio no responde."""
        code, data = self._call(API_CHECK_MODE)
        if code == 0 and data:
            try:
                return code, json.loads(data)
            except json.JSONDecodeError:
                return code, {"raw": data}
        return code, None

    def release_mode(self):
        """⚠ SUELTA EL CONTROLADOR DE ALTO NIVEL. El robot deja de equilibrarse."""
        return self._call(API_RELEASE_MODE)

    def select_mode(self, name: str):
        """⚠ Cambia el controlador de alto nivel (p. ej. 'ai', 'normal')."""
        return self._call(API_SELECT_MODE, {"name": name})

    def close(self):
        try:
            if self._executor is not None and self._owns_node:
                self._executor.remove_node(self.node)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
