"""Prueba rápida de la cámara de cabeza. Lo primero que hay que correr.

    ros2 run h1_2_vision camera_test                          # pregunta al robot
    ros2 run h1_2_vision camera_test --ros-args -p mode:=topic  # escucha el nodo

Dos modos, que responden a dos preguntas distintas:

  `service` (por defecto)
      Le pregunta al `videohub` del robot directamente, sin que haga falta
      tener el nodo `head_camera` corriendo. Responde a «¿hay cámara?»: si esto
      falla, el problema está en el robot o en la red, no en nuestro código.

  `topic`
      Escucha lo que publica `head_camera`, que tiene que estar corriendo en
      otra terminal. Responde a «¿llega la imagen a ROS?».

En los dos casos mide la tasa de fotogramas NUEVOS, que es la que importa: el
servicio devuelve el último fotograma en caché, así que las respuestas que se
repiten no son imagen nueva. Al terminar guarda el último fotograma en disco
para poder mirarlo (`-p save:=''` lo desactiva).
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from .videohub_client import VideoHub

PARAMS = {
    "mode": "service",          # service | topic
    "seconds": 5.0,
    "poll_hz": 30.0,
    "save": "/tmp/h1_2_head_camera.jpg",
    "topic": "/head_camera/image_raw/compressed",
    "best_effort": False,       # tiene que coincidir con el del nodo
}


class Medidor:
    """Cuenta fotogramas y separa los nuevos de las repeticiones del caché."""

    def __init__(self):
        self.total = 0
        self.nuevos = 0
        self.bytes = 0
        self.ultimo = None
        self.primero_en = None
        self.t0 = time.monotonic()

    def anota(self, jpeg: bytes):
        self.total += 1
        self.bytes += len(jpeg)
        if self.primero_en is None:
            self.primero_en = time.monotonic() - self.t0
        if jpeg != self.ultimo:
            self.nuevos += 1
            self.ultimo = jpeg


def _decodifica(jpeg: bytes):
    import cv2
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    return cv2, img


def _informe(m: Medidor, duracion: float, fuente: str, save: str) -> int:
    print()
    if m.total == 0:
        print(f"  ✗ no llegó ni un fotograma de {fuente} en {duracion:.0f} s")
        return 1

    cv2, img = _decodifica(m.ultimo)
    if img is None:
        print(f"  ✗ llegaron {m.total} fotogramas de {fuente}, pero el último "
              f"no es un JPEG válido ({len(m.ultimo)} bytes)")
        return 1

    print(f"  ✔ {fuente}")
    print(f"      resolución        {img.shape[1]}x{img.shape[0]}")
    print(f"      tamaño            {m.bytes / m.total / 1024:.0f} KB por fotograma")
    print(f"      recibidos         {m.total / duracion:.1f} Hz")
    print(f"      fotogramas NUEVOS {m.nuevos / duracion:.1f} Hz   ← la tasa real")
    print(f"      primer fotograma  {m.primero_en * 1e3:.0f} ms tras arrancar")
    if m.nuevos < 2:
        print("      ⚠ la imagen no cambia: ¿está la cámara tapada o congelada?")

    if save:
        cv2.imwrite(save, img)
        print(f"      guardado en       {save}")
    return 0


def prueba_servicio(nodo: Node) -> int:
    duracion = float(nodo.get_parameter("seconds").value)
    poll_hz = float(nodo.get_parameter("poll_hz").value)
    m = Medidor()

    vh = VideoHub(nodo, on_frame=lambda jpeg, lat: m.anota(jpeg), timeout=2.0)
    print(f"  preguntando al servicio `videohub` a {poll_hz:.0f} Hz "
          f"durante {duracion:.0f} s...")
    if not vh.wait_for_service(2.0):
        print("\n  ✗ nadie escucha en /api/videohub/request.")
        print("      · ¿está el robot encendido y el cable en la NIC de "
              "192.168.123.0/24?")
        print("      · ¿se hizo `source setup_env.sh`? Fija CYCLONEDDS_URI y la NIC.")
        print("      · comprueba con: ros2 topic list | grep videohub")
        return 1

    intervalo = 1.0 / poll_hz
    m.t0 = siguiente = time.monotonic()
    fin = m.t0 + duracion
    while time.monotonic() < fin:
        vh.request()
        siguiente += intervalo
        while (espera := siguiente - time.monotonic()) > 0:
            rclpy.spin_once(nodo, timeout_sec=espera)
        if espera < -intervalo:
            siguiente = time.monotonic()
    rclpy.spin_once(nodo, timeout_sec=0.2)      # las últimas en vuelo

    print(f"      {vh.stats()}")
    return _informe(m, duracion, "el servicio `videohub` del robot",
                    nodo.get_parameter("save").value)


def prueba_topico(nodo: Node) -> int:
    duracion = float(nodo.get_parameter("seconds").value)
    topico = nodo.get_parameter("topic").value
    qos = (qos_profile_sensor_data
           if nodo.get_parameter("best_effort").value else 10)
    m = Medidor()

    nodo.create_subscription(CompressedImage, topico,
                             lambda msg: m.anota(bytes(msg.data)), qos)
    print(f"  escuchando {topico} durante {duracion:.0f} s...")

    m.t0 = time.monotonic()
    fin = m.t0 + duracion
    while time.monotonic() < fin:
        rclpy.spin_once(nodo, timeout_sec=0.1)

    if m.total == 0:
        print(f"\n  ✗ nadie publica en {topico}")
        print("      · arranca el nodo:  ros2 run h1_2_vision head_camera")
        print("      · si el nodo va con `-p best_effort:=true`, este también "
              "lo necesita: las QoS tienen que casar o no se emparejan.")
        return 1
    return _informe(m, duracion, topico, nodo.get_parameter("save").value)


def main(args=None):
    rclpy.init(args=args)
    nodo = None
    codigo = 1
    try:
        nodo = Node("h1_2_camera_test")
        for k, v in PARAMS.items():
            nodo.declare_parameter(k, v)

        modo = nodo.get_parameter("mode").value
        print(f"\n  prueba de la cámara de cabeza (D435i por `videohub`) — modo {modo}\n")
        if modo == "service":
            codigo = prueba_servicio(nodo)
        elif modo == "topic":
            codigo = prueba_topico(nodo)
        else:
            print(f"  ✗ modo desconocido '{modo}': service | topic")

        if codigo == 0 and modo == "service":
            print("\n  siguiente paso:")
            print("      ros2 run h1_2_vision head_camera")
            print("      ros2 run rqt_image_view rqt_image_view "
                  "/head_camera/image_raw/compressed")
        print()
    except KeyboardInterrupt:
        print("\n  interrumpido")
        codigo = 130
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return codigo


if __name__ == "__main__":
    raise SystemExit(main())
