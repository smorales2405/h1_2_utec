"""Lo que comparten los nodos: parámetros, cliente y apagado ordenado.

Los nodos de este paquete no son nodos que giren indefinidamente: hacen una
cosa —colocar una postura, recorrer una trayectoria— y terminan. Aun así son
nodos ROS y se lanzan con `ros2 run`, así que los parámetros se declaran como
parámetros ROS y se pasan con `--ros-args -p`.

El apagado tiene que ser ordenado SIEMPRE. Si el proceso muere dejando de
publicar de golpe, los motores se quedan con la última consigna y la última
ganancia; por eso todo va dentro de un `with` sobre el cliente, que al salir
—también por Ctrl-C o por excepción— baja las ganancias en rampa.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from .arm_client import H12Client, SafetyAbort
from . import gains as cfg
from .joints import ARM_INDICES, BY_INDEX, BY_NAME


PARAMS_COMUNES = {
    "gains": "",            # conjunto de gains.yaml; "" = el marcado `active`
    "channel": "lowcmd",    # lowcmd (modo debug) o arm_sdk
    "rate_hz": 250.0,
    "gravity": True,        # feedforward de gravedad; `tuned_gff` lo NECESITA
    "legs": "hold",         # free | damp | hold
    "speed": 0.15,          # rad/s de las rampas
    "dry_run": False,       # publica en un tópico que nadie escucha
}


class ArmNode(Node):
    """Nodo que toma el control de los brazos, hace algo, y lo devuelve."""

    def __init__(self, name: str, extra: dict | None = None):
        super().__init__(name)
        for k, v in {**PARAMS_COMUNES, **(extra or {})}.items():
            self.declare_parameter(k, v)

    def p(self, nombre):
        return self.get_parameter(nombre).value

    def cliente(self, controlled=None) -> H12Client:
        """Cliente configurado con los parámetros del nodo."""
        conjunto = self.p("gains") or None
        g = cfg.load(conjunto)
        gravedad = None
        if self.p("gravity"):
            try:
                from .gravity import GravityModel
                gravedad = GravityModel(verbose=False)
            except Exception as exc:
                self.get_logger().warn(
                    f"sin compensación de gravedad ({exc}). "
                    f"OJO: el conjunto '{g.set_name}' puede estar sintonizado "
                    f"CON ella; si es 'tuned_gff', usa 'tuned' en su lugar.")
        return H12Client(
            controlled=list(controlled if controlled is not None else ARM_INDICES),
            gains=g, channel=self.p("channel"), rate_hz=float(self.p("rate_hz")),
            verbose=True, dry_run=bool(self.p("dry_run")),
            legs_policy=self.p("legs"), gravity=gravedad)

    def tabla(self, cli, q0: dict, titulo: str = "resultado"):
        """Imprime de dónde salió cada articulación y dónde acabó."""
        print(f"\n  {'articulación':<20}{'salió de':>11}{'llegó a':>10}{'par':>9}")
        peor, quien = 0.0, None
        for i in sorted(q0):
            q = cli.q(i)
            print(f"    {BY_INDEX[i].name:<18}{math.degrees(q0[i]):>10.2f}°"
                  f"{math.degrees(q):>9.2f}°{cli.tau(i):>8.2f}N")
        deriva, idx = cli.leg_drift()
        if deriva > math.radians(2.0):
            print(f"\n  ⚠ las piernas se movieron: {BY_INDEX[idx].name} "
                  f"{math.degrees(deriva):.1f}°")
        if cli.collision_clamps:
            print(f"  ⚠ la protección de autocolisión actuó en "
                  f"{cli.collision_clamps} ciclos")
        print(f"\n  {cli.loop_health()}")


def ejecuta(clase, nombre: str, args=None):
    """Arranque y apagado comunes. Devuelve el código de salida."""
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = clase(nombre)
        return nodo.run()
    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n  interrumpido")
        return 130
    finally:
        if nodo is not None:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
