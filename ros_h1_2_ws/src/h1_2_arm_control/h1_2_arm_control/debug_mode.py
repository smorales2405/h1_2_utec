#!/usr/bin/env python3
"""Suelta o recupera el controlador de alto nivel del robot.

Por qué hace falta. El controlador `ai` del robot publica en `/lowcmd` a
500 Hz. Si publicamos ahí a la vez, los dos mensajes se alternan y el motor
recibe consignas contradictorias: la articulación ve una fracción de la
ganancia que se le pide y vibra. Es exactamente el problema que aparecía con
los primeros scripts.

`enter` suelta ese controlador y deja `/lowcmd` libre. `status` solo mira.

    ros2 run h1_2_arm_control debug_mode --ros-args -p action:=status
    ros2 run h1_2_arm_control debug_mode --ros-args -p action:=enter
    ros2 run h1_2_arm_control debug_mode --ros-args -p action:=exit

CON EL ROBOT COLGADO DEL ARNÉS. Si está de pie sosteniéndose solo, al soltar
el controlador SE CAE. Paro de emergencia del mando: L2 + B.
"""
import sys

import rclpy
from rclpy.node import Node

from .motion_switcher import MotionSwitcher


BANNER = """
╔══════════════════════════════════════════════════════════════════════════╗
║  SOLTAR EL CONTROLADOR DE ALTO NIVEL                                     ║
║                                                                          ║
║  Después de esto NADIE manda en los 27 motores salvo este paquete.       ║
║  Si el robot está de pie sosteniéndose solo, SE CAE.                     ║
║                                                                          ║
║  Solo con el robot COLGADO DEL ARNÉS o firmemente sujeto.                ║
║  Mando a mano. Paro de emergencia: L2 + B.                               ║
╚══════════════════════════════════════════════════════════════════════════╝
"""


class DebugMode(Node):
    def __init__(self, nombre="h1_2_debug_mode"):
        super().__init__(nombre)
        self.declare_parameter("action", "status")
        self.declare_parameter("yes", False)

    def run(self) -> int:
        accion = self.get_parameter("action").value
        ms = MotionSwitcher()
        try:
            activo = ms.active_mode()
            if activo is None:
                print("  el servicio motion_switcher no responde.")
                return 1
            if accion == "status":
                print(f"\n  Controlador activo: "
                      f"{activo if activo else 'ninguno (modo debug)'}")
                if activo:
                    print(f"\n  Para tomar el control de los motores:\n"
                          f"      ros2 run h1_2_arm_control debug_mode "
                          f"--ros-args -p action:=enter")
                return 0
            if accion == "enter":
                if not activo:
                    print("  ya está en modo debug: /lowcmd es nuestro.")
                    return 0
                print(BANNER)
                if not self.get_parameter("yes").value:
                    try:
                        r = input("  Escribe 'si' para soltar el controlador: ")
                    except EOFError:
                        r = ""
                    if r.strip().lower() not in ("si", "sí", "s", "yes", "y"):
                        print("  cancelado.")
                        return 1
                ms.release_mode()
                print("  ✔ modo debug activo: /lowcmd es nuestro.")
                return 0
            if accion == "exit":
                ms.select_mode("ai")
                print("  ✔ controlador de alto nivel devuelto.")
                return 0
            print(f"  acción desconocida: '{accion}' (status | enter | exit)")
            return 1
        finally:
            ms.close()


def main(args=None):
    rclpy.init(args=args)
    n = DebugMode()
    try:
        sys.exit(n.run())
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
