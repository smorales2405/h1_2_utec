#!/usr/bin/env python3
"""Entra y sale del modo debug del H1-2. Es la única puerta a `/lowcmd`.

    status   Solo mira: qué controlador está activo y si /lowcmd lleva tráfico.
    enter    ⚠ Suelta el controlador de alto nivel. A partir de ahí nadie
             publica en /lowcmd salvo nosotros.
    exit     Devuelve el mando al controlador 'ai'.

⚠ LEER ANTES DE `enter`

Soltar el controlador de alto nivel deja los motores sin nadie que los mande.
Si el robot está de pie sosteniéndose solo, **se cae**. Solo con el robot
COLGADO DEL ARNÉS o firmemente sujeto.

`status` dice además qué está mandando el servicio ahora mismo. Si sale
`kp = 0, kd = 0` en los 27 motores, el robot está en reposo con los motores
libres: soltar el servicio no cambia nada físicamente, porque ya no aplica par.
Ese es el caso menos arriesgado, y conviene comprobarlo antes de entrar.

Uso:
    python3 scripts/06_debug_mode.py status
    python3 scripts/06_debug_mode.py enter --yes
    python3 scripts/06_debug_mode.py exit
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_hg.msg import LowCmd

from h1_2_joint_control.joints import BY_INDEX, NUM_CMD_MOTOR
from h1_2_joint_control.motion_switcher import MotionSwitcher

OK, WARN = "✔", "⚠"

AVISO = """
╔══════════════════════════════════════════════════════════════════════════╗
║  SOLTAR EL CONTROLADOR DE ALTO NIVEL                                     ║
║                                                                          ║
║  Después de esto NADIE manda en los 27 motores salvo los scripts de      ║
║  este paquete. Si el robot está de pie sosteniéndose solo, SE CAE.       ║
║                                                                          ║
║  Solo con el robot COLGADO DEL ARNÉS o firmemente sujeto.                ║
║  Mando a mano. Paro de emergencia: L2 + B.                               ║
║                                                                          ║
║  Al terminar: python3 scripts/06_debug_mode.py exit                      ║
╚══════════════════════════════════════════════════════════════════════════╝
"""


class CmdPeek(Node):
    """Escucha /lowcmd para ver qué manda el servicio."""

    def __init__(self):
        super().__init__("h1_2_debug_mode")
        self.n = 0
        self.t0 = None
        self.last: LowCmd | None = None
        self.create_subscription(LowCmd, "/lowcmd", self._cb, qos_profile_sensor_data)

    def _cb(self, msg: LowCmd):
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.n += 1
        self.last = msg

    def reset(self):
        self.n, self.t0 = 0, None

    def rate(self) -> float:
        if self.t0 is None or self.n < 2:
            return 0.0
        return (self.n - 1) / max(time.monotonic() - self.t0, 1e-9)


def show_lowcmd(peek: CmdPeek):
    hz = peek.rate()
    print(f"\n  /lowcmd: {hz:.0f} Hz" + ("" if hz > 1 else "   (en silencio)"))
    m = peek.last
    if m is None:
        print("  Nadie ha publicado en /lowcmd durante la escucha.")
        return hz
    kps = [m.motor_cmd[i].kp for i in range(NUM_CMD_MOTOR)]
    kds = [m.motor_cmd[i].kd for i in range(NUM_CMD_MOTOR)]
    print(f"  Último comando visto: mode_pr={m.mode_pr} mode_machine={m.mode_machine}")
    print(f"    kp: mín {min(kps):.1f}  máx {max(kps):.1f}     "
          f"kd: mín {min(kds):.1f}  máx {max(kds):.1f}")
    if max(kps) == 0.0 and max(kds) == 0.0:
        print(f"  {OK} kp = kd = 0 en los 27 motores: el servicio NO aplica par.")
        print( "     Los motores están habilitados pero libres. Soltar el servicio")
        print( "     no cambia nada físicamente.")
    else:
        hot = [BY_INDEX[i].name for i in range(NUM_CMD_MOTOR)
               if m.motor_cmd[i].kp > 0]
        print(f"  {WARN} el servicio SÍ aplica par en {len(hot)} motor(es): "
              f"{', '.join(hot[:6])}{'…' if len(hot) > 6 else ''}")
        print( "     El robot se está sosteniendo. Soltarlo tiene consecuencias.")
    return hz


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["status", "enter", "exit"])
    ap.add_argument("--mode", default="ai", help="controlador al que volver en `exit`")
    ap.add_argument("--yes", action="store_true", help="no preguntar en `enter`")
    ap.add_argument("--seconds", type=float, default=3.0, help="escucha de /lowcmd")
    a = ap.parse_args()

    rclpy.init()
    peek = CmdPeek()
    ex = SingleThreadedExecutor()
    ex.add_node(peek)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    ms = MotionSwitcher(executor=ex)
    rc = 0
    try:
        name = ms.active_mode()
        if name is None:
            print(f"  {WARN} el servicio motion_switcher no responde.")
            return 1
        print(f"\n  Controlador activo: " + (f"'{name}'" if name else "ninguno (modo debug)"))

        peek.reset()
        time.sleep(a.seconds)
        show_lowcmd(peek)

        if a.cmd == "status":
            print("\n  Para tomar el control de los motores:  "
                  "python3 scripts/06_debug_mode.py enter")
            return 0

        if a.cmd == "enter":
            if not name:
                print(f"\n  {OK} ya estaba en modo debug. Nada que hacer.")
                return 0
            print(AVISO)
            if not a.yes:
                try:
                    r = input("  Escribe 'si' para soltar el controlador: ").strip().lower()
                except EOFError:
                    r = ""
                if r not in ("si", "sí", "s", "yes", "y"):
                    print("  cancelado.")
                    return 1
            print("\n  Soltando…")
            ok, detail = ms.enter_debug_mode()
            print(f"  {OK if ok else WARN} " +
                  ("modo debug activo." if ok else f"no se pudo: {detail}"))
            peek.reset()
            time.sleep(a.seconds)
            hz = peek.rate()
            if hz < 1:
                print(f"  {OK} /lowcmd en silencio: el canal es nuestro.")
            else:
                print(f"  {WARN} /lowcmd sigue a {hz:.0f} Hz. NO usar ese canal todavía.")
                ok = False
            if ok:
                print("\n  Siguiente: python3 scripts/01_hold.py --channel lowcmd --seconds 10")
                print("  Al terminar: python3 scripts/06_debug_mode.py exit")
            rc = 0 if ok else 2

        elif a.cmd == "exit":
            print(f"\n  Devolviendo el mando a '{a.mode}'…")
            ok, detail = ms.exit_debug_mode(a.mode)
            print(f"  {OK if ok else WARN} " +
                  (f"controlador '{detail}' activo." if ok else f"no se pudo: {detail}"))
            rc = 0 if ok else 2
        return rc
    finally:
        ms.close()
        ex.shutdown()
        spin.join(timeout=2.0)
        peek.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
