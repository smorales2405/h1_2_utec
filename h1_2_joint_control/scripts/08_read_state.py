#!/usr/bin/env python3
"""Lee el estado de las articulaciones. SOLO LECTURA: no publica nada.

Se suscribe a `/lowstate`, imprime y sale. El robot no se entera: no hay
ningún publicador en este script, ni siquiera al tópico de pruebas.

    python3 scripts/08_read_state.py                  los 14 de los brazos
    python3 scripts/08_read_state.py --all            los 27 motores
    python3 scripts/08_read_state.py --joints L_elbow,R_elbow
    python3 scripts/08_read_state.py --watch          refresco continuo
    python3 scripts/08_read_state.py --json           para encadenar con otros scripts

Con `--watch` se refresca en el sitio hasta que se pulsa Ctrl-C; es cómodo para
mover un brazo a mano y ver los ángulos cambiar.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_hg.msg import LowState

from h1_2_joint_control.joints import ARM_INDICES, BY_INDEX, JOINTS, resolve


class Reader(Node):
    def __init__(self):
        super().__init__("h1_2_read_state")
        self.last: LowState | None = None
        self.n = 0
        self.create_subscription(LowState, "/lowstate", self._cb,
                                 qos_profile_sensor_data)

    def _cb(self, msg: LowState):
        self.last = msg
        self.n += 1

    def wait(self, timeout: float) -> LowState | None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.last is not None:
                return self.last
            time.sleep(0.01)
        return None


def table(msg: LowState, idxs: list[int]) -> list[str]:
    out = [f"  mode_machine={msg.mode_machine}  mode_pr={getattr(msg, 'mode_pr', '?')}",
           "",
           f"  {'idx':>3} {'articulación':<18} {'q (rad)':>9} {'q (°)':>8} "
           f"{'dq':>7} {'tau_est':>8} {'T °C':>5} {'modo':>5}"]
    for i in idxs:
        j = BY_INDEX[i]
        m = msg.motor_state[i]
        out.append(f"  {i:>3} {j.name:<18} {m.q:>9.3f} {math.degrees(m.q):>8.1f} "
                   f"{m.dq:>7.3f} {m.tau_est:>8.2f} {m.temperature[0]:>5} {m.mode:>5}")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joints", default="arms",
                    help="arms (por defecto), all, left_arm, wrist, L_elbow, 16…")
    ap.add_argument("--all", action="store_true", help="atajo de --joints all")
    ap.add_argument("--watch", action="store_true",
                    help="refrescar en el sitio hasta Ctrl-C")
    ap.add_argument("--hz", type=float, default=4.0, help="refresco de --watch")
    ap.add_argument("--json", action="store_true", help="salida en JSON")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="espera máxima al primer /lowstate")
    a = ap.parse_args()

    idxs = resolve("all" if a.all else a.joints)

    rclpy.init()
    node = Reader()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        msg = node.wait(a.timeout)
        if msg is None:
            print(f"✗ no llega /lowstate en {a.timeout:.0f} s.\n"
                  f"  · ¿El robot está encendido y el cable conectado?\n"
                  f"  · ¿Has hecho 'source scripts/env.sh' en esta terminal?\n"
                  f"  · Si acaba de morir mal algún proceso ROS, el dominio DDS\n"
                  f"    tarda ~10 s en recuperarse; vuelve a intentarlo.",
                  file=sys.stderr)
            return 1

        if a.json:
            print(json.dumps({
                "mode_machine": int(msg.mode_machine),
                "joints": {BY_INDEX[i].name: {
                    "idx": i,
                    "q": round(float(msg.motor_state[i].q), 6),
                    "q_deg": round(math.degrees(msg.motor_state[i].q), 3),
                    "dq": round(float(msg.motor_state[i].dq), 6),
                    "tau_est": round(float(msg.motor_state[i].tau_est), 4),
                    "temperature": int(msg.motor_state[i].temperature[0]),
                    "mode": int(msg.motor_state[i].mode),
                } for i in idxs},
            }, indent=2, ensure_ascii=False))
            return 0

        if not a.watch:
            print()
            for line in table(msg, idxs):
                print(line)
            # Vector listo para copiar y pegar, en el orden de los 14 del brazo
            # que usan get_current_dual_arm_q() y ctrl_dual_arm().
            if set(idxs) >= set(ARM_INDICES):
                q = [round(float(msg.motor_state[i].q), 4) for i in ARM_INDICES]
                print(f"\n  postura de los brazos (orden de H1_2_JointArmIndex):"
                      f"\n  {q}")
            print(f"\n  Solo lectura: no se ha publicado nada.")
            return 0

        # --watch: se reimprime encima con el cursor.
        # La salida va por un Event y no por KeyboardInterrupt a secas: bajo
        # rclpy, SIGTERM cierra el contexto pero NO interrumpe este bucle, y el
        # proceso se quedaría girando para siempre (`timeout` no podría con él).
        stop = threading.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, lambda *_: stop.set())
            except (ValueError, OSError):
                pass
        n_lines = 0
        period = 1.0 / max(a.hz, 0.5)
        print("\n  Ctrl-C para salir\n")
        while not stop.is_set() and rclpy.ok():
            lines = table(node.last, idxs)
            if n_lines:
                sys.stdout.write(f"\033[{n_lines}A")
            for line in lines:
                sys.stdout.write("\033[2K" + line + "\n")
            sys.stdout.flush()
            n_lines = len(lines)
            stop.wait(period)
        print("\n  fin.")
        return 0
    finally:
        # Este orden importa: destruir el nodo con el ejecutor girando deja el
        # participante DDS a medias y la SIGUIENTE ejecución no recibe nada.
        ex.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
