#!/usr/bin/env python3
"""Diagnóstico de SOLO LECTURA. No publica ningún comando: el robot no se toca.

Responde a las tres preguntas que hay que contestar antes de mover nada:

  1. ¿Llega el estado del robot, y a qué frecuencia?
  2. ¿Quién más está publicando en `/lowcmd`?  ← la causa de que un script de
     bajo nivel «no mueva nada» o «mueva pero vibrando».
  3. ¿En qué estado están los 27 motores (ángulo, par, temperatura)?

Uso:
    source scripts/env.sh
    python3 scripts/00_diagnose.py
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_hg.msg import LowCmd, LowState

from h1_2_joint_control import config as cfg
from h1_2_joint_control.joints import ARM_INDICES, BY_INDEX, NUM_CMD_MOTOR
from h1_2_joint_control.motion_switcher import MotionSwitcher

OK, WARN, BAD = "✔", "⚠", "✗"


def rule(title: str):
    print(f"\n\033[1m{title}\033[0m\n" + "─" * 78)


# NO usar `subprocess` con `ros2 topic hz`: al vencer el timeout, subprocess
# mata el proceso, el participante DDS no se despide y el dominio local se
# degrada durante ~10 s. Medido: /lowstate cae de 500 Hz a 62 Hz justo
# después, y tarda esos 10 s en recuperarse. Se mide todo aquí dentro.


class Counter:
    """Contador de mensajes con una ventana que se puede reiniciar."""

    def __init__(self):
        self.n = 0
        self.t0 = None

    def tick(self):
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.n += 1

    def reset(self):
        self.n, self.t0 = 0, None

    def rate(self) -> float:
        if self.t0 is None or self.n < 2:
            return -1.0
        el = time.monotonic() - self.t0
        return (self.n - 1) / el if el > 0 else -1.0


class StateProbe(Node):
    """Escucha /lowstate y /lowcmd. Solo lee: no publica nada."""

    def __init__(self):
        super().__init__("h1_2_diagnose")
        self.state = Counter()
        self.cmd = Counter()
        self.last: LowState | None = None
        self.create_subscription(LowState, "/lowstate", self._on_state,
                                 qos_profile_sensor_data)
        # Escuchar /lowcmd es la prueba directa de si hay otro controlador
        # mandando en los motores.
        self._cmd_sub = self.create_subscription(
            LowCmd, "/lowcmd", self._on_cmd, qos_profile_sensor_data)
        self.last_cmd: LowCmd | None = None

    def stop_watching_lowcmd(self):
        """/lowstate y /lowcmd van a 500 Hz cada uno. Escuchar los dos satura
        el ejecutor de un solo hilo y falsea a la baja la tasa de /lowstate
        (medido: 72 Hz en vez de 500). Una vez contado /lowcmd, se suelta."""
        if self._cmd_sub is not None:
            self.destroy_subscription(self._cmd_sub)
            self._cmd_sub = None

    def _on_state(self, msg: LowState):
        self.state.tick()
        self.last = msg

    def _on_cmd(self, msg: LowCmd):
        self.cmd.tick()
        self.last_cmd = msg

    def reset(self):
        self.state.reset()
        self.cmd.reset()

    def counts(self, topic: str) -> tuple[int, int]:
        return self.count_publishers(topic), self.count_subscribers(topic)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=5.0,
                    help="tiempo de escucha de /lowstate")
    ap.add_argument("--all-joints", action="store_true",
                    help="mostrar los 27 motores, no solo los 14 de los brazos")
    a = ap.parse_args()

    problems: list[str] = []

    # El sondeo de /lowstate se arranca ANTES que nada: el descubrimiento DDS
    # tarda, y así aprovecha el tiempo de las secciones 1 y 2.
    if not rclpy.ok():
        rclpy.init()
    probe = StateProbe()
    ex = SingleThreadedExecutor()
    ex.add_node(probe)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()

    def teardown():
        """En este orden, o el proceso muere dejando el participante DDS a
        medias y la siguiente ejecución no recibe /lowstate."""
        ex.shutdown()
        spin_thread.join(timeout=2.0)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------- tópicos
    rule("1. Canales DDS")
    print("  Escuchando 4 s…")
    probe.reset()
    time.sleep(4.0)
    lowcmd_hz = probe.cmd.rate()
    # Nuestros propios contadores no publican nada, así que lo que se vea en
    # /lowcmd viene entero del robot.
    for t in ("/lowstate", "/lowcmd", "/arm_sdk"):
        pubs, subs = probe.counts(t)
        print(f"  {t:<12} publicadores={pubs}  suscriptores={subs}")
    armsdk_pubs, armsdk_subs = probe.counts("/arm_sdk")
    probe.stop_watching_lowcmd()

    if lowcmd_hz > 1.0:
        print(f"\n  /lowcmd lleva tráfico a {lowcmd_hz:.0f} Hz y no lo ponemos nosotros:")
        print( "     es el servicio de control del robot. /lowcmd YA ESTÁ OCUPADO.")
        print( "     Un script que publique ahí a la vez se alterna con el servicio,")
        print( "     y el motor recibe consignas contradictorias -> la articulación")
        print( "     no llega a su referencia, o llega temblando.")
        problems.append(
            f"/lowcmd ocupado por el servicio del robot a {lowcmd_hz:.0f} Hz: ese "
            f"canal no sirve sin soltar antes el controlador de alto nivel")
    else:
        print(f"\n  {OK} /lowcmd está en silencio: nadie más manda por ahí.")

    if armsdk_pubs == 0 and armsdk_subs >= 1:
        print(f"\n  {OK} /arm_sdk: {armsdk_subs} suscriptor(es), 0 publicadores.")
        print( "     El servicio arm_sdk del robot escucha y nadie le habla.")
        print( "     Ese es el canal correcto para mover los brazos.")
    elif armsdk_subs == 0:
        print(f"\n  {BAD} /arm_sdk no tiene suscriptores: el servicio arm_sdk no")
        print( "     está activo en el robot. Por ese canal no se moverá nada.")
        problems.append("el servicio arm_sdk del robot no parece estar activo")

    # ------------------------------------------------- controlador de alto nivel
    rule("2. Controlador de alto nivel (motion_switcher)")
    ms = MotionSwitcher(executor=ex)
    code, mode = ms.check_mode()
    ms.close()
    if code is None:
        print(f"  {WARN} el servicio motion_switcher no responde.")
    elif code == 0:
        name = (mode or {}).get("name", "")
        form = (mode or {}).get("form", "")
        print(f"  CheckMode -> code=0  name='{name}'  form='{form}'")
        if name:
            print(f"  {WARN} hay un controlador activo ('{name}'). Es quien publica")
            print( "     en /lowcmd. Para usar ese canal habría que soltarlo")
            print( "     (ReleaseMode), lo que deja al robot sin equilibrarse.")
            print(f"  {OK} Con el canal arm_sdk NO hace falta: el controlador sigue")
            print( "     llevando las piernas y solo cede los brazos.")
        else:
            print(f"  {OK} sin controlador de alto nivel activo (modo debug o amortiguación).")
    else:
        print(f"  {WARN} CheckMode devolvió code={code}")

    # ------------------------------------------------------ estado de motores
    rule(f"3. Estado de los motores ({a.seconds:.0f} s de escucha de /lowstate)")
    # El sondeo lleva ya girando desde el principio del script. Con
    # spin_once() en el hilo principal la tasa medida saldría falseada a la
    # baja (~160 Hz); con un ejecutor en su propio hilo se leen los 500 Hz
    # reales, que es además como trabaja el cliente de control.
    probe.reset()
    time.sleep(a.seconds)
    rate = probe.state.rate()
    msg = probe.last

    if msg is None:
        print(f"  {BAD} no llega /lowstate. Comprueba el cable, la NIC y CYCLONEDDS_URI.")
        teardown()
        return 1

    print(f"  {OK} /lowstate a {rate:.0f} Hz   mode_machine={msg.mode_machine}  "
          f"mode_pr={getattr(msg, 'mode_pr', '?')}")

    shown = list(range(NUM_CMD_MOTOR)) if a.all_joints else list(ARM_INDICES)
    print(f"\n  {'idx':>3} {'articulación':<18} {'q (rad)':>9} {'q (°)':>8} "
          f"{'dq':>7} {'tau_est':>8} {'tau_max':>8} {'T °C':>5} {'modo':>5}")
    hot, loaded = [], []
    for i in shown:
        j = BY_INDEX[i]
        m = msg.motor_state[i]
        frac = abs(m.tau_est) / j.tau_max
        temp = m.temperature[0]
        mark = ""
        if frac > 0.5:
            mark += f"  {WARN} {frac*100:.0f} % del par"
            loaded.append(j.name)
        if temp > 60:
            mark += f"  {WARN} caliente"
            hot.append(j.name)
        print(f"  {i:>3} {j.name:<18} {m.q:>9.3f} {math.degrees(m.q):>8.1f} "
              f"{m.dq:>7.3f} {m.tau_est:>8.2f} {j.tau_max:>8.1f} {temp:>5} "
              f"{m.mode:>5}{mark}")

    modes = {msg.motor_state[i].mode for i in ARM_INDICES}
    print(f"\n  modos de los motores de los brazos: {sorted(modes)}  "
          f"(1 = habilitado, 0 = deshabilitado)")
    if modes == {0}:
        print(f"  {WARN} los motores de los brazos están deshabilitados.")
        problems.append("motores de brazos en modo 0 (deshabilitados)")

    if hot:
        problems.append(f"motores calientes: {', '.join(hot)}")
    if loaded:
        problems.append(f"motores por encima del 50 % de su par: {', '.join(loaded)}")

    # ---------------------------------------------------------- ganancias
    rule("4. Ganancias configuradas")
    g = cfg.load()
    print(f"  conjunto activo: '{g.set_name}' — {g.description.strip()}")
    print(f"\n  {'articulación':<18} {'kp':>7} {'kd':>6} {'tau_max':>8} "
          f"{'err@sat':>9}   (error de posición que satura el motor)")
    for i in ARM_INDICES:
        j = BY_INDEX[i]
        kp, kd = g.for_index(i)
        q_sat = j.tau_max / kp if kp > 0 else float("inf")
        print(f"  {j.name:<18} {kp:>7.1f} {kd:>6.2f} {j.tau_max:>8.1f} "
              f"{q_sat:>9.3f} rad ({math.degrees(q_sat):5.1f}°)")

    # ------------------------------------------------------------ veredicto
    rule("Veredicto")
    if problems:
        for p in problems:
            print(f"  {WARN} {p}")
    else:
        print(f"  {OK} nada que objetar.")
    print("\n  Siguiente paso recomendado:")
    print("     python3 scripts/01_hold.py --seconds 10")
    print("     (toma el control de los brazos por arm_sdk SIN moverlos)")

    teardown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
