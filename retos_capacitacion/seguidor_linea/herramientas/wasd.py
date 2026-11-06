#!/usr/bin/env python3
"""Teleoperacion WASD del H1-2 desde el teclado, por SSH.

Se ejecuta EN EL ROBOT (PC2). Manda velocidades por el LocoClient (RPC) y lee
rt/lowstate para vigilar juntas, IMU y modo.

    /home/unitree/unitree_sdk2_python/.venv/bin/python wasd.py --iface eth0

ANTES DE USARLO
---------------
* El robot tiene que estar en un estado que camine: **FSM 201** (L2+UP en el mando).
  El SDK NO puede cambiar la FSM; eso es competencia exclusiva del mando.
* **L2+B es la parada de emergencia.** Tenerlo a mano siempre.
* Espacio libre alrededor: el controlador de Unitree no evita obstaculos.

SEGURIDAD QUE TRAE
------------------
* Cada orden se manda con duration=1 s, asi que **si el script muere el robot
  para solo en 1 segundo**. Es la parada de seguridad gratis del propio firmware.
* Sin tecla durante --soltar segundos, la velocidad cae a cero (el terminal no
  da eventos de "tecla soltada", se usa la autorrepeticion).
* Vigila motor_state[i].motorstate: si alguno sale distinto de 0, para y avisa.

TECLAS
------
    w / s     adelante / atras
    a / d     desplazamiento lateral izquierda / derecha
    q / e     girar izquierda / derecha
    espacio   parada inmediata
    + / -     subir / bajar la escala de velocidad
    r         reiniciar el contador de rumbo
    ?         ayuda
    x / Ctrl-C  salir (parando el robot)
"""

import argparse
import math
import select
import signal
import sys
import termios
import time
import tty

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

TOPIC_LOWSTATE = "rt/lowstate"

# Topes de velocidad. El ejemplo oficial camina a vx=0.3; estos son los maximos
# que deja pedir la escala, no lo que se manda de entrada.
VX_MAX = 0.6
VY_MAX = 0.3
VYAW_MAX = 0.5


class Estado:
    """Lo ultimo que se ha leido de rt/lowstate."""

    def __init__(self):
        self.msg = None
        self.n = 0
        self.t_primero = None
        self.yaw0 = None

    def al_recibir(self, msg: LowState_):
        self.msg = msg
        self.n += 1
        if self.t_primero is None:
            self.t_primero = time.time()

    def hz(self):
        if self.t_primero is None or self.n < 2:
            return 0.0
        dt = time.time() - self.t_primero
        return self.n / dt if dt > 0 else 0.0

    def yaw(self):
        """Rumbo en grados a partir del cuaternio del IMU."""
        if self.msg is None:
            return None
        w, x, y, z = self.msg.imu_state.quaternion
        return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                       1.0 - 2.0 * (y * y + z * z)))

    def rumbo_relativo(self):
        y = self.yaw()
        if y is None:
            return None
        if self.yaw0 is None:
            self.yaw0 = y
        d = y - self.yaw0
        return (d + 180.0) % 360.0 - 180.0

    def marcar_rumbo(self):
        self.yaw0 = self.yaw()

    def fallos(self):
        """Indices de motor con motorstate != 0."""
        if self.msg is None:
            return []
        return [i for i, m in enumerate(self.msg.motor_state[:27]) if m.motorstate != 0]

    def temp_max(self):
        if self.msg is None:
            return None
        temps = [m.temperature[0] for m in self.msg.motor_state[:27]]
        return max(temps) if temps else None


def leer_teclas():
    """Todas las teclas pendientes en stdin, sin bloquear."""
    teclas = []
    while select.select([sys.stdin], [], [], 0)[0]:
        c = sys.stdin.read(1)
        if not c:
            break
        teclas.append(c)
    return teclas


def ayuda():
    return ("  w/s adelante-atras   a/d lateral   q/e girar   "
            "espacio parar   +/- escala   r marcar rumbo   x salir")


def main():
    ap = argparse.ArgumentParser(description="Teleoperacion WASD del H1-2")
    ap.add_argument("--iface", default="eth0",
                    help="interfaz de red del DDS (la del robot, no la wifi)")
    ap.add_argument("--vx", type=float, default=0.3, help="velocidad adelante/atras (m/s)")
    ap.add_argument("--vy", type=float, default=0.2, help="velocidad lateral (m/s)")
    ap.add_argument("--vyaw", type=float, default=0.4, help="velocidad de giro (rad/s)")
    ap.add_argument("--hz", type=float, default=10.0, help="frecuencia de envio")
    ap.add_argument("--soltar", type=float, default=0.4,
                    help="segundos sin tecla antes de parar")
    args = ap.parse_args()

    if not sys.stdin.isatty():
        sys.exit("Esto necesita un terminal interactivo (ssh -t).")

    ChannelFactoryInitialize(0, args.iface)

    estado = Estado()
    sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
    sub.Init(estado.al_recibir, 10)

    # Sin lowstate no hay robot al otro lado: mejor saberlo antes de mandar nada.
    t0 = time.time()
    while estado.msg is None and time.time() - t0 < 3.0:
        time.sleep(0.05)
    if estado.msg is None:
        sys.exit(f"No llega {TOPIC_LOWSTATE} por {args.iface}. "
                 "¿Robot encendido? ¿Interfaz correcta?")

    print(f"lowstate OK  ·  mode_machine={estado.msg.mode_machine}  "
          f"mode_pr={estado.msg.mode_pr}  motores={len(estado.msg.motor_state)}")
    fallos = estado.fallos()
    if fallos:
        sys.exit(f"Hay motores en fallo antes de empezar: {fallos}. No se arranca.")

    cliente = LocoClient()
    cliente.SetTimeout(10.0)
    cliente.Init()

    print("\n  El robot debe estar en FSM 201 (L2+UP en el mando).")
    print("  L2+B es la parada de emergencia.")
    print(ayuda())
    try:
        input("\n  ENTER para empezar, Ctrl-C para salir: ")
    except (EOFError, KeyboardInterrupt):
        return

    escala = 1.0
    vx = vy = vyaw = 0.0
    t_tecla = 0.0
    salir = False
    aviso = ""

    fd = sys.stdin.fileno()
    previo = termios.tcgetattr(fd)
    periodo = 1.0 / args.hz

    def parar(*_):
        nonlocal salir
        salir = True

    signal.signal(signal.SIGINT, parar)
    signal.signal(signal.SIGTERM, parar)

    try:
        tty.setcbreak(fd)
        while not salir:
            ciclo = time.time()

            for c in leer_teclas():
                cb = c.lower()
                if cb == "w":
                    vx, vy, vyaw = args.vx, 0.0, 0.0
                elif cb == "s":
                    vx, vy, vyaw = -args.vx, 0.0, 0.0
                elif cb == "a":
                    vx, vy, vyaw = 0.0, args.vy, 0.0
                elif cb == "d":
                    vx, vy, vyaw = 0.0, -args.vy, 0.0
                elif cb == "q":
                    vx, vy, vyaw = 0.0, 0.0, args.vyaw
                elif cb == "e":
                    vx, vy, vyaw = 0.0, 0.0, -args.vyaw
                elif c == " ":
                    vx = vy = vyaw = 0.0
                elif c in "+=":
                    escala = min(2.0, escala + 0.1)
                elif c in "-_":
                    escala = max(0.1, escala - 0.1)
                elif cb == "r":
                    estado.marcar_rumbo()
                elif cb == "?":
                    aviso = ayuda()
                elif cb == "x":
                    salir = True
                else:
                    continue
                if cb in "wsadqe ":
                    t_tecla = time.time()

            # El terminal no avisa de teclas soltadas: se para solo por silencio.
            if time.time() - t_tecla > args.soltar:
                vx = vy = vyaw = 0.0

            fallos = estado.fallos()
            if fallos:
                vx = vy = vyaw = 0.0
                aviso = f"MOTOR EN FALLO {fallos} - parando"
                salir = True

            ex = max(-VX_MAX, min(VX_MAX, vx * escala))
            ey = max(-VY_MAX, min(VY_MAX, vy * escala))
            eyaw = max(-VYAW_MAX, min(VYAW_MAX, vyaw * escala))

            # duration=1 s: si esto muere, el robot para solo en 1 segundo.
            cliente.Move(ex, ey, eyaw)

            rumbo = estado.rumbo_relativo()
            temp = estado.temp_max()
            linea = (f"\rvx {ex:+.2f}  vy {ey:+.2f}  w {eyaw:+.2f}  "
                     f"| escala {escala:.1f}  | rumbo {rumbo:+6.1f}d  "
                     f"| {estado.hz():5.0f} Hz  | Tmax {temp}C   {aviso[:40]:<40}")
            sys.stdout.write(linea)
            sys.stdout.flush()

            resto = periodo - (time.time() - ciclo)
            if resto > 0:
                time.sleep(resto)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previo)
        try:
            cliente.StopMove()
        except Exception as e:
            print(f"\nNo se pudo mandar StopMove: {e}")
        print("\nParado. El robot se detiene solo 1 s despues del ultimo comando.")


if __name__ == "__main__":
    main()
