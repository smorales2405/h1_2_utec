#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Selector numerado de rutinas JSON para el Unitree H1-2 REAL: brazos Y MANOS.
#
# Parte del selector de code_cap (selector_poses_real/.../h1_2_robot_selector.py),
# que no se toca, y le añade las manos Inspire RH56DFTP. Los brazos van igual:
#   - Dominio DDS 0 sobre la interfaz del robot (eth0), no el 1 del simulador.
#   - Comprueba el modo Debug en dos pasos: CheckMode del MotionSwitcher tiene
#     que dar name '' (si da 'ai', ofrece ReleaseMode() tras teclear SOLTAR; con
#     'ai' el robot ignora nuestros rt/lowcmd sin dar error), y además escucha
#     rt/lowcmd 3 s y, si alguien más publica, NO arranca.
#   - Pide confirmación tecleada antes de mandar el primer comando.
#   - Recorta cada objetivo a la intersección de los límites de la web de
#     Unitree y del URDF, con 0.05 rad de margen.
#   - El primer movimiento dura al menos 3 s, desde la postura medida.
#   - Si un motor reporta fallo (motorstate != 0), pasa a amortiguación.
#   - Al salir no deja los motores rígidos: rampa de kp a 0 con kd (amortiguación).
#
# Las manos (Modbus TCP, puerto 6000; izquierda .211, derecha .210):
#   - Cada paso puede traer "manos": {"izq": {...}, "der": {...}} con las 6 juntas
#     actuadas del URDF en radianes (0 = abierta). Se pasan a ANGLE_SET con
#     conversion_angle_set.py, que corrige la no linealidad de la mano.
#   - Durante el paso se manda ANGLE_SET interpolado a HZ_MANO, desde lo último
#     mandado: los dedos llegan a la vez que el brazo, por el mismo camino que en
#     el editor de MuJoCo. El gesto 'cerrada' va por fases, en el orden de puno()
#     (gestos_mano.py), esperando a que cada una termine.
#   - Si un dedo pasa de FMAX_MANO gf o la mano da err, ese dedo se abre y la mano
#     deja de recibir órdenes hasta el final (los brazos siguen).
#   - Al salir, o si el robot pasa a amortiguación, las manos se abren.
#   - Los pasos sin "manos" las dejan como están: las rutinas del selector de
#     code_cap valen tal cual (solo brazos).
#
# La POSE SEGURA (poses/0_pose_segura.json, manos abiertas) es la postura por
# defecto: el selector va a ella al empezar (tras MOVER), la añade al final de
# cada rutina que no termine en ella (no hace falta guardarla en la rutina) y se
# queda sujetándola entre rutinas. Solo al salir pasa a amortiguación.
#
# REQUISITOS FÍSICOS, antes de lanzarlo:
#   1. Robot COLGADO del arnés, con los pies sin tocar el suelo. En Debug el
#      robot no se equilibra: las piernas se quedan en la postura inicial.
#   2. Modo Debug: L2+R2 en el mando (L2+A debe dar la postura de diagnóstico).
#   3. Alguien con el mando en la mano: L2+B es la parada de emergencia.
#   4. Ningún otro programa mandando a las manos ni a los brazos (selector_manos.sh
#      lo comprueba): ni el driver de manos del teleop ni el servicio h1_2_six_seven.
#
# Se ejecuta EN EL PC2 del robot, con un python que tenga unitree_sdk2py y pymodbus
# (~/teleop_venv):
#   python h1_2_robot_selector_manos.py                   # eth0, rutinas de poses/
#   python h1_2_robot_selector_manos.py eth0 /ruta/a/rutinas
#   python h1_2_robot_selector_manos.py --sin-manos       # solo brazos
#   python h1_2_robot_selector_manos.py lo --sin-manos    # contra unitree_mujoco (no tiene manos)
#
# Comandos:
#   número = ejecutar rutina
#   l      = listar rutinas otra vez
#   x      = salir: vuelve a la pose segura, abre las manos y suelta en amortiguación
#            (entre rutinas no: se queda en la pose segura)
# -----------------------------------------------------------------------------

from __future__ import annotations

import argparse
import time
import sys
import json
import re
import struct
import threading
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gestos_mano as gm                                                  # noqa: E402
from conversion_angle_set import CLAVES, rad_a_angle_set, angle_set_a_rad  # noqa: E402


H1_2_NUM_MOTOR = 27

# Ganancias del ejemplo oficial h1_27dof_example.cpp, por tipo de reductor.
Kp = [
    100, 100, 100, 200, 80, 80,
    100, 100, 100, 200, 80, 80,
    100,
    80, 80, 80, 80, 80, 80, 80,
    80, 80, 80, 80, 80, 80, 80
]

Kd = [
    3, 3, 3, 5, 2, 2,
    3, 3, 3, 5, 2, 2,
    3,
    2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2
]

# Límites del tren superior: intersección de la web (About_H1-2) y del URDF
# h1_2_handless, que no coinciden (ver docs/CAPACITACION_H1_2.md, §5).
# El lado derecho es el espejo del izquierdo en roll, yaw y wrist_roll.
# El editor de MuJoCo (h1_2_mujoco/) lee LIMITES y MARGEN de aquí.
LIMITES = {
    12: (-2.35, 1.57),      # torso
    13: (-3.14, 1.57),      # shoulder pitch
    14: (-0.38, 3.40),      # shoulder roll
    15: (-2.66, 2.66),      # shoulder yaw
    16: (-0.95, 1.60),      # elbow
    17: (-2.967, 2.75),     # wrist roll
    18: (-0.4625, 0.349),   # wrist pitch
    19: (-1.012, 1.012),    # wrist yaw
    20: (-3.14, 1.57),
    21: (-3.40, 0.38),
    22: (-2.66, 2.66),
    23: (-0.95, 1.60),
    24: (-2.75, 2.967),
    25: (-0.4625, 0.349),
    26: (-1.012, 1.012),
}
MARGEN = 0.05

DURACION_MINIMA_PRIMER_MOVIMIENTO = 3.0
T_SOLTAR = 2.0
# Pose segura: la postura por defecto (al empezar, al final de cada rutina y al salir). El editor de MuJoCo
# lee estas dos constantes de aquí.
DUR_POSE_SEGURA = 3.0      # s para ir a ella
TOL_POSE_SEGURA = 0.01     # rad: un paso "es" la pose segura si brazos y dedos están a menos de esto

# Manos Inspire RH56DFTP por Modbus TCP (registros: code_cap/manos/README.md).
IP_MANO = {"izq": "192.168.124.211", "der": "192.168.124.210"}   # al revés que la doc de Unitree
PUERTO_MANO = 6000
ANGLE_SET, SPEED_SET, ANGLE_ACT, FORCE_ACT, ERR_MANO = 1486, 1522, 1546, 1582, 1606
VELOCIDAD_MANO = 1000      # SPEED_SET durante la sesión: la rampa la marca el ANGLE_SET interpolado
HZ_MANO = 25               # ANGLE_SET por segundo y mano
FMAX_MANO = 800            # gf: por encima, el dedo empuja algo (el mismo corte que puno() en caja_cuadrado.py)
TOL_LLEGADA = 40           # unidades de ANGLE_SET: fase de 'cerrada' terminada (la rotación no pasa de ~973)
T_LLEGADA_MAX = 2.0        # s de espera, como mucho, al final de cada fase de 'cerrada'
T_ABRIR = 2.0              # s para abrir las manos al salir


class Mode:
    PR = 0
    AB = 1


def recortar(j, q):
    lo, hi = LIMITES[j]
    return min(max(q, lo + MARGEN), hi - MARGEN)


class Mano:
    """Una mano Inspire por Modbus TCP. Al crearla solo LEE; preparar() fija la velocidad.
    tramo() lleva la mano a una postura (radianes del URDF) mandando ANGLE_SET interpolado."""

    def __init__(self, lado: str, ip: str, puerto: int):
        from pymodbus.client import ModbusTcpClient
        self.lado, self.ip = lado, ip
        self.c = ModbusTcpClient(ip, port=puerto, timeout=3)
        if not self.c.connect():
            raise ConnectionError(f"no conecta con la mano {lado} ({ip}:{puerto})")
        self.cerrojo = threading.Lock()
        self.detenida = ""             # motivo, si la protección la paró
        self.cancelar = False
        self.primer_movimiento = True
        self.velocidad_original = None
        self.t_err = 0.0
        err = self.errores()
        if any(err):
            raise RuntimeError(f"la mano {lado} tiene err {err}: manos.sh borrar-error {lado}, y repite")
        self.q_cmd = dict(zip(CLAVES, angle_set_a_rad(self.regs(ANGLE_ACT, 6))))

    # ------------------------------------------------- Modbus (igual que la clase Mano de caja_cuadrado.py)
    def regs(self, a, n, signo=True):
        with self.cerrojo:
            r = self.c.read_holding_registers(a, n, 1)
        if r.isError():
            raise IOError(f"mano {self.lado}: error leyendo el registro {a}: {r}")
        f = "h" if signo else "H"
        return list(struct.unpack(f">{n}{f}", struct.pack(f">{n}H", *r.registers)))

    def escribir(self, a, v):
        with self.cerrojo:
            r = self.c.write_registers(a, [int(x) & 0xFFFF for x in v], 1)
        if r.isError():
            raise IOError(f"mano {self.lado}: error escribiendo el registro {a}: {r}")

    def errores(self):
        """err de los 6 DOF (1 byte por DOF); se lee aparte: en bloque con otros da basura."""
        return [int(x) & 0xFF for r in self.regs(ERR_MANO, 3, False) for x in (r >> 8, r)]

    # ------------------------------------------------- órdenes
    def preparar(self):
        self.velocidad_original = self.regs(SPEED_SET, 6, False)
        self.escribir(SPEED_SET, [VELOCIDAD_MANO] * 6)

    def restaurar(self):
        if self.velocidad_original is not None:
            self.escribir(SPEED_SET, self.velocidad_original)

    def cerrar(self):
        with self.cerrojo:
            self.c.close()

    def enviar(self, q) -> bool:
        """Manda la postura q y vigila fuerza y err. False si la protección paró la mano."""
        self.escribir(ANGLE_SET, rad_a_angle_set(q))
        f = self.regs(FORCE_ACT, 6)
        malos = [d for d in range(6) if f[d] > FMAX_MANO]
        if malos:
            self.escribir(ANGLE_SET, [1000 if d in malos else -1 for d in range(6)])
            self.detenida = (f"{[gm.NOMBRE_DOF[CLAVES[d]] for d in malos]} con {[f[d] for d in malos]} gf "
                             f"> {FMAX_MANO}: se abren")
            return False
        if time.monotonic() - self.t_err > 0.5:
            self.t_err = time.monotonic()
            err = self.errores()
            if any(err):
                self.detenida = f"err {err} (manos.sh borrar-error {self.lado})"
                return False
        return True

    def esperar_llegada(self, objetivo) -> bool:
        meta = rad_a_angle_set(objetivo)
        t0 = time.monotonic()
        while time.monotonic() - t0 < T_LLEGADA_MAX and not self.cancelar:
            act = self.regs(ANGLE_ACT, 6)
            if max(abs(act[d] - meta[d]) for d in range(6)) <= TOL_LLEGADA:
                return True
            if not self.enviar(objetivo):
                return False
            time.sleep(1.0 / HZ_MANO)
        if not self.cancelar:
            act = self.regs(ANGLE_ACT, 6)
            self.detenida = f"una fase de 'cerrada' no terminó en {T_LLEGADA_MAX:.0f} s (ANGLE_ACT {act}, objetivo {meta})"
        return False

    def tramo(self, q_fin, duracion):
        """Lleva la mano a q_fin en `duracion` s, desde lo último mandado. 'cerrada', por fases."""
        try:
            if self.detenida:
                return
            q_fin = gm.normalizar(q_fin, self.q_cmd)
            if self.primer_movimiento:
                duracion = max(duracion, DURACION_MINIMA_PRIMER_MOVIMIENTO)
                self.primer_movimiento = False
            fases = gm.fases(self.q_cmd, q_fin)
            secuencia = gm.nombre_gesto(q_fin) == "cerrada"
            for objetivo, tf in zip(fases, gm.duraciones(duracion, len(fases))):
                q0, t0 = dict(self.q_cmd), time.monotonic()
                while not self.cancelar:
                    s = 1.0 if tf <= 0 else min(1.0, (time.monotonic() - t0) / tf)
                    q = gm.interpolar(q0, objetivo, s)
                    if not self.enviar(q):
                        return
                    self.q_cmd = q
                    if s >= 1.0:
                        break
                    time.sleep(1.0 / HZ_MANO)
                if self.cancelar or (secuencia and not self.esperar_llegada(objetivo)):
                    return
        except Exception as e:                  # el hilo no debe morir en silencio
            self.detenida = f"{type(e).__name__}: {e}"


class H1_2RobotSelector:

    def __init__(self, poses_dir: Path, control_dt: float = 0.002):
        self.poses_dir = poses_dir
        self.control_dt = control_dt
        self.crc = CRC()

        self.lowcmd_publisher_ = None
        self.lowstate_subscriber = None
        self.low_state = None
        self.t_low_state = 0.0
        self.mode_machine_ = 0
        self._writer_thread = None
        self._cerrojo = threading.Lock()

        self.controlled_joints = list(range(12, 27))

        self.target_pos = {i: 0.0 for i in range(H1_2_NUM_MOTOR)}
        self.q_init = {i: 0.0 for i in range(H1_2_NUM_MOTOR)}
        self.current_cmd_pos = {i: 0.0 for i in range(H1_2_NUM_MOTOR)}
        self.hold_pos = {i: 0.0 for i in range(H1_2_NUM_MOTOR)}

        self.t = 0.0
        self.T = 1.0
        self.primer_movimiento = True

        # Factor de 1 a 0 sobre kp: 1 = control normal, 0 = solo amortiguación.
        self.factor_kp = 1.0
        self.amortiguado = False
        self.motivo_amortiguacion = ""

        self.manos = {}                 # lado -> Mano; vacío con --sin-manos
        self._hilos_manos = []
        self._aviso_sin_manos = False

    # ---------------------------------------------------------
    # Comunicación
    # ---------------------------------------------------------

    def Init(self):
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def comprobar_modo_debug(self, segundos=3.0):
        """Cuenta los rt/lowcmd de OTROS antes de crear nuestro publicador."""
        n = [0]
        kp_max = [0.0]

        def al_recibir(msg: LowCmd_):
            n[0] += 1
            kp_max[0] = max(kp_max[0], max(msg.motor_cmd[i].kp for i in range(H1_2_NUM_MOTOR)))

        espia = ChannelSubscriber("rt/lowcmd", LowCmd_)
        espia.Init(al_recibir, 10)
        print(f"[INFO] Escuchando rt/lowcmd {segundos:.0f} s para comprobar el modo Debug...")
        time.sleep(segundos)
        espia.Close()

        if n[0] > 0:
            print(f"[ERROR] Hay otro controlador en rt/lowcmd: {n[0]} mensajes "
                  f"(~{n[0] / segundos:.0f} Hz), kp max = {kp_max[0]:.0f}.")
            print("        El robot NO está en modo Debug. Pon L2+R2 en el mando y repite.")
            return False

        print("[OK] Nadie publica en rt/lowcmd.")
        return True

    @staticmethod
    def comprobar_servicio_movimiento(intentos=3):
        """El servicio de movimiento de Unitree ('ai') tiene que estar liberado.

        Que nadie publique en rt/lowcmd NO basta: el 2026-09-25 el robot estaba en FSM 0
        con rt/lowcmd en silencio, pero CheckMode seguía dando 'ai' y el robot ignoraba
        nuestros rt/lowcmd sin dar error. El Debug de verdad es CheckMode con name ''.
        """
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        msc = MotionSwitcherClient()
        msc.SetTimeout(3.0)
        msc.Init()
        status, result = msc.CheckMode()
        nombre = (result or {}).get("name")
        print(f"[INFO] CheckMode: {status} {result}")
        if status == 0 and not nombre:
            print("[OK] Servicio de movimiento liberado: modo Debug confirmado.")
            return True

        print(f"[AVISO] El servicio de movimiento '{nombre}' sigue activo: el robot ignoraría los rt/lowcmd.")
        print("        ReleaseMode() lo libera y las PIERNAS SE SUELTAN: el robot tiene que estar COLGADO.")
        if input("        Escribe SOLTAR para llamar a ReleaseMode() (cualquier otra cosa sale): ").strip() != "SOLTAR":
            return False
        for n in range(1, intentos + 1):
            msc.ReleaseMode()
            time.sleep(1.0)
            status, result = msc.CheckMode()
            nombre = (result or {}).get("name")
            print(f"[INFO] ReleaseMode {n}/{intentos} -> CheckMode {status} {result}")
            if status == 0 and not nombre:
                print("[OK] Servicio de movimiento liberado: modo Debug confirmado.")
                return True
        print(f"[ERROR] El modo sigue siendo '{nombre}'. No se arranca.")
        return False

    def CrearPublicador(self):
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        self.t_low_state = time.monotonic()
        if hasattr(msg, "mode_machine"):
            self.mode_machine_ = msg.mode_machine

    def wait_lowstate(self, timeout=8.0):
        print("[INFO] Esperando rt/lowstate...")
        start = time.time()

        while self.low_state is None:
            if time.time() - start > timeout:
                print("[ERROR] LowState no recibido dentro del timeout.")
                print("Verifica que el cuerpo del robot esté encendido y la interfaz sea eth0.")
                return False
            time.sleep(0.05)

        for i in range(H1_2_NUM_MOTOR):
            q0 = float(self.low_state.motor_state[i].q)
            self.hold_pos[i] = q0
            self.q_init[i] = q0
            self.target_pos[i] = q0
            self.current_cmd_pos[i] = q0

        fallos = [i for i in range(H1_2_NUM_MOTOR) if self.low_state.motor_state[i].motorstate != 0]
        temps = [self.low_state.motor_state[i].temperature[0] for i in range(H1_2_NUM_MOTOR)]

        print(f"\n[ESTADO] mode_machine={self.mode_machine_}  mode_pr={self.low_state.mode_pr}  "
              f"T motor max={max(temps)} C")
        if fallos:
            print(f"[ERROR] Motores con fallo (motorstate != 0): {fallos}. No se arranca.")
            return False

        print("[POSE INICIAL - TORSO Y BRAZOS]")
        for j in self.controlled_joints:
            print(f'"{j}": {float(self.low_state.motor_state[j].q):.4f},')
        print("[FIN POSE INICIAL]\n")
        return True

    # ---------------------------------------------------------
    # Manos
    # ---------------------------------------------------------

    def conectar_manos(self, ips: dict, puerto: int):
        """Solo lee: conexión, err y postura de cada mano. False si alguna falla."""
        for lado, ip in ips.items():
            try:
                mano = Mano(lado, ip, puerto)
            except Exception as e:
                print(f"[ERROR] Mano {lado}: {e}")
                for m in self.manos.values():
                    m.cerrar()
                self.manos = {}
                return False
            self.manos[lado] = mano
            print(f"[OK] Mano {lado} ({ip}): sin err, ANGLE_ACT {rad_a_angle_set(mano.q_cmd)}.")
        return True

    def lanzar_manos(self, manos: dict | None, duracion: float):
        """Un hilo por mano con postura en el paso; corren a la vez que el brazo."""
        self.esperar_manos()            # si un paso anterior se cortó, que no manden dos hilos a la vez
        if not manos:
            return
        if not self.manos:
            if not self._aviso_sin_manos:
                print("     [MANOS] la rutina trae manos, pero el selector va sin ellas (--sin-manos).")
                self._aviso_sin_manos = True
            return
        for lado, q in manos.items():
            mano = self.manos.get(lado)
            if mano is None:
                continue
            if mano.detenida:
                print(f"     [MANOS] {lado} parada ({mano.detenida}): no se mueve.")
                continue
            hilo = threading.Thread(target=mano.tramo, args=(q, duracion), daemon=True)
            hilo.start()
            self._hilos_manos.append((mano, hilo))

    def esperar_manos(self):
        for mano, hilo in self._hilos_manos:
            hilo.join()
            if mano.detenida:
                print(f"     [MANOS] {mano.lado} PARADA: {mano.detenida}")
        self._hilos_manos = []

    def abrir_manos(self):
        """Cancela lo que estén haciendo y las abre en T_ABRIR s (aunque la protección las parara)."""
        for mano, hilo in self._hilos_manos:
            mano.cancelar = True
        for mano, hilo in self._hilos_manos:
            hilo.join(timeout=5.0)
        self._hilos_manos = []
        hilos = []
        for mano in self.manos.values():
            mano.cancelar, mano.detenida = False, ""
            hilo = threading.Thread(target=mano.tramo, args=(gm.gesto("abierta"), T_ABRIR), daemon=True)
            hilo.start()
            hilos.append((mano, hilo))
        for mano, hilo in hilos:
            hilo.join(timeout=T_ABRIR + 10.0)
            if mano.detenida:
                print(f"[WARN] Mano {mano.lado}: no se pudo abrir del todo ({mano.detenida}).")

    def cerrar_manos(self):
        for mano in self.manos.values():
            try:
                mano.restaurar()
            except Exception as e:
                print(f"[WARN] Mano {mano.lado}: no se pudo restaurar SPEED_SET ({e}).")
            mano.cerrar()

    # ---------------------------------------------------------
    # Interpolación y envío LowCmd
    # ---------------------------------------------------------

    def interpolate_position(self, q_init, q_target):
        if self.T <= 0:
            return q_target
        s = max(0.0, min(self.t / self.T, 1.0))
        return q_init + (q_target - q_init) * s

    def pasar_a_amortiguacion(self, motivo):
        if not self.amortiguado:
            self.amortiguado = True
            self.motivo_amortiguacion = motivo
            print(f"\n[PARADA] {motivo} -> amortiguación (kp=0).")

    def LowCmdWrite(self):
        if self.low_state is None:
            return

        # Vigilancia: estado viejo o motor en fallo -> amortiguación.
        if time.monotonic() - self.t_low_state > 0.5:
            self.pasar_a_amortiguacion("rt/lowstate lleva más de 0.5 s sin llegar")
        else:
            fallos = [i for i in range(H1_2_NUM_MOTOR)
                      if self.low_state.motor_state[i].motorstate != 0]
            if fallos:
                self.pasar_a_amortiguacion(f"motor con fallo: {fallos}")

        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = Mode.PR
        cmd.mode_machine = self.mode_machine_

        factor = 0.0 if self.amortiguado else self.factor_kp

        for i in range(H1_2_NUM_MOTOR):
            cmd.motor_cmd[i].mode = 1
            cmd.motor_cmd[i].kp = Kp[i] * factor
            cmd.motor_cmd[i].kd = Kd[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0

            if i in self.controlled_joints:
                q0 = self.q_init.get(i, self.current_cmd_pos.get(i, 0.0))
                q1 = self.target_pos.get(i, q0)
                cmd.motor_cmd[i].q = self.interpolate_position(q0, q1)
            else:
                cmd.motor_cmd[i].q = self.hold_pos.get(i, 0.0)

        cmd.crc = self.crc.Crc(cmd)
        self.lowcmd_publisher_.Write(cmd)

        self.t += self.control_dt

    def StartWriter(self):
        if self._writer_thread is None:
            self._writer_thread = RecurrentThread(
                self.control_dt,
                target=self.LowCmdWrite,
                name="h1_2_robot_lowcmd_writer"
            )
            self._writer_thread.Start()
            print("[INFO] LowCmd writer iniciado (mantiene la postura medida).")

    def move_to(self, updates: dict, duration: float = 1.0):
        if self.amortiguado:
            raise RuntimeError(f"en amortiguación ({self.motivo_amortiguacion}); reinicia el selector")

        for j in self.controlled_joints:
            self.q_init[j] = float(self.current_cmd_pos[j])

        new_targets = {j: float(self.current_cmd_pos[j]) for j in self.controlled_joints}
        recortados = []

        for k, v in updates.items():
            try:
                jidx = int(k)
                value = float(v)
            except Exception:
                continue
            if jidx in self.controlled_joints:
                r = recortar(jidx, value)
                if abs(r - value) > 1e-6:
                    recortados.append((jidx, round(value, 3), round(r, 3)))
                new_targets[jidx] = r

        if recortados:
            print(f"     [LÍMITE] recortados (junta, pedido, usado): {recortados}")

        duration = float(duration) if float(duration) > 0 else 0.001
        if self.primer_movimiento:
            duration = max(duration, DURACION_MINIMA_PRIMER_MOVIMIENTO)
            self.primer_movimiento = False

        for j in self.controlled_joints:
            self.target_pos[j] = new_targets[j]

        self.T = duration
        self.t = 0.0

        while self.t < self.T:
            if self.amortiguado:
                raise RuntimeError(self.motivo_amortiguacion)
            time.sleep(self.control_dt)

        self.t = self.T
        for j in self.controlled_joints:
            self.current_cmd_pos[j] = self.target_pos[j]
            self.q_init[j] = self.target_pos[j]

        time.sleep(max(self.control_dt, 0.002))

    # ---------------------------------------------------------
    # Carga y ejecución de rutinas
    # ---------------------------------------------------------

    def load_routine(self, filepath: Path):
        with open(filepath, "r", encoding="utf-8") as f:
            routine = json.load(f)
        robot = routine.get("robot")
        if robot is not None and robot != "unitree_h1_2":
            raise ValueError(f"la rutina es para '{robot}', no para el H1-2")
        for n, paso in enumerate(routine.get("pasos", []), 1):
            for lado, q in (paso.get("manos") or {}).items():
                if lado not in gm.LADOS:
                    raise ValueError(f"paso {n}: mano '{lado}' desconocida (izq o der)")
                gm.normalizar(q)        # KeyError si trae un DOF que no existe
        return routine

    def cargar_pose_segura(self):
        """El paso de poses/0_pose_segura.json (manos abiertas si no las trae). Sin él no se arranca."""
        paso = self.load_routine(self.poses_dir / "0_pose_segura.json")["pasos"][0]
        manos = paso.get("manos") or {lado: gm.gesto("abierta") for lado in gm.LADOS}
        self.pose_segura = {"nombre": "pose segura", "duracion": DUR_POSE_SEGURA,
                            "posiciones": {k: float(v) for k, v in paso["posiciones"].items()},
                            "manos": {lado: gm.normalizar(q) for lado, q in manos.items()}}

    def es_pose_segura(self, paso: dict) -> bool:
        """Si el paso deja brazos y manos en la pose segura (a menos de TOL_POSE_SEGURA)."""
        seg, pos, manos = self.pose_segura, paso.get("posiciones", {}), paso.get("manos") or {}
        if any(abs(float(pos.get(k, float("inf"))) - v) > TOL_POSE_SEGURA for k, v in seg["posiciones"].items()):
            return False
        return all(lado in manos and all(abs(gm.normalizar(manos[lado])[k] - x) <= TOL_POSE_SEGURA
                                         for k, x in q.items())
                   for lado, q in seg["manos"].items())

    def ejecutar_paso(self, paso: dict, titulo: str):
        dur = float(paso.get("duracion", 1.0))
        manos = paso.get("manos") or {}
        texto = ", ".join(f"{lado} {gm.nombre_gesto(gm.normalizar(q)) or 'propia'}" for lado, q in manos.items())
        print(f"  -> {titulo} | dur={dur:.2f}s{f' | manos: {texto}' if texto else ''}")
        self.lanzar_manos(manos, dur)
        self.move_to(paso.get("posiciones", {}), duration=dur)
        self.esperar_manos()

    def PlayRoutine(self, routine: dict):
        name = routine.get("nombre_rutina", "routine")
        pasos = routine.get("pasos", [])

        print("\n" + "=" * 72)
        print(f"[INFO] Ejecutando rutina: {name}  ({len(pasos)} pasos)")
        print("=" * 72)

        for idx, paso in enumerate(pasos, 1):
            self.ejecutar_paso(paso, f"{idx:02d}. {paso.get('nombre', f'Paso {idx}')}")
        if not pasos or not self.es_pose_segura(pasos[-1]):
            self.ejecutar_paso(self.pose_segura, "pose segura (automática)")

        print("[INFO] Rutina finalizada: en la pose segura.")

    # ---------------------------------------------------------
    # Catálogo
    # ---------------------------------------------------------

    @staticmethod
    def extract_number(path: Path):
        match = re.match(r"^\s*(\d+)", path.stem)
        return int(match.group(1)) if match else None

    def build_catalog(self):
        if not self.poses_dir.is_dir():
            return []
        files = sorted(self.poses_dir.glob("*.json"),
                       key=lambda p: (self.extract_number(p) if self.extract_number(p) is not None else 9999, p.name))
        catalog, used, fallback = [], set(), 1
        for path in files:
            number = self.extract_number(path)
            if number is None or number in used:
                while fallback in used:
                    fallback += 1
                number = fallback
            used.add(number)
            catalog.append({"number": number, "path": path})
        return catalog

    def print_menu(self):
        print("\n" + "=" * 72)
        print("SELECTOR DE RUTINAS H1-2 CON MANOS - ROBOT REAL  (L2+B = parada de emergencia)")
        print("=" * 72)
        print(f"Carpeta de rutinas: {self.poses_dir}")
        print(f"Manos: {', '.join(self.manos) if self.manos else 'NO (solo brazos)'}")
        print("Comandos: número = ejecutar | l = listar | x = salir (pose segura y amortiguación)")
        print("Cada rutina termina en la pose segura, y el robot se queda sujetándola.")
        print("-" * 72)
        for item in self.build_catalog():
            print(f"{item['number']:02d}. {item['path'].name}")
        print("-" * 72)

    def selector_loop(self):
        self.print_menu()
        while True:
            choice = input("\nNúmero de rutina / l / x: ").strip().lower()
            if choice == "":
                continue
            if choice == "l":
                self.print_menu()
                continue
            if choice == "x":
                print("[INFO] Saliendo del selector.")
                return
            if not choice.isdigit():
                print("[WARN] Entrada inválida. Usa un número, l o x.")
                continue
            item = next((it for it in self.build_catalog() if it["number"] == int(choice)), None)
            if item is None:
                print(f"[WARN] No existe rutina con número: {choice}")
                continue
            print(f"\n[RUN] #{item['number']:02d} -> {item['path'].name}")
            try:
                self.PlayRoutine(self.load_routine(item["path"]))
            except Exception as e:
                print(f"[ERROR] {item['path'].name}: {e}")
                if self.amortiguado:
                    return

    # ---------------------------------------------------------
    # Cierre
    # ---------------------------------------------------------

    def StopAndShutdown(self):
        if self._writer_thread is None:
            print("[INFO] No se llegó a mandar ningún comando.")
            self.cerrar_manos()
            return

        # 1. Volver a la pose segura, si se puede, y abrir las manos a la vez.
        hilo_manos = None
        if self.manos:
            print(f"[INFO] Abriendo las manos ({T_ABRIR:.0f} s)...")
            hilo_manos = threading.Thread(target=self.abrir_manos, daemon=True)
            hilo_manos.start()
        if not self.amortiguado and getattr(self, "pose_segura", None):
            try:
                print(f"[INFO] Volviendo a la pose segura ({DUR_POSE_SEGURA:.0f} s)...")
                self.move_to(self.pose_segura["posiciones"], duration=DUR_POSE_SEGURA)
            except Exception as e:
                print(f"[WARN] No se pudo volver a la pose segura: {e}")
        if hilo_manos is not None:
            hilo_manos.join()
        habia_manos = bool(self.manos)
        self.cerrar_manos()

        # 2. Soltar en rampa: kp -> 0 manteniendo kd (amortiguación).
        print(f"[INFO] Soltando a amortiguación en {T_SOLTAR:.0f} s...")
        pasos = int(T_SOLTAR / 0.02)
        for k in range(pasos + 1):
            self.factor_kp = 1.0 - k / pasos
            time.sleep(0.02)
        self.amortiguado = True
        time.sleep(0.5)

        try:
            self._writer_thread.Wait()
        except Exception:
            pass
        self._writer_thread = None
        print(f"[INFO] Cierre completado: motores en amortiguación{', manos abiertas' if habia_manos else ''}, "
              f"sin publicador.")


def main():
    script_dir = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="Selector de rutinas brazos + manos del H1-2 real.")
    ap.add_argument("interface", nargs="?", default="eth0", help="eth0 en el robot; lo = unitree_mujoco (dominio 1)")
    ap.add_argument("poses", nargs="?", default=str(script_dir / "poses"),
                    help="carpeta de rutinas")
    ap.add_argument("--sin-manos", action="store_true", help="solo brazos: no conecta con las manos")
    ap.add_argument("--ip-izq", default=IP_MANO["izq"])
    ap.add_argument("--ip-der", default=IP_MANO["der"])
    ap.add_argument("--puerto-manos", type=int, default=PUERTO_MANO)
    a = ap.parse_args()
    interface = a.interface
    poses_dir = Path(a.poses).expanduser().resolve()

    print("=" * 72)
    print("H1-2 REAL - control de bajo nivel por rt/lowcmd, y manos por Modbus TCP")
    print("  1. Robot COLGADO, pies sin tocar el suelo")
    print("  2. Modo Debug (L2+R2)")
    print("  3. Alguien con el mando: L2+B = parada de emergencia")
    print("=" * 72)
    # "lo" es el simulador (dominio 1), para probar el selector en MuJoCo
    # antes de llevarlo al robot. Cualquier otra interfaz es el robot (dominio 0).
    dominio = 1 if interface == "lo" else 0
    print(f"[INFO] Interface: {interface}  (dominio DDS {dominio})")
    print(f"[INFO] Poses dir: {poses_dir}")

    ChannelFactoryInitialize(dominio, interface)

    selector = H1_2RobotSelector(poses_dir=poses_dir, control_dt=0.002)
    selector.Init()

    if not selector.wait_lowstate(timeout=8.0):
        return
    # En el simulador no hay servicio de movimiento que liberar.
    if interface != "lo" and not selector.comprobar_servicio_movimiento():
        return
    if not selector.comprobar_modo_debug():
        return
    try:
        selector.cargar_pose_segura()
    except Exception as e:
        print(f"[ERROR] La pose segura es la postura por defecto y no se puede leer ({e}). No se arranca.")
        return
    if not a.sin_manos and not selector.conectar_manos({"izq": a.ip_izq, "der": a.ip_der}, a.puerto_manos):
        print("[ERROR] Sin las dos manos no se arranca. Para mover solo los brazos: --sin-manos")
        return

    confirmacion = input("Escribe MOVER para empezar a mandar comandos (cualquier otra cosa sale): ").strip()
    if confirmacion != "MOVER":
        print("[INFO] Cancelado. No se ha mandado ningún comando.")
        selector.cerrar_manos()
        return

    try:
        for mano in selector.manos.values():
            mano.preparar()
        selector.CrearPublicador()
        selector.StartWriter()
        selector.ejecutar_paso(selector.pose_segura, "pose segura (inicio)")
        selector.selector_loop()
    except (KeyboardInterrupt, EOFError):
        print("\n[INFO] Interrumpido. Cerrando...")
    except RuntimeError as e:
        print(f"[ERROR] {e}")
    finally:
        selector.StopAndShutdown()
        print("[INFO] Programa terminado.")


if __name__ == "__main__":
    main()
