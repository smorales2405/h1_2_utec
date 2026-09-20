"""Cliente `arm_sdk` sobre DDS directo, para correr EN EL ROBOT.

Por qué existe, habiendo ya un cliente en `ros_h1_2_ws`: el PC2 del robot
**no tiene ROS 2**. Tiene Python 3.10 y `unitree_sdk2py` sobre `cyclonedds`,
instalados sin Internet desde un bundle. El cliente de ROS depende de `rclpy` y
de los mensajes `unitree_hg` de `unitree_ros2`, así que ahí no arranca.

Esto es la misma lógica con otro transporte: publicar en `rt/arm_sdk` a
frecuencia fija, vigilar el estado, y soltar en rampa. Se ha quedado fuera todo
lo que aquí no hace falta —canal `lowcmd`, política de piernas, compensación de
gravedad, registro de medidas— porque menos código es menos que revisar en algo
que va a arrancar solo con el robot de pie.

QUÉ SIGNIFICA EL PESO
─────────────────────
`arm_sdk` no toma los brazos: los **mezcla**. El peso viaja en el `q` del motor
27 —`kNotUsedJoint0` del H1-2, no el 29 que usa el G1— y va de 0 a 1. Medido
sobre el robot el 2026-09-17: con peso 0.5 y pidiendo +3° en un codo, el brazo
se movió +1.80°, o sea el 60 %. La mezcla es lineal y el resto lo sigue
mandando el controlador de locomoción, que es lo que mantiene al robot de pie.

Por eso este cliente nunca suelta de golpe: bajar el peso en rampa devuelve el
brazo al controlador sin escalón.
"""
from __future__ import annotations

import math
import signal
import struct
import threading
import time

import numpy as np
from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                         ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from joints import ARM_INDICES, ARM_SDK_INDICES, BY_INDEX, WEIGHT_INDEX

TOPIC_ARM_SDK = "rt/arm_sdk"
TOPIC_LOWSTATE = "rt/lowstate"


class SafetyAbort(RuntimeError):
    """Algo se salió de lo previsto y se ha soltado el brazo."""


class ArmSdkClient:
    def __init__(self, gains, rate_hz: float = 250.0, max_weight: float = 1.0,
                 tau_abort_fraction: float = 0.7, temp_max: float = 80.0,
                 state_timeout: float = 0.5, verbose: bool = True):
        self.gains = gains
        self.dt = 1.0 / float(rate_hz)
        self.max_weight = float(max_weight)
        self.tau_abort_fraction = tau_abort_fraction
        self.temp_max = temp_max
        self.state_timeout = state_timeout
        self.verbose = verbose

        self.commanded = list(ARM_SDK_INDICES)
        self._lock = threading.Lock()
        self._state: LowState_ | None = None
        self._t_state = 0.0
        self._running = False
        self._abort: str | None = None
        self._hilo: threading.Thread | None = None

        self._q_des = np.zeros(35)
        self._dq_des = np.zeros(35)
        self._q_prev = np.zeros(35)
        self._kp = np.zeros(35)
        self._kd = np.zeros(35)
        self._traj: dict[int, tuple] = {}
        self._weight = 0.0
        self._weight_target = 0.0
        self._weight_rate = 1.0
        self.q0 = np.zeros(35)
        self.señal: int | None = None
        # Combinaciones del mando que ABORTAN el gesto en cuanto aparecen.
        # Las pone el demonio; el cliente solo las vigila, porque es el único
        # que mira el estado en cada ciclo.
        self.teclas_de_aborto: dict[str, int] = {}
        self.collision_clamps = 0
        self.cycles = 0
        self.late = 0
        self._tau_alto_desde: dict[int, float] = {}

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self._pub = ChannelPublisher(TOPIC_ARM_SDK, LowCmd_)
        self._pub.Init()
        self._sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self._sub.Init(self._on_state, 10)

    # ── estado ─────────────────────────────────────────────────────────────
    def _on_state(self, msg: LowState_):
        with self._lock:
            self._state = msg
            self._t_state = time.monotonic()

    def state(self) -> LowState_ | None:
        with self._lock:
            return self._state

    def wait_for_state(self, timeout: float = 15.0) -> LowState_:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            s = self.state()
            if s is not None:
                return s
            time.sleep(0.02)
        raise SafetyAbort(
            f"no llega {TOPIC_LOWSTATE} en {timeout:.0f} s. ¿Está el robot "
            f"encendido? ¿La interfaz de red es la correcta?")

    def q(self, i: int) -> float:
        s = self.state()
        return float(s.motor_state[i].q) if s else 0.0

    def dq(self, i: int) -> float:
        s = self.state()
        return float(s.motor_state[i].dq) if s else 0.0

    def tau(self, i: int) -> float:
        s = self.state()
        return float(s.motor_state[i].tau_est) if s else 0.0

    def temp(self, i: int) -> float:
        s = self.state()
        return float(s.motor_state[i].temperature[0]) if s else 0.0

    # ── control ────────────────────────────────────────────────────────────
    def engage(self, ramp: float = 1.0) -> None:
        """Toma los brazos, sin moverlos: la consigna parte de donde están."""
        s = self.wait_for_state()
        self.msg.mode_pr = 0
        self.msg.mode_machine = s.mode_machine
        with self._lock:
            # TODOS los motores se inicializan a su posición MEDIDA, no a cero,
            # aunque solo comandemos quince. Con kp = 0 un `q` erróneo es
            # inofensivo, pero dejarlo en cero es sembrar una trampa para el
            # día que alguien suba una ganancia sin mirar. Es lo que hace el
            # controlador de `xr_teleoperate`, y no hay razón para desviarse.
            for i in range(35):
                self.msg.motor_cmd[i].mode = 1
                self.msg.motor_cmd[i].q = float(s.motor_state[i].q)
                self.msg.motor_cmd[i].dq = 0.0
                self.msg.motor_cmd[i].tau = 0.0
                self.msg.motor_cmd[i].kp = 0.0
                self.msg.motor_cmd[i].kd = 0.0
            for i in self.commanded:
                self.q0[i] = float(s.motor_state[i].q)
                self._q_des[i] = self.q0[i]
                self._q_prev[i] = self.q0[i]
                kp, kd = self.gains.for_index(i)
                self._kp[i], self._kd[i] = kp, kd
            self._dq_des[:] = 0.0
            self._weight = 0.0
            self._weight_target = self.max_weight
            self._weight_rate = 1.0 / max(ramp, 1e-3)
        self._arranca()
        self._log(f"tomando los brazos por {TOPIC_ARM_SDK}, peso hasta "
                  f"{self.max_weight:.2f} en {ramp:.1f} s…")
        self.sleep(ramp + 0.3)

    def release(self, home_to: dict | None = None, home_speed: float = 0.3,
                weight_ramp: float = 1.5) -> None:
        """Devuelve el brazo y baja el peso en rampa. Nunca suelta de golpe."""
        if not self._running:
            return
        try:
            destino = ({i: float(self.q0[i]) for i in self.commanded}
                       if home_to is None else dict(home_to))
            if destino:
                self._log("devolviendo a la postura de partida…")
                self.ramp_to(destino, speed=home_speed)
            self._log(f"bajando el peso a 0 en {weight_ramp:.1f} s…")
            with self._lock:
                self._weight_target = 0.0
                self._weight_rate = 1.0 / max(weight_ramp, 1e-3)
            self.sleep(weight_ramp + 0.3)
        except SafetyAbort as e:
            self._log(f"  (seguridad al salir: {e})")
            self._suelta_ya()
        finally:
            self._para()
            self._log("brazos devueltos al robot.")

    def _suelta_ya(self) -> None:
        """Peso a 0 lo más rápido que permita no dar un tirón. Solo para abortos."""
        with self._lock:
            self._weight_target = 0.0
            self._weight_rate = 1.0 / 0.4
            self._traj.clear()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.8 and self._weight > 1e-3:
            time.sleep(0.02)

    def set_trajectory(self, i: int, func, q_base: float | None = None) -> None:
        with self._lock:
            base = float(self._q_des[i]) if q_base is None else float(q_base)
            self._traj[i] = (func, base, time.monotonic())

    def clear_trajectory(self) -> None:
        with self._lock:
            self._traj.clear()
            self._dq_des[:] = 0.0

    def ramp_to(self, targets: dict, speed: float = 0.3) -> None:
        """Coseno alzado hasta `targets`, bloqueando. Velocidad nula en los extremos."""
        self.clear_trajectory()
        with self._lock:
            inicio = {i: float(self._q_des[i]) for i in targets}
        # El tope del hombro depende del codo, así que se recorta con el codo
        # MÁS ESTIRADO entre el de ahora y el de destino: es el que manda
        # durante toda la rampa.
        pares = self.gains.cond_pairs()
        meta = {}
        for i, q in targets.items():
            e = pares.get(i)
            if e is None:
                meta[i] = self.gains.clamp(i, q)
            else:
                q_codo = max(abs(float(self._q_des[e])),
                             abs(float(targets.get(e, self._q_des[e]))))
                lo, hi = self.gains.limits_dynamic(i, q_codo)
                meta[i] = min(max(q, lo), hi)
        dist = max((abs(meta[i] - inicio[i]) for i in targets), default=0.0)
        if dist < 1e-6:
            return
        dur = dist / max(abs(speed), 1e-3)
        t0 = time.monotonic()
        while True:
            self._si_aborto()
            a = min((time.monotonic() - t0) / dur, 1.0)
            s = 0.5 - 0.5 * math.cos(math.pi * a)
            with self._lock:
                for i in targets:
                    self._q_des[i] = inicio[i] + s * (meta[i] - inicio[i])
            if a >= 1.0:
                return
            time.sleep(self.dt)

    def wait_all_settled(self, indices, dq_tol: float = 0.05,
                         timeout: float = 1.5, estable: float = 0.15) -> bool:
        """Espera a que TODAS estén quietas a la vez, con un contador único.

        `dq_tol` por defecto 0.05 y no 0.02: de pie, el controlador de
        equilibrio microajusta los brazos sin parar. Medido sobre el robot
        parado, las catorce cumplen 0.02 a la vez solo el 37 % del tiempo y
        0.05 el 99.8 %.
        """
        idx = list(indices)
        t0 = time.monotonic()
        desde = None
        while time.monotonic() - t0 < timeout:
            self._si_aborto()
            if all(abs(self.dq(i)) < dq_tol for i in idx):
                if desde is None:
                    desde = time.monotonic()
                elif time.monotonic() - desde >= estable:
                    return True
            else:
                desde = None
            time.sleep(0.02)
        return False

    def sleep(self, seconds: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self._si_aborto()
            time.sleep(min(0.02, seconds))

    # ── lazo ───────────────────────────────────────────────────────────────
    def _arranca(self) -> None:
        if self._running:
            return
        self._abort = None
        self._running = True
        self._hilo = threading.Thread(target=self._lazo, daemon=True)
        self._hilo.start()

    def _para(self) -> None:
        self._running = False
        if self._hilo is not None:
            self._hilo.join(timeout=1.0)
            self._hilo = None

    def _si_aborto(self) -> None:
        if self._abort:
            raise SafetyAbort(self._abort)

    def _lazo(self) -> None:
        siguiente = time.monotonic()
        while self._running:
            try:
                self._un_ciclo()
            except Exception as e:                    # nunca dejar de publicar
                self._abort = self._abort or f"fallo en el lazo: {e}"
            siguiente += self.dt
            espera = siguiente - time.monotonic()
            if espera > 0:
                time.sleep(espera)
            else:
                self.late += 1
                siguiente = time.monotonic()

    def _un_ciclo(self) -> None:
        self.cycles += 1
        ahora = time.monotonic()
        with self._lock:
            s = self._state
            t_s = self._t_state
        if s is None:
            return
        # Estado rancio: si deja de llegar, no sabemos dónde está el brazo.
        if ahora - t_s > self.state_timeout and self._abort is None:
            self._abort = (f"{TOPIC_LOWSTATE} lleva {ahora - t_s:.2f} s sin "
                           f"llegar")

        with self._lock:
            for i, (f, base, t0) in list(self._traj.items()):
                q_rel, dq_rel = f(ahora - t0)
                self._q_des[i] = self.gains.clamp(i, base + q_rel)
                self._dq_des[i] = dq_rel
            # Autocolisión sobre la consigna, no sobre la medida: cuando el
            # brazo real ha llegado a una postura en colisión ya es tarde.
            self.collision_clamps += self.gains.enforce_pairs(
                self._q_des, self._q_prev, tuple(self._traj), False,
                self.commanded)
            self._q_prev[:] = self._q_des

            # rampa de peso
            if self._weight < self._weight_target:
                self._weight = min(self._weight_target,
                                   self._weight + self._weight_rate * self.dt)
            elif self._weight > self._weight_target:
                self._weight = max(self._weight_target,
                                   self._weight - self._weight_rate * self.dt)
            w = self._weight
            q_des = self._q_des.copy()
            dq_des = self._dq_des.copy()
            kp = self._kp.copy()
            kd = self._kd.copy()

        self._seguridad(s)

        for i in self.commanded:
            c = self.msg.motor_cmd[i]
            c.mode = 1
            c.q = float(q_des[i])
            c.dq = float(dq_des[i])
            c.tau = 0.0
            c.kp = float(kp[i])
            c.kd = float(kd[i])
        # El peso de la mezcla viaja aquí, en un motor que no existe.
        self.msg.motor_cmd[WEIGHT_INDEX].q = float(w)
        self.msg.crc = self.crc.Crc(self.msg)
        self._pub.Write(self.msg)

        if self._abort and w <= 1e-3:
            self._running = False

    @staticmethod
    def _teclas(s) -> int:
        b = bytes(bytearray(s.wireless_remote))
        return struct.unpack_from("<H", b, 2)[0] if len(b) >= 4 else 0

    def _seguridad(self, s) -> None:
        if self._abort:
            # ya abortando: el lazo sigue publicando mientras baja el peso
            with self._lock:
                self._weight_target = 0.0
                self._weight_rate = 1.0 / 0.4
                self._traj.clear()
            return
        # El mando manda. Si aparece una combinación de cambio de modo
        # —`L2+B` es amortiguación, que se pulsa cuando algo va mal— hay que
        # soltar los brazos YA, no dentro de diez segundos: el robot está
        # intentando cambiar de modo y nosotros se los estamos sujetando.
        if self.teclas_de_aborto:
            k = self._teclas(s)
            for nombre, m in self.teclas_de_aborto.items():
                if m and (k & m) == m:
                    self._abort = f"el mando pidió «{nombre}»"
                    return

        ahora = time.monotonic()
        for i in ARM_INDICES:
            j = BY_INDEX[i]
            tau = abs(float(s.motor_state[i].tau_est))
            if tau > self.tau_abort_fraction * j.tau_max:
                t0 = self._tau_alto_desde.setdefault(i, ahora)
                if ahora - t0 > 0.3:
                    self._abort = (f"{j.name} lleva 0.3 s por encima del "
                                   f"{self.tau_abort_fraction*100:.0f} % de su "
                                   f"par máximo ({tau:.1f} de {j.tau_max:.0f} N)")
                    return
            else:
                self._tau_alto_desde.pop(i, None)
            t = float(s.motor_state[i].temperature[0])
            if t > self.temp_max:
                self._abort = f"{j.name} a {t:.0f} °C"
                return

    def _log(self, m: str) -> None:
        if self.verbose:
            print(f"  {m}", flush=True)

    # ── apagado ────────────────────────────────────────────────────────────
    def cierra(self) -> None:
        """Suelta pase lo que pase. Idempotente."""
        try:
            if self._running:
                self._suelta_ya()
        finally:
            self._para()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        try:
            self.release()
        except Exception:
            self.cierra()
        return False


def inicializa_dds(nic: str | None = None, dominio: int = 0) -> None:
    """Arranca DDS. En el propio robot la interfaz suele ser `lo`."""
    if nic:
        ChannelFactoryInitialize(dominio, nic)
    else:
        ChannelFactoryInitialize(dominio)


def instala_señales(cliente: "ArmSdkClient") -> None:
    """SIGTERM y SIGINT sueltan el brazo antes de morir.

    Importa más aquí que en la laptop: esto va a ser un servicio que systemd
    para con SIGTERM. Sin esto, el proceso muere con el peso donde estuviera y
    el robot se queda con la última consigna.
    """
    def _cae(signum, _frame):
        print(f"\n  señal {signum}: soltando los brazos…", flush=True)
        # Se anota cuál fue: que systemd te pida parar con SIGTERM no es un
        # fallo, y salir con 130 hace que la unidad quede marcada como
        # «Failed». Con `Restart=always` eso no rompe nada, pero ensucia el
        # diagnóstico justo donde se va a mirar cuando algo vaya mal de verdad.
        cliente.señal = signum
        try:
            cliente.cierra()
        finally:
            raise KeyboardInterrupt(f"señal {signum}")

    for s in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(s, _cae)
        except (ValueError, OSError):
            pass
