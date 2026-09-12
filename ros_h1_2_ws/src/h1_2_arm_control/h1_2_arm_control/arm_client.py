"""Cliente de control articular de bajo nivel para el H1-2 sobre unitree_ros2.

Dos canales, y la diferencia importa mucho:

``arm_sdk``  (por defecto, y el que hay que usar)
    Publica en ``/arm_sdk``. El servicio de control del robot sigue mandando
    en las piernas y solo cede los 14 motores de los brazos + la cintura,
    mezclados con un peso ``w`` que viaja en ``motor_cmd[27].q`` (0 = manda el
    robot, 1 = mandamos nosotros). Nadie más publica en este tópico, así que
    no hay dos controladores peleándose.

``lowcmd``   (solo con el robot COLGADO DEL ARNÉS y en modo debug)
    Publica en ``/lowcmd``, el mismo tópico que usa el servicio interno del
    robot **a 500 Hz**. Si el servicio sigue vivo, los dos publicadores se
    alternan y el motor recibe consignas contradictorias: eso es exactamente
    la vibración que se observó con ``test_mandar_modificado.py``. En este
    canal hay que comandar los 27 motores, piernas incluidas.

Uso típico::

    with H12Client(controlled=ARM_INDICES) as cli:
        cli.engage()                       # toma el control, sin mover nada
        cli.ramp_to({16: 0.0}, speed=0.15) # el codo a 0°, despacio
        cli.release()                      # devuelve el control

Al salir del ``with`` —también por Ctrl-C o por una excepción— se ejecuta
siempre ``release()``: la articulación vuelve despacio a donde estaba y el peso
baja a 0. El robot nunca se queda sin nadie que lo mande.
"""
from __future__ import annotations

import math
import signal
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_hg.msg import LowCmd, LowState

from . import gains as cfg
from .crc import set_crc
from .joints import (ARM_SDK_INDICES, BY_INDEX, LEG_INDICES, NUM_CMD_MOTOR,
                     WEIGHT_INDEX)

TOPIC_STATE = "/lowstate"
TOPIC_ARM_SDK = "/arm_sdk"
TOPIC_LOWCMD = "/lowcmd"
# Tópico de pruebas: nadie lo escucha. Sirve para ejercitar toda la cadena
# —construcción del mensaje, CRC, lazo a frecuencia fija, registro, métricas—
# con la certeza de que el robot no puede enterarse.
TOPIC_DRY_RUN = "/h1_2_arm_control/dry_run"


class SafetyAbort(RuntimeError):
    """Se ha superado un tope de seguridad. El cliente ya está soltando."""


def _install_sigterm_handler():
    """Hacer que SIGTERM se comporte como Ctrl-C.

    Ctrl-C ya funciona: `rclpy` deja que Python lance `KeyboardInterrupt`, el
    hilo principal sale de `sleep()` y se ejecuta la salida ordenada.

    SIGTERM no. Comprobado: `rclpy` cierra su contexto (`rclpy.ok()` pasa a
    False) pero el bucle del hilo principal sigue corriendo tan campante. En un
    script que está mandando al robot eso es serio: un `kill <pid>` o un
    `timeout` dejarían el proceso vivo PUBLICANDO, sin nadie mirando.

    Lanzar `KeyboardInterrupt` desde el manejador lo arregla: el manejador corre
    en el hilo principal, así que la excepción sale por donde estuviera y
    dispara el mismo `finally` que Ctrl-C.
    """
    if threading.current_thread() is not threading.main_thread():
        return                       # solo el hilo principal puede
    try:
        if signal.getsignal(signal.SIGTERM) is signal.SIG_DFL:
            signal.signal(
                signal.SIGTERM,
                lambda signum, frame: (_ for _ in ()).throw(
                    KeyboardInterrupt(f"SIGTERM ({signum})")))
    except (ValueError, OSError):
        pass                         # entornos sin señales; no es crítico


def _install_sigterm_handler():
    """Hacer que SIGTERM se comporte como Ctrl-C.

    Ctrl-C ya funciona: `rclpy` deja que Python lance `KeyboardInterrupt`, el
    hilo principal sale de `sleep()` y se ejecuta la salida ordenada.

    SIGTERM no. Comprobado: `rclpy` cierra su contexto (`rclpy.ok()` pasa a
    False) pero el bucle del hilo principal sigue corriendo tan campante. En un
    script que está mandando al robot eso es serio: un `kill <pid>` o un
    `timeout` dejarían el proceso vivo PUBLICANDO, sin nadie mirando.

    Lanzar `KeyboardInterrupt` desde el manejador lo arregla: el manejador corre
    en el hilo principal, así que la excepción sale por donde estuviera y
    dispara el mismo `finally` que Ctrl-C.
    """
    if threading.current_thread() is not threading.main_thread():
        return                       # solo el hilo principal puede
    try:
        if signal.getsignal(signal.SIGTERM) is signal.SIG_DFL:
            signal.signal(
                signal.SIGTERM,
                lambda signum, frame: (_ for _ in ()).throw(
                    KeyboardInterrupt(f"SIGTERM ({signum})")))
    except (ValueError, OSError):
        pass                         # entornos sin señales; no es crítico


class H12Client:
    """Publica lowcmd/arm_sdk a frecuencia fija y vigila el estado del robot."""

    def __init__(
        self,
        controlled: list[int],
        gains: cfg.Gains | None = None,
        channel: str = "arm_sdk",
        rate_hz: float = 250.0,
        node_name: str = "h1_2_arm_control",
        verbose: bool = True,
        dry_run: bool = False,
        max_weight: float = 1.0,
        legs_policy: str = "free",
        gravity=None,
        gravity_ramp: float = 1.0,
        gravity_frac: float = 0.5,
        gravity_at: str = "meas",
    ):
        """`dry_run`: publica en un tópico que nadie escucha. El robot no se
        entera de nada; sirve para validar el software.

        `max_weight`: hasta dónde sube el peso de arm_sdk en `engage()`. Con
        0.0 se publica en el tópico bueno pero con autoridad nula, que es la
        primera prueba prudente sobre el robot real. Con 0.3, un tercio de
        autoridad: el brazo obedece, pero flojo.

        `legs_policy`: qué hacer con piernas y cintura en el canal `lowcmd`,
        donde somos los únicos que mandamos. Solo aplica a ese canal.

            free  kp = kd = 0. Reproduce EXACTAMENTE lo que el servicio `ai`
                  estaba publicando en reposo, así que soltarlo no cambia nada
                  para las piernas. Es lo correcto con el robot colgado.
            damp  kp = 0, kd = 2. Igual de libre, pero amortiguado: si el robot
                  cuelga, las piernas dejan de bambolearse.
            hold  las ganancias de `legs_hold` (kp = 300). Sostiene las piernas
                  rígidas donde estén. NO usar si el robot no está colgado:
                  pasar de par nulo a kp = 300 de golpe es un tirón."""
        if legs_policy not in ("free", "damp", "hold"):
            raise ValueError("legs_policy debe ser 'free', 'damp' o 'hold'")
        self.legs_policy = legs_policy

        # Compensación de gravedad. Se evalúa en `q_des` y a la frecuencia del
        # LAZO, no a la de la trayectoria: el par que hace falta depende de
        # dónde se le está pidiendo estar en ese instante.
        self.gravity = gravity
        self.gravity_ramp = float(gravity_ramp)
        # Tope propio, independiente del aborto por par: un fallo de signo o un
        # modelo disparatado no puede meter más de esta fracción del par máximo.
        self.gravity_frac = float(gravity_frac)
        # Dónde se evalúa g(): en la consigna o en la posición MEDIDA.
        #
        # El protocolo pide `q_des`, y para sostener una postura da igual. Pero
        # durante un movimiento la articulación va por detrás de la consigna,
        # así que g(q_des) es el par que hará falta al LLEGAR, no el que hace
        # falta ahora: se adelanta, y añade empuje justo cuando el PD ya está
        # empujando. Evaluar en la posición medida cancela la gravedad que
        # realmente actúa, que es lo que se quiere de una compensación.
        if gravity_at not in ("des", "meas"):
            raise ValueError("gravity_at debe ser 'des' o 'meas'")
        self.gravity_at = gravity_at
        self._g_scale = 0.0
        self._g_clip = 0
        if channel not in ("arm_sdk", "lowcmd"):
            raise ValueError("channel debe ser 'arm_sdk' o 'lowcmd'")
        self.channel = channel
        self.gains = gains or cfg.load()
        self.safety = self.gains.safety
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz
        self.verbose = verbose

        # En arm_sdk hay que comandar los 15 motores que el servicio cede, no
        # solo el que se está probando: los demás se dejan clavados donde
        # estaban. Si no, quedarían con kp=0 y el brazo se descolgaría.
        if channel == "arm_sdk":
            self.commanded = list(ARM_SDK_INDICES)
        else:
            self.commanded = list(range(NUM_CMD_MOTOR))
        self.controlled = list(controlled)
        for i in self.controlled:
            if i not in self.commanded:
                raise ValueError(
                    f"el motor {i} ({BY_INDEX[i].name}) no se puede comandar por "
                    f"el canal '{channel}'. En arm_sdk solo valen brazos y cintura."
                )

        # --- estado compartido con el hilo de control ---------------------
        self._lock = threading.Lock()
        self._q_des = np.zeros(NUM_CMD_MOTOR)
        self._dq_des = np.zeros(NUM_CMD_MOTOR)
        self._tau_ff = np.zeros(NUM_CMD_MOTOR)
        self._tau_g = np.zeros(NUM_CMD_MOTOR)
        self._kp = np.zeros(NUM_CMD_MOTOR)
        self._kd = np.zeros(NUM_CMD_MOTOR)
        self._weight = 0.0
        self._weight_target = 0.0
        self._weight_rate = 1.0          # 1/s -> 1 s de rampa completa
        # Trayectorias vivas: idx -> (f, q_base, t0). Las evalúa el HILO DE
        # CONTROL, no quien llama. Si la referencia se actualizara desde el
        # hilo principal, su jitter (decenas de ms en Python) se colaría en la
        # medida y se confundiría con mal seguimiento de la articulación.
        self._traj: dict[int, tuple] = {}
        self._abort: str | None = None
        self._tau_hot_since: dict[int, float] = {}

        # `q0` es donde estaba el robot al ceder el control: ahí se le devuelve
        # al terminar. `q_base` es desde dónde parten los ensayos, que NO es lo
        # mismo en cuanto se aplica una postura de ensayo.
        self.q0 = np.zeros(NUM_CMD_MOTOR)
        self.q_base = np.zeros(NUM_CMD_MOTOR)

        # Autocolisión hombro-codo. El margen extra se aplica cuando de verdad
        # se MUEVE una muñeca, no cuando simplemente está en la lista de
        # controladas.
        #
        # Deducirlo de `controlled` fue un error caro: en un barrido están las
        # siete controladas pero solo una se mueve, así que el margen extra se
        # aplicaba siempre y el tope del hombro subía de 10° a 15°. Un ensayo
        # que va de 18° a 11.1° chocaba con ese tope, el portero lo recortaba en
        # el 45 % de los ciclos, y el escalón se quedaba a medias: `q_final` muy
        # cerca de `q_start` y un "sobreimpulso" del 61 % que no existía.
        # Es lo que hacía irreproducibles los barridos de F2.3.
        self._wrist_idx = {i for i in controlled if BY_INDEX[i].group == "wrist"}
        self._cond_pairs = {r: e for r, e in self.gains.cond_pairs().items()
                            if r in self.commanded and e in self.commanded}
        self.collision_clamps = 0
        self._q_prev = np.zeros(NUM_CMD_MOTOR)
        self.cycles = 0
        self.late_cycles = 0
        self._loop_t0 = 0.0
        self._loop_t1 = 0.0

        # --- ROS ----------------------------------------------------------
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        try:
            self.node = Node(node_name)
        except Exception as exc:
            # "rcl node's rmw handle is invalid" no dice la causa, y la causa
            # casi siempre es que la NIC del CYCLONEDDS_URI está caída.
            raise RuntimeError(
                f"no se pudo crear el nodo ROS ({exc}).\n"
                f"    Causa habitual: la interfaz de red del CYCLONEDDS_URI\n"
                f"    está caída. Comprueba con:  ip -br addr show\n"
                f"    Si el robot está apagado o el cable desconectado, es eso."
            ) from exc
        self._state: LowState | None = None
        self._state_stamp = 0.0
        self._state_lock = threading.Lock()
        self.node.create_subscription(
            LowState, TOPIC_STATE, self._on_state, qos_profile_sensor_data)
        self.dry_run = bool(dry_run)
        self.max_weight = float(np.clip(max_weight, 0.0, 1.0))
        if self.dry_run:
            topic = TOPIC_DRY_RUN
        else:
            topic = TOPIC_ARM_SDK if channel == "arm_sdk" else TOPIC_LOWCMD
        self.topic = topic
        self._pub = self.node.create_publisher(LowCmd, topic, 10)
        self._msg = LowCmd()

        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self._running = False
        self._ctrl_thread: threading.Thread | None = None
        _install_sigterm_handler()

    # ================================================================== ROS
    def _spin(self):
        try:
            self._exec.spin()
        except Exception:
            pass
        finally:
            self._spin_stop.set()

    def _on_state(self, msg: LowState):
        with self._state_lock:
            self._state = msg
            self._state_stamp = time.monotonic()

    def state(self) -> LowState:
        with self._state_lock:
            if self._state is None:
                raise RuntimeError("todavía no ha llegado ningún /lowstate")
            return self._state

    def wait_for_state(self, timeout: float = 15.0) -> LowState:
        """Espera al primer /lowstate.

        El descubrimiento DDS no es instantáneo y, si en la máquina acaba de
        morir mal otro participante, puede tardar varios segundos de más. Por
        eso el plazo es generoso: agotarlo a los 5 s producía fallos
        intermitentes que no tenían nada que ver con el robot.
        """
        t0 = time.monotonic()
        warned = False
        while time.monotonic() - t0 < timeout:
            with self._state_lock:
                if self._state is not None:
                    return self._state
            if not warned and time.monotonic() - t0 > 2.0:
                warned = True
                self._log("  esperando a /lowstate (descubrimiento DDS)…")
            time.sleep(0.01)
        raise TimeoutError(
            f"no llega /lowstate en {timeout:.0f} s.\n"
            f"    · ¿El robot está encendido y el cable conectado?\n"
            f"    · ¿CYCLONEDDS_URI apunta a la NIC correcta? "
            f"(source setup_env.sh del workspace)\n"
            f"    · ¿Ha muerto mal algún proceso ROS hace poco? El dominio DDS\n"
            f"      tarda ~10 s en recuperarse; vuelve a intentarlo.\n"
            f"    Comprobación rápida:  ros2 run h1_2_arm_control read_state")

    # -- lecturas cómodas ------------------------------------------------
    def q(self, idx: int) -> float:
        return float(self.state().motor_state[idx].q)

    def dq(self, idx: int) -> float:
        return float(self.state().motor_state[idx].dq)

    def tau(self, idx: int) -> float:
        return float(self.state().motor_state[idx].tau_est)

    def temp(self, idx: int) -> float:
        return float(self.state().motor_state[idx].temperature[0])

    def q_des(self, idx: int) -> float:
        """La consigna que se está publicando ahora mismo."""
        with self._lock:
            return float(self._q_des[idx])

    def q_all(self) -> np.ndarray:
        s = self.state()
        return np.array([s.motor_state[i].q for i in range(NUM_CMD_MOTOR)])

    # ============================================================== control
    def engage(self, ramp: float = 1.5) -> None:
        """Toma el control SIN mover nada: consigna = postura actual.

        Sube el peso de 0 a 1 en `ramp` segundos. Mientras sube, la consigna es
        exactamente donde está el brazo, así que no debe haber movimiento
        visible: si lo hay, es que algo no cuadra y conviene parar.
        """
        self.wait_for_state()
        self.q0 = self.q_all()
        self.q_base = self.q0.copy()

        with self._lock:
            self._q_des[:] = self.q0
            self._q_prev[:] = self.q0
            self._dq_des[:] = 0.0
            self._tau_ff[:] = 0.0
            self._tau_g[:] = 0.0
            self._g_scale = 0.0
            for i in self.commanded:
                kp, kd = self.gains.for_index(i)
                self._kp[i], self._kd[i] = kp, kd
            if self.channel == "lowcmd":
                # En debug ya no publica nadie más: las piernas dependen de
                # nosotros. Qué hacer con ellas lo decide `legs_policy`.
                for i in LEG_INDICES:
                    if self.legs_policy == "hold":
                        kp, kd = self.gains.for_index(i)
                    elif self.legs_policy == "damp":
                        kp, kd = 0.0, 2.0
                    else:                       # free
                        kp, kd = 0.0, 0.0
                    self._kp[i], self._kd[i] = kp, kd
            self._weight = 0.0
            self._weight_target = self.max_weight
            self._weight_rate = 1.0 / max(ramp, 1e-3)

        self._start_loop()
        where = f"DRY RUN -> {self.topic}" if self.dry_run else self.topic
        if self.channel == "arm_sdk":
            self._log(f"cediendo el control ({ramp:.1f} s de rampa de peso hasta "
                      f"{self.max_weight:.2f}) por {where}…")
        else:
            self._log(f"tomando el control por {where}  "
                      f"(piernas: {self.legs_policy}, {ramp:.1f} s de rampa de "
                      f"ganancias)…")
        self.sleep(ramp + 0.3)
        drift = float(np.max(np.abs(self.q_all()[self.commanded] - self.q0[self.commanded])))
        flag = "OK" if drift < 0.02 else "⚠ REVISAR"
        self._log(f"control tomado. Deriva al ceder: {drift:.4f} rad "
                  f"({math.degrees(drift):.2f}°) {flag}")

    def release(self, home_speed: float = 0.4, weight_ramp: float = 2.0,
                home_to: dict[int, float] | None = None) -> None:
        """Devuelve el brazo a la postura inicial y suelta el control.

        Primero vuelve despacio a `q0` (la postura que había al llamar a
        `engage`), que es también la que el servicio del robot estaba
        sosteniendo; así, al bajar el peso a 0, no hay tirón.

        `home_to` cambia ese destino. Lo necesita quien coloca el robot a
        propósito y NO quiere que se deshaga al soltar: un ensayo mide y
        devuelve, pero preparar una postura para la teleoperación es lo
        contrario. Pasar `{}` suelta donde esté, sin mover nada.
        """
        if not self._running:
            return
        try:
            destino = ({i: float(self.q0[i]) for i in self.commanded}
                       if home_to is None else dict(home_to))
            if destino:
                self._log("volviendo a la postura inicial…"
                          if home_to is None else "colocando la postura de salida…")
                # Todas las comandadas, no solo las que se estaban midiendo: si se
                # aplicó una postura de ensayo (p. ej. hombros a ±18°), esas
                # articulaciones también hay que devolverlas a donde estaban.
                self.ramp_to(destino, speed=home_speed)
            self._log(("bajando el peso a 0" if self.channel == "arm_sdk"
                       else "bajando las ganancias a 0")
                      + f" en {weight_ramp:.1f} s…")
            with self._lock:
                self._weight_target = 0.0
                self._weight_rate = 1.0 / max(weight_ramp, 1e-3)
            self.sleep(weight_ramp + 0.3)
        finally:
            self._stop_loop()
            self._log("control devuelto al robot.")
            self._log("  " + self.loop_health())
            if self.gravity is not None:
                self._log(f"  gravedad: activa, {self._g_clip} ciclos recortados "
                          f"al {self.gravity_frac*100:.0f} % del par máximo")
            if self.collision_clamps:
                self._log(f"  ⚠ la protección de autocolisión recortó la consigna "
                          f"en {self.collision_clamps} ciclos "
                          f"({100*self.collision_clamps/max(self.cycles,1):.1f} %). "
                          f"La postura pedida no era compatible con el codo.")

    def emergency_release(self, reason: str) -> None:
        """Suelta ya, sin volver a la postura inicial. Solo para abortos.

        No para el lazo: `_trip` ya ha puesto el peso a bajar y el lazo SIGUE
        publicando hasta llegar a 0. Cortar la publicación de golpe dejaría al
        servicio del robot con el último peso que le mandamos —posiblemente
        1.0— y el relevo sería brusco. Aquí solo se espera a que termine.
        """
        with self._lock:
            self._weight_target = 0.0
            self._weight_rate = 1.0 / 0.4
        self._log(f"⚠ SUELTA DE EMERGENCIA: {reason}")
        t0 = time.monotonic()
        while self._running and time.monotonic() - t0 < 2.0:
            time.sleep(0.02)
        self._stop_loop()

    # -- consignas --------------------------------------------------------
    def set_target(self, idx: int, q: float, dq: float = 0.0,
                   tau_ff: float = 0.0) -> None:
        """Consigna instantánea (sin rampa). Recorta a los topes efectivos:
        los del URDF con margen, y los blandos de autocolisión."""
        with self._lock:
            self._q_des[idx] = self.gains.clamp(idx, q)
            self._dq_des[idx] = dq
            self._tau_ff[idx] = tau_ff

    def set_targets(self, targets: dict[int, float]) -> None:
        for i, q in targets.items():
            self.set_target(i, q)

    def set_trajectory(self, idx: int, func, q_base: float | None = None) -> None:
        """Engancha `func(t) -> (q_rel, dq_rel)` a una articulación.

        `t` cuenta desde este instante y `q_rel` es relativo a `q_base` (por
        defecto, la consigna actual). El lazo de control la evalúa cada ciclo.
        """
        with self._lock:
            base = float(self._q_des[idx]) if q_base is None else float(q_base)
            self._traj[idx] = (func, base, time.monotonic())

    def clear_trajectory(self, idx: int | None = None) -> None:
        """Congela la consigna donde esté y deja de evaluar la trayectoria."""
        with self._lock:
            if idx is None:
                self._traj.clear()
            else:
                self._traj.pop(idx, None)
            self._dq_des[:] = 0.0

    def trajectory_time(self, idx: int) -> float:
        with self._lock:
            entry = self._traj.get(idx)
        return time.monotonic() - entry[2] if entry else 0.0

    def set_gains(self, idx: int, kp: float, kd: float) -> None:
        with self._lock:
            self._kp[idx], self._kd[idx] = float(kp), float(kd)

    def goto_posture(self, speed: float = 0.25,
                           extra: dict[int, float] | None = None,
                           tol: float = 0.0087, iters: int = 6,
                           replace: bool = False) -> dict[int, float]:
        """Lleva las articulaciones de `test_posture_deg` a su ángulo REAL.

        Se hace DESPUÉS de `engage()` y despacio: es un movimiento real del
        robot, no una lectura.

        La postura de ensayo es una posición FÍSICA (anticolisión): lo que
        importa es dónde acaba el brazo, no qué se le pidió. Y con un PD sin
        integral esas dos cosas no coinciden: sosteniendo contra la gravedad
        queda un error permanente `tau_g/kp` que no se va nunca. Medido en
        `R_shoulder_roll` con kp=140: se le pide −18.0° y se queda en −15.5°,
        2.5° más cerca del cuerpo de lo previsto. En una postura que existe
        precisamente para no tocarse, eso no vale.

        Así que se corrige: se manda, se mide, y se desplaza la consigna por lo
        que falte, hasta que el ángulo real entre en `tol` (0.5° por defecto) o
        se agoten las iteraciones. Es acción integral, aplicada una vez.

        Devuelve {índice: ángulo objetivo} de lo que se ha movido.
        """
        # `replace` es para las posturas con nombre de F3: cada una es una
        # descripción COMPLETA de dónde va el brazo, y fundirla con la de
        # ensayo daría una cuarta postura que nadie ha verificado.
        posture = dict(extra or {})
        posture = {i: q for i, q in posture.items() if i in self.commanded}
        if not posture:
            return {}
        desde = {i: float(self.q0[i]) for i in posture}
        movidas = {i: q for i, q in posture.items()
                   if abs(q - desde[i]) > 1e-3}
        if not movidas:
            self._log("  postura de ensayo: ya estaba en su sitio.")
            return {}
        for i, q in movidas.items():
            j = BY_INDEX[i]
            self._log(f"  postura de ensayo: {j.name} "
                      f"{math.degrees(desde[i]):+.1f}° -> {math.degrees(q):+.1f}°")
        self.ramp_to(movidas, speed=speed)
        self.sleep(0.4)

        if self.dry_run:
            for i, q in movidas.items():
                self.q_base[i] = q
            return movidas

        # Corrección en lazo cerrado del error permanente de gravedad
        cmd = dict(movidas)
        for k in range(iters):
            self.sleep(0.35)                       # dejar asentar
            errs = {i: self.q(i) - q for i, q in movidas.items()}
            if max(abs(e) for e in errs.values()) <= tol:
                break
            for i, e in errs.items():
                # el signo: si el brazo se queda corto, se le pide de más
                cmd[i] = self.gains.clamp(i, cmd[i] - e)
            self.ramp_to(cmd, speed=speed)

        for i, q in movidas.items():
            real = self.q(i)
            err = real - q
            j = BY_INDEX[i]
            extra_cmd = cmd[i] - q
            flag = "" if abs(err) <= tol else "   ⚠ NO LLEGA"
            self._log(f"    {j.name}: real {math.degrees(real):+6.2f}°  "
                      f"objetivo {math.degrees(q):+6.2f}°  "
                      f"(error {math.degrees(err):+5.2f}°; hubo que pedir "
                      f"{math.degrees(extra_cmd):+5.2f}° de más){flag}")
            # Los ensayos parten de la CONSIGNA corregida: es la que mantiene
            # el brazo donde se quiere.
            self.q_base[i] = cmd[i]
        return movidas

    def ramp_to(self, targets: dict[int, float], speed: float = 0.4) -> None:
        """Lleva la consigna a `targets` a `speed` rad/s, bloqueando."""
        speed = min(abs(speed), self.safety.max_ref_velocity)
        self.clear_trajectory()      # una rampa manda sobre cualquier trayectoria
        with self._lock:
            start = {i: float(self._q_des[i]) for i in targets}
        # Recorte con el tope CONDICIONADO al codo, no con el fijo. El fijo de
        # ±10° es el respaldo para cuando no se sabe dónde está el codo; aquí
        # sí se sabe, y con el codo flexionado el hombro puede llegar a 0°.
        # Sin esto la tabla `shoulder_roll_vs_elbow_deg` no autorizaba nada:
        # solo actuaba a través de `enforce_pairs`, que impide empeorar pero
        # no permite acercarse.
        #
        # El codo que se usa es el MÁS ESTIRADO entre el de ahora y el de
        # destino, porque durante la rampa se pasa por los dos y el requisito
        # crece con la extensión.
        pares = self.gains.cond_pairs()
        goal = {}
        for i, q in targets.items():
            e = pares.get(i)
            if e is None:
                goal[i] = self.gains.clamp(i, q)
                continue
            q_codo = max(abs(float(self._q_des[e])), abs(float(targets.get(e, self._q_des[e]))))
            lo, hi = self.gains.limits_dynamic(i, q_codo)
            goal[i] = min(max(q, lo), hi)
        dist = max((abs(goal[i] - start[i]) for i in targets), default=0.0)
        if dist < 1e-6:
            return
        duration = dist / speed
        t0 = time.monotonic()
        while True:
            self._raise_if_aborted()
            a = min((time.monotonic() - t0) / duration, 1.0)
            # coseno alzado: velocidad 0 al principio y al final, sin tirones
            s = 0.5 - 0.5 * math.cos(math.pi * a)
            with self._lock:
                for i in targets:
                    self._q_des[i] = start[i] + s * (goal[i] - start[i])
            if a >= 1.0:
                return
            time.sleep(self.dt)

    def leg_drift(self) -> tuple[float, int]:
        """(radianes, índice) de la pierna que más se ha movido desde `engage`.

        En los ensayos de brazo las piernas no deberían moverse. Si lo hacen
        —por reacción al movimiento del brazo en el arnés, o porque alguien las
        dejó libres— conviene enterarse por un número y no de vista.
        """
        peor, quien = 0.0, -1
        for i in LEG_INDICES:
            d = abs(float(self.q(i)) - float(self.q0[i]))
            if d > peor:
                peor, quien = d, i
        return peor, quien

    def wait_settled(self, idx: int, dq_tol: float = 0.02,
                     timeout: float = 3.0, estable: float = 0.15) -> bool:
        """Espera a que la articulación esté REALMENTE quieta.

        Una pausa fija no basta y el precio de suponer que sí lo es está
        medido: los barridos con `--repeats 2` daban 29-69 % de sobreimpulso en
        `L_shoulder_roll` donde el mismo kp con una sola repetición da 1.4-17 %.
        La segunda repetición empezaba con la articulación aún volviendo de la
        primera, y `step_response` tomaba como punto de partida una posición
        que se estaba moviendo.

        Devuelve False si se agota el plazo, para que quien llame pueda decidir.
        """
        t0 = time.monotonic()
        quieta_desde = None
        while time.monotonic() - t0 < timeout:
            self._raise_if_aborted()
            # Solo la VELOCIDAD. Comparar q con q_des no vale como criterio de
            # reposo: con gravedad hay un error permanente legítimo, mayor que
            # cualquier tolerancia razonable a kp bajo, y la condición no se
            # cumpliría nunca.
            ok = abs(self.dq(idx)) < dq_tol
            if ok:
                if quieta_desde is None:
                    quieta_desde = time.monotonic()
                elif time.monotonic() - quieta_desde >= estable:
                    return True
            else:
                quieta_desde = None
            time.sleep(0.02)
        return False

    def sleep(self, seconds: float) -> None:
        """Como time.sleep, pero corta si salta la seguridad."""
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self._raise_if_aborted()
            time.sleep(min(0.01, max(t_end - time.monotonic(), 0.0)))
        self._raise_if_aborted()

    # -- registro ---------------------------------------------------------
    def _start_loop(self):
        if self._running:
            return
        self._running = True
        self._abort = None
        self._ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._ctrl_thread.start()

    def _stop_loop(self):
        self._running = False
        if self._ctrl_thread is not None:
            self._ctrl_thread.join(timeout=2.0)
            self._ctrl_thread = None

    def _raise_if_aborted(self):
        if self._abort is not None:
            raise SafetyAbort(self._abort)

    def loop_health(self) -> str:
        """Frecuencia real del lazo y ciclos que llegaron tarde.

        Importa: si el lazo no llega a su frecuencia, la consigna se actualiza
        a saltos y eso se confunde con un mal seguimiento de la articulación.
        """
        el = max(self._loop_t1 - self._loop_t0, 1e-9)
        hz = self.cycles / el
        pct = 100.0 * self.late_cycles / max(self.cycles, 1)
        flag = "" if pct < 2.0 else "   ⚠ el lazo no llega a su frecuencia"
        return (f"lazo: {hz:.1f} Hz reales de {self.rate_hz:.0f} pedidos, "
                f"{self.cycles} ciclos, {self.late_cycles} tarde ({pct:.1f} %){flag}")

    def _control_loop(self):
        self._loop_t0 = time.monotonic()
        next_tick = time.perf_counter()
        while self._running:
            next_tick += self.dt
            try:
                self._one_cycle()
            except SafetyAbort:
                # No se sale: `_trip` ha puesto el peso a bajar y hay que
                # SEGUIR publicando hasta que llegue a 0. Dejar de publicar de
                # golpe le dejaría al robot el último peso enviado.
                pass
            except Exception as exc:               # nunca dejar morir el hilo
                self._abort = f"error en el lazo de control: {exc!r}"
                with self._lock:
                    self._weight_target = 0.0
                    self._weight_rate = 1.0 / 0.4
            if self._abort is not None and self._weight <= 0.0:
                self._running = False
                break
            slack = next_tick - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                self.late_cycles += 1
                next_tick = time.perf_counter()    # re-sincroniza tras un atasco
            self.cycles += 1
            self._loop_t1 = time.monotonic()

    def _one_cycle(self):
        with self._state_lock:
            state = self._state
            age = time.monotonic() - self._state_stamp
        if state is None:
            return                       # todavía no ha llegado nada
        if age > self.safety.lowstate_timeout:
            # Se avisa, pero NO se corta: se sigue publicando con el último
            # mode_machine conocido, que es lo que hace falta para que la
            # rampa de bajada del peso llegue a su destino.
            self._trip(f"sin /lowstate desde hace {age:.2f} s")
        else:
            self._check_safety(state)

        now = time.monotonic()
        with self._lock:
            # trayectorias: se evalúan aquí, en el ciclo de control
            for i, (func, base, t0) in self._traj.items():
                q_rel, dq_rel = func(now - t0)
                self._q_des[i] = self.gains.clamp(i, base + q_rel)
                self._dq_des[i] = dq_rel

            # Autocolisión: la restricción acopla hombro y codo, así que se
            # comprueba sobre la PAREJA y se frena al que se esté moviendo.
            # Se mira `q_des`, no `q`: cuando la articulación real ha llegado
            # a una postura en colisión ya es tarde; hay que vetar la orden.
            movida = bool(self._wrist_idx & set(self._traj))
            self.collision_clamps += self.gains.enforce_pairs(
                self._q_des, self._q_prev, self._traj,
                movida, self.commanded)
            self._q_prev[:] = self._q_des

            # gravedad: rampa de entrada para no meter un escalón de par
            if self.gravity is not None:
                self._g_scale = min(1.0, self._g_scale + self.dt / max(self.gravity_ramp, 1e-3))
                if self.gravity_at == "des":
                    q_ref = self._q_des
                else:
                    q_ref = [state.motor_state[k].q for k in range(NUM_CMD_MOTOR)]
                g = self.gravity.tau(q_ref)
                for i in self.commanded:
                    # Una articulación LIBRE no recibe par. Con kp = kd = 0 no
                    # hay nada que sostenga la postura, así que un tau_ff es
                    # una orden de acelerar sin más: el motor integra y la
                    # articulación se va. Es el caso de las piernas con
                    # `--legs free`, que es el valor por defecto.
                    if self._kp[i] == 0.0 and self._kd[i] == 0.0:
                        self._tau_g[i] = 0.0
                        continue
                    tope = self.gravity_frac * BY_INDEX[i].tau_max
                    v = float(np.clip(g.get(i, 0.0), -tope, tope))
                    if abs(g.get(i, 0.0)) > tope:
                        self._g_clip += 1
                    self._tau_g[i] = v * self._g_scale

            # rampa de peso
            step = self._weight_rate * self.dt
            if self._weight < self._weight_target:
                self._weight = min(self._weight + step, self._weight_target)
            elif self._weight > self._weight_target:
                self._weight = max(self._weight - step, self._weight_target)
            weight = self._weight
            q_des = self._q_des.copy()
            dq_des = self._dq_des.copy()
            tau_ff = self._tau_ff + self._tau_g
            kp, kd = self._kp.copy(), self._kd.copy()

        m = self._msg
        m.mode_pr = 0                       # tobillos en modo Pitch/Roll
        m.mode_machine = int(state.mode_machine)
        # `weight` es la autoridad, de 0 a 1, y significa lo mismo en los dos
        # canales aunque se aplique distinto:
        #   arm_sdk  el servicio del robot mezcla nuestra consigna con la suya
        #   lowcmd   no hay nadie con quien mezclar, así que escalamos kp, kd y
        #            tau_ff. Con 0 el motor queda libre; con 1, mandamos del
        #            todo. Así entrar y salir del control es una rampa en los
        #            dos casos, y no un corte.
        scale = float(weight) if self.channel == "lowcmd" else 1.0
        for i in self.commanded:
            c = m.motor_cmd[i]
            c.mode = 1                      # 1 = habilitado
            c.q = float(q_des[i])
            c.dq = float(dq_des[i])
            c.tau = float(tau_ff[i]) * scale
            c.kp = float(kp[i]) * scale
            c.kd = float(kd[i]) * scale
        if self.channel == "arm_sdk":
            # El peso de mezcla viaja en el `q` de un motor que no existe.
            m.motor_cmd[WEIGHT_INDEX].q = float(weight)
        set_crc(m)
        self._pub.publish(m)


    def _check_safety(self, state: LowState):
        if self._abort is not None:
            return                       # ya se está soltando; no insistir
        now = time.monotonic()
        for i in self.controlled:
            j = BY_INDEX[i]
            ms = state.motor_state[i]
            limit = self.safety.tau_abort_fraction * j.tau_max
            if abs(ms.tau_est) > limit:
                since = self._tau_hot_since.setdefault(i, now)
                # Un escalón produce un pico legítimo. Solo aborta si el par
                # se mantiene alto: eso es un atasco o un choque, no un
                # transitorio.
                if now - since > 0.30:
                    self._trip(f"{j.name}: |tau|={abs(ms.tau_est):.1f} Nm por encima "
                               f"de {limit:.1f} Nm durante más de 0.3 s")
            else:
                self._tau_hot_since.pop(i, None)
            if ms.temperature[0] > self.safety.temperature_abort:
                self._trip(f"{j.name}: {ms.temperature[0]} °C, por encima de "
                           f"{self.safety.temperature_abort:.0f} °C")

    def _trip(self, reason: str):
        """Marca el aborto y pone el peso a bajar. Solo lanza la primera vez:
        si lanzara en cada ciclo, el lazo no llegaría a publicar la rampa."""
        if self._abort is not None:
            return
        self._abort = reason
        with self._lock:
            self._weight_target = 0.0
            self._weight_rate = 1.0 / 0.4
        self._log(f"\n⚠ ABORTO: {reason}")
        raise SafetyAbort(reason)

    def _log(self, msg: str):
        if self.verbose:
            print(msg, flush=True)

    # ================================================= gestión del contexto
    def __enter__(self) -> "H12Client":
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            if self._abort is not None:
                self.emergency_release(self._abort)
            else:
                self.release()
        except SafetyAbort as e:
            # Ha saltado la seguridad mientras volvíamos a casa. No debe tapar
            # la excepción que trajera el que llama.
            self._log(f"  (seguridad durante la salida: {e})")
            self.emergency_release(str(e))
        except Exception as e:
            self._log(f"  (error durante la salida: {e!r})")
        finally:
            self.shutdown()
        return False

    def shutdown(self):
        """Cierre ordenado. El orden NO es opcional.

        Si se destruye el nodo mientras el ejecutor sigue girando en su hilo,
        el proceso muere con `terminate called without an active exception` y
        el participante DDS se queda a medio destruir. La consecuencia no se ve
        en esa ejecución sino en la SIGUIENTE: el descubrimiento se atasca y
        `/lowstate` deja de llegar aunque el robot lo esté publicando.
        """
        self._stop_loop()
        try:
            self._exec.shutdown()
        except Exception:
            pass
        self._spin_stop.wait(timeout=2.0)
        self._spin_thread.join(timeout=2.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
