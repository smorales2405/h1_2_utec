#!/usr/bin/env python3
"""Lanza teleop_hand_and_arm.py sin tocar el codigo de Unitree, con estos cambios:

1. Enter_Debug_Mode() con 3 intentos como maximo. El original repite ReleaseMode() mientras CheckMode()
   devuelva un nombre de modo. En el H1-2 el SDK no puede cambiar la FSM (el Debug se pone con L2+R2 en el
   mando), asi que ese bucle podria no acabar nunca. Aqui se intenta 3 veces, se registra lo que responde y
   se sigue.
2. Al salir (q, Ctrl-C o un error), en este orden:
   a) los dos brazos van a 0 grados (antebrazos horizontales hacia delante);
   b) se abren del todo las dos manos, con los brazos quietos en 0 grados;
   c) los brazos vuelven a la postura medida al arrancar, antes de que se moviera nada. El programa termina
      ahi: en Debug los brazos se quedan sin fuerza al salir, y desde 0 grados los antebrazos caerian.
3. Si se corta el SSH, el teleop sale como con q (y por tanto hace lo mismo). El corte se detecta:
   - por el terminal colgado: se cerro la terminal o el ssh del PC;
   - porque el PC deja de responder al ping SIN_PING_S segundos: WiFi caida, portatil suspendido. Hace falta
     porque el sshd del robot no tiene ClientAliveInterval y con un corte silencioso no se entera en minutos.
   SIGHUP se ignora, tambien en los procesos hijos, para que el corte no los mate antes de salir.
"""
import os, sys, time, runpy, signal, subprocess, threading, multiprocessing
import numpy as np
TELEOP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "xr_teleoperate", "teleop")
SCRIPT = os.path.join(TELEOP, "teleop_hand_and_arm.py")
sys.path.insert(0, os.path.dirname(TELEOP))
os.chdir(TELEOP)
import logging_mp
# logging_mp exige basicConfig() antes de cualquier getLogger(), y robot_arm y robot_hand_inspire lo llaman al
# importarse: se configura aqui igual que en la primera linea del teleop y se anula la llamada del teleop.
logging_mp.basicConfig(level=logging_mp.INFO)
logging_mp.basicConfig = lambda *args, **kwargs: None
from teleop.utils import motion_switcher as ms
from teleop.robot_control import robot_arm, robot_hand_inspire

MANO_ABIERTA = 0.95   # angle_act/1000 de una mano abierta: ~0.99 los dedos, ~0.986 la rotacion del pulgar
ESPERA_MANOS_S = 3.0  # si no terminan de abrir en este tiempo, los brazos vuelven igual
VEL_BRAZOS = 1.0      # rad/s de media, en la junta que mas se mueve, en los dos movimientos de la salida
TOL_BRAZOS = 0.05     # rad; la misma que ctrl_dual_arm_go_home de Unitree
ESPERA_BRAZOS_S = 1.0 # tras la rampa, si no han llegado en este tiempo se sigue igual
SIN_PING_S = 3.0      # sin respuesta del PC durante esto -> q (la WiFi tiene picos de 0.5 s)


def aviso(texto):
    # Tras un corte del SSH puede no quedar donde escribir: que eso no tumbe el cierre ordenado
    try:
        print(f"[lanzar_teleop] {texto}", flush=True)
    except OSError:
        pass


def enter_debug_limitado(self, intentos=3):
    try:
        status, result = self.msc.CheckMode()
        aviso(f"CheckMode antes: {status} {result}")
        n = 0
        while result.get("name") and n < intentos:
            self.msc.ReleaseMode(); time.sleep(1); n += 1
            status, result = self.msc.CheckMode()
            aviso(f"ReleaseMode {n}/{intentos} -> CheckMode {status} {result}")
        if result.get("name"):
            aviso("AVISO: el modo sigue siendo %r. Pon Debug con L2+R2 en el mando." % result["name"])
        return status, result
    except Exception as e:
        aviso(f"Enter_Debug_Mode fallo: {e}")
        return None, None


# --- 2. salida: brazos a 0 grados, abrir las manos, brazos a la postura inicial ---
# Se crea antes de que el teleop arranque el proceso de las manos (fork), asi que lo comparten
ABRIR_MANOS = multiprocessing.Value("b", False)
MANOS = []   # el Inspire_Controller_FTP del teleop

_init_mano = robot_hand_inspire.Inspire_Controller_FTP.__init__
_enviar_mano = robot_hand_inspire.Inspire_Controller_FTP._send_hand_command
_ir_a_cero_unitree = robot_arm.H1_2_ArmController.ctrl_dual_arm_go_home


def init_mano(self, *args, **kwargs):
    _init_mano(self, *args, **kwargs)
    MANOS.append(self)


def enviar_mano(self, izq, der):
    # Corre en el proceso de las manos, a 100 Hz: con ABRIR_MANOS manda abrir en vez del retargeting
    if ABRIR_MANOS.value:
        izq = der = [1000] * robot_hand_inspire.Inspire_Num_Motors
    _enviar_mano(self, izq, der)


def abrir_manos():
    if not MANOS:
        return
    mano = MANOS[0]
    aviso("abriendo las manos")
    ABRIR_MANOS.value = True
    t0 = time.monotonic()
    while True:
        angulos = list(mano.left_hand_state_array[:]) + list(mano.right_hand_state_array[:])
        if min(angulos) >= MANO_ABIERTA:
            aviso(f"manos abiertas en {time.monotonic() - t0:.1f} s")
            return
        if time.monotonic() - t0 > ESPERA_MANOS_S:
            aviso(f"AVISO: las manos no terminaron de abrir en {ESPERA_MANOS_S:.0f} s "
                  f"(izq {[round(a, 2) for a in angulos[:6]]}, der {[round(a, 2) for a in angulos[6:]]}); "
                  "los brazos vuelven igual")
            return
        time.sleep(0.05)


def mover_brazos(self, q_final, destino):
    # Rampa suave desde la q medida. ctrl_dual_arm_go_home de Unitree salta al objetivo (en MuJoCo, 3.2 rad/s de
    # pico frente a 1.6 de esta rampa) y deja puesto el par de gravedad del ultimo instante del teleop, que ya no
    # corresponde a la postura. Aqui ese par baja a 0 con la rampa.
    q_ini = self.get_current_dual_arm_q()
    with self.ctrl_lock:
        tau_ini = np.array(self.tauff_target)
    duracion = max(np.max(np.abs(q_final - q_ini)) / VEL_BRAZOS, 0.2)
    aviso(f"brazos a {destino}: {duracion:.1f} s")
    t0 = time.monotonic()
    while (s := (time.monotonic() - t0) / duracion) < 1.0:
        suave = 0.5 - 0.5 * np.cos(np.pi * s)
        self.ctrl_dual_arm(q_ini + (q_final - q_ini) * suave, tau_ini * (1.0 - suave))
        time.sleep(0.004)
    self.ctrl_dual_arm(q_final, np.zeros_like(tau_ini))
    t0 = time.monotonic()
    while (error := np.max(np.abs(self.get_current_dual_arm_q() - q_final))) > TOL_BRAZOS:
        if time.monotonic() - t0 > ESPERA_BRAZOS_S:
            aviso(f"AVISO: brazos a {error:.3f} rad de {destino}; se sigue igual")
            return
        time.sleep(0.02)


def intentar(paso, *args):
    # Que el fallo de un paso no impida los siguientes
    try:
        paso(*args)
    except Exception as e:
        aviso(f"{paso.__name__} fallo: {e}")


def salida(self):
    # En Motion Mode (--motion, arrancar.sh no lo pasa) los brazos no se quedan sin fuerza: Unitree baja el peso
    # de arm_sdk y vuelven solos a su postura. No hay que llevarlos a ningun sitio ni medir nada al arrancar.
    if self.motion_mode:
        intentar(_ir_a_cero_unitree, self)
        intentar(abrir_manos)
        return
    # all_motor_q se mide en __init__, antes de que el controlador publique nada
    q_inicio = self.all_motor_q[[int(j) for j in robot_arm.H1_2_JointArmIndex]]
    intentar(mover_brazos, self, np.zeros(q_inicio.shape), "0 grados")
    intentar(abrir_manos)
    intentar(mover_brazos, self, q_inicio, "la postura inicial")


# --- 3. corte del SSH = q ---
def modulo_teleop():
    # Mientras corre, runpy.run_path deja el modulo del teleop en sys.modules["__main__"]
    g = sys.modules["__main__"].__dict__
    return g if g.get("__file__") == SCRIPT else None


def hay_terminal():
    # Al colgarse el terminal, el kernel se lo quita a toda la sesion y abrir /dev/tty da ENXIO
    try:
        os.close(os.open("/dev/tty", os.O_RDONLY | os.O_NOCTTY))
        return True
    except OSError:
        return False


def responde(ip):
    return subprocess.run(["ping", "-c", "1", "-W", "1", "-q", ip],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def vigilar_conexion():
    con_terminal = hay_terminal()
    ip = os.environ.get("SSH_CLIENT", "").split(" ")[0]
    # Se espera a que el teleop haya arrancado (y creado sus procesos) antes de lanzar pings
    while (g := modulo_teleop()) is None or not g.get("READY"):
        if g is not None and g.get("STOP"):
            return
        time.sleep(0.1)
    ultimo_ping = [None]   # no se vigila el ping hasta la primera respuesta, por si el PC no contesta pings

    def hacer_ping():
        while True:
            if responde(ip):
                ultimo_ping[0] = time.monotonic()
            time.sleep(0.5)

    if ip:
        threading.Thread(target=hacer_ping, name="ping_pc", daemon=True).start()
    aviso(f"vigilando el SSH: terminal {'si' if con_terminal else 'no'}, ping a {ip or '-'} "
          f"(q si no responde {SIN_PING_S:.0f} s)")
    t0, avisado = time.monotonic(), False
    while not g.get("STOP"):
        motivo = None
        if con_terminal and not hay_terminal():
            motivo = "se colgo el terminal del SSH"
        elif ultimo_ping[0] is not None and time.monotonic() - ultimo_ping[0] > SIN_PING_S:
            motivo = f"el PC {ip} no responde al ping desde hace {SIN_PING_S:.0f} s"
        elif ip and ultimo_ping[0] is None and not avisado and time.monotonic() - t0 > 5:
            aviso(f"AVISO: el PC {ip} no responde al ping: solo se vigila el terminal")
            avisado = True
        if motivo:
            aviso(f"CORTE: {motivo} -> salgo como con q")
            g["on_press"]("q")
            return
        time.sleep(0.2)


if __name__ == "__main__":
    # Antes de que se cree ningun proceso: el SIG_IGN lo heredan todos los hijos, por fork o por exec
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    # Precarga PIL y TODOS sus plugins antes de que televuer haga fork() del proceso de Vuer: si no, el hijo
    # hereda el cerrojo de importacion tomado por otro hilo y se bloquea al importar PngImagePlugin en el
    # primer JPEG que manda a las gafas (visto con py-spy el 2026-09-25: Vuer al 0 % y sockets sin leer).
    from PIL import Image as _PIL_Image; _PIL_Image.init()
    ms.MotionSwitcher.Enter_Debug_Mode = enter_debug_limitado
    robot_hand_inspire.Inspire_Controller_FTP.__init__ = init_mano
    robot_hand_inspire.Inspire_Controller_FTP._send_hand_command = enviar_mano
    robot_arm.H1_2_ArmController.ctrl_dual_arm_go_home = salida
    threading.Thread(target=vigilar_conexion, name="vigilar_conexion", daemon=True).start()
    sys.argv = ["teleop_hand_and_arm.py"] + sys.argv[1:]
    runpy.run_path(SCRIPT, run_name="__main__")
