#!/usr/bin/env python3
"""
Prueba de UNA articulación del brazo del H1-2, en escalera de tres peldaños.

Pensado como primer movimiento de los brazos, ANTES de lanzar la teleoperación
completa: mueve una sola articulación, despacio, un ángulo pequeño, y vuelve.

    read   Solo lectura. NO publica nada, no toca el robot.
           Muestra el modo actual y el ángulo/par/temperatura de las 14
           articulaciones de los brazos.

    hold   Entra en modo debug y mantiene la postura ACTUAL. No comanda ningún
           movimiento: el robot solo se pone rígido donde ya está. Sirve para
           comprobar que el control responde antes de mover nada.

    move   Como `hold`, y además mueve UNA articulación: delta pequeño, vuelta,
           y restauración del ángulo inicial al terminar.

⚠️  LEE ESTO ANTES DE USAR `hold` O `move`

1. `H1_2_ArmController` energiza el ROBOT ENTERO, no solo los brazos. Fija
   mode=1 y ganancias a las 27 articulaciones —piernas incluidas, con
   kp=300/kd=5— con q = la posición actual de cada una. Nada más construirlo, el
   robot se pone rígido en la postura en la que esté.

2. Entrar en modo debug SUELTA el servicio de locomoción (`ReleaseMode`). Si el
   robot está de pie sosteniéndose solo, deja de equilibrarse. **El robot debe
   estar colgado del arnés o firmemente apoyado**, nunca libre de pie.

3. El `H1_2_ArmController` original arranca yendo a 0°: fija
   `q_target = np.zeros(14)` al principio de `__init__` y lanza el hilo
   publicador (250 Hz) al final. En cuanto el constructor retorna, las 14
   articulaciones del brazo ya van hacia 0° a 30 rad/s. Este script usa una
   subclase que lo evita —ver `controlador_seguro()`— y comprueba la deriva
   real para demostrarlo.

4. PARO DE EMERGENCIA: `L2 + B` en el mando. Funciona también en modo debug
   (el manual del H1 lo confirma: dentro de debug, L2+B vuelve a amortiguación).
   Al hacerlo el robot se deja caer despacio al suelo: de ahí el arnés.
   Después pulsa Ctrl-C aquí para que el script deje de publicar.

Ejemplos:
    python3 arm_joint_test.py read  --network enp0s31f6
    python3 arm_joint_test.py hold  --network enp0s31f6 --seconds 10
    python3 arm_joint_test.py move  --network enp0s31f6 --joint L_elbow_pitch --delta 0.15
"""
import argparse
import sys
import time

import numpy as np

# Índice en el vector de 14 que usan get_current_dual_arm_q() / ctrl_dual_arm(),
# en el orden de H1_2_JointArmIndex (motores 13..26 del cuerpo).
ARTICULACIONES = {
    "L_shoulder_pitch": 0,  "L_shoulder_roll": 1,  "L_shoulder_yaw": 2,
    "L_elbow_pitch":    3,  "L_elbow_roll":    4,  "L_wrist_pitch":  5, "L_wrist_yaw": 6,
    "R_shoulder_pitch": 7,  "R_shoulder_roll": 8,  "R_shoulder_yaw": 9,
    "R_elbow_pitch":   10,  "R_elbow_roll":   11,  "R_wrist_pitch": 12, "R_wrist_yaw": 13,
}
MOTOR_DE_ARRAY = {v: 13 + v for v in ARTICULACIONES.values()}   # 0->13 ... 13->26


def leer_lowstate(network, segundos=8.0):
    """Suscripción pura a rt/lowstate. No publica nada."""
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()
    msg, t0 = None, time.time()
    while time.time() - t0 < segundos and msg is None:
        msg = sub.Read()
        time.sleep(0.02)
    return msg


def tabla(msg, resaltar=None):
    print(f"\n  mode_machine = {msg.mode_machine}")
    print(f"  {'idx':>3} {'articulacion':<18} {'q (rad)':>9} {'q (deg)':>9} "
          f"{'dq':>8} {'tau_est':>9} {'temp':>5}")
    for nombre, i in ARTICULACIONES.items():
        m = msg.motor_state[MOTOR_DE_ARRAY[i]]
        marca = " <—" if nombre == resaltar else ""
        print(f"  {i:>3} {nombre:<18} {m.q:>9.3f} {np.degrees(m.q):>9.1f} "
              f"{m.dq:>8.3f} {m.tau_est:>9.2f} {m.temperature[0]:>5}{marca}")
    piernas = [abs(msg.motor_state[i].tau_est) for i in range(13)]
    print(f"\n  |tau| máximo en piernas y torso (idx 0-12): {max(piernas):.2f} Nm")


def controlador_seguro(velocity_limit):
    """`H1_2_ArmController` que NO arranca llevando los brazos a 0°.

    El original fija `q_target = np.zeros(14)` en la primera línea de `__init__`
    y arranca el hilo publicador en la última. Entre esos dos momentos no hay
    forma de intervenir desde fuera: cuando el constructor retorna, el hilo ya
    lleva un rato publicando `rt/lowcmd` a 250 Hz con objetivo cero, y
    `clip_arm_q_target` avanza hasta `30 rad/s * 1/250 s = 0.12 rad` por paso.
    Bajar el límite después de construir llega tarde.

    Se intercepta el propio hilo: su `target` es `self._ctrl_motor_state`, que
    Python resuelve a esta subclase. Así lo PRIMERO que corre el hilo —antes de
    publicar nada— es fijar el objetivo en la postura ACTUAL y bajar el límite
    de velocidad. `ctrl_lock` y `lowstate_buffer` ya existen en ese punto:
    el `__init__` los deja listos antes de `start()`.
    """
    from teleop.robot_control.robot_arm import H1_2_ArmController

    class _ControladorSeguro(H1_2_ArmController):
        def _ctrl_motor_state(self):
            self.set_arm_velocity_limit(velocity_limit)
            actual = self.get_current_dual_arm_q()
            with self.ctrl_lock:
                self.q_target = actual
                self.tauff_target = np.zeros(14)
            super()._ctrl_motor_state()

    return _ControladorSeguro(motion_mode=False, simulation_mode=False)


def cuenta_atras(segundos, mensaje):
    print(f"\n  {mensaje}")
    for s in range(segundos, 0, -1):
        print(f"    {s}...", end="", flush=True)
        time.sleep(1)
    print(" ya")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["read", "hold", "move"])
    ap.add_argument("--network", default="enp0s31f6", help="NIC hacia el robot")
    ap.add_argument("--joint", default="L_elbow_pitch", choices=list(ARTICULACIONES),
                    help="articulación a mover (solo para `move`)")
    ap.add_argument("--delta", type=float, default=0.15,
                    help="desplazamiento en radianes (0.15 rad ≈ 8.6°)")
    ap.add_argument("--velocity-limit", type=float, default=0.15,
                    help="rad/s. El repo usa 30.0 por defecto: aquí se baja 200x a propósito")
    ap.add_argument("--seconds", type=float, default=8.0, help="duración de `hold`")
    ap.add_argument("--tau-max", type=float, default=25.0,
                    help="aborta si |tau_est| de la articulación supera esto (Nm)")
    ap.add_argument("--skip-debug-mode", action="store_true",
                    help="no llamar a Enter_Debug_Mode (el servicio de locomoción puede pelear el mando)")
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(0, a.network)

    # ---------- read: sin publicar nada ----------
    msg = leer_lowstate(a.network)
    if msg is None:
        print("✗ no llega rt/lowstate. ¿Robot encendido y cable conectado?")
        return 1
    print("=== Estado actual (solo lectura) ===")
    tabla(msg, resaltar=a.joint if a.cmd == "move" else None)
    if a.cmd == "read":
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
            msc = MotionSwitcherClient(); msc.SetTimeout(3.0); msc.Init()
            print(f"\n  MotionSwitcher.CheckMode() -> {msc.CheckMode()}")
        except Exception as e:
            print(f"\n  (CheckMode no disponible: {type(e).__name__}: {e})")
        print("\n  No se publicó nada. El robot no ha sido tocado.")
        return 0

    # ---------- a partir de aquí SÍ se toma el control ----------
    print("\n" + "!" * 72)
    print("  Lo siguiente ENERGIZA EL ROBOT ENTERO (piernas incluidas, kp=300)")
    print("  para sostener su postura actual, y suelta el servicio de locomoción.")
    print("  El robot debe estar COLGADO DEL ARNÉS o firmemente apoyado.")
    print("  Ctrl-C ahora para abortar.")
    print("!" * 72)
    cuenta_atras(5, "Empezando en")

    sys.path.insert(0, "/home/utec/Documents/h1_2_teleoperation/xr_teleoperate")

    # Postura de los brazos ANTES de tomar el control, para medir la deriva.
    q_antes = np.array([msg.motor_state[MOTOR_DE_ARRAY[i]].q
                        for i in sorted(ARTICULACIONES.values())])

    if not a.skip_debug_mode:
        from teleop.utils.motion_switcher import MotionSwitcher
        status, _ = MotionSwitcher().Enter_Debug_Mode()
        print(f"  modo debug: {'OK' if status == 0 else 'FALLÓ (el servicio puede pelear el mando)'}")

    arm = controlador_seguro(a.velocity_limit)
    q0 = arm.get_current_dual_arm_q().copy()
    tau0 = np.zeros(14)
    deriva = np.max(np.abs(q0 - q_antes))
    print(f"  límite de velocidad: {a.velocity_limit} rad/s (el repo usa 30.0)")
    print(f"  deriva al tomar el control: {deriva:.4f} rad ({np.degrees(deriva):.2f}°)"
          f"  {'✔ los brazos no se movieron' if deriva < 0.02 else '⚠ REVISAR'}")
    print(f"  postura inicial de los brazos: {np.round(q0, 3).tolist()}")

    try:
        if a.cmd == "hold":
            print(f"\n  Manteniendo la postura actual {a.seconds:.0f} s. No se comanda movimiento.")
            arm.ctrl_dual_arm(q0, tau0)
            t0 = time.time()
            while time.time() - t0 < a.seconds:
                q = arm.get_current_dual_arm_q()
                print(f"    deriva máx respecto al inicio: "
                      f"{np.max(np.abs(q - q0)):.4f} rad", end="\r", flush=True)
                time.sleep(0.5)
            print("\n  Sin deriva apreciable = el control responde bien.")
            return 0

        # ---------- move ----------
        i = ARTICULACIONES[a.joint]
        destino = q0.copy()
        destino[i] = q0[i] + a.delta
        print(f"\n  Moviendo SOLO {a.joint} (índice {i}, motor {MOTOR_DE_ARRAY[i]})")
        print(f"    {q0[i]:.3f} rad ({np.degrees(q0[i]):.1f}°)  ->  "
              f"{destino[i]:.3f} rad ({np.degrees(destino[i]):.1f}°)  y vuelta")
        print(f"    aborta si |tau_est| > {a.tau_max} Nm\n")

        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        estado = ChannelSubscriber("rt/lowstate", LowState_); estado.Init()
        motor = MOTOR_DE_ARRAY[i]

        def tramo(objetivo, etiqueta):
            arm.ctrl_dual_arm(objetivo, tau0)
            t0 = time.time()
            while time.time() - t0 < 12.0:
                q = arm.get_current_dual_arm_q()
                err = abs(q[i] - objetivo[i])
                s = estado.Read()
                tau = s.motor_state[motor].tau_est if s else 0.0
                print(f"    {etiqueta}: q={q[i]:+.3f} err={err:.4f} tau={tau:+.2f} Nm   ",
                      end="\r", flush=True)
                if abs(tau) > a.tau_max:
                    print(f"\n    ⚠ ABORTO: |tau| = {abs(tau):.2f} Nm > {a.tau_max}")
                    return False
                if err < 0.01:
                    print(f"\n    {etiqueta}: alcanzado (err {err:.4f} rad, tau {tau:+.2f} Nm)")
                    return True
                time.sleep(0.1)
            print(f"\n    ⚠ no alcanzó el objetivo en 12 s (err {err:.4f}) — ¿articulación trabada?")
            return False

        if tramo(destino, "ida"):
            time.sleep(1.0)
            tramo(q0, "vuelta")
        return 0

    except KeyboardInterrupt:
        print("\n  interrumpido")
        return 1
    finally:
        print("\n  Restaurando la postura inicial de los brazos...")
        arm.set_arm_velocity_limit(a.velocity_limit)
        arm.ctrl_dual_arm(q0, tau0)
        time.sleep(4.0)
        q = arm.get_current_dual_arm_q()
        print(f"  desviación final respecto al inicio: {np.max(np.abs(q - q0)):.4f} rad")
        print("\n  NOTA: el proceso sigue publicando lowcmd mientras viva. Al salir,")
        print("  el robot deja de recibir comandos: asegúrate de que está sujeto.")


if __name__ == "__main__":
    sys.exit(main())
