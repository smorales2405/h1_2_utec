#!/usr/bin/env python3
"""Demonio del gesto «six-seven», para correr EN EL ROBOT sin ordenador.

Arranca con el robot, se queda escuchando el mando, y al pulsar `R2+up` hace el
gesto y devuelve los brazos donde estaban. No necesita nada conectado.

    python3 six_seven_daemon.py                       # con los valores buenos
    python3 six_seven_daemon.py --once --trigger-now   # una vez, sin mando
    python3 six_seven_daemon.py --dry-run              # sin publicar nada

Equivale a esto, que es lo que se validó desde la laptop:

    ros2 run h1_2_arm_control six_seven_remote --ros-args \
        -p amplitude_deg:=10.0 -p speed:=1.0 -p duration:=10.0 \
        -p approach_speed:=0.75

───────────────────────────────────────────────────────────────────────────
POR QUÉ NO ES EL MISMO PROGRAMA QUE EN LA LAPTOP
───────────────────────────────────────────────────────────────────────────

El PC2 del robot **no tiene ROS 2**: tiene Python 3.10 y `unitree_sdk2py`
sobre `cyclonedds`, instalados sin Internet. El nodo de `ros_h1_2_ws` depende
de `rclpy` y de los mensajes de `unitree_ros2`, así que ahí no arranca.

Lo que cambia es el transporte. La matemática del gesto, la tabla de motores,
las ganancias y la protección de autocolisión son los mismos ficheros.

───────────────────────────────────────────────────────────────────────────
SEGURIDAD DE UN SERVICIO QUE ARRANCA SOLO
───────────────────────────────────────────────────────────────────────────

Esto va a estar siempre corriendo, así que:

* **Mientras espera no publica nada.** El robot tiene sus brazos enteros y el
  controlador de equilibrio ni se entera de que estamos aquí.
* **SIGTERM y SIGINT sueltan antes de morir.** `systemd` para con SIGTERM; sin
  eso el proceso moriría con el peso donde estuviera.
* **Un gesto cada vez.** Pulsar la combinación mientras se ejecuta no hace nada.
* **Si algo aborta** —par, temperatura, estado rancio— el peso baja a cero y el
  demonio vuelve a esperar en vez de caerse.
"""
from __future__ import annotations

import argparse
import math
import signal
import struct
import sys
import time

import gains as cfg
from arm_sdk_client import (ArmSdkClient, SafetyAbort, inicializa_dds,
                            instala_señales)
from joints import ARM_INDICES, BY_INDEX, BY_NAME

# Mapa de bits del mando, de `xRockerBtnDataStruct`. Verificado sobre el robot
# leyendo `wireless_remote`: `select` sale 0x0008, `A` 0x0100, `R2+up` 0x1010.
BOTONES = {"R1": 0, "L1": 1, "start": 2, "select": 3,
           "R2": 4, "L2": 5, "F1": 6, "F3": 7,
           "A": 8, "B": 9, "X": 10, "Y": 11,
           "up": 12, "right": 13, "down": 14, "left": 15}


def mascara(combo: str) -> int:
    m = 0
    for parte in combo.replace(" ", "").split("+"):
        if not parte:
            continue
        k = next((k for k in BOTONES if k.lower() == parte.lower()), None)
        if k is None:
            raise SystemExit(f"botón desconocido: '{parte}'")
        m |= 1 << BOTONES[k]
    return m


def lee_teclas(state) -> int | None:
    """Botones desde `wireless_remote`: 2 de cabecera, 2 de teclas, 5 floats."""
    if state is None:
        return None
    b = bytes(bytearray(state.wireless_remote))
    return struct.unpack_from("<H", b, 2)[0] if len(b) >= 4 else None


# ── el gesto ───────────────────────────────────────────────────────────────
def _envolvente(t: float, dur: float, fade: float) -> tuple[float, float]:
    if t <= 0.0 or t >= dur:
        return 0.0, 0.0
    if fade <= 1e-6:
        return 1.0, 0.0
    if t < fade:
        return (0.5 - 0.5 * math.cos(math.pi * t / fade),
                (math.pi / (2.0 * fade)) * math.sin(math.pi * t / fade))
    if t > dur - fade:
        u = dur - t
        return (0.5 - 0.5 * math.cos(math.pi * u / fade),
                -(math.pi / (2.0 * fade)) * math.sin(math.pi * u / fade))
    return 1.0, 0.0


def balancin(amp: float, freq: float, dur: float, fade: float, signo: float):
    """`f(t) -> (q, dq)` con envolvente, para que empiece y acabe con dq = 0.

    La `dq` lleva el término de la envolvente, `w'(t)·sin(ωt)`, y no solo la
    derivada del seno: mandar una velocidad que no es la derivada de la
    posición que se manda cuesta caro —medido en F5, `dq_des = 0` divide por
    4.6 el ancho de banda del codo—.
    """
    w = 2.0 * math.pi * freq

    def f(t: float):
        e, de = _envolvente(t, dur, fade)
        s, c = math.sin(w * t), math.cos(w * t)
        return signo * amp * e * s, signo * amp * (de * s + e * w * c)
    return f


def camino_libre(g, desde: dict, hasta: dict, n: int = 40):
    """¿Es segura la recta de `desde` a `hasta`? Devuelve (ok, peor margen)."""
    peor = float("inf")
    for k in range(n + 1):
        f = k / n
        q = {i: desde[i] + f * (hasta[i] - desde[i]) for i in hasta}
        for i_roll, i_elb in g.cond_pairs().items():
            if i_roll in q and i_elb in q:
                m = g.roll_min_abs(i_roll, q[i_elb])
                if m is not None:
                    peor = min(peor, abs(q[i_roll]) - m)
    return peor > 0.0, peor


class Gesto:
    def __init__(self, a):
        self.a = a
        self.g = cfg.load(a.gains)

    def postura(self, cli) -> dict:
        a = self.a
        if a.keep_posture:
            p = {i: cli.q(i) for i in ARM_INDICES}
            if a.palms_up:
                p[BY_NAME["L_wrist_roll"].idx] = math.radians(a.palm_up_left_deg)
                p[BY_NAME["R_wrist_roll"].idx] = math.radians(a.palm_up_right_deg)
            return p
        r = abs(math.radians(a.shoulder_roll_deg))
        return {
            BY_NAME["L_shoulder_pitch"].idx: math.radians(a.shoulder_pitch_deg),
            BY_NAME["R_shoulder_pitch"].idx: math.radians(a.shoulder_pitch_deg),
            BY_NAME["L_shoulder_roll"].idx: +r,
            BY_NAME["R_shoulder_roll"].idx: -r,
            BY_NAME["L_shoulder_yaw"].idx: 0.0,
            BY_NAME["R_shoulder_yaw"].idx: 0.0,
            BY_NAME["L_elbow"].idx: math.radians(a.elbow_deg),
            BY_NAME["R_elbow"].idx: math.radians(a.elbow_deg),
            BY_NAME["L_wrist_roll"].idx: math.radians(a.palm_up_left_deg),
            BY_NAME["R_wrist_roll"].idx: math.radians(a.palm_up_right_deg),
            BY_NAME["L_wrist_pitch"].idx: 0.0,
            BY_NAME["R_wrist_pitch"].idx: 0.0,
            BY_NAME["L_wrist_yaw"].idx: 0.0,
            BY_NAME["R_wrist_yaw"].idx: 0.0,
        }

    def ejecuta(self, cli) -> None:
        a = self.a
        q_motion = {i: cli.q(i) for i in ARM_INDICES}
        postura = self.postura(cli)
        amp = abs(math.radians(a.amplitude_deg))
        freq = abs(a.speed) / (2.0 * math.pi * amp)
        fade = a.fade or min(0.5 / max(freq, 1e-3), a.duration / 3.0)
        i_l = BY_NAME[f"L_{a.moving_joint}"].idx
        i_r = BY_NAME[f"R_{a.moving_joint}"].idx

        t = {}
        t0 = time.monotonic()
        cli.engage(ramp=a.engage_ramp)
        t["control"] = time.monotonic() - t0

        mueve = {i: q for i, q in postura.items()
                 if abs(q - q_motion[i]) > math.radians(0.2)}
        if mueve:
            libre, margen = camino_libre(self.g, q_motion, postura)
            t1 = time.monotonic()
            if libre:
                # Todo a la vez: el hombro hace de contrapeso del codo, y si se
                # mueven por separado la compensación llega tarde y el robot da
                # un paso. Observado el 2026-09-17.
                print(f"  recolocando a la vez (margen {math.degrees(margen):.1f}°)",
                      flush=True)
                cli.ramp_to(postura, speed=a.approach_speed)
            else:
                print(f"  ⚠ el camino recto choca; se escalona", flush=True)
                codos = [i for i in mueve if BY_INDEX[i].name.endswith("elbow")]
                otros = [i for i in mueve if i not in codos]
                cli.ramp_to({i: postura[i] for i in codos}, speed=a.approach_speed)
                cli.ramp_to({i: postura[i] for i in otros}, speed=a.approach_speed)
            t["recolocar"] = time.monotonic() - t1
            t2 = time.monotonic()
            cli.wait_all_settled(mueve, dq_tol=a.settle_dq_tol,
                                 timeout=a.settle_timeout)
            t["quietud"] = time.monotonic() - t2

        print(f"  oscilando {a.moving_joint} ±{a.amplitude_deg:.1f}° a "
              f"{freq:.2f} Hz, {a.duration:.0f} s", flush=True)
        cli.set_trajectory(i_l, balancin(amp, freq, a.duration, fade, -1.0),
                           q_base=postura[i_l])
        cli.set_trajectory(i_r, balancin(amp, freq, a.duration, fade, +1.0),
                           q_base=postura[i_r])
        t["hasta el gesto"] = time.monotonic() - t0
        cli.sleep(a.duration)
        cli.clear_trajectory()
        cli.wait_all_settled((i_l, i_r), dq_tol=a.settle_dq_tol,
                             timeout=a.settle_timeout)

        cli.release(home_to=q_motion, home_speed=a.approach_speed,
                    weight_ramp=a.engage_ramp + 0.5)
        print("  tiempos: " + "  ".join(f"{k} {v:.2f}s" for k, v in t.items()),
              flush=True)
        if cli.collision_clamps:
            print(f"  ⚠ autocolisión: {cli.collision_clamps} ciclos recortados",
                  flush=True)


def argumentos(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--combo", default="R2+up")
    p.add_argument("--nic", default=None,
                   help="interfaz de red para DDS; en el robot suele sobrar")
    p.add_argument("--gains", default="tuned",
                   help="conjunto de gains.yaml. `tuned` y no `tuned_gff`: en "
                        "arm_sdk el robot aplica SU compensación de gravedad")
    # postura
    p.add_argument("--shoulder-roll-deg", type=float, default=7.5)
    p.add_argument("--shoulder-pitch-deg", type=float, default=25.0)
    p.add_argument("--elbow-deg", type=float, default=0.0)
    p.add_argument("--palm-up-left-deg", type=float, default=-90.0)
    p.add_argument("--palm-up-right-deg", type=float, default=90.0)
    p.add_argument("--keep-posture", action="store_true")
    p.add_argument("--palms-up", action="store_true", default=True)
    # oscilación
    p.add_argument("--moving-joint", default="elbow",
                   choices=("elbow", "shoulder_pitch"))
    p.add_argument("--amplitude-deg", type=float, default=10.0)
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--fade", type=float, default=0.0)
    # control
    p.add_argument("--weight", type=float, default=1.0)
    p.add_argument("--approach-speed", type=float, default=0.75)
    p.add_argument("--engage-ramp", type=float, default=1.0)
    p.add_argument("--settle-dq-tol", type=float, default=0.05)
    p.add_argument("--settle-timeout", type=float, default=1.5)
    p.add_argument("--rate-hz", type=float, default=250.0)
    # modos
    p.add_argument("--once", action="store_true")
    p.add_argument("--trigger-now", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    return p.parse_args(argv)


def main(argv=None) -> int:
    a = argumentos(argv)
    m = mascara(a.combo)
    inicializa_dds(a.nic)
    gesto = Gesto(a)
    cli = ArmSdkClient(gesto.g, rate_hz=a.rate_hz, max_weight=a.weight)
    instala_señales(cli)
    try:
        cli.wait_for_state()
        print(f"\n  six-seven · esperando «{a.combo}» (0x{m:04X})", flush=True)
        print(f"  ganancias '{gesto.g.set_name}', peso {a.weight:.2f}, "
              f"{a.amplitude_deg:.1f}° a {a.speed:.2f} rad/s, "
              f"{a.duration:.0f} s", flush=True)
        if a.dry_run:
            print("  PRUEBA EN SECO: no se publica nada.", flush=True)
        print("  mientras espera NO publica nada.\n", flush=True)

        anterior = 0
        while True:
            if a.trigger_now:
                a.trigger_now = False
                disparar = True
            else:
                time.sleep(0.02)
                k = lee_teclas(cli.state())
                if k is None:
                    continue
                disparar = (k & m) == m and (anterior & m) != m
                anterior = k
            if not disparar:
                continue
            print(f"  ▶ «{a.combo}»", flush=True)
            if a.dry_run:
                print("  (en seco: no se mueve)", flush=True)
            else:
                try:
                    gesto.ejecuta(cli)
                except SafetyAbort as e:
                    # Abortar no debe tumbar el servicio: se suelta y se sigue
                    # esperando, que es lo que un demonio tiene que hacer.
                    print(f"  ⚠ abortado: {e}", flush=True)
                    cli.cierra()
            if a.once:
                return 0
            print(f"\n  esperando «{a.combo}»…\n", flush=True)
    except KeyboardInterrupt:
        # SIGTERM es una parada pedida, no un fallo: 0. SIGINT desde una
        # terminal conserva el 130 de siempre.
        return 0 if cli.señal == signal.SIGTERM else 130
    finally:
        cli.cierra()


if __name__ == "__main__":
    sys.exit(main())
