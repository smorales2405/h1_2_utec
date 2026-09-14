#!/usr/bin/env python3
"""Gesto «six-seven»: las dos manos en balanza, palmas arriba.

Una sube mientras la otra baja, en contrafase, suave y periódico. El
movimiento vive en los `shoulder_pitch`; el resto de articulaciones se queda
en la postura base.

    ros2 run h1_2_arm_control six_seven
    ros2 run h1_2_arm_control six_seven --ros-args -p duration:=8.0
    ros2 run h1_2_arm_control six_seven --ros-args -p duration:=6.0 \
                                                   -p amplitude:=0.10 \
                                                   -p frequency:=0.5

La secuencia completa es **colocar en 0° → gesto → reposo**, y va entera en
este proceso. Encadenar comandos no valdría: al soltar, las ganancias bajan a
cero y la gravedad se lleva los codos —medido, de 1° a 79° en segundos— así
que el siguiente proceso ya no encontraría el brazo donde lo dejaron.

───────────────────────────────────────────────────────────────────────────
POR QUÉ LA AMPLITUD ENTRA Y SALE CON UNA ENVOLVENTE
───────────────────────────────────────────────────────────────────────────

Un seno puro `A·sin(ωt)` cortado a los `duration` segundos termina con
velocidad `A·ω·cos(ωD)`, que salvo coincidencia NO es cero: el brazo se queda
a media carrera y la consigna desaparece de golpe. La especificación pide
acabar «en un punto con velocidad cercana a cero».

Con una envolvente de coseno alzado que sube al principio y baja al final:

    q(t)  = q0 + s·A·w(t)·sin(ωt)
    dq(t) = s·A·[w'(t)·sin(ωt) + w(t)·ω·cos(ωt)]

en t=0 y en t=D valen w=0 y w'=0, así que el gesto **empieza y acaba en el
centro y con velocidad nula**, sin ningún corte. Y `dq` lleva el término de la
envolvente: mandar la derivada del seno sin él sería mandar una velocidad que
no corresponde a la posición, que es justo el error que la especificación
advierte en su §9.

───────────────────────────────────────────────────────────────────────────
DOS COSAS EN LAS QUE ESTO SE APARTA DE LA ESPECIFICACIÓN, A PROPÓSITO
───────────────────────────────────────────────────────────────────────────

**El canal es `lowcmd`, no `arm_sdk`.** La especificación (§13) prefiere
`arm_sdk`, y tendría razón si funcionara: es el mecanismo que cede solo los
brazos sin soltar la locomoción. Pero en este robot se probó con cinco
variantes de mensaje y **con el robot en reposo no hace nada**. Todo el
paquete, y toda la sintonización de la que salen estas ganancias, va por
`lowcmd` con el controlador de alto nivel soltado. Seguir la especificación
aquí daría un gesto que no mueve el robot.

**Las palmas van a ±90°, no a 1.2–1.5 rad.** La especificación (§4) daba ese
rango y dejaba los signos por verificar, precisamente porque las manos Inspire
van sobre una adaptación y el cero visual de la palma no coincide con el cero
del joint. Verificado sobre el robot: `L_wrist_roll = −90°` y
`R_wrist_roll = +90°`. Son parámetros, así que se cambian sin tocar el código.

───────────────────────────────────────────────────────────────────────────
"""
from __future__ import annotations

import math
import sys

from .._node_base import ArmNode, ejecuta
from ..joints import ARM_INDICES, BY_INDEX, BY_NAME
from ..postures import to_rest, to_zero


# ── Postura base del gesto ─────────────────────────────────────────────────
# Todos en radianes. El `roll` va en valor absoluto: el signo lo pone el lado,
# positivo separa el brazo izquierdo del cuerpo y negativo el derecho.
CENTER_PITCH = -0.75        # -43°, centro de la oscilación
SHOULDER_ROLL = 0.30        # ±17°, separa el brazo del torso
ELBOW_CENTER = 0.45         # 26°, antebrazo flexionado

# Palmas arriba. Estos dos valores NO salen de la especificación —que proponía
# 1.2–1.5 rad y dejaba el signo por verificar— sino del montaje real de las
# manos Inspire, comprobado sobre el robot: el cero visual de la palma no
# coincide con el cero del joint porque las manos van sobre una adaptación.
PALM_UP_L = -math.pi / 2    # -90°
PALM_UP_R = +math.pi / 2    # +90°

# ── Oscilación ─────────────────────────────────────────────────────────────
AMPLITUDE = 0.15            # rad; el pitch recorre [-0.90, -0.60]
FREQUENCY = 0.75            # Hz
DURATION = 4.0              # s, unos 3 ciclos a 0.75 Hz


def _envolvente(t: float, dur: float, fade: float) -> tuple[float, float]:
    """(w, dw/dt) del coseno alzado que abre y cierra el gesto."""
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
    """`f(t) -> (q, dq)` relativa al centro, con envolvente.

    `signo` −1 para el brazo izquierdo y +1 para el derecho, que es lo que los
    pone en contrafase: cuando uno sube el otro baja.
    """
    w = 2.0 * math.pi * freq

    def f(t: float):
        e, de = _envolvente(t, dur, fade)
        s, c = math.sin(w * t), math.cos(w * t)
        return signo * amp * e * s, signo * amp * (de * s + e * w * c)

    return f


def pico_velocidad(amp, freq, dur, fade, n=2000) -> float:
    """|dq| máximo de la trayectoria, muestreado. Barato y exacto de sobra."""
    f = balancin(amp, freq, dur, fade, 1.0)
    return max(abs(f(dur * k / n)[1]) for k in range(n + 1))


class SixSeven(ArmNode):
    def __init__(self, nombre="h1_2_six_seven"):
        super().__init__(nombre, {
            "duration": DURATION,
            "frequency": FREQUENCY,
            "amplitude": AMPLITUDE,
            "center_pitch": CENTER_PITCH,
            "shoulder_roll": SHOULDER_ROLL,
            "elbow_center": ELBOW_CENTER,
            "palm_up_left": PALM_UP_L,
            "palm_up_right": PALM_UP_R,
            # Oscilación del codo. La especificación la deja para después de
            # validar la versión básica, así que va apagada por defecto.
            "elbow_swing": 0.0,
            # Segundos de entrada y de salida de la amplitud. 0 = automático,
            # medio periodo, que es lo que hace el gesto reconocible sin comerse
            # los ciclos centrales.
            "fade": 0.0,
            "return_home": True,
        })

    # ── postura base ───────────────────────────────────────────────────────
    def _postura(self) -> dict[int, float]:
        c = float(self.p("center_pitch"))
        r = abs(float(self.p("shoulder_roll")))
        e = float(self.p("elbow_center"))
        return {
            BY_NAME["L_shoulder_pitch"].idx: c,
            BY_NAME["R_shoulder_pitch"].idx: c,
            BY_NAME["L_shoulder_roll"].idx: +r,
            BY_NAME["R_shoulder_roll"].idx: -r,
            BY_NAME["L_shoulder_yaw"].idx: 0.0,
            BY_NAME["R_shoulder_yaw"].idx: 0.0,
            BY_NAME["L_elbow"].idx: e,
            BY_NAME["R_elbow"].idx: e,
            BY_NAME["L_wrist_roll"].idx: float(self.p("palm_up_left")),
            BY_NAME["R_wrist_roll"].idx: float(self.p("palm_up_right")),
            BY_NAME["L_wrist_pitch"].idx: 0.0,
            BY_NAME["R_wrist_pitch"].idx: 0.0,
            BY_NAME["L_wrist_yaw"].idx: 0.0,
            BY_NAME["R_wrist_yaw"].idx: 0.0,
        }

    def _comprueba(self, cli, postura, fade) -> bool:
        """Topes, velocidad de pico y margen de par, ANTES de mover nada."""
        ok = True
        amp = float(self.p("amplitude"))
        freq = float(self.p("frequency"))
        dur = float(self.p("duration"))
        c = float(self.p("center_pitch"))

        # 1. la postura base y los extremos de la oscilación, contra los topes
        for i, v in postura.items():
            lo, hi = cli.gains.limits(i)
            if not (lo <= v <= hi):
                print(f"  ✗ {BY_INDEX[i].name} pide {math.degrees(v):.1f}°, "
                      f"fuera de [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")
                ok = False
        for n in ("L_shoulder_pitch", "R_shoulder_pitch"):
            i = BY_NAME[n].idx
            lo, hi = cli.gains.limits(i)
            for extremo in (c - amp, c + amp):
                if not (lo <= extremo <= hi):
                    print(f"  ✗ {n} oscilaría hasta {math.degrees(extremo):.1f}°, "
                          f"fuera de sus topes")
                    ok = False

        # 2. autocolisión de la postura base
        for i_roll, i_elb in cli.gains.cond_pairs().items():
            if i_roll in postura and i_elb in postura:
                if not cli.gains.pair_ok(i_roll, postura[i_roll], postura[i_elb]):
                    minimo = cli.gains.roll_min_abs(i_roll, postura[i_elb])
                    print(f"  ✗ {BY_INDEX[i_roll].name} a "
                          f"{math.degrees(postura[i_roll]):.1f}° con el codo a "
                          f"{math.degrees(postura[i_elb]):.1f}°: la envolvente "
                          f"exige |roll| ≥ {math.degrees(minimo):.1f}°")
                    ok = False

        # 3. velocidad de pico contra el límite del paquete
        pico = pico_velocidad(amp, freq, dur, fade)
        tope_v = cli.safety.max_ref_velocity
        marca = "" if pico <= tope_v else "   ✗ POR ENCIMA DEL LÍMITE"
        print(f"  velocidad de pico {pico:.3f} rad/s   (límite {tope_v:.2f})"
              f"{marca}")
        if pico > tope_v:
            print(f"    baja la amplitud o la frecuencia: el pico va como "
                  f"amplitud × 2π × frecuencia.")
            ok = False

        # 4. margen de par. La gravedad ya consume buena parte en esta postura.
        if cli.gravity is not None:
            import numpy as np
            q = cli.q_all().copy()
            for i, v in postura.items():
                q[i] = v
            peor = 0.0
            for extremo in (c - amp, c + amp):
                q[BY_NAME["L_shoulder_pitch"].idx] = extremo
                q[BY_NAME["R_shoulder_pitch"].idx] = extremo
                t = cli.gravity.tau(q)
                peor = max(peor, abs(t.get(BY_NAME["L_shoulder_pitch"].idx, 0.0)))
            tope_t = (cli.safety.tau_abort_fraction
                      * BY_INDEX[BY_NAME["L_shoulder_pitch"].idx].tau_max)
            print(f"  par de gravedad en el hombro {peor:.1f} N de {tope_t:.1f} "
                  f"que aborta   (margen {tope_t - peor:.1f} N para el "
                  f"transitorio)")
            if tope_t - peor < 5.0:
                print(f"    ⚠ margen escaso. Si aborta a mitad, baja la "
                      f"amplitud o acerca `center_pitch` a 0.")
        return ok

    def run(self) -> int:
        dur = float(self.p("duration"))
        freq = float(self.p("frequency"))
        amp = float(self.p("amplitude"))
        swing = float(self.p("elbow_swing"))
        fade = float(self.p("fade")) or min(0.5 / max(freq, 1e-3), dur / 3.0)
        v = float(self.p("speed"))
        postura = self._postura()

        with self.cliente() as cli:
            cli.wait_for_state()

            print(f"\n  six-seven · {dur:.1f} s a {freq:.2f} Hz, amplitud "
                  f"{math.degrees(amp):.1f}°  ({dur * freq:.1f} ciclos, "
                  f"entrada y salida de {fade:.2f} s)")
            if not self._comprueba(cli, postura, fade):
                print("\n  no se ejecuta.")
                return 1

            cli.engage()

            print("\n── 1/4 · colocando en 0° ────────────────────────────")
            to_zero(cli, speed=v)

            print("\n── 2/4 · postura del gesto ──────────────────────────")
            # Los brazos primero y las palmas después: girar la muñeca 90° con
            # el brazo todavía colgando la llevaría cerca de la pierna.
            munecas = {i: q for i, q in postura.items()
                       if BY_INDEX[i].group == "wrist"}
            brazos = {i: q for i, q in postura.items() if i not in munecas}
            print("  brazos…")
            cli.ramp_to(brazos, speed=v)
            print("  palmas arriba…")
            cli.ramp_to(munecas, speed=v)
            for i in postura:
                cli.wait_settled(i)

            print(f"\n── 3/4 · gesto ──────────────────────────────────────")
            i_l = BY_NAME["L_shoulder_pitch"].idx
            i_r = BY_NAME["R_shoulder_pitch"].idx
            cli.set_trajectory(i_l, balancin(amp, freq, dur, fade, -1.0),
                               q_base=postura[i_l])
            cli.set_trajectory(i_r, balancin(amp, freq, dur, fade, +1.0),
                               q_base=postura[i_r])
            if swing > 0.0:
                for lado, signo in (("L", -1.0), ("R", +1.0)):
                    i = BY_NAME[f"{lado}_elbow"].idx
                    cli.set_trajectory(i, balancin(swing, freq, dur, fade, signo),
                                       q_base=postura[i])
            cli.sleep(dur)
            # La envolvente ya ha devuelto la consigna al centro con velocidad
            # nula, así que soltarla aquí no es un corte.
            cli.clear_trajectory()
            for i in (i_l, i_r):
                cli.wait_settled(i)
            print(f"  acabó en L {math.degrees(cli.q(i_l)):+.2f}°   "
                  f"R {math.degrees(cli.q(i_r)):+.2f}°   "
                  f"(centro {math.degrees(float(self.p('center_pitch'))):+.1f}°)")

            print("\n── 4/4 · a reposo ───────────────────────────────────")
            if self.p("return_home"):
                reposo = to_rest(cli, speed=v)
            else:
                reposo = {i: cli.q(i) for i in ARM_INDICES}

            if cli.collision_clamps:
                print(f"  ⚠ autocolisión: consigna recortada en "
                      f"{cli.collision_clamps} ciclos")
            deriva, idx = cli.leg_drift()
            if deriva > math.radians(2.0):
                print(f"  ⚠ las piernas se movieron: {BY_INDEX[idx].name} "
                      f"{math.degrees(deriva):.1f}°")
            print(f"  {cli.loop_health()}")
            cli.release(home_to=reposo)
        return 0


def main(args=None):
    sys.exit(ejecuta(SixSeven, "h1_2_six_seven", args))


if __name__ == "__main__":
    main()
