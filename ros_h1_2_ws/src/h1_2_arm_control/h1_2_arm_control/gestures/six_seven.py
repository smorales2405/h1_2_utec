#!/usr/bin/env python3
"""Gesto «six-seven»: las dos manos en balanza, palmas arriba.

Una sube mientras la otra baja, en contrafase, suave y periódico. Se mueve
**una sola articulación** —el codo o el hombro-pitch, a elegir— y el resto se
queda clavado en la postura que se le diga.

    # codo oscilando ±20° alrededor de 0°, con hombro y roll en 0°, a 0.5 rad/s
    ros2 run h1_2_arm_control six_seven --ros-args \
        -p moving_joint:=elbow \
        -p shoulder_roll_deg:=0.0 \
        -p shoulder_pitch_deg:=0.0 \
        -p elbow_deg:=0.0 \
        -p amplitude_deg:=20.0 \
        -p speed:=0.5 \
        -p duration:=8.0

Todos los ángulos en GRADOS sexagesimales; la velocidad en rad/s.

    moving_joint        `elbow` o `shoulder_pitch`: la que oscila
    shoulder_roll_deg   fijo, en valor absoluto (el signo lo pone el lado)
    shoulder_pitch_deg  fijo si oscila el codo; referencia si oscila él
    elbow_deg           fijo si oscila el hombro; referencia si oscila él
    amplitude_deg       ± desde la referencia, el mismo hacia arriba y abajo
    speed               velocidad angular de PICO de la oscilación, rad/s

La frecuencia no se pide: sale de la velocidad y la amplitud, que es como se
relacionan de verdad. Para un seno de amplitud A y pulsación ω la velocidad de
pico es A·ω, así que

    f = velocidad / (2π · amplitud)

El script la calcula, la enseña, y comprueba que el brazo puede seguirla.

───────────────────────────────────────────────────────────────────────────
POR QUÉ LA AMPLITUD ENTRA Y SALE CON UNA ENVOLVENTE
───────────────────────────────────────────────────────────────────────────

Un seno puro `A·sin(ωt)` cortado a los `duration` segundos termina con
velocidad `A·ω·cos(ωD)`, que salvo coincidencia NO es cero: el brazo se queda
a media carrera y la consigna desaparece de golpe.

Con una envolvente de coseno alzado que sube al principio y baja al final:

    q(t)  = q0 + s·A·w(t)·sin(ωt)
    dq(t) = s·A·[w'(t)·sin(ωt) + w(t)·ω·cos(ωt)]

en t=0 y en t=D valen w=0 y w'=0, así que el gesto **empieza y acaba en la
referencia y con velocidad nula**, sin ningún corte. Y `dq` lleva el término
de la envolvente: mandar la derivada del seno sin él sería mandar una
velocidad que no corresponde a la posición, y eso cuesta caro —medido en F5,
mandar `dq_des = 0` divide por 4.6 el ancho de banda del codo—.

───────────────────────────────────────────────────────────────────────────
DOS COSAS EN LAS QUE ESTO SE APARTA DE LA ESPECIFICACIÓN, A PROPÓSITO
───────────────────────────────────────────────────────────────────────────

**El canal es `lowcmd`, no `arm_sdk`.** La especificación (§13) prefiere
`arm_sdk`, y tendría razón si funcionara: es el mecanismo que cede solo los
brazos sin soltar la locomoción. Pero en este robot se probó con cinco
variantes de mensaje y **con el robot en reposo no hace nada**.

**Las palmas van a ±90°, no a 1.2–1.5 rad.** La especificación (§4) daba ese
rango y dejaba los signos por verificar, porque las manos Inspire van sobre
una adaptación y el cero visual de la palma no coincide con el del joint.
Verificado sobre el robot: `L_wrist_roll = −90°`, `R_wrist_roll = +90°`.

───────────────────────────────────────────────────────────────────────────
"""
from __future__ import annotations

import math
import sys

from .._node_base import ArmNode, ejecuta
from ..joints import ARM_INDICES, BY_INDEX, BY_NAME
from ..postures import to_rest, to_zero


# Ancho de banda a −3 dB medido en F5 con `tuned_gff`, chirp logarítmico
# 0.2→5 Hz y coherencia ≥ 0.98 (ver 06_RESULTADOS.md §17). Por encima de esto
# la articulación deja de seguir: la amplitud que sale es menor que la pedida.
ANCHO_BANDA_HZ = {"shoulder_pitch": 1.21, "elbow": 3.07}

# Fracción del ancho de banda hasta la que el seguimiento es fiel. A BW/3 la
# caída de amplitud es de décimas de dB; a BW ya son los 3 dB que definen el
# ancho de banda, o sea un 30 % de amplitud perdida.
FRACCION_BW = 1.0 / 3.0

# Suelo de velocidad. El ruido del canal de velocidad medido en los ensayos va
# de 0.007 a 0.014 rad/s (`chatter_dq`); por debajo de unas pocas veces eso, lo
# que se manda queda enterrado en el ruido y el movimiento sale a tirones.
VELOCIDAD_MINIMA = 0.05

MOVILES = ("elbow", "shoulder_pitch")


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
    """`f(t) -> (q, dq)` relativa a la referencia, con envolvente.

    `signo` −1 para el brazo izquierdo y +1 para el derecho, que es lo que los
    pone en contrafase: cuando uno sube el otro baja.
    """
    w = 2.0 * math.pi * freq

    def f(t: float):
        e, de = _envolvente(t, dur, fade)
        s, c = math.sin(w * t), math.cos(w * t)
        return signo * amp * e * s, signo * amp * (de * s + e * w * c)

    return f


def pico_velocidad(amp, freq, dur, fade, n=4000) -> float:
    """|dq| máximo real de la trayectoria, envolvente incluida."""
    f = balancin(amp, freq, dur, fade, 1.0)
    return max(abs(f(dur * k / n)[1]) for k in range(n + 1))


class SixSeven(ArmNode):
    def __init__(self, nombre="h1_2_six_seven"):
        super().__init__(nombre, {
            # qué se mueve
            "moving_joint": "elbow",
            # ángulos, en GRADOS
            "shoulder_roll_deg": 17.2,
            "shoulder_pitch_deg": 0.0,
            "elbow_deg": 0.0,
            "amplitude_deg": 20.0,
            "palm_up_left_deg": -90.0,
            "palm_up_right_deg": 90.0,
            # velocidad de pico de la oscilación, rad/s
            "speed": 0.5,
            "duration": 8.0,
            # Segundos de entrada y de salida de la amplitud. 0 = automático,
            # medio periodo, que es lo que hace el gesto reconocible sin
            # comerse los ciclos centrales.
            "fade": 0.0,
            "return_home": True,
            # velocidad de las rampas de colocación, rad/s
            "approach_speed": 0.15,
        })

    # ── postura ────────────────────────────────────────────────────────────
    def _postura(self) -> dict[int, float]:
        r = abs(math.radians(float(self.p("shoulder_roll_deg"))))
        p = math.radians(float(self.p("shoulder_pitch_deg")))
        e = math.radians(float(self.p("elbow_deg")))
        return {
            BY_NAME["L_shoulder_pitch"].idx: p,
            BY_NAME["R_shoulder_pitch"].idx: p,
            BY_NAME["L_shoulder_roll"].idx: +r,
            BY_NAME["R_shoulder_roll"].idx: -r,
            BY_NAME["L_shoulder_yaw"].idx: 0.0,
            BY_NAME["R_shoulder_yaw"].idx: 0.0,
            BY_NAME["L_elbow"].idx: e,
            BY_NAME["R_elbow"].idx: e,
            BY_NAME["L_wrist_roll"].idx: math.radians(float(self.p("palm_up_left_deg"))),
            BY_NAME["R_wrist_roll"].idx: math.radians(float(self.p("palm_up_right_deg"))),
            BY_NAME["L_wrist_pitch"].idx: 0.0,
            BY_NAME["R_wrist_pitch"].idx: 0.0,
            BY_NAME["L_wrist_yaw"].idx: 0.0,
            BY_NAME["R_wrist_yaw"].idx: 0.0,
        }

    def _moviles(self) -> tuple[int, int]:
        m = self.p("moving_joint")
        if m not in MOVILES:
            raise SystemExit(f"  moving_joint='{m}': tiene que ser uno de "
                             f"{', '.join(MOVILES)}")
        return BY_NAME[f"L_{m}"].idx, BY_NAME[f"R_{m}"].idx

    # ── comprobaciones, antes de mover nada ────────────────────────────────
    def _comprueba(self, cli, postura, i_l, i_r, amp, freq, dur, fade) -> bool:
        ok = True
        m = self.p("moving_joint")
        ref = postura[i_l]

        # 1. la postura fija, contra los topes.
        #
        #    Para los `shoulder_roll` hay que usar el tope CONDICIONADO al
        #    codo, no el fijo de ±10°. El fijo es el respaldo para cuando no se
        #    sabe dónde está el codo; aquí sí se sabe, y con el codo flexionado
        #    el hombro puede llegar a 0°. Usar el fijo aquí haría esta
        #    comprobación más estricta que el `ramp_to` que luego ejecuta el
        #    movimiento, y rechazaría posturas que el robot sí adopta —de hecho
        #    `init_pose` lleva los hombros a 0° todos los días—.
        #    El recorrido completo lo cubre la comprobación 3.
        pares = cli.gains.cond_pairs()
        for i, v in postura.items():
            i_elb = pares.get(i)
            if i_elb is not None and i_elb in postura:
                lo, hi = cli.gains.limits_dynamic(i, postura[i_elb])
            else:
                lo, hi = cli.gains.limits(i)
            if not (lo <= v <= hi):
                print(f"  ✗ {BY_INDEX[i].name} pide {math.degrees(v):.1f}°, "
                      f"fuera de [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")
                ok = False

        # 2. los EXTREMOS de la oscilación, que es lo que la postura no dice
        for i in (i_l, i_r):
            lo, hi = cli.gains.limits(i)
            for extremo in (postura[i] - amp, postura[i] + amp):
                if not (lo <= extremo <= hi):
                    print(f"  ✗ {BY_INDEX[i].name} oscilaría hasta "
                          f"{math.degrees(extremo):.1f}°, fuera de "
                          f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")
                    ok = False

        # 3. autocolisión EN TODO EL RECORRIDO, no solo en la referencia.
        #    Con el codo oscilando, el |roll| que la envolvente exige cambia a
        #    lo largo del ciclo: crece con la extensión del codo. Hay que mirar
        #    el extremo que más pide, no el punto medio.
        for i_roll, i_elb in pares.items():
            if i_roll not in postura or i_elb not in postura:
                continue
            codos = ([postura[i_elb] - amp, postura[i_elb], postura[i_elb] + amp]
                     if i_elb in (i_l, i_r) else [postura[i_elb]])
            rolls = ([postura[i_roll] - amp, postura[i_roll], postura[i_roll] + amp]
                     if i_roll in (i_l, i_r) else [postura[i_roll]])
            for qe in codos:
                for qr in rolls:
                    if cli.gains.pair_ok(i_roll, qr, qe):
                        continue
                    minimo = cli.gains.roll_min_abs(i_roll, qe)
                    print(f"  ✗ autocolisión: con {BY_INDEX[i_elb].name} a "
                          f"{math.degrees(qe):.1f}° la envolvente exige "
                          f"|{BY_INDEX[i_roll].name}| ≥ "
                          f"{math.degrees(minimo):.1f}°, y se piden "
                          f"{abs(math.degrees(qr)):.1f}°")
                    print(f"     sube `shoulder_roll_deg` a "
                          f"{math.degrees(minimo):.1f}° o más, o baja "
                          f"`amplitude_deg`.")
                    ok = False

        # 4. velocidad de pico contra el límite del paquete y contra el suelo
        pico = pico_velocidad(amp, freq, dur, fade)
        tope_v = cli.safety.max_ref_velocity
        print(f"  velocidad de pico {pico:.3f} rad/s   "
              f"(pedida {float(self.p('speed')):.3f}, límite {tope_v:.2f})")
        if pico > tope_v:
            print(f"  ✗ por encima de `max_ref_velocity`. Baja `speed` o la "
                  f"amplitud.")
            ok = False
        if float(self.p("speed")) < VELOCIDAD_MINIMA:
            print(f"  ⚠ por debajo de {VELOCIDAD_MINIMA} rad/s el movimiento "
                  f"queda cerca del ruido de velocidad medido (0.007–0.014 "
                  f"rad/s) y puede salir a tirones.")

        # 5. ¿puede la articulación seguir esa frecuencia? F5 lo midió.
        bw = ANCHO_BANDA_HZ.get(m)
        if bw:
            print(f"  frecuencia {freq:.3f} Hz, periodo {1.0/max(freq,1e-9):.2f} s"
                  f"   (ancho de banda medido de {m}: {bw:.2f} Hz)")
            if freq > bw:
                print(f"  ✗ por encima del ancho de banda: el brazo NO va a "
                      f"llegar a la amplitud pedida.")
                ok = False
            elif freq > bw * FRACCION_BW:
                print(f"  ⚠ por encima de BW/3 ({bw*FRACCION_BW:.2f} Hz): el "
                      f"seguimiento empieza a perder amplitud.")

        # 6. margen de par
        if cli.gravity is not None:
            q = cli.q_all().copy()
            for i, v in postura.items():
                q[i] = v
            peor, donde = 0.0, None
            for extremo in (ref - amp, ref, ref + amp):
                q[i_l] = extremo
                q[i_r] = extremo
                t = cli.gravity.tau(q)
                for i in (i_l, i_r):
                    if abs(t.get(i, 0.0)) > peor:
                        peor, donde = abs(t.get(i, 0.0)), i
            if donde is not None:
                tope_t = (cli.safety.tau_abort_fraction * BY_INDEX[donde].tau_max)
                print(f"  par de gravedad en {BY_INDEX[donde].name} {peor:.1f} N "
                      f"de {tope_t:.1f} que aborta   (margen {tope_t - peor:.1f} N)")
                if tope_t - peor < 5.0:
                    print(f"  ⚠ margen escaso para el transitorio.")
        return ok

    # ── ejecución ──────────────────────────────────────────────────────────
    def run(self) -> int:
        m = self.p("moving_joint")
        i_l, i_r = self._moviles()
        postura = self._postura()
        amp = abs(math.radians(float(self.p("amplitude_deg"))))
        vel = abs(float(self.p("speed")))
        dur = float(self.p("duration"))
        v_ramp = float(self.p("approach_speed"))

        if amp < 1e-6:
            print("  amplitude_deg es 0: no hay gesto que hacer.")
            return 1
        # La frecuencia sale de la velocidad pedida y la amplitud: para un seno
        # la velocidad de pico es A·ω, así que ω = v/A.
        freq = vel / (2.0 * math.pi * amp)
        fade = float(self.p("fade")) or min(0.5 / max(freq, 1e-3), dur / 3.0)

        with self.cliente() as cli:
            cli.wait_for_state()
            print(f"\n  six-seven · oscila {m} ±{float(self.p('amplitude_deg')):.1f}° "
                  f"alrededor de {math.degrees(postura[i_l]):.1f}°, "
                  f"{dur:.1f} s")
            print(f"  fijos: shoulder_roll ±{abs(float(self.p('shoulder_roll_deg'))):.1f}°, "
                  f"shoulder_pitch {float(self.p('shoulder_pitch_deg')):.1f}°, "
                  f"elbow {float(self.p('elbow_deg')):.1f}°")
            if not self._comprueba(cli, postura, i_l, i_r, amp, freq, dur, fade):
                print("\n  no se ejecuta.")
                return 1

            cli.engage()

            print("\n── 1/4 · colocando en 0° ────────────────────────────")
            to_zero(cli, speed=v_ramp)

            print("\n── 2/4 · postura del gesto ──────────────────────────")
            # Los brazos primero y las palmas después: girar la muñeca 90° con
            # el brazo todavía colgando la llevaría cerca de la pierna.
            munecas = {i: q for i, q in postura.items()
                       if BY_INDEX[i].group == "wrist"}
            brazos = {i: q for i, q in postura.items() if i not in munecas}
            print("  brazos…")
            cli.ramp_to(brazos, speed=v_ramp)
            print("  palmas arriba…")
            cli.ramp_to(munecas, speed=v_ramp)
            for i in postura:
                cli.wait_settled(i)

            print(f"\n── 3/4 · gesto ({dur * freq:.1f} ciclos) ────────────")
            cli.set_trajectory(i_l, balancin(amp, freq, dur, fade, -1.0),
                               q_base=postura[i_l])
            cli.set_trajectory(i_r, balancin(amp, freq, dur, fade, +1.0),
                               q_base=postura[i_r])
            cli.sleep(dur)
            # La envolvente ya ha devuelto la consigna a la referencia con
            # velocidad nula, así que soltarla aquí no es un corte.
            cli.clear_trajectory()
            for i in (i_l, i_r):
                cli.wait_settled(i)
            print(f"  acabó en L {math.degrees(cli.q(i_l)):+.2f}°   "
                  f"R {math.degrees(cli.q(i_r)):+.2f}°   "
                  f"(referencia {math.degrees(postura[i_l]):+.1f}°)")

            print("\n── 4/4 · a reposo ───────────────────────────────────")
            if self.p("return_home"):
                reposo = to_rest(cli, speed=v_ramp)
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
