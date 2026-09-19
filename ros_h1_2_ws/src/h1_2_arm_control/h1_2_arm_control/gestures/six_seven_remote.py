#!/usr/bin/env python3
"""«six-seven» disparado desde el mando, con el robot DE PIE en Motion Mode.

Se queda escuchando el mando. Al pulsar la combinación —`R2+up` por defecto—
toma los brazos, hace el gesto, y los devuelve exactamente a donde estaban.
Igual que las acciones de fábrica de saludar o dar la mano.

    ros2 run h1_2_arm_control six_seven_remote --ros-args \
        -p amplitude_deg:=7.5 -p speed:=0.75 -p duration:=10.0

Ctrl-C para salir. Mientras espera **no publica nada**: el robot tiene sus
brazos enteros y el control de estabilidad no se entera de que estamos aquí.

───────────────────────────────────────────────────────────────────────────
POR QUÉ `arm_sdk` Y NO `lowcmd`
───────────────────────────────────────────────────────────────────────────

`lowcmd` es el canal del resto del paquete, y ahí hay que soltar antes el
controlador de alto nivel. Con el robot de pie eso **lo tira**: nadie
equilibra.

`arm_sdk` existe justo para esto: cede los brazos mientras el controlador de
locomoción sigue mandando en las piernas. El peso, que viaja en el `q` del
motor 27, es el factor de mezcla entre nuestra consigna y la suya.

**Comprobado sobre el robot el 2026-09-17** (F7.3 del plan, que llevaba
abierto): de pie y en Motion Mode, con peso 0.5, sostener la postura dio 0.28°
de deriva, y al pedir +3° en un codo se movió +1.80° — el 60 % de lo pedido,
que es lo que corresponde a ese peso. Cuando se probó en reposo no hacía nada,
y ahora se entiende por qué: sin controlador de locomoción corriendo no hay con
quién mezclar.

───────────────────────────────────────────────────────────────────────────
POR QUÉ EL CONJUNTO DE GANANCIAS ES `tuned` Y NO `tuned_gff`
───────────────────────────────────────────────────────────────────────────

En `arm_sdk` el servicio del robot aplica **su** compensación de gravedad.
Sumar la nuestra sería contarla dos veces, y el brazo saldría disparado hacia
arriba. `tuned` está sintonizado sin feedforward, que es lo que corresponde
aquí.

───────────────────────────────────────────────────────────────────────────
POR QUÉ LA RECOLOCACIÓN VA TODA A LA VEZ
───────────────────────────────────────────────────────────────────────────

Porque el hombro hace de **contrapeso** del codo. Al flexionarse, el codo lleva
el antebrazo y la mano hacia adelante y el centro de masas se va con ellos; el
hombro retrocediendo lo compensa. Si se mueve el codo primero y el hombro
después, la compensación llega tarde y el robot da un paso para no caerse —
observado el 2026-09-17: echaba a andar y solo paraba con `start`.

Antes esto iba escalonado por miedo a la autocolisión, porque el tope del
hombro depende del codo. Medido: con el codo en los ~39° de Motion Mode el tope
exige |roll| >= 4.88°, y el camino recto hasta la postura del gesto **nunca se
acerca a él** —el roll empieza en 13.5° y baja más despacio de lo que el codo
relaja el requisito, así que el margen mínimo es de 5° a 7.5°—. Lo que iba
justo no era el camino sino el recorte conservador de `ramp_to`, que evalúa el
tope con el codo más estirado de los dos extremos.

Así que se comprueba el camino de verdad, muestreándolo, y solo se escalona si
ese camino choca. Con un `shoulder_roll` holgado nunca hace falta.
"""
from __future__ import annotations

import math
import struct
import sys
import time

import rclpy

from .._node_base import ejecuta
from ..arm_client import H12Client
from .. import gains as cfg
from ..joints import ARM_INDICES, BY_INDEX, BY_NAME
from .six_seven import balancin, pico_velocidad, ANCHO_BANDA_HZ, FRACCION_BW

from rclpy.node import Node


# Mapa de bits del mando, de `xRockerBtnDataStruct`. Verificado sobre el robot
# el 2026-09-17 leyendo `wireless_remote` mientras se pulsaban teclas: `select`
# salió 0x0008 (bit 3) y `A` 0x0100 (bit 8), que es lo que dice esta tabla.
BOTONES = {"R1": 0, "L1": 1, "start": 2, "select": 3,
           "R2": 4, "L2": 5, "F1": 6, "F3": 7,
           "A": 8, "B": 9, "X": 10, "Y": 11,
           "up": 12, "right": 13, "down": 14, "left": 15}


def mascara(combo: str) -> int:
    """'R2+up' -> mapa de bits. Distingue mayúsculas solo donde hace falta."""
    m = 0
    for parte in combo.replace(" ", "").split("+"):
        if not parte:
            continue
        clave = next((k for k in BOTONES if k.lower() == parte.lower()), None)
        if clave is None:
            raise SystemExit(f"  botón desconocido: '{parte}'. "
                             f"Hay: {', '.join(BOTONES)}")
        m |= 1 << BOTONES[clave]
    if not m:
        raise SystemExit("  la combinación está vacía")
    return m


def lee_teclas(state) -> int | None:
    """Mapa de bits del mando dentro de `LowState.wireless_remote`.

    Los 40 bytes son `xRockerBtnDataStruct`: dos de cabecera, dos de teclas, y
    cinco `float` con los ejes. Se lee de aquí y no del tópico
    `/wirelesscontroller` porque ese no publica nada en este robot —tiene
    publicador pero no emite, comprobado— mientras que `/lowstate` llega a
    250 Hz y ya estamos suscritos.
    """
    if state is None:
        return None
    b = bytes(bytearray(state.wireless_remote))
    if len(b) < 4:
        return None
    return struct.unpack_from("<H", b, 2)[0]


def camino_libre(gains, desde: dict, hasta: dict, n: int = 40):
    """¿Es segura la recta de `desde` a `hasta` en el espacio articular?

    `ramp_to` aplica el mismo factor a todas las articulaciones, así que el
    camino ES una recta y basta muestrearla. Devuelve (ok, peor_margen, dónde).

    Esto sustituye al escalonado. Mover el codo primero y el hombro después es
    lo PEOR que se puede hacer cuando el hombro va de contrapeso: el codo tira
    el centro de masas adelante y la compensación llega tarde. Si el camino
    simultáneo es seguro —y con un `shoulder_roll` holgado lo es— hay que
    moverlo todo a la vez.
    """
    peor, donde = float("inf"), 0.0
    for k in range(n + 1):
        f = k / n
        q = {i: desde[i] + f * (hasta[i] - desde[i]) for i in hasta}
        for i_roll, i_elb in gains.cond_pairs().items():
            if i_roll not in q or i_elb not in q:
                continue
            minimo = gains.roll_min_abs(i_roll, q[i_elb])
            if minimo is None:
                continue
            margen = abs(q[i_roll]) - minimo
            if margen < peor:
                peor, donde = margen, f
    return peor > 0.0, peor, donde


class SixSevenRemote(Node):
    def __init__(self, nombre="h1_2_six_seven_remote"):
        super().__init__(nombre)
        p = {
            "combo": "R2+up",
            # postura del gesto, en GRADOS
            "shoulder_roll_deg": 7.5,
            "shoulder_pitch_deg": 25.0,
            "elbow_deg": 0.0,
            "amplitude_deg": 7.5,
            "palm_up_left_deg": -90.0,
            "palm_up_right_deg": 90.0,
            # oscilación
            "moving_joint": "elbow",
            "speed": 0.75,
            "duration": 10.0,
            "fade": 0.0,
            # control
            "gains": "tuned",
            "weight": 1.0,
            "approach_speed": 0.25,
            # Rampa de peso al tomar el control. Es lo que evita el tirón al
            # entrar, así que bajarla mucho no compensa; 1 s sigue siendo una
            # rampa. Va ANTES de la recolocación, no entre ésta y el gesto.
            "engage_ramp": 1.0,
            # Quietud exigida antes de arrancar el gesto. Con `wait_all_settled`
            # el coste es este valor una sola vez, no una vez por articulación.
            "settle": 0.15,
            # Tolerancia de velocidad para dar una articulación por quieta.
            #
            # 0.05 y no el 0.02 que usa el resto del paquete, y hay una razón
            # medida: con el robot DE PIE los brazos nunca están del todo
            # quietos, porque el controlador de equilibrio los microajusta.
            # Medido el 2026-09-18 sobre el robot parado, 6000 muestras:
            # los picos llegan a 0.062 rad/s, y las catorce cumplen |dq| < 0.02
            # a la vez solo el 37 % del tiempo —conseguir 0.15 s SEGUIDOS es
            # casi imposible, así que se agotaba el plazo de 3 s siempre y eso
            # eran 3 de los 5 segundos muertos antes del gesto—. Con 0.05
            # cumplen el 99.8 %.
            #
            # Colgado del arnés y sin controlador, 0.02 sigue siendo lo
            # correcto; por eso el valor va aquí y no en el cliente.
            "settle_dq_tol": 0.05,
            "settle_timeout": 1.5,
            "rate_hz": 250.0,
            "once": False,          # salir tras el primer gesto
            # Dispara el gesto nada más arrancar, sin esperar al mando. Es
            # para validar el MOVIMIENTO por separado de la detección del
            # mando: dos cosas que pueden fallar de forma independiente y que
            # conviene no depurar a la vez.
            "trigger_now": False,
            # Oscilar ALREDEDOR DE DONDE YA ESTÁN los brazos, sin recolocarlos.
            #
            # Recolocar es lo que desestabiliza: en Motion Mode el codo está a
            # ~39° y la postura del gesto lo lleva a 0°, o sea 39 grados de
            # recorrido en los dos brazos a la vez, más 90° de giro de muñeca.
            # Eso mueve el centro de masas de un robot que se está
            # equilibrando. Las acciones de fábrica no hacen eso: parten de la
            # postura de pie y se mueven poco.
            #
            # Con `keep_posture`, el gesto es solo la oscilación del codo
            # alrededor de su valor de Motion Mode. La perturbación pasa de 39°
            # a la amplitud pedida.
            # `False` porque con el hombro haciendo de contrapeso —25°, que
            # retrocede mientras el codo se flexiona— la recolocación ya no
            # desestabiliza, y el gesto se ve mucho mejor desde la postura
            # propia que desde la de pie. Ponerlo a `True` vuelve a oscilar
            # alrededor de la postura de Motion Mode, sin recolocar: es la
            # opción conservadora si el robot vuelve a moverse.
            "keep_posture": False,
            # Girar las palmas cuando se CONSERVA la postura. Sin conservarla,
            # las palmas ya van en `_postura()`. Las muñecas pesan poco, así
            # que perturban mucho menos que el codo.
            "palms_up": True,
            "dry_run": False,
        }
        for k, v in p.items():
            self.declare_parameter(k, v)

    def p(self, n):
        return self.get_parameter(n).value

    # ── postura ────────────────────────────────────────────────────────────
    def _postura(self):
        r = abs(math.radians(float(self.p("shoulder_roll_deg"))))
        pi = math.radians(float(self.p("shoulder_pitch_deg")))
        e = math.radians(float(self.p("elbow_deg")))
        return {
            BY_NAME["L_shoulder_pitch"].idx: pi,
            BY_NAME["R_shoulder_pitch"].idx: pi,
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

    # ── el gesto, una vez ──────────────────────────────────────────────────
    def _postura_objetivo(self, cli) -> dict:
        """Dónde poner los brazos antes de oscilar.

        Con `keep_posture` es donde ya están —la postura de pie del
        controlador de locomoción— y entonces la única perturbación es la
        propia oscilación.
        """
        if not self.p("keep_posture"):
            return self._postura()
        postura = {i: cli.q(i) for i in ARM_INDICES}
        if self.p("palms_up"):
            postura[BY_NAME["L_wrist_roll"].idx] = math.radians(
                float(self.p("palm_up_left_deg")))
            postura[BY_NAME["R_wrist_roll"].idx] = math.radians(
                float(self.p("palm_up_right_deg")))
        return postura

    def _gesto(self, cli) -> None:
        q_motion = {i: cli.q(i) for i in ARM_INDICES}
        postura = self._postura_objetivo(cli)
        amp = abs(math.radians(float(self.p("amplitude_deg"))))
        vel = abs(float(self.p("speed")))
        dur = float(self.p("duration"))
        v = float(self.p("approach_speed"))
        m = self.p("moving_joint")
        i_l, i_r = BY_NAME[f"L_{m}"].idx, BY_NAME[f"R_{m}"].idx
        freq = vel / (2.0 * math.pi * amp)
        fade = float(self.p("fade")) or min(0.5 / max(freq, 1e-3), dur / 3.0)

        print(f"\n  ── gesto ──────────────────────────────────────────")
        t = {}
        t0 = time.monotonic()
        cli.engage(ramp=float(self.p("engage_ramp")))
        t["tomar el control"] = time.monotonic() - t0

        mueve = {i: q for i, q in postura.items()
                 if abs(q - q_motion[i]) > math.radians(0.2)}
        if mueve:
            recorrido = max(abs(postura[i] - q_motion[i]) for i in mueve)
            libre, margen, donde = camino_libre(cli.gains, q_motion, postura)
            print(f"  recolocando {len(mueve)} articulaciones, "
                  f"recorrido mayor {math.degrees(recorrido):.1f}°")
            if libre:
                # TODO A LA VEZ. Es lo que hace que el contrapeso funcione: si
                # el codo se flexiona antes de que el hombro retroceda, el
                # centro de masas se va adelante y el robot da un paso. Moviendo
                # las dos cosas juntas, la compensación es instantánea.
                print(f"    · todo a la vez (margen de autocolisión "
                      f"{math.degrees(margen):.1f}°)")
                t1 = time.monotonic()
                cli.ramp_to(postura, speed=v)
                t["recolocar"] = time.monotonic() - t1
            else:
                # El camino recto choca, así que hay que escalonar: flexionar el
                # codo primero relaja el tope del hombro. Se pierde el efecto de
                # contrapeso, pero es eso o rozar la pierna.
                print(f"    ⚠ el camino recto viola la envolvente en "
                      f"{math.degrees(margen):.1f}° (a mitad {donde:.0%}); "
                      f"se escalona, y el contrapeso será peor")
                codos = [i for i in mueve if BY_INDEX[i].name.endswith("elbow")]
                otros = [i for i in mueve if i not in codos]
                t1 = time.monotonic()
                cli.ramp_to({i: postura[i] for i in codos}, speed=v)
                cli.ramp_to({i: postura[i] for i in otros}, speed=v)
                t["recolocar"] = time.monotonic() - t1
            t1 = time.monotonic()
            cli.wait_all_settled(mueve,
                                 dq_tol=float(self.p("settle_dq_tol")),
                                 timeout=float(self.p("settle_timeout")),
                                 estable=float(self.p("settle")))
            t["esperar quietud"] = time.monotonic() - t1
        else:
            print(f"  sin recolocar: se oscila alrededor de la postura de "
                  f"Motion Mode")

        print(f"  oscilando {m} ±{float(self.p('amplitude_deg')):.1f}° "
              f"alrededor de {math.degrees(postura[i_l]):.1f}°, "
              f"a {freq:.2f} Hz, {dur:.0f} s…")
        cli.set_trajectory(i_l, balancin(amp, freq, dur, fade, -1.0),
                           q_base=postura[i_l])
        cli.set_trajectory(i_r, balancin(amp, freq, dur, fade, +1.0),
                           q_base=postura[i_r])
        t["hasta el gesto"] = time.monotonic() - t0
        cli.sleep(dur)
        cli.clear_trajectory()
        cli.wait_all_settled((i_l, i_r),
                             dq_tol=float(self.p("settle_dq_tol")),
                             timeout=float(self.p("settle_timeout")))

        # `release()` devuelve a `q0`, que es la postura de Motion Mode que
        # había al enganchar, y baja el peso en rampa. Es exactamente lo que
        # hace falta: el robot recupera sus brazos donde los dejó.
        print("  devolviendo a la postura de Motion Mode…")
        cli.release(home_speed=v, weight_ramp=2.0)

        print("  tiempos: " + "  ".join(f"{k} {v:.2f}s" for k, v in t.items()))
        peor = max(ARM_INDICES, key=lambda i: abs(cli.q(i) - q_motion[i]))
        print(f"  vuelta: desviación máxima {math.degrees(abs(cli.q(peor) - q_motion[peor])):.2f}° "
              f"({BY_INDEX[peor].name})")
        if cli.collision_clamps:
            print(f"  ⚠ autocolisión: consigna recortada en "
                  f"{cli.collision_clamps} ciclos")
        print(f"  {cli.loop_health()}")

    # ── comprobaciones previas ─────────────────────────────────────────────
    def _comprueba(self, cli) -> bool:
        ok = True
        postura = self._postura_objetivo(cli)
        amp = abs(math.radians(float(self.p("amplitude_deg"))))
        vel = abs(float(self.p("speed")))
        dur = float(self.p("duration"))
        m = self.p("moving_joint")
        i_l = BY_NAME[f"L_{m}"].idx
        freq = vel / (2.0 * math.pi * amp)
        fade = float(self.p("fade")) or min(0.5 / max(freq, 1e-3), dur / 3.0)
        pares = cli.gains.cond_pairs()

        for i, q in postura.items():
            i_e = pares.get(i)
            lo, hi = (cli.gains.limits_dynamic(i, postura[i_e]) if i_e in postura
                      else cli.gains.limits(i))
            if not (lo <= q <= hi):
                print(f"  ✗ {BY_INDEX[i].name} pide {math.degrees(q):.1f}°, fuera "
                      f"de [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")
                ok = False
        for i_roll, i_elb in pares.items():
            if i_roll not in postura or i_elb not in postura:
                continue
            for qe in (postura[i_elb] - amp, postura[i_elb], postura[i_elb] + amp):
                if not cli.gains.pair_ok(i_roll, postura[i_roll], qe):
                    mn = cli.gains.roll_min_abs(i_roll, qe)
                    print(f"  ✗ autocolisión: con {BY_INDEX[i_elb].name} a "
                          f"{math.degrees(qe):.1f}° hace falta "
                          f"|{BY_INDEX[i_roll].name}| ≥ {math.degrees(mn):.1f}°")
                    ok = False

        pico = pico_velocidad(amp, freq, dur, fade)
        print(f"  velocidad de pico {pico:.3f} rad/s de {cli.safety.max_ref_velocity:.2f}"
              f"   ·   frecuencia {freq:.3f} Hz", end="")
        bw = ANCHO_BANDA_HZ.get(m)
        if bw:
            print(f" de {bw:.2f} de ancho de banda", end="")
            if freq > bw:
                print("\n  ✗ por encima del ancho de banda: no llegará a la amplitud")
                ok = False
            elif freq > bw * FRACCION_BW:
                print(f"\n  ⚠ por encima de BW/3, el seguimiento pierde amplitud", end="")
        print()
        if pico > cli.safety.max_ref_velocity:
            print(f"  ✗ velocidad de pico por encima de `max_ref_velocity`")
            ok = False
        return ok

    # ── bucle ──────────────────────────────────────────────────────────────
    def run(self) -> int:
        combo = self.p("combo")
        mask = mascara(combo)
        g = cfg.load(self.p("gains"))
        cli = H12Client(
            controlled=list(ARM_INDICES), gains=g, channel="arm_sdk",
            rate_hz=float(self.p("rate_hz")), verbose=True,
            dry_run=bool(self.p("dry_run")),
            max_weight=float(self.p("weight")),
            node_name="h1_2_six_seven_remote_cli")
        try:
            cli.wait_for_state()
            print(f"\n  six-seven por mando · esperando «{combo}» "
                  f"(0x{mask:04X})")
            print(f"  ganancias '{g.set_name}', canal arm_sdk, peso "
                  f"{float(self.p('weight')):.2f}")
            if not self._comprueba(cli):
                print("\n  no se arranca.")
                return 1
            if bool(self.p("dry_run")):
                print("\n  ╔═══════════════════════════════════════════════════╗")
                print("  ║  PRUEBA EN SECO: el robot NO se va a mover.       ║")
                print("  ║  Se publica en un tópico que nadie escucha, para  ║")
                print("  ║  validar la detección del mando y la secuencia.   ║")
                print("  ║                                                   ║")
                print("  ║  Para que se mueva, quita  -p dry_run:=true       ║")
                print("  ╚═══════════════════════════════════════════════════╝")
            print(f"\n  Mientras espera NO publica nada: el robot tiene sus\n"
                  f"  brazos enteros. Ctrl-C para salir.\n")

            if self.p("trigger_now"):
                print(f"  ▶ trigger_now: sin esperar al mando")
                self._gesto(cli)
                if self.p("once"):
                    return 0
                print(f"\n  esperando «{combo}»…\n")

            anterior = 0
            visto = set()
            while rclpy.ok():
                time.sleep(0.02)
                keys = lee_teclas(cli.state())
                if keys is None:
                    continue
                if keys and keys not in visto:
                    visto.add(keys)
                    pulsados = [n for n, b in BOTONES.items() if keys & (1 << b)]
                    print(f"  mando: 0x{keys:04X}  {'+'.join(pulsados)}")
                # flanco de subida de la combinación completa
                if (keys & mask) == mask and (anterior & mask) != mask:
                    print(f"\n  ▶ «{combo}»")
                    self._gesto(cli)
                    if self.p("once"):
                        return 0
                    print(f"\n  esperando «{combo}» otra vez…\n")
                anterior = keys
        finally:
            cli.__exit__(None, None, None)
        return 0


def main(args=None):
    sys.exit(ejecuta(SixSevenRemote, "h1_2_six_seven_remote", args))


if __name__ == "__main__":
    main()
