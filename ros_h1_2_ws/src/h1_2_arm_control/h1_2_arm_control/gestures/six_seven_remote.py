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
POR QUÉ LA TRANSICIÓN VA POR ETAPAS
───────────────────────────────────────────────────────────────────────────

El tope de autocolisión del hombro depende del codo. En Motion Mode el codo
está a ~39°, que exige |roll| >= 4.88°, y el gesto pide 5.0°: **0.12° de
margen**. En un solo tramo `ramp_to` recorta con el codo más estirado de los
dos, así que rozaría el tope todo el camino.

Flexionando el codo primero, el requisito cae a 0.94° y sobran 4°.
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


class SixSevenRemote(Node):
    def __init__(self, nombre="h1_2_six_seven_remote"):
        super().__init__(nombre)
        p = {
            "combo": "R2+up",
            # postura del gesto, en GRADOS
            "shoulder_roll_deg": 5.0,
            "shoulder_pitch_deg": 0.0,
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
            "rate_hz": 250.0,
            "once": False,          # salir tras el primer gesto
            # Dispara el gesto nada más arrancar, sin esperar al mando. Es
            # para validar el MOVIMIENTO por separado de la detección del
            # mando: dos cosas que pueden fallar de forma independiente y que
            # conviene no depurar a la vez.
            "trigger_now": False,
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
    def _gesto(self, cli) -> None:
        postura = self._postura()
        amp = abs(math.radians(float(self.p("amplitude_deg"))))
        vel = abs(float(self.p("speed")))
        dur = float(self.p("duration"))
        v = float(self.p("approach_speed"))
        m = self.p("moving_joint")
        i_l, i_r = BY_NAME[f"L_{m}"].idx, BY_NAME[f"R_{m}"].idx
        freq = vel / (2.0 * math.pi * amp)
        fade = float(self.p("fade")) or min(0.5 / max(freq, 1e-3), dur / 3.0)

        # Donde estaba en Motion Mode. `release()` vuelve solo aquí, pero se
        # guarda para enseñarlo y para comprobar el retorno.
        q_motion = {i: cli.q(i) for i in ARM_INDICES}

        print(f"\n  ── gesto ──────────────────────────────────────────")
        cli.engage(ramp=2.0)

        # Por etapas, y el orden importa: con el codo en la postura de Motion
        # Mode (~39°) el tope de autocolisión exige casi exactamente el roll que
        # el gesto pide. Flexionando primero, sobra margen.
        codos = [BY_NAME["L_elbow"].idx, BY_NAME["R_elbow"].idx]
        munecas = [i for i in postura if BY_INDEX[i].group == "wrist"]
        resto = [i for i in postura if i not in codos and i not in munecas]
        print("  1/4 · flexionando los codos…")
        cli.ramp_to({i: postura[i] for i in codos}, speed=v)
        print("  2/4 · hombros…")
        cli.ramp_to({i: postura[i] for i in resto}, speed=v)
        print("  3/4 · palmas arriba…")
        cli.ramp_to({i: postura[i] for i in munecas}, speed=v)
        for i in postura:
            cli.wait_settled(i)

        print(f"  4/4 · oscilando {m} ±{float(self.p('amplitude_deg')):.1f}° "
              f"a {freq:.2f} Hz, {dur:.0f} s…")
        cli.set_trajectory(i_l, balancin(amp, freq, dur, fade, -1.0),
                           q_base=postura[i_l])
        cli.set_trajectory(i_r, balancin(amp, freq, dur, fade, +1.0),
                           q_base=postura[i_r])
        cli.sleep(dur)
        cli.clear_trajectory()
        for i in (i_l, i_r):
            cli.wait_settled(i)

        # `release()` devuelve a `q0`, que es la postura de Motion Mode que
        # había al enganchar, y baja el peso en rampa. Es exactamente lo que
        # hace falta: el robot recupera sus brazos donde los dejó.
        print("  devolviendo a la postura de Motion Mode…")
        cli.release(home_speed=v, weight_ramp=2.0)

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
        postura = self._postura()
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
