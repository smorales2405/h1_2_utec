#!/usr/bin/env python3
"""Envía una trayectoria de referencia a una articulación y la sigue.

La consigna se evalúa en el hilo de control contra reloj monótono, no contando
ciclos: si un ciclo llega tarde la trayectoria no se deforma, que es lo que
hace comparables dos ejecuciones.

Trayectorias disponibles:

    smooth_step   escalón con coseno alzado. Lo que más se parece a una orden
                  de un algoritmo: va de A a B sin tirones.
    step          escalón puro. Duro, para ver el peor caso.
    sine          seno a una frecuencia. Seguimiento y desfase.
    chirp         barrido de frecuencia f0 -> f1, para ver dónde deja de seguir.
    hold          no mover. Mide el temblor en reposo.

    ros2 run h1_2_arm_control move_joint --ros-args \
        -p joint:=L_elbow -p traj:=smooth_step -p amp_deg:=10.0

    ros2 run h1_2_arm_control move_joint --ros-args \
        -p joint:=L_elbow -p traj:=sine -p amp_deg:=7.0 -p freq:=0.5 -p cycles:=3.0

La amplitud es RELATIVA a donde esté la articulación, y se recorta sola contra
los topes articulares y los de autocolisión.
"""
import math
import sys

from . import trajectories as traj
from ._node_base import ArmNode, ejecuta
from .gains import joint_or_die
from .joints import BY_INDEX


class MoveJoint(ArmNode):
    def __init__(self, nombre="h1_2_move_joint"):
        super().__init__(nombre, {
            "joint": "L_elbow",
            "traj": "smooth_step",
            "amp_deg": 7.0,
            "freq": 0.5,
            "cycles": 3.0,
            "rise": 0.3,
            "f0": 0.2,
            "f1": 3.0,
            "duration": 15.0,
            "settle": 2.0,
        })

    def _construye(self, amp):
        t = self.p("traj")
        if t == "step":
            return traj.step(amp), float(self.p("settle")) + 0.5
        if t == "smooth_step":
            return (traj.smooth_step(amp, rise=float(self.p("rise"))),
                    float(self.p("settle")) + 0.5)
        if t == "sine":
            f = float(self.p("freq"))
            return traj.sine(amp, f), float(self.p("cycles")) / max(f, 1e-3)
        if t == "chirp":
            d = float(self.p("duration"))
            return traj.chirp(amp, float(self.p("f0")), float(self.p("f1")), d), d
        if t == "hold":
            return traj.hold(), float(self.p("duration"))
        raise SystemExit(f"  trayectoria desconocida: '{t}'")

    def run(self) -> int:
        idx = joint_or_die(self.p("joint"))
        j = BY_INDEX[idx]
        with self.cliente([idx]) as cli:
            cli.wait_for_state()
            q0 = cli.q(idx)
            amp = math.radians(float(self.p("amp_deg")))

            # recortar la amplitud contra los topes, en vez de abortar a medias
            lo, hi = cli.gains.limits(idx)
            if q0 + amp > hi:
                amp = hi - q0
            if q0 + amp < lo:
                amp = lo - q0
            if abs(amp) < 1e-4 and self.p("traj") != "hold":
                print(f"  {j.name} está pegada a su tope: no hay recorrido.")
                return 1

            f, dur = self._construye(amp)
            print(f"\n  {j.name}: {math.degrees(q0):+.1f}° "
                  f"-> {math.degrees(q0 + amp):+.1f}°   "
                  f"{self.p('traj')}, {dur:.1f} s")

            cli.engage()
            cli.set_trajectory(idx, f)
            cli.sleep(dur)
            cli.clear_trajectory(idx)
            cli.wait_settled(idx)

            print(f"\n  acabó en {math.degrees(cli.q(idx)):+.2f}°   "
                  f"par {cli.tau(idx):+.2f} Nm")
            if cli.collision_clamps:
                print(f"  ⚠ autocolisión: consigna recortada en "
                      f"{cli.collision_clamps} ciclos")
            print(f"  {cli.loop_health()}")
            cli.release()
        return 0


def main(args=None):
    sys.exit(ejecuta(MoveJoint, "h1_2_move_joint", args))


if __name__ == "__main__":
    main()
