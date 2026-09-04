"""Consignas de prueba, todas centradas en la postura de partida.

Cada función devuelve `f(t) -> (q, dq)` en radianes y rad/s, **relativa** al
punto de partida: el que llama suma `q0`. Devolver también `dq` importa: sin
velocidad de referencia el término `kd*(0 - dq)` frena el movimiento y mete
retardo, que es una de las causas típicas del temblor en seguimiento.
"""
from __future__ import annotations

import math
from typing import Callable, Tuple

Traj = Callable[[float], Tuple[float, float]]


def step(amplitude: float, t_step: float = 0.5) -> Traj:
    """Escalón puro. Para medir sobreimpulso, tiempo de subida y error final."""
    def f(t: float):
        return (amplitude if t >= t_step else 0.0), 0.0
    return f


def smooth_step(amplitude: float, rise: float = 0.3, t_step: float = 0.5) -> Traj:
    """Escalón suavizado con coseno alzado. Lo que de verdad se parece a la
    teleoperación: la mano del operador tampoco da saltos infinitos."""
    def f(t: float):
        a = (t - t_step) / rise
        if a <= 0.0:
            return 0.0, 0.0
        if a >= 1.0:
            return amplitude, 0.0
        s = 0.5 - 0.5 * math.cos(math.pi * a)
        ds = 0.5 * math.pi * math.sin(math.pi * a) / rise
        return amplitude * s, amplitude * ds
    return f


def sine(amplitude: float, freq: float) -> Traj:
    """Seno. Mide ganancia y desfase del lazo a una frecuencia concreta."""
    w = 2.0 * math.pi * freq
    def f(t: float):
        return amplitude * math.sin(w * t), amplitude * w * math.cos(w * t)
    return f


def chirp(amplitude: float, f0: float, f1: float, duration: float) -> Traj:
    """Barrido lineal de frecuencia f0 -> f1. Enseña de un vistazo a partir de
    qué frecuencia la articulación deja de seguir y empieza a resonar."""
    k = (f1 - f0) / duration
    def f(t: float):
        tc = min(max(t, 0.0), duration)
        phase = 2.0 * math.pi * (f0 * tc + 0.5 * k * tc * tc)
        w = 2.0 * math.pi * (f0 + k * tc)
        return amplitude * math.sin(phase), amplitude * w * math.cos(phase)
    return f


def hold() -> Traj:
    """No mover. Mide el ruido y el temblor en reposo."""
    def f(t: float):
        return 0.0, 0.0
    return f


def zero_velocity(traj: Traj) -> Traj:
    """Envuelve una trayectoria anulando `dq`, como hace xr_teleoperate.

    `H1_2_ArmController._ctrl_motor_state` fija `msg.motor_cmd[id].dq = 0`
    siempre. Sirve para medir cuánto cuesta esa decisión.
    """
    def f(t: float):
        q, _ = traj(t)
        return q, 0.0
    return f


BUILDERS = {
    "step": step,
    "smooth_step": smooth_step,
    "sine": sine,
    "chirp": chirp,
    "hold": hold,
}
