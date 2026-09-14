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


def log_chirp(v_max: float, f0: float, f1: float, duration: float,
              fade: float = 2.0, amp_max: float | None = None) -> Traj:
    """Barrido LOGARÍTMICO de frecuencia con velocidad de pico constante.

    Es la señal que pide F5, y se diferencia de `chirp` en dos cosas.

    **La frecuencia sube geométricamente**, `f(t) = f0·(f1/f0)^(t/T)`, así que
    cada década recibe el mismo tiempo de medida. Con el barrido lineal, ir de
    0.2 a 1 Hz ocupa una quinta parte del ensayo y de 1 a 5 Hz las cuatro
    quintas: justo al revés de lo que interesa para un Bode, donde la década
    baja es tan importante como la alta.

    **La amplitud se escala como `A(f) = v_max / (2πf)`**, que mantiene la
    velocidad de pico constante en `v_max`. Sin eso hay que elegir entre
    saturar arriba o no excitar abajo: a amplitud fija, la velocidad pedida
    crece proporcionalmente a la frecuencia, y a 5 Hz con 0.12 rad serían
    3.8 rad/s, por encima de lo que la articulación puede dar. El resultado
    sería un «ancho de banda» que mide el límite de velocidad, no la dinámica.

    La amplitud, por tanto, es máxima al principio: `v_max/(2πf0)`. Con los
    valores de F5 —0.3 rad/s y 0.2 Hz— son 0.239 rad, o sea 13.7°. Conviene
    acotarla con `amp_max` según el recorrido que quede en la articulación.

    `fade` suaviza la entrada con un coseno alzado. Sin él, `dq` vale `v_max`
    en t=0: un escalón de velocidad de 0.3 rad/s, que además ensucia el
    espectro con su propio transitorio.

    Devuelve (q, dq) relativos, como el resto de trayectorias.
    """
    if f0 <= 0.0 or f1 <= f0:
        raise ValueError("hace falta 0 < f0 < f1")
    T = float(duration)
    L = math.log(f1 / f0)
    k = 2.0 * math.pi * f0 * T / L      # factor de la fase

    def f(t: float):
        tc = min(max(t, 0.0), T)
        r = math.exp(L * tc / T)        # (f1/f0)^(t/T)
        fi = f0 * r                     # frecuencia instantánea
        phase = k * (r - 1.0)
        amp = v_max / (2.0 * math.pi * fi)
        if amp_max is not None:
            amp = min(amp, amp_max)

        # ventana de entrada y su derivada
        if fade > 0.0 and tc < fade:
            w = 0.5 - 0.5 * math.cos(math.pi * tc / fade)
            dw = (math.pi / (2.0 * fade)) * math.sin(math.pi * tc / fade)
        else:
            w, dw = 1.0, 0.0

        sin_p, cos_p = math.sin(phase), math.cos(phase)
        q = w * amp * sin_p
        # dq = d/dt [w·A·sin(φ)] = (w'·A + w·A')·sin(φ) + w·A·φ'·cos(φ)
        # con A' = −A·L/T  y  A·φ' = v_max (por construcción, si no se recortó)
        dA = -amp * L / T
        dq = (dw * amp + w * dA) * sin_p + w * amp * (2.0 * math.pi * fi) * cos_p
        return q, dq

    return f


def chirp_freq(t: float, f0: float, f1: float, duration: float) -> float:
    """Frecuencia instantánea del `log_chirp` en el instante `t`, en Hz.

    Hace falta para etiquetar el eje de los análisis y para saber qué tramo
    del registro corresponde a cada banda.
    """
    tc = min(max(t, 0.0), duration)
    return f0 * math.exp(math.log(f1 / f0) * tc / duration)


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
