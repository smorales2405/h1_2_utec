"""Métricas de calidad de seguimiento a partir de un registro de `Sample`.

Todo se calcula con numpy; no hace falta scipy. El muestreo es el propio ciclo
de control (250 Hz por defecto), así que la Nyquist ronda los 125 Hz: de sobra
para ver el temblor de un motor, que vive entre 10 y 60 Hz.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class TrackingMetrics:
    joint: str
    kp: float
    kd: float
    n: int
    fs: float
    # --- seguimiento ---
    rms_error: float          # rad
    max_error: float          # rad
    mean_error: float         # rad, el sesgo: gravedad no compensada
    # --- temblor ---
    chatter_dq: float         # rad/s, energía de dq por encima de f_hp
    chatter_tau: float        # Nm,    ídem para el par estimado
    peak_freq: float          # Hz, frecuencia dominante del temblor
    # --- esfuerzo ---
    rms_tau: float            # Nm
    max_tau: float            # Nm
    tau_headroom: float       # fracción del par máximo del URDF que queda libre

    def as_row(self) -> dict:
        return asdict(self)

    def summary(self) -> str:
        return (f"{self.joint:<17} kp={self.kp:6.1f} kd={self.kd:5.2f} │ "
                f"err rms {self.rms_error*1000:6.2f} mrad  máx {self.max_error*1000:6.2f} "
                f"sesgo {self.mean_error*1000:+7.2f} │ "
                f"temblor dq {self.chatter_dq:5.3f} rad/s @ {self.peak_freq:5.1f} Hz │ "
                f"tau rms {self.rms_tau:5.2f} máx {self.max_tau:5.2f} Nm "
                f"(queda {self.tau_headroom*100:4.0f} %)")


@dataclass
class StepMetrics:
    joint: str
    kp: float
    kd: float
    amplitude: float          # rad
    rise_time: float          # s, del 10 % al 90 %
    overshoot: float          # fracción (0.12 = 12 %)
    settling_time: float      # s, hasta quedarse dentro del ±2 %
    steady_error: float       # rad, media del último 20 % del tramo
    peak_tau: float           # Nm
    oscillations: int         # cruces por el valor final tras el primer pico

    def as_row(self) -> dict:
        return asdict(self)

    def summary(self) -> str:
        return (f"{self.joint:<17} kp={self.kp:6.1f} kd={self.kd:5.2f} │ "
                f"subida {self.rise_time*1000:6.1f} ms  sobreimp {self.overshoot*100:6.1f} % "
                f"establec {self.settling_time*1000:7.1f} ms │ "
                f"err final {self.steady_error*1000:+7.2f} mrad │ "
                f"{self.oscillations} oscilaciones  pico tau {self.peak_tau:5.2f} Nm")


def _arrays(samples, idx):
    t = np.array([s.t for s in samples])
    return (t,
            np.array([s.q_des[idx] for s in samples]),
            np.array([s.q[idx] for s in samples]),
            np.array([s.dq[idx] for s in samples]),
            np.array([s.tau[idx] for s in samples]))


def _hf_rms(x: np.ndarray, fs: float, f_hp: float) -> tuple[float, float]:
    """RMS de la parte de `x` por encima de `f_hp`, y su frecuencia dominante.

    Se usa Parseval sobre la FFT real: la energía por encima del corte se lee
    directamente de los coeficientes, sin diseñar ningún filtro.
    """
    n = len(x)
    if n < 16:
        return 0.0, 0.0
    x = x - x.mean()
    win = np.hanning(n)
    # el factor corrige la pérdida de potencia que introduce la ventana
    scale = np.sqrt(np.mean(win ** 2))
    spec = np.fft.rfft(x * win) / (n * scale)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    band = freqs >= f_hp
    if not band.any():
        return 0.0, 0.0
    power = 2.0 * np.abs(spec[band]) ** 2
    rms = float(np.sqrt(power.sum()))
    peak = float(freqs[band][int(np.argmax(power))]) if power.size else 0.0
    return rms, peak


def tracking(samples, idx, kp, kd, tau_max, f_hp: float = 8.0,
             skip: float = 0.0) -> TrackingMetrics:
    """Métricas de un tramo de seguimiento (seno, chirp, o simple reposo).

    `skip` descarta los primeros segundos, para no contaminar el resultado con
    el transitorio de arranque.
    """
    from .joints import BY_INDEX
    t, q_des, q, dq, tau = _arrays(samples, idx)
    if skip > 0:
        keep = t >= (t[0] + skip)
        t, q_des, q, dq, tau = (a[keep] for a in (t, q_des, q, dq, tau))
    n = len(t)
    fs = (n - 1) / (t[-1] - t[0]) if n > 1 and t[-1] > t[0] else 0.0
    err = q - q_des
    c_dq, peak = _hf_rms(dq, fs, f_hp)
    c_tau, _ = _hf_rms(tau, fs, f_hp)
    max_tau = float(np.max(np.abs(tau))) if n else 0.0
    return TrackingMetrics(
        joint=BY_INDEX[idx].name, kp=float(kp), kd=float(kd), n=n, fs=float(fs),
        rms_error=float(np.sqrt(np.mean(err ** 2))),
        max_error=float(np.max(np.abs(err))),
        mean_error=float(np.mean(err)),
        chatter_dq=c_dq, chatter_tau=c_tau, peak_freq=peak,
        rms_tau=float(np.sqrt(np.mean(tau ** 2))),
        max_tau=max_tau,
        tau_headroom=float(max(0.0, 1.0 - max_tau / tau_max)),
    )


def step_response(samples, idx, kp, kd, t_step: float) -> StepMetrics:
    """Métricas clásicas de un escalón. `t_step` es cuándo se dio el salto."""
    from .joints import BY_INDEX
    t, q_des, q, dq, tau = _arrays(samples, idx)
    post = t >= t_step
    if post.sum() < 10:
        raise ValueError("no hay suficientes muestras después del escalón")
    q_start = float(np.mean(q[t < t_step])) if (t < t_step).any() else float(q[0])
    q_cmd = float(q_des[post][-1])
    amp = q_cmd - q_start
    tp, qp, taup = t[post] - t_step, q[post], tau[post]

    tail = qp[int(0.8 * len(qp)):]
    q_final = float(np.mean(tail))
    steady_error = q_cmd - q_final

    if abs(amp) < 1e-6:
        return StepMetrics(BY_INDEX[idx].name, kp, kd, amp, float("nan"), 0.0,
                           float("nan"), steady_error,
                           float(np.max(np.abs(taup))), 0)

    prog = (qp - q_start) / amp        # 0 al empezar, 1 en la consigna
    def _first(mask):
        w = np.flatnonzero(mask)
        return float(tp[w[0]]) if w.size else float("nan")
    rise = _first(prog >= 0.9) - _first(prog >= 0.1)

    overshoot = max(0.0, float(np.max(prog)) - 1.0)

    # asentamiento: último instante fuera de la banda del ±2 % de la amplitud
    outside = np.flatnonzero(np.abs(qp - q_final) > 0.02 * abs(amp))
    settling = float(tp[outside[-1]]) if outside.size else 0.0

    # Oscilaciones: alternancias a un lado y otro del valor final, después del
    # primer pico. Contar cambios de signo sin más da cientos de "oscilaciones"
    # con la articulación quieta, porque el ruido del encoder cruza el cero
    # continuamente. Se exige salir de una banda del ±2 % de la amplitud.
    i_peak = int(np.argmax(prog))
    band = 0.02 * abs(amp)
    side = np.where(qp[i_peak:] - q_final > band, 1,
                    np.where(qp[i_peak:] - q_final < -band, -1, 0))
    side = side[side != 0]
    crossings = int(np.sum(np.diff(side) != 0)) if side.size else 0

    return StepMetrics(
        joint=BY_INDEX[idx].name, kp=float(kp), kd=float(kd), amplitude=amp,
        rise_time=rise, overshoot=overshoot, settling_time=settling,
        steady_error=steady_error, peak_tau=float(np.max(np.abs(taup))),
        oscillations=crossings,
    )


def cost(track: TrackingMetrics, step: StepMetrics | None = None,
         w_err: float = 1.0, w_chatter: float = 1.0,
         w_overshoot: float = 0.5) -> float:
    """Número único para ordenar candidatos en el barrido de ganancias.

    Mezcla, en unidades comparables:
      * error rms de seguimiento en mrad,
      * temblor (dq de alta frecuencia) en centésimas de rad/s,
      * sobreimpulso del escalón en puntos porcentuales.
    Los pesos son deliberadamente visibles para poder discutirlos: si lo que
    molesta es la vibración, sube `w_chatter`.
    """
    j = w_err * (track.rms_error * 1000.0) + w_chatter * (track.chatter_dq * 100.0)
    if step is not None and np.isfinite(step.overshoot):
        j += w_overshoot * (step.overshoot * 100.0)
    return float(j)
