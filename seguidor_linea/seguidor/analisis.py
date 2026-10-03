"""Analisis de los registros: respuesta a un escalon de vyaw (pregunta 6.3.1 del PDF).

Se mide sobre el yaw de la IMU y no sobre gz: la marcha mete en gz una oscilacion de
~0.27 rad/s de desviacion (datos_cuadrado), del orden de la propia orden.
"""

import math

import numpy as np


def cadencia(t, senal, fmin=0.5, fmax=3.0):
    """Frecuencia (Hz) del pico del espectro de `senal` entre fmin y fmax: la de la marcha si se
    le pasa el roll de la IMU (el torso se balancea una vez por zancada)."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(senal, dtype=float) - np.mean(senal)
    espectro = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    frec = np.fft.rfftfreq(len(x), np.median(np.diff(t)))
    banda = (frec >= fmin) & (frec <= fmax)
    return float(frec[banda][np.argmax(espectro[banda])])


def media_centrada(t, x, ventana_s):
    """Media movil centrada de `ventana_s` (p. ej. un periodo de la marcha): quita el balanceo
    sin retrasar la senal. Solo para analizar despues, no en vivo (necesita el futuro)."""
    n = max(1, int(round(ventana_s / np.median(np.diff(t)))))
    return np.convolve(np.asarray(x, dtype=float), np.ones(n) / n, mode="same")


def tiempo_hasta(t, x, t0, desde, hacia, fraccion):
    """s desde t0 hasta que x recorre `fraccion` del camino de `desde` a `hacia` (NaN si no llega)."""
    objetivo = desde + fraccion * (hacia - desde)
    signo = 1.0 if hacia >= desde else -1.0
    idx = np.flatnonzero((np.asarray(t) >= t0) & (signo * (np.asarray(x) - objetivo) >= 0))
    return float(t[idx[0]] - t0) if len(idx) else math.nan


def respuesta_escalon(t, yaw, t_base0, t_esc0, t_esc1, amplitud, umbral_deg=2.0):
    """t (s) y yaw (rad, se desenvuelve aqui) de la IMU; la base va de t_base0 a t_esc0 (vyaw = 0)
    y el escalon de t_esc0 a t_esc1 (vyaw = amplitud). Devuelve:
      deriva       rad/s de la base (la deriva del control de Unitree), que se descuenta
      vel_regimen  rad/s: pendiente del giro en la segunda mitad del escalon
      ganancia     vel_regimen / amplitud
      retardo      s: donde la recta de regimen corta el cero (tiempo muerto + media subida)
      t_umbral     s: primera vez que el giro pasa de `umbral_deg` en el sentido de la orden
      giro         rad girados al acabar el escalon, sin la deriva
    """
    t = np.asarray(t, dtype=float)
    yaw = np.unwrap(np.asarray(yaw, dtype=float))
    base = (t >= t_base0) & (t < t_esc0)
    esc = (t >= t_esc0) & (t <= t_esc1)
    if base.sum() < 10 or esc.sum() < 20:
        raise ValueError(f"pocas muestras: {base.sum()} en la base y {esc.sum()} en el escalon")
    deriva, yaw0 = np.polyfit(t[base] - t_esc0, yaw[base], 1)
    te = t[esc] - t_esc0
    giro = yaw[esc] - (yaw0 + deriva * te)
    regimen = te >= te[-1] / 2
    vel, corte = np.polyfit(te[regimen], giro[regimen], 1)
    signo = 1.0 if amplitud >= 0 else -1.0
    pasa = te[signo * giro > math.radians(umbral_deg)]
    return {"amplitud": float(amplitud), "deriva": float(deriva), "vel_regimen": float(vel),
            "ganancia": float(vel / amplitud) if amplitud else math.nan,
            "retardo": float(-corte / vel) if abs(vel) > 1e-6 else math.nan,
            "t_umbral": float(pasa[0]) if len(pasa) else math.nan,
            "giro": float(giro[-1])}
