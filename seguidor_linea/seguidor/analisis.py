"""Analisis de los registros: respuesta a un escalon de vyaw (pregunta 6.3.1 del PDF) y, sobre los
datasets, la velocidad y la direccion reales de avance a partir de como se mueve la linea.

El giro se mide sobre el yaw de la IMU y no sobre gz: la marcha mete en gz una oscilacion de
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


def andando(t, gz, ventana_s=1.0, umbral=0.08):
    """True donde el robot da pasos: la desviacion de gz en una ventana centrada pasa de `umbral`
    rad/s. De pie gz es casi plano (~0.01); andando, la marcha mete ~0.27 rad/s."""
    gz = np.asarray(gz, dtype=float)
    media = media_centrada(t, gz, ventana_s)
    return np.sqrt(np.maximum(media_centrada(t, gz ** 2, ventana_s) - media ** 2, 0.0)) > umbral


def rellenar(x):
    """NaN -> interpolacion lineal entre los valores validos (para poder filtrar)."""
    x = np.asarray(x, dtype=float).copy()
    ok = np.isfinite(x)
    if ok.sum() >= 2:
        x[~ok] = np.interp(np.flatnonzero(~ok), np.flatnonzero(ok), x[ok])
    return x


def direccion_de_avance(t, a, theta, mascara, omega=None, ventana_s=1.0, paso_s=0.25, puntos=False):
    """Velocidad y direccion reales de avance, sin odometria, con una linea recta fija en el suelo.

    Vista desde la camara, la linea es y = a + tan(theta) x. Si la camara avanza a v en la direccion
    phi (rad, en su propio marco) y el robot gira a omega alrededor de un punto que esta d por detras
    de la camara, para angulos pequenos

        da/dt = v (theta - phi) - d omega

    y la regresion de da/dt frente a theta (y omega, si se da: el yaw de la IMU derivado) da v, phi
    y d. a, theta y omega se suavizan con una media centrada de `ventana_s` (quita el balanceo de
    cada paso) y da/dt sale de una diferencia centrada de +-`paso_s`. Solo cuentan los fotogramas de
    `mascara` (andando, linea bien vista) con toda la ventana dentro. El yaw de la camara respecto
    del avance es -phi.
    Devuelve dict(v, phi, d, n, residuo) (d = NaN sin omega) o None si hay menos de 30 puntos; con
    puntos=True, ademas theta, omega y dadt de la regresion (para dibujarla)."""
    t = np.asarray(t, dtype=float)
    mascara = np.asarray(mascara, dtype=bool) & np.isfinite(a) & np.isfinite(theta)
    a_s = media_centrada(t, rellenar(a), ventana_s)
    th_s = media_centrada(t, rellenar(theta), ventana_s)
    om_s = media_centrada(t, rellenar(omega), ventana_s) if omega is not None else np.zeros(len(t))
    k = max(1, int(round(paso_s / np.median(np.diff(t)))))
    n_ventana = max(k, int(round(ventana_s / 2 / np.median(np.diff(t)))))
    entera = np.convolve(mascara.astype(float), np.ones(2 * n_ventana + 1), mode="same") >= 2 * n_ventana + 1
    idx = np.flatnonzero(entera)
    idx = idx[(idx >= k) & (idx < len(t) - k)]
    if len(idx) < 30:
        return None
    dadt = (a_s[idx + k] - a_s[idx - k]) / (t[idx + k] - t[idx - k])
    x, w = th_s[idx], om_s[idx]
    cols = [x, w, np.ones(len(x))] if omega is not None else [x, np.ones(len(x))]
    buenos = np.ones(len(x), dtype=bool)
    for _ in range(2):   # la segunda vez, sin los saltos de la deteccion (> 3 desviaciones)
        coef, *_ = np.linalg.lstsq(np.stack(cols, axis=1)[buenos], dadt[buenos], rcond=None)
        res = dadt - np.stack(cols, axis=1) @ coef
        buenos = np.abs(res) < 3 * np.std(res[buenos])
    v, corte = coef[0], coef[-1]
    out = {"v": float(v), "phi": float(-corte / v) if abs(v) > 1e-6 else math.nan,
           "d": float(-coef[1]) if omega is not None else math.nan,
           "n": int(buenos.sum()), "residuo": float(np.std(res[buenos]))}
    if puntos:
        out.update(theta=x[buenos], omega=w[buenos], dadt=dadt[buenos])
    return out


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
