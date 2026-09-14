#!/usr/bin/env python3
"""F5 — Respuesta en frecuencia de una articulación. Ancho de banda a −3 dB.

El chirp ya se usaba de forma comparativa —«a partir de aquí deja de seguir»—
pero para la tesis hace falta un número. Esto lo da: ancho de banda, fase, pico
de resonancia y coherencia, por articulación y por conjunto de ganancias.

**La señal** es `trajectories.log_chirp`: barrido logarítmico 0.2 → 5 Hz con la
amplitud escalada como `A(f) = v_max/(2πf)`, o sea velocidad de pico constante.
Las dos cosas importan. El barrido logarítmico da el mismo tiempo de medida a
cada década, que es lo que un Bode necesita. Y la amplitud escalada evita tener
que elegir entre saturar arriba o no excitar abajo: a amplitud fija, pedir
0.12 rad a 5 Hz son 3.8 rad/s, y el «ancho de banda» que saldría sería el
límite de velocidad del motor, no la dinámica del lazo.

**El análisis** estima la función de transferencia por promediado espectral
(Welch, 50 % de solape) entre la consigna y la posición medida:

    H(f) = Sxy(f) / Sxx(f)          coherencia = |Sxy|² / (Sxx·Syy)

La coherencia es el control de calidad: dice qué fracción de la salida está
linealmente explicada por la entrada. Por debajo de 0.9 la estimación no vale
—fricción, zona muerta, o simplemente poca excitación— y hay que subir `--vmax`.

    python3 scripts/18_bode.py --joint L_elbow
    python3 scripts/18_bode.py --joint L_elbow --gains xr_teleoperate
    python3 scripts/18_bode.py --joint L_elbow --zero-dq      # el coste de dq=0
    python3 scripts/18_bode.py --analiza logs/bode_*.csv      # sin robot

Las nueve corridas de F5 son tres articulaciones (`shoulder_pitch`, `elbow`,
`wrist_pitch`) por tres configuraciones (`xr_teleoperate`, `tuned_gff`, y
`tuned_gff` con `dq_des = 0`).
"""
from __future__ import annotations

import argparse
import glob
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _common import (add_common_args, apply_test_posture, build_client, confirm,
                     joint_index)
from h1_2_joint_control import config as cfg
from h1_2_joint_control import recorder as rec
from h1_2_joint_control import trajectories as tr
from h1_2_joint_control.client import SafetyAbort
from h1_2_joint_control.joints import BY_INDEX


def carga_traza(ruta):
    """(t, q_des, q) de un CSV de `recorder.save_samples`.

    Se lee la cabecera con `csv` y se accede por ÍNDICE, no por el nombre que
    invente numpy. `genfromtxt(names=True)` borra el punto de `L_elbow.q` y lo
    deja en `L_elbowq`, así que cualquier selector con `_q` falla en silencio;
    y `L_elbowdq` también acaba en `q`, de modo que un selector laxo escogería
    la velocidad creyendo que es la posición.
    """
    import csv as _csv
    with open(ruta, newline="", encoding="utf-8") as fh:
        cab = next(_csv.reader(fh))
    i_des = next(i for i, c in enumerate(cab) if c.endswith(".q_des"))
    i_q = next(i for i, c in enumerate(cab) if c.endswith(".q"))
    i_t = cab.index("t")
    d = np.loadtxt(ruta, delimiter=",", skiprows=1)
    return d[:, i_t], d[:, i_des], d[:, i_q]


def analiza(t, q_des, q, f0, f1, nperseg=None):
    """|H|, fase y coherencia por Welch. Devuelve (f, mag_dB, fase_deg, coh)."""
    from scipy import signal
    fs = (len(t) - 1) / (t[-1] - t[0])
    if nperseg is None:
        # Compromiso: segmentos largos dan resolución en baja frecuencia, que
        # es donde está el 0.2 Hz; cortos dan más promedios y menos varianza.
        # 16 s cubre tres ciclos del tono más lento.
        nperseg = int(2 ** round(math.log2(16.0 * fs)))
    nperseg = min(nperseg, len(t))
    kw = dict(fs=fs, nperseg=nperseg, noverlap=nperseg // 2, window="hann")
    f, Sxx = signal.welch(q_des, **kw)
    _, Syy = signal.welch(q, **kw)
    _, Sxy = signal.csd(q_des, q, **kw)
    banda = (f >= f0) & (f <= f1)
    f = f[banda]
    H = Sxy[banda] / np.maximum(Sxx[banda], 1e-30)
    coh = (np.abs(Sxy[banda]) ** 2
           / np.maximum(Sxx[banda] * Syy[banda], 1e-30))
    return f, 20.0 * np.log10(np.maximum(np.abs(H), 1e-12)), \
           np.degrees(np.unwrap(np.angle(H))), coh


def resume(f, mag, fase, coh, f0, f1):
    """Las cuatro métricas que pide F5, más lo que hace falta para juzgarlas."""
    # Referencia de 0 dB: la media en la banda baja, donde el lazo sigue bien.
    baja = f <= min(0.5, f0 * 2.5)
    ref = float(np.mean(mag[baja])) if baja.any() else float(mag[0])

    def cruce(y, objetivo, creciente=False):
        """Primera frecuencia donde `y` cruza `objetivo`, interpolando.

        `nan` significa que no cruza dentro de la banda, y es un resultado, no
        un fallo: si la fase no llega a −90° hasta 5 Hz, la articulación es
        más rápida que el barrido.
        """
        d = (y - objetivo) if creciente else (objetivo - y)
        w = np.flatnonzero(d[:-1] * d[1:] < 0)
        if not w.size:
            return float("nan")
        i = w[0]
        x0, x1, y0, y1 = f[i], f[i + 1], y[i], y[i + 1]
        return float(x0 + (objetivo - y0) * (x1 - x0) / (y1 - y0))

    # Todo lo que dependa de la fase o del máximo se calcula SOLO en el tramo
    # fiable, que es el contiguo desde la banda baja con coherencia ≥ 0.9.
    # Arriba del barrido la amplitud es de 0.65° y el estimador se descompone:
    # `np.unwrap` acumula saltos de 360° y la fase sale POSITIVA —hasta +228°
    # medidos—, que es imposible en un sistema causal.
    malo = np.flatnonzero(coh < 0.9)
    n_fiable = int(malo[0]) if malo.size else len(f)
    sl = slice(0, max(n_fiable, 2))

    bw = cruce(mag, ref - 3.0)
    f90 = cruce(fase[sl], -90.0) if n_fiable > 2 else float("nan")

    # El pico se busca SOLO donde la coherencia dice que la estimación vale.
    # Arriba del barrido la amplitud es mínima —0.65° a 5 Hz con v_max 0.3— y
    # el ruido del encoder domina: |H| = Sxy/Sxx se dispara y `argmax` sobre
    # toda la banda devuelve un pico que no existe. Validado con segundos
    # órdenes de resonancia conocida: sin la máscara, un sistema con pico real
    # de 2.70 dB a 1.24 Hz se reportaba como 5.46 dB a 4.82 Hz.
    i_pico = int(np.argmax(mag[sl]))
    # Un máximo en el borde del tramo NO es una resonancia: significa que la
    # respuesta cae de forma monótona y el máximo es simplemente el primer
    # punto. Reportarlo como «pico a 0.24 Hz» sería engañoso.
    hay_pico = 0 < i_pico < (sl.stop - 1)
    banda_coh = (f >= 0.2) & (f <= 3.0)
    return {
        "ref_dB": ref,
        "bw_3dB": bw,
        "f_fase90": f90,
        "pico_dB": float(mag[i_pico] - ref) if hay_pico else float("nan"),
        "f_pico": float(f[i_pico]) if hay_pico else float("nan"),
        "f_fiable": float(f[max(n_fiable - 1, 0)]),
        "coh_media": float(np.mean(coh[banda_coh])) if banda_coh.any() else float("nan"),
        "coh_min": float(np.min(coh[banda_coh])) if banda_coh.any() else float("nan"),
    }


def imprime(nombre, r, f, mag, fase, coh):
    print(f"\n  ── {nombre} " + "─" * max(0, 52 - len(nombre)))
    print(f"    ancho de banda a −3 dB      {r['bw_3dB']:>8.2f} Hz")
    if math.isnan(r["f_fase90"]):
        print(f"    fase −90°                     no llega dentro de la banda")
    else:
        print(f"    fase −90°                   {r['f_fase90']:>8.2f} Hz")
    if math.isnan(r["pico_dB"]):
        print(f"    resonancia                    ninguna: cae monótonamente")
    else:
        print(f"    pico de resonancia          {r['pico_dB']:>8.2f} dB"
              f"  a {r['f_pico']:.2f} Hz")
    print(f"    banda fiable hasta          {r['f_fiable']:>8.2f} Hz")
    print(f"    coherencia media 0.2–3 Hz   {r['coh_media']:>8.3f}"
          + ("   ✔" if r["coh_media"] > 0.9 else "   ⚠ POR DEBAJO DE 0.9"))
    print(f"    coherencia mínima           {r['coh_min']:>8.3f}")
    if r["coh_media"] <= 0.9:
        print("    La estimación NO es fiable: la salida no está explicada por\n"
              "    la entrada. Sospechosos: fricción o zona muerta. Sube --vmax.")
    print(f"\n    {'f [Hz]':>8}{'|H| [dB]':>10}{'fase [°]':>10}{'coh':>8}")
    for objetivo in (0.2, 0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0):
        i = int(np.argmin(np.abs(f - objetivo)))
        if abs(f[i] - objetivo) > objetivo * 0.25:
            continue
        print(f"    {f[i]:>8.2f}{mag[i]-r['ref_dB']:>10.2f}{fase[i]:>10.1f}"
              f"{coh[i]:>8.3f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.set_defaults(channel="lowcmd", gains="tuned_gff", gravity_ff=True)
    ap.add_argument("--joint", default=None)
    ap.add_argument("--vmax", type=float, default=0.3,
                    help="velocidad de pico, rad/s (F5 pide 0.3)")
    ap.add_argument("--f0", type=float, default=0.2)
    ap.add_argument("--f1", type=float, default=5.0)
    ap.add_argument("--duration", type=float, default=90.0)
    ap.add_argument("--fade", type=float, default=2.0)
    ap.add_argument("--zero-dq", action="store_true",
                    help="mandar dq_des = 0, como xr_teleoperate sin el parche")
    ap.add_argument("--analiza", default=None,
                    help="analizar un CSV ya grabado, sin robot")
    a = ap.parse_args()

    if a.analiza:
        for ruta in sorted(glob.glob(a.analiza)):
            t, qd, q = carga_traza(ruta)
            f, mag, fase, coh = analiza(t, qd, q, a.f0, a.f1)
            r = resume(f, mag, fase, coh, a.f0, a.f1)
            imprime(Path(ruta).name, r, f, mag, fase, coh)
        return 0

    if not a.joint:
        raise SystemExit("hace falta --joint (o --analiza)")
    idx = joint_index(a.joint)
    j = BY_INDEX[idx]
    gains = cfg.load(a.gains)

    # La amplitud manda al principio del barrido: v_max/(2πf0). Se acota a lo
    # que quede de recorrido para que el chirp no se coma el tope.
    amp0 = a.vmax / (2.0 * math.pi * a.f0)
    print(f"\n  {j.name} · chirp log {a.f0}→{a.f1} Hz en {a.duration:.0f} s")
    print(f"  velocidad de pico {a.vmax} rad/s  ->  amplitud "
          f"{math.degrees(amp0):.1f}° a {a.f0} Hz, "
          f"{math.degrees(a.vmax/(2*math.pi*a.f1)):.2f}° a {a.f1} Hz")

    cli = build_client(a, [idx])
    try:
        cli.wait_for_state()
        cli.engage()
        apply_test_posture(cli, a, gains)
        q0 = float(cli.q_base[idx])
        lo, hi = gains.limits(idx)
        margen = min(q0 - lo, hi - q0)
        amp = min(amp0, max(margen * 0.8, 0.0))
        if amp < amp0:
            print(f"  ⚠ amplitud acotada a {math.degrees(amp):.1f}° por el "
                  f"recorrido disponible ({math.degrees(margen):.1f}°): la banda "
                  f"baja quedará subexcitada y la coherencia lo dirá.")

        traj = tr.log_chirp(a.vmax, a.f0, a.f1, a.duration, a.fade, amp_max=amp)
        if a.zero_dq:
            traj = tr.zero_velocity(traj)

        confirm(a, f"Barrido de 90 s en {j.name}. Robot COLGADO DEL ARNÉS.")

        cli.record(True)
        cli.set_trajectory(idx, traj)
        cli.sleep(a.duration + 1.0)
        cli.clear_trajectory(idx)
        samples = cli.record(False)
        cli.ramp_to({idx: q0}, speed=0.3)
    except SafetyAbort as e:
        print(f"\n  ⚠ abortado por seguridad: {e}")
        return 1
    finally:
        cli.__exit__(None, None, None)

    stamp = rec.stamp()
    et = f"{a.gains}{'_zerodq' if a.zero_dq else ''}"
    ruta = cfg.LOG_DIR / f"bode_{j.name}_{et}_{stamp}.csv"
    rec.save_samples(samples, [idx], ruta)

    t = np.array([s.t for s in samples])
    qd = np.array([s.q_des[idx] for s in samples])
    q = np.array([s.q[idx] for s in samples])
    f, mag, fase, coh = analiza(t, qd, q, a.f0, a.f1)
    r = resume(f, mag, fase, coh, a.f0, a.f1)
    imprime(f"{j.name} · {et}", r, f, mag, fase, coh)
    print(f"\n  traza -> {ruta}")

    rec.append_index({"stamp": stamp, "test": "bode", "channel": a.channel,
                      "gains": et, "rate_hz": a.rate, "joint": j.name,
                      "kp": gains.for_index(idx)[0], "kd": gains.for_index(idx)[1],
                      "csv": ruta.name, "vmax": a.vmax,
                      "zero_dq": int(a.zero_dq), **r})
    return 0


if __name__ == "__main__":
    sys.exit(main())
