#!/usr/bin/env python3
"""Gráficas de un ensayo guardado. No toca el robot: solo lee CSV.

Por cada articulación del CSV dibuja tres paneles:
  1. consigna contra medida,
  2. error de seguimiento,
  3. par estimado, con el límite del URDF marcado.
Y, si el ensayo dura lo bastante, el espectro de la velocidad: ahí se ve el
temblor como un pico, y a qué frecuencia.

Ejemplos:
    python3 scripts/05_plot.py logs/step_L_elbow_kp50_kd1.0_20260904_161500.csv
    python3 scripts/05_plot.py logs/sweep_*_L_elbow_*.csv     # varios superpuestos
    python3 scripts/05_plot.py --last                          # el más reciente
"""
from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")           # sin ventana: guarda PNG y ya
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from h1_2_joint_control.config import LOG_DIR          # noqa: E402
from h1_2_joint_control.joints import BY_NAME          # noqa: E402


def load(path: Path):
    with path.open(encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
    data = np.genfromtxt(path, delimiter=",", skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    cols = {name: data[:, i] for i, name in enumerate(header)}
    joints = sorted({c.split(".")[0] for c in header if "." in c})
    return cols, joints


def spectrum(x, fs, f_min=0.0):
    n = len(x)
    x = (x - x.mean()) * np.hanning(n)
    mag = np.abs(np.fft.rfft(x)) * 2.0 / n
    f = np.fft.rfftfreq(n, 1.0 / fs)
    keep = f >= f_min
    return f[keep], mag[keep]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="*", help="ficheros CSV (admite comodines)")
    ap.add_argument("--last", action="store_true", help="usar el CSV más reciente")
    ap.add_argument("--out", default=None, help="PNG de salida")
    ap.add_argument("--no-spectrum", action="store_true")
    a = ap.parse_args()

    paths: list[Path] = []
    for pat in a.csv:
        paths += [Path(p) for p in sorted(glob.glob(pat))] or (
            [Path(LOG_DIR / p) for p in sorted(glob.glob(str(LOG_DIR / pat)))])
    if a.last or not paths:
        found = sorted(LOG_DIR.glob("*.csv"), key=lambda p: p.stat().st_mtime)
        found = [p for p in found if p.name != "index.csv"]
        if not found:
            raise SystemExit(f"no hay CSV en {LOG_DIR}")
        paths = [found[-1]]
    paths = [p for p in paths if p.name != "index.csv"]
    print(f"  {len(paths)} fichero(s):")
    for p in paths:
        print(f"    {p.name}")

    joints = sorted({j for p in paths for j in load(p)[1]})
    n_rows = len(joints) * (1 if a.no_spectrum else 1)
    fig, axes = plt.subplots(len(joints), 4 if not a.no_spectrum else 3,
                             figsize=(20 if not a.no_spectrum else 15,
                                      3.4 * len(joints)), squeeze=False)

    for r, jname in enumerate(joints):
        ax_q, ax_e, ax_t = axes[r][0], axes[r][1], axes[r][2]
        for p in paths:
            cols, js = load(p)
            if jname not in js:
                continue
            t = cols["t"]
            q_des, q = cols[f"{jname}.q_des"], cols[f"{jname}.q"]
            dq, tau = cols[f"{jname}.dq"], cols[f"{jname}.tau"]
            label = p.stem[:44]
            ax_q.plot(t, np.degrees(q), lw=1.2, label=label)
            if p is paths[0]:
                ax_q.plot(t, np.degrees(q_des), "k--", lw=1.0, label="consigna")
            ax_e.plot(t, np.degrees(q - q_des), lw=1.0, label=label)
            ax_t.plot(t, tau, lw=1.0, label=label)
            if not a.no_spectrum and len(t) > 64:
                fs = (len(t) - 1) / (t[-1] - t[0])
                f, mag = spectrum(dq, fs, f_min=0.5)
                axes[r][3].semilogy(f, np.maximum(mag, 1e-6), lw=0.9, label=label)

        j = BY_NAME.get(jname)
        ax_q.set_title(f"{jname} — posición")
        ax_q.set_ylabel("q [°]")
        ax_q.grid(alpha=0.3)
        ax_q.legend(fontsize=6)
        ax_e.set_title("error de seguimiento (medida − consigna)")
        ax_e.set_ylabel("error [°]")
        ax_e.axhline(0, color="k", lw=0.6)
        ax_e.grid(alpha=0.3)
        ax_t.set_title("par estimado")
        ax_t.set_ylabel("tau_est [Nm]")
        ax_t.grid(alpha=0.3)
        if j is not None:
            for s in (1, -1):
                ax_t.axhline(s * j.tau_max, color="r", ls=":", lw=0.9)
            ax_t.text(0.01, 0.92, f"límite URDF ±{j.tau_max:.0f} Nm",
                      transform=ax_t.transAxes, color="r", fontsize=7)
        for ax in axes[r][:3]:
            ax.set_xlabel("t [s]")
        if not a.no_spectrum:
            axes[r][3].set_title("espectro de la velocidad (temblor)")
            axes[r][3].set_xlabel("frecuencia [Hz]")
            axes[r][3].set_ylabel("|dq| [rad/s]")
            axes[r][3].axvline(8.0, color="orange", ls=":", lw=0.9)
            axes[r][3].text(8.3, axes[r][3].get_ylim()[1] * 0.3,
                            "corte del índice\nde temblor", fontsize=6, color="orange")
            axes[r][3].grid(alpha=0.3, which="both")

    fig.tight_layout()
    out = Path(a.out) if a.out else (LOG_DIR / (paths[0].stem + ".png"
                                                if len(paths) == 1
                                                else f"comparacion_{joints[0]}.png"))
    fig.savefig(out, dpi=110)
    print(f"\n  -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
