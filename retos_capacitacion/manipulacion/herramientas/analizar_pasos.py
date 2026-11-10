"""Analisis de herramientas/curva_pasos.py (PDF 6.3.1-2): inclinacion y giro del cuerpo frente a la duracion.

Por movimiento, duracion y sentido (ida: de 0 a la pose; vuelta: de la pose a 0), respecto de lo medido en los
2 s quietos de antes:
  pico       la mayor desviacion (con su signo) mientras se mueve y en los s quietos de despues
  queda      la media de los 2 ultimos s quietos: lo que deja el centro de masas desplazado (el estabilizador y su
             integral lo van quitando despacio)
  exceso     |pico| - |queda|: lo que pone la dinamica del movimiento por encima de lo estatico
del pitch y del roll de la IMU (grados), el yaw (giro; pico y al final), la carga en los pies y el par de los
tobillos. Y del movimiento mandado (la consigna de arnes.py): el mayor recorrido de una junta, su velocidad y su
aceleracion de pico, y el desplazamiento, la velocidad y la aceleracion de pico del centro de masas del robot entero
(manip/dinamica.py, con las masas identificadas de los brazos).

Pasos: de pie por rt/lowcmd en el arnes las piernas siguen su postura, no hay marcha que de pasos: el cuerpo
responde inclinandose y girando, y eso es lo que se mide.

Despues, para cada medida, el r2 de una recta contra cada candidato (velocidad, aceleracion o desplazamiento del
centro de masas, velocidad o aceleracion articular): lo que mejor la explica. Y la duracion minima que deja el
exceso de pitch y el giro por debajo de los umbrales en todos los movimientos.

    python3 herramientas/analizar_pasos.py datos/pasos_<fecha> [--umbral-exceso 0.2] [--umbral-yaw 0.5]
Escribe en la carpeta analisis.csv, analisis.json y curva.png.
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from manip.cuerpo import BRAZOS  # noqa: E402

VENTANA_S = 2.0
TOBILLO_PITCH, TOBILLO_ROLL = (4, 10), (5, 11)
CANDIDATOS = {"dcom_cm": "desplazamiento del c. de masas", "vcom_cms": "velocidad del c. de masas",
              "acom_cms2": "aceleracion del c. de masas", "vq": "velocidad articular", "aq": "aceleracion articular"}


def cargar(carpeta):
    with open(os.path.join(carpeta, "muestras.csv")) as f:
        filas = list(csv.DictReader(f))
    if not filas:
        raise ValueError(f"{carpeta}/muestras.csv sin muestras")
    etiquetas = {k: [r[k] for r in filas] for k in ("mov", "T", "rep", "tramo")}
    num = {}
    for k in filas[0]:
        if k in etiquetas:
            continue
        num[k] = np.array([float(r[k]) if r[k] != "" else np.nan for r in filas])
    return etiquetas, num


def _media_final(t, x, idx, s=VENTANA_S):
    if len(idx) == 0:
        return np.nan
    sel = idx[t[idx] >= t[idx[-1]] - s]
    return float(np.nanmean(x[sel]))


def _pico(x):
    """El valor de mayor modulo, con su signo."""
    if len(x) == 0 or np.all(np.isnan(x)):
        return np.nan
    return float(x[np.nanargmax(np.abs(x))])


def senales(num):
    """Lo que se mide del cuerpo, en las unidades del informe."""
    yaw = np.degrees(np.unwrap(num["yaw"]))
    return {"pitch": np.degrees(num["pitch"]), "roll": np.degrees(num["roll"]), "yaw": yaw,
            "carga": 100.0 * num["carga"],
            "tob_pitch": num[f"tau{TOBILLO_PITCH[0]}"] + num[f"tau{TOBILLO_PITCH[1]}"],
            "tob_roll": num[f"tau{TOBILLO_ROLL[0]}"] + num[f"tau{TOBILLO_ROLL[1]}"]}


def cinematica(t, num, idx, din=None, cada_s=0.05):
    """Del movimiento mandado entre los indices idx: recorrido, velocidad y aceleracion de pico de las juntas y del
    centro de masas (horizontal, x adelante e y izquierda)."""
    tt = t[idx]
    c = np.stack([num[f"c{j}"][idx] for j in BRAZOS], axis=1)
    rejilla = np.arange(tt[0], tt[-1] + 1e-9, cada_s)
    if len(rejilla) < 5:
        return {}
    ci = np.stack([np.interp(rejilla, tt, c[:, k]) for k in range(c.shape[1])], axis=1)
    v = np.gradient(ci, cada_s, axis=0)
    acc = np.gradient(v, cada_s, axis=0)
    r = {"dq": float(np.abs(ci[-1] - ci[0]).max()), "vq": float(np.abs(v).max()), "aq": float(np.abs(acc).max())}
    if din is not None:
        piernas = np.array([np.nanmean(num[f"q{j}"][idx]) for j in range(13)])
        com = []
        for fila in ci:
            q = np.zeros(27)
            q[:13] = piernas
            q[BRAZOS] = fila
            com.append(din.centro_de_masas(q))
        com = np.array(com)
        vc = np.gradient(com[:, :2], cada_s, axis=0)
        ac = np.gradient(vc, cada_s, axis=0)
        d = com[-1] - com[0]
        r.update(dcom_x_cm=100 * float(d[0]), dcom_y_cm=100 * float(d[1]), dcom_z_cm=100 * float(d[2]),
                 dcom_cm=100 * float(np.hypot(d[0], d[1])), vcom_cms=100 * float(np.hypot(*vc.T).max()),
                 acom_cms2=100 * float(np.hypot(*ac.T).max()))
    return r


def analizar(carpeta, din=None):
    """[filas]: una por movimiento, duracion, repeticion y sentido completos."""
    et, num = cargar(carpeta)
    t = num["t"]
    s = senales(num)
    err = np.degrees(np.max(np.abs(np.stack([num[f"q{j}"] - num[f"c{j}"] for j in BRAZOS], axis=1)), axis=1))
    grupos = {}
    for i, clave in enumerate(zip(et["mov"], et["T"], et["rep"], et["tramo"])):
        grupos.setdefault(clave, []).append(i)
    grupos = {k: np.array(v) for k, v in grupos.items()}
    filas = []
    for (mov, T, rep) in sorted({k[:3] for k in grupos if k[0]}, key=lambda k: (k[0], -float(k[1]), int(k[2]))):
        g = {tr: grupos.get((mov, T, rep, tr), np.array([], int)) for tr in
             ("antes", "ida", "en_pose", "vuelta", "despues")}
        for sentido, (base, mueve, quieto) in {"ida": ("antes", "ida", "en_pose"),
                                               "vuelta": ("en_pose", "vuelta", "despues")}.items():
            if min(len(g[base]), len(g[mueve]), len(g[quieto])) < 3:
                continue
            fila = {"mov": mov, "T": float(T), "rep": int(rep), "sentido": sentido,
                    "T_real": float(t[g[mueve][-1]] - t[g[mueve][0]])}
            resp = np.r_[g[mueve], g[quieto]]
            for k, x in s.items():
                b = _media_final(t, x, g[base])
                d = x[resp] - b
                fila[f"{k}_pico"] = _pico(d)
                fila[f"{k}_queda"] = _media_final(t, x, g[quieto]) - b
            for k in ("pitch", "roll"):
                fila[f"{k}_exceso"] = abs(fila[f"{k}_pico"]) - abs(fila[f"{k}_queda"])
            fila["err_brazo_deg"] = float(np.nanmax(err[g[mueve]]))
            fila.update(cinematica(t, num, g[mueve], din))
            filas.append(fila)
    return filas


def r2(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = ~(np.isnan(x) | np.isnan(y))
    x, y = x[ok], y[ok]
    if len(x) < 3 or np.ptp(x) == 0 or np.ptp(y) == 0:
        return np.nan
    return float(np.corrcoef(x, y)[0, 1] ** 2)


def explicacion(filas):
    """{medida: [(candidato, r2)] de mejor a peor} con el modulo de cada medida."""
    sal = {}
    for medida in ("pitch_pico", "pitch_exceso", "pitch_queda", "roll_pico", "yaw_pico"):
        y = [abs(f[medida]) for f in filas]
        cands = [(c, r2([abs(f.get(c, np.nan)) for f in filas], y)) for c in CANDIDATOS]
        sal[medida] = sorted(cands, key=lambda c: (bool(np.isnan(c[1])), -np.nan_to_num(c[1])))
    return sal


def duracion_minima(filas, umbral_exceso, umbral_yaw):
    """La menor duracion probada en la que, en todos los movimientos y sentidos, el exceso de pitch y de roll y el
    giro quedan por debajo de los umbrales (None si ninguna)."""
    buenas = []
    for T in sorted({f["T"] for f in filas}):
        fs = [f for f in filas if f["T"] == T]
        if all(f["pitch_exceso"] <= umbral_exceso and f["roll_exceso"] <= umbral_exceso and
               abs(f["yaw_pico"]) <= umbral_yaw for f in fs):
            buenas.append(T)
    return min(buenas) if buenas else None


def dibujar(filas, ruta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    for mov in sorted({f["mov"] for f in filas}):
        for sentido, estilo in (("ida", "-o"), ("vuelta", "--s")):
            fs = sorted((f for f in filas if f["mov"] == mov and f["sentido"] == sentido), key=lambda f: f["T"])
            if not fs:
                continue
            T = [f["T"] for f in fs]
            ax[0].plot(T, [f["pitch_pico"] for f in fs], estilo, label=f"{mov} {sentido}")
            ax[1].plot(T, [f["yaw_pico"] for f in fs], estilo, label=f"{mov} {sentido}")
            ax[2].plot([f.get("acom_cms2", np.nan) for f in fs], [f["pitch_exceso"] for f in fs], estilo,
                       label=f"{mov} {sentido}")
    ax[0].set(xlabel="duracion del movimiento (s)", ylabel="pitch, pico (grados)", title="Inclinacion")
    ax[1].set(xlabel="duracion del movimiento (s)", ylabel="yaw, pico (grados)", title="Giro")
    ax[2].set(xlabel="aceleracion de pico del c. de masas (cm/s2)", ylabel="pitch, exceso sobre lo estatico (grados)",
              title="Lo dinamico")
    for a in ax:
        a.grid(alpha=0.3)
        a.axhline(0, color="k", lw=0.5)
    ax[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(ruta, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("carpeta")
    ap.add_argument("--umbral-exceso", type=float, default=0.2, help="grados de pitch/roll por encima de lo estatico")
    ap.add_argument("--umbral-yaw", type=float, default=0.5, help="grados de giro")
    ap.add_argument("--sin-modelo", action="store_true", help="sin el centro de masas (sin pinocchio)")
    a = ap.parse_args()
    din = None
    if not a.sin_modelo:
        from manip.dinamica import Dinamica
        din = Dinamica()
    filas = analizar(a.carpeta, din)
    if not filas:
        sys.exit("Ningun movimiento completo en el registro.")
    print(f"\n  {a.carpeta}: {len(filas)} movimientos completos. Grados respecto de los 2 s quietos de antes; "
          "carga en % del peso; tobillos, suma de los dos (N m).\n")
    cab = (f"  {'movimiento':12s} {'T':>4s} {'sentido':7s} {'dq':>5s} {'vq':>5s} {'aq':>5s} {'dCOMx':>6s} {'aCOM':>6s} "
           f"{'pitch':>6s} {'queda':>6s} {'exceso':>6s} {'roll':>6s} {'yaw':>6s} {'yawfin':>6s} {'carga':>6s} "
           f"{'tobP':>6s} {'errbr':>5s}")
    print(cab + "\n  " + " " * 12 + "    s          rad  rad/s rad/s2     cm  cm/s2" + " " * 18 + "grados")
    for f in filas:
        print(f"  {f['mov']:12s} {f['T']:4.0f} {f['sentido']:7s} {f.get('dq', np.nan):5.2f} {f.get('vq', np.nan):5.2f} "
              f"{f.get('aq', np.nan):5.2f} {f.get('dcom_x_cm', np.nan):+6.1f} {f.get('acom_cms2', np.nan):6.2f} "
              f"{f['pitch_pico']:+6.2f} {f['pitch_queda']:+6.2f} {f['pitch_exceso']:+6.2f} {f['roll_pico']:+6.2f} "
              f"{f['yaw_pico']:+6.2f} {f['yaw_queda']:+6.2f} {f['carga_pico']:+6.1f} {f['tob_pitch_pico']:+6.1f} "
              f"{f['err_brazo_deg']:5.1f}")
    print("\n  Pasos: 0 (de pie por rt/lowcmd las piernas siguen su postura: el cuerpo se inclina y gira, no anda).")
    exp = explicacion(filas)
    print("\n  Que explica cada medida (r2 de una recta con el modulo):")
    for medida, cands in exp.items():
        print(f"    {medida:13s} " + ", ".join(f"{CANDIDATOS[c]} {v:.2f}" for c, v in cands if not np.isnan(v)))
    tmin = duracion_minima(filas, a.umbral_exceso, a.umbral_yaw)
    print(f"\n  Duracion minima con exceso <= {a.umbral_exceso:g} grados y giro <= {a.umbral_yaw:g} grados en todo: "
          + (f"{tmin:g} s" if tmin is not None else "ninguna de las probadas") + " (el PDF no deja bajar de 8 s).")
    with open(os.path.join(a.carpeta, "analisis.csv"), "w", newline="") as fh:
        claves = sorted({k for f in filas for k in f}, key=lambda k: list(filas[0]).index(k) if k in filas[0] else 99)
        w = csv.DictWriter(fh, claves)
        w.writeheader()
        w.writerows(filas)
    with open(os.path.join(a.carpeta, "analisis.json"), "w") as fh:
        json.dump({"filas": filas, "explicacion": {k: [[c, None if np.isnan(v) else round(v, 3)] for c, v in cs]
                                                   for k, cs in exp.items()},
                   "duracion_minima_s": tmin, "umbral_exceso": a.umbral_exceso, "umbral_yaw": a.umbral_yaw},
                  fh, indent=1, default=lambda x: None if isinstance(x, float) and math.isnan(x) else x)
    try:
        dibujar(filas, os.path.join(a.carpeta, "curva.png"))
        print(f"  Grafica: {os.path.join(a.carpeta, 'curva.png')}")
    except ImportError:
        print("  (sin matplotlib: sin grafica)")


if __name__ == "__main__":
    main()
