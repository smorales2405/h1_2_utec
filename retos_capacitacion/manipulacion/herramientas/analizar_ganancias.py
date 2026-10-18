#!/usr/bin/env python3
"""Analiza las pasadas de comparar_ganancias.py y compara los juegos de ganancias (en la PC, tras
./desplegar.sh --traer).

    python3 herramientas/analizar_ganancias.py datos/ganancias_<fecha>_actual datos/ganancias_<fecha>_tuned
    python3 herramientas/analizar_ganancias.py          # la ultima pasada de cada juego en datos/ (sin las _sim)

De lowstate.csv (la columna estado dice el tramo: Tnn en movimiento, Pnn en la pausa del final), por tramo y por
junta del brazo:
  - error de seguimiento (medida - consigna): RMS y maximo en movimiento; estable = media del ultimo 0.5 s de
    la pausa (con PD sin compensar la gravedad, el brazo se queda corto: tau_g / kp)
  - sobreimpulso: lo que la junta pasa de su valor estable en el sentido del movimiento
  - asentamiento: desde el final del movimiento hasta quedar a menos de 0.3 grados del valor estable
  - temblor: desviacion tipica de la medida en la pausa, desde 0.5 s
  - par: RMS y maximo; temperatura: subida en la pasada
y del cuerpo: roll y pitch (maximo respecto al inicio de la pasada), giro (yaw), velocidad angular RMS y la
mayor desviacion de una junta de pierna (un paso). Escribe datos/analisis_ganancias_<fecha>/ con resumen.md,
por_tramo.csv y graficas.
"""

import argparse
import csv
import glob
import json
import os
import sys
import time
import warnings

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from manip import config  # noqa: E402
from manip.registro import NOMBRES  # noqa: E402

BRAZOS = list(range(13, 27))
PIERNAS = list(range(12))
ASENTADO_DEG = 0.3
CORTO = {n: n.replace("Shoulder", "S").replace("Wrist", "W").replace("Elbow", "Codo") for n in NOMBRES}


def cargar(carpeta):
    with open(os.path.join(carpeta, "resumen.json")) as f:
        r = json.load(f)
    with open(os.path.join(carpeta, "lowstate.csv")) as f:
        rd = csv.reader(f)
        cab = next(rd)
        filas = list(rd)
    i = {c: k for k, c in enumerate(cab)}
    cols = (["t", "peso", "roll_deg", "pitch_deg", "yaw_deg", "gx", "gy", "gz"]
            + [f"{NOMBRES[j]}_q" for j in PIERNAS]
            + [f"{NOMBRES[j]}_{s}" for s in ("q", "dq", "tau", "T") for j in BRAZOS]
            + [f"{NOMBRES[j]}_obj" for j in BRAZOS])
    idx = [i[c] for c in cols]
    M = np.array([[float(x[k]) if x[k] != "" else np.nan for k in idx] for x in filas])
    c = {n: M[:, k] for k, n in enumerate(cols)}
    nb = len(BRAZOS)
    d = {"juego": r.get("juego", os.path.basename(carpeta)), "carpeta": carpeta, "resumen": r,
         "estado": np.array([x[i["estado"]] for x in filas]), "t": c["t"], "peso": c["peso"],
         "roll": c["roll_deg"], "pitch": c["pitch_deg"], "yaw": c["yaw_deg"],
         "gyro": np.degrees(np.sqrt(c["gx"] ** 2 + c["gy"] ** 2 + c["gz"] ** 2)),
         "piernas": np.column_stack([c[f"{NOMBRES[j]}_q"] for j in PIERNAS])}
    for s in ("q", "dq", "tau", "T", "obj"):
        d[s] = np.column_stack([c[f"{NOMBRES[j]}_{s}"] for j in BRAZOS])
    assert d["q"].shape[1] == nb
    d["err"] = np.degrees(d["q"] - d["obj"])
    return d


def por_tramo(d):
    """[{metricas}] de cada tramo completo (con su pausa) de la pasada."""
    est, t = d["estado"], d["t"]
    ref = {"roll": d["roll"][0], "pitch": d["pitch"][0], "yaw": d["yaw"][0], "piernas": d["piernas"][0]}
    tramos = {m["n"]: m for m in d["resumen"].get("metricas", [])}
    sal = []
    for n in sorted(tramos):
        mov, pau = est == f"T{n:02d}", est == f"P{n:02d}"
        if mov.sum() < 2 or pau.sum() < 2:
            continue
        todo = mov | pau
        tp = t[pau]
        ss = pau & (t >= tp[-1] - 0.5)
        q = np.degrees(d["q"])
        q_ss = q[ss].mean(axis=0)
        err_mov, err_ss = d["err"][mov], d["err"][ss].mean(axis=0)
        obj = np.degrees(d["obj"])
        k0 = np.argmax(mov)
        delta = obj[np.where(todo)[0][-1]] - obj[k0 - 1 if k0 > 0 else k0]
        sentido = np.sign(delta) * (np.abs(delta) > 2.0)
        sobre = np.nanmax(np.maximum(0.0, (q[todo] - q_ss) * sentido), axis=0)
        fuera = np.abs(q[pau] - q_ss) > ASENTADO_DEG
        asent = np.array([tp[np.where(fuera[:, k])[0][-1]] - tp[0] if fuera[:, k].any() else 0.0 for k in range(len(BRAZOS))])
        quieto = pau & (t >= tp[0] + 0.5)
        temblor = q[quieto].std(axis=0) if quieto.sum() > 5 else np.full(len(BRAZOS), np.nan)
        rms = np.sqrt(np.nanmean(err_mov ** 2, axis=0))
        tau = d["tau"][todo]
        info = tramos[n]
        sal.append({
            "juego": d["juego"], "n": n, "nombre": info["nombre"], "T_s": info["T_s"],
            "err_rms_mov_deg": float(np.nanmean(rms)), "err_max_mov_deg": float(np.nanmax(np.abs(err_mov))),
            "junta_err_max": CORTO[NOMBRES[BRAZOS[int(np.nanargmax(np.abs(err_mov).max(axis=0)))]]],
            "err_estable_medio_deg": float(np.nanmean(np.abs(err_ss))), "err_estable_max_deg": float(np.nanmax(np.abs(err_ss))),
            "junta_err_estable": CORTO[NOMBRES[BRAZOS[int(np.nanargmax(np.abs(err_ss)))]]],
            "sobreimpulso_max_deg": float(np.nanmax(sobre)), "asentamiento_max_s": float(asent.max()),
            "temblor_max_deg": float(np.nanmax(temblor)) if np.isfinite(temblor).any() else np.nan,
            "roll_max_deg": float(np.nanmax(np.abs(d["roll"][todo] - ref["roll"]))),
            "pitch_max_deg": float(np.nanmax(np.abs(d["pitch"][todo] - ref["pitch"]))),
            "yaw_fin_deg": float(((d["yaw"][np.where(todo)[0][-1]] - ref["yaw"] + 180) % 360) - 180),
            "gyro_rms_deg_s": float(np.sqrt(np.nanmean(d["gyro"][todo] ** 2))),
            "pierna_max_deg": float(np.degrees(np.nanmax(np.abs(d["piernas"][todo] - ref["piernas"])))),
            "tau_rms_nm": float(np.sqrt(np.nanmean(tau ** 2))), "tau_max_nm": float(np.nanmax(np.abs(tau))),
            "_rms": rms, "_ss": err_ss, "_sobre": sobre, "_temblor": temblor, "_tau_max": np.nanmax(np.abs(tau), axis=0)})
    return sal


METRICAS = [("err_rms_mov_deg", "error RMS en movimiento (grados, media de tramos)", np.mean),
            ("err_max_mov_deg", "error maximo en movimiento (grados, peor tramo)", np.max),
            ("err_estable_medio_deg", "error estable medio (grados, media de tramos)", np.mean),
            ("err_estable_max_deg", "error estable maximo (grados, peor tramo)", np.max),
            ("sobreimpulso_max_deg", "sobreimpulso (grados, peor tramo)", np.max),
            ("asentamiento_max_s", "asentamiento a 0.3 grados (s, media de tramos)", np.mean),
            ("temblor_max_deg", "temblor en la pausa (grados, media de tramos)", np.nanmean),
            ("roll_max_deg", "roll respecto al inicio (grados, peor tramo)", np.max),
            ("pitch_max_deg", "pitch respecto al inicio (grados, peor tramo)", np.max),
            ("yaw_fin_deg", "giro al final de la pasada (grados)", lambda v: v[-1]),
            ("gyro_rms_deg_s", "velocidad angular del torso RMS (grados/s, media)", np.mean),
            ("pierna_max_deg", "junta de pierna, mayor desviacion (grados)", np.max),
            ("tau_rms_nm", "par RMS de los brazos (N m, media de tramos)", np.mean),
            ("tau_max_nm", "par maximo de los brazos (N m, peor tramo)", np.max)]


def ultimas():
    base = config.ruta_datos(config.cargar())
    todas = sorted(glob.glob(os.path.join(base, "ganancias_*")))
    reales = [c for c in todas if not c.endswith("_sim")] or todas
    por_juego = {}
    for c in reales:
        try:
            with open(os.path.join(c, "resumen.json")) as f:
                por_juego[json.load(f).get("juego")] = c
        except (OSError, ValueError):
            continue
    return list(por_juego.values())


def graficas(datos, tablas, salida):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colores = ["tab:blue", "tab:red", "tab:green", "tab:orange"]
    # por tramo
    fig, ax = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    for k, (d, tab) in enumerate(zip(datos, tablas)):
        n = [x["n"] for x in tab]
        ax[0].plot(n, [x["err_rms_mov_deg"] for x in tab], "o-", color=colores[k], label=d["juego"])
        ax[1].plot(n, [x["err_estable_max_deg"] for x in tab], "o-", color=colores[k])
        ax[1].plot(n, [x["err_estable_medio_deg"] for x in tab], "o--", color=colores[k], alpha=0.5)
        ax[2].plot(n, [x["roll_max_deg"] for x in tab], "o-", color=colores[k])
        ax[2].plot(n, [x["pitch_max_deg"] for x in tab], "s--", color=colores[k], alpha=0.6)
        ax[3].plot(n, [x["tau_max_nm"] for x in tab], "o-", color=colores[k])
    for a_, tit in zip(ax, ["error RMS en movimiento (grados)", "error estable: max (-) y medio (--) (grados)",
                            "roll (-) y pitch (--) max respecto al inicio (grados)", "par maximo del brazo (N m)"]):
        a_.set_title(tit, fontsize=10)
        a_.grid(alpha=0.3)
    ax[0].legend()
    ax[3].set_xlabel("tramo")
    fig.tight_layout()
    fig.savefig(os.path.join(salida, "por_tramo.png"), dpi=110)
    plt.close(fig)
    # error de cada junta en el tiempo, desde el primer tramo
    fig, ax = plt.subplots(7, 2, figsize=(14, 16), sharex=True)
    for k, d in enumerate(datos):
        t0 = d["t"][np.argmax(d["estado"] == "T01")]
        t = d["t"] - t0
        for c in range(len(BRAZOS)):
            a_ = ax[c % 7, c // 7]
            a_.plot(t, d["err"][:, c], color=colores[k], lw=0.7, label=d["juego"] if c == 0 else None)
            a_.set_title(CORTO[NOMBRES[BRAZOS[c]]], fontsize=9)
            a_.grid(alpha=0.3)
    ax[0, 0].legend()
    for a_ in ax[-1]:
        a_.set_xlabel("s desde el primer tramo")
    fig.suptitle("error de seguimiento: medida - consigna (grados)")
    fig.tight_layout()
    fig.savefig(os.path.join(salida, "error_juntas.png"), dpi=100)
    plt.close(fig)
    # cuerpo
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for k, d in enumerate(datos):
        t0 = d["t"][np.argmax(d["estado"] == "T01")]
        t = d["t"] - t0
        ax[0].plot(t, d["roll"] - d["roll"][0], color=colores[k], lw=0.8, label=d["juego"])
        ax[1].plot(t, d["pitch"] - d["pitch"][0], color=colores[k], lw=0.8)
        ax[2].plot(t, ((d["yaw"] - d["yaw"][0] + 180) % 360) - 180, color=colores[k], lw=0.8)
        ax[3].plot(t, np.degrees(np.abs(d["piernas"] - d["piernas"][0]).max(axis=1)), color=colores[k], lw=0.8)
    for a_, tit in zip(ax, ["roll (grados)", "pitch (grados)", "yaw (grados)", "pierna: mayor desviacion (grados)"]):
        a_.set_title(tit, fontsize=10)
        a_.grid(alpha=0.3)
    ax[0].legend()
    ax[3].set_xlabel("s desde el primer tramo")
    fig.tight_layout()
    fig.savefig(os.path.join(salida, "cuerpo.png"), dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("carpetas", nargs="*")
    a = ap.parse_args()
    warnings.simplefilter("ignore", RuntimeWarning)     # medias de tramos sin pausa larga: nan
    carpetas = a.carpetas or ultimas()
    if not carpetas:
        sys.exit("No hay pasadas de comparar_ganancias.py en datos/ (./desplegar.sh --traer)")
    datos = [cargar(c) for c in carpetas]
    tablas = [por_tramo(d) for d in datos]
    salida = os.path.join(config.ruta_datos(config.cargar()), f"analisis_ganancias_{time.strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(salida, exist_ok=True)
    juegos = [d["juego"] for d in datos]
    L = ["# Comparacion de ganancias", ""]
    for d, tab in zip(datos, tablas):
        r = d["resumen"]
        sube = {k: r.get("temperatura_final", {}).get(k, v) - v for k, v in r.get("temperatura_inicial", {}).items()}
        peor = max(sube, key=sube.get) if sube else None
        L.append(f"- **{d['juego']}**: `{os.path.relpath(d['carpeta'], RAIZ)}`, {r.get('motivo')}, {len(tab)} tramos completos, "
                 f"escala x{r['args']['escala_tiempo']:g}, {r.get('duracion_s', 0) / 60:.1f} min"
                 + (f"; temperatura +{sube[peor]} grados C como mucho ({peor})" if peor else ""))
    comunes = sorted(set.intersection(*[{x["n"] for x in tab} for tab in tablas]))
    if len(datos) > 1:
        L += ["", f"Tramos comparados (los completos en todas las pasadas): {len(comunes)}", ""]
    L += ["| metrica | " + " | ".join(juegos) + " |", "|---|" + "---|" * len(juegos)]
    for clave, texto, f in METRICAS:
        vals = []
        for tab in tablas:
            v = np.array([x[clave] for x in tab if x["n"] in comunes] if len(datos) > 1 else [x[clave] for x in tab], float)
            vals.append(f(v) if len(v) else np.nan)
        L.append(f"| {texto} | " + " | ".join(f"{v:.2f}" for v in vals) + " |")
    L += ["", "Por junta (media de los tramos comparados): error RMS en movimiento / error estable / sobreimpulso / par max", "",
          "| junta | " + " | ".join(juegos) + " |", "|---|" + "---|" * len(juegos)]
    for c, j in enumerate(BRAZOS):
        celdas = []
        for tab in tablas:
            sel = [x for x in tab if x["n"] in comunes] if len(datos) > 1 else tab
            if not sel:
                celdas.append("-")
                continue
            celdas.append(f"{np.mean([x['_rms'][c] for x in sel]):.2f} / {np.mean([abs(x['_ss'][c]) for x in sel]):.2f} / "
                          f"{np.mean([x['_sobre'][c] for x in sel]):.2f} / {np.max([x['_tau_max'][c] for x in sel]):.1f}")
        L.append(f"| {CORTO[NOMBRES[j]]} | " + " | ".join(celdas) + " |")
    with open(os.path.join(salida, "resumen.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    claves = [k for k in tablas[0][0] if not k.startswith("_")] if tablas and tablas[0] else []
    with open(os.path.join(salida, "por_tramo.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(claves)
        for tab in tablas:
            for x in tab:
                w.writerow([f"{x[k]:.4g}" if isinstance(x[k], float) else x[k] for k in claves])
    graficas(datos, tablas, salida)
    print("\n".join(L))
    print(f"\n{os.path.relpath(salida, RAIZ)}: resumen.md, por_tramo.csv, por_tramo.png, error_juntas.png, cuerpo.png")


if __name__ == "__main__":
    main()
