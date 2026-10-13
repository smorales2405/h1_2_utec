#!/usr/bin/env python3
"""Analisis de una prueba de escalon_vyaw.py, en la PC (pregunta 6.3.1 del PDF). No usa el robot.

Por cada escalon, sobre el lowstate.csv (100 Hz):
  - retardo efectivo y ganancia: recta de regimen sobre el yaw, descontando la deriva de la base
    (para el 2o escalon, solo la 2a mitad de la vuelta: al principio el robot aun para el giro);
  - t63 de arranque y de parada: gz con una media centrada de un periodo de la marcha (quita el
    balanceo sin retrasar; por eso el t10 sale adelantado y no se da);
  - la cadencia de la marcha (pico del espectro del roll).

    python3 herramientas/analizar_escalon.py datos/escalon_20261003_055636_vx0_a0.3 [--figura]

Escribe analisis.json (y analisis.png con --figura) en la carpeta de la prueba.
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import registro  # noqa: E402
from seguidor.analisis import cadencia, media_centrada, respuesta_escalon, tiempo_hasta  # noqa: E402


def fases(t, estado, vyaw):
    """[(estado, t_ini, t_fin, vyaw)] seguidas, a partir de las columnas del lowstate.csv."""
    out, ini = [], 0
    for i in range(1, len(t) + 1):
        if i == len(t) or estado[i] != estado[ini] or vyaw[i] != vyaw[ini]:
            out.append((estado[ini], t[ini], t[i - 1], vyaw[ini]))
            ini = i
    return out


def main():
    ap = argparse.ArgumentParser(description="Analisis de una prueba de escalones de vyaw")
    ap.add_argument("carpeta")
    ap.add_argument("--figura", action="store_true", help="guardar analisis.png (matplotlib)")
    args = ap.parse_args()

    with open(os.path.join(args.carpeta, "lowstate.csv")) as f:
        filas = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in filas])
    yaw = np.unwrap(np.radians([float(r["yaw_deg"]) for r in filas]))
    gz = np.array([float(r["gz"]) for r in filas])
    roll = np.array([float(r["roll_deg"]) for r in filas])
    estado = [r["estado"] for r in filas]
    vyaw = np.array([float(r["vyaw_cmd"]) for r in filas])

    tramos = fases(t, estado, vyaw)
    marcha = np.isin(estado, ["base", "escalon", "vuelta"])
    f_marcha = cadencia(t[marcha], roll[marcha])
    gzf = media_centrada(t, gz, 1.0 / f_marcha)
    print(f"cadencia de la marcha {f_marcha:.2f} Hz (periodo {1 / f_marcha:.2f} s)")

    resultados = []
    for i, (nombre, t_ini, t_fin, w) in enumerate(tramos):
        if nombre != "escalon" or i == 0:
            continue
        nombre_b, b_ini, b_fin, _ = tramos[i - 1]
        if nombre_b == "vuelta":
            b_ini = (b_ini + b_fin) / 2
        r = respuesta_escalon(t, yaw, b_ini, t_ini, t_fin, w)
        base = gzf[(t >= b_ini) & (t < t_ini)].mean()
        regimen = gzf[(t >= (t_ini + t_fin) / 2) & (t <= t_fin)].mean()
        despues = (t > t_fin + 1.0) & (t <= t_fin + 2.0)
        final = gzf[despues].mean() if despues.any() else base
        r.update(t63_arranque=tiempo_hasta(t, gzf, t_ini, base, regimen, 0.63),
                 t90_arranque=tiempo_hasta(t, gzf, t_ini, base, regimen, 0.90),
                 t63_parada=tiempo_hasta(t, gzf, t_fin, regimen, final, 0.63))
        resultados.append(r)

    print("orden    ganancia  retardo  t63 arranque  t90 arranque  t63 parada  deriva base")
    for r in resultados:
        print(f"{r['amplitud']:+.2f}    {r['ganancia']:6.2f}   {r['retardo']:5.2f} s   {r['t63_arranque']:5.2f} s"
              f"       {r['t90_arranque']:5.2f} s      {r['t63_parada']:5.2f} s   {math.degrees(r['deriva']):+.1f} grados/s")
    registro.guardar_json(os.path.join(args.carpeta, "analisis.json"),
                          {"cadencia_hz": f_marcha, "escalones": resultados})

    if args.figura:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        t0 = t[0]
        a1.plot(t - t0, np.degrees(yaw - yaw[0]), lw=1)
        a1.set_ylabel("yaw (grados)")
        a2.plot(t - t0, gz, lw=0.5, alpha=0.4, label="gz")
        a2.plot(t - t0, gzf, lw=1.5, label=f"gz, media de {1 / f_marcha:.2f} s")
        a2.step(t - t0, vyaw, where="post", lw=1.5, label="vyaw mandada")
        a2.set_ylabel("rad/s")
        a2.set_xlabel("s")
        a2.legend(loc="upper right", fontsize=8)
        fig.suptitle(os.path.basename(os.path.normpath(args.carpeta)))
        fig.tight_layout()
        fig.savefig(os.path.join(args.carpeta, "analisis.png"), dpi=110)
    print(f"Escrito {os.path.join(args.carpeta, 'analisis.json')}")


if __name__ == "__main__":
    main()
