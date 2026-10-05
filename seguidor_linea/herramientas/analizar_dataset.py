#!/usr/bin/env python3
"""Analisis de un dataset grabado, en la PC. No usa el robot.

  - calidad: fotogramas, fps, perdidos y tramos andando (oscilacion de gz)
  - linea vista: fotogramas en que el detector minimo de la calibracion (seguidor/calibracion.py)
    ajusta la recta con al menos --min-puntos puntos
  - postura: pitch y roll medios del torso de pie y andando (la camara se inclina con ellos)
  - balanceo: cadencia y oscilacion rapida del angulo de la linea, sin y con el yaw de la IMU
  - avance: velocidad real, yaw de la camara respecto de la direccion real de avance y distancia
    al centro de giro (seguidor/analisis.py: direccion_de_avance), sin odometria

La geometria es la que se uso al grabar (resumen.json del dataset), sin el yaw de la camara: el
yaw es justo lo que se mide aqui.

    python3 herramientas/analizar_dataset.py datos/dataset_<fecha>_<nombre> [--figura]

Escribe analisis.json (y analisis.png con --figura) en la carpeta del dataset.
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import analisis, calibracion, config, registro  # noqa: E402
from seguidor.fuentes import FuenteDataset, ImuDataset  # noqa: E402
from seguidor.geometria import Intrinsecos, ModeloSuelo  # noqa: E402

DESFASE_IMU_S = 0.010   # la IMU, 10 ms antes de que llegue el fotograma (RESULTADOS.md, apartado 4)
ARRANQUE_S = 1.5        # s tras empezar a andar que no cuentan para el avance
MIN_PUNTOS_AVANCE = 100     # por debajo, la regresion del avance no es fiable
MIN_RANGO_AVANCE_DEG = 5.0  # ni si el angulo de la linea recorre menos que esto


def main():
    ap = argparse.ArgumentParser(description="Analisis de un dataset grabado (sin robot)")
    ap.add_argument("carpeta")
    ap.add_argument("--min-puntos", type=int, default=60, help="puntos de linea para dar el fotograma por bueno")
    ap.add_argument("--figura", action="store_true", help="guardar analisis.png (matplotlib)")
    args = ap.parse_args()

    with open(os.path.join(args.carpeta, "resumen.json")) as f:
        resumen = json.load(f)
    geo = resumen.get("geometria") or config.cargar()["geometria"]
    fuente = FuenteDataset(args.carpeta)
    intr = Intrinsecos.desde_dict(fuente.meta["ir"])
    modelo = ModeloSuelo(intr, geo["altura_m"], math.radians(geo["inclinacion_deg"]), math.radians(geo.get("roll_deg", 0.0)))

    t, a, theta, npts = [], [], [], []
    for fot in fuente:
        puntos = calibracion.puntos_de_linea(fot.ir, modelo, x_min=0.3, x_max=2.5)
        t.append(fot.t_rx)
        if len(puntos) >= args.min_puntos:
            a0, b, k = calibracion.ajustar_recta(puntos)
            a.append(a0 if k >= args.min_puntos // 2 else math.nan)
            theta.append(math.atan(b) if k >= args.min_puntos // 2 else math.nan)
        else:
            a.append(math.nan)
            theta.append(math.nan)
        npts.append(len(puntos))
    t, a, theta = np.array(t), np.array(a), np.array(theta)
    vista = np.isfinite(a)

    with open(os.path.join(args.carpeta, "lowstate.csv")) as f:
        filas = list(csv.DictReader(f))
    tl = np.array([float(r["t"]) for r in filas])
    gz = np.array([float(r["gz"]) for r in filas])
    roll = np.array([float(r["roll_deg"]) for r in filas])
    pitch = np.array([float(r["pitch_deg"]) for r in filas])
    and_l = analisis.andando(tl, gz)
    anda = np.interp(t, tl, and_l.astype(float)) > 0.5
    imu = ImuDataset(os.path.join(args.carpeta, "lowstate.csv"))
    yaw = np.array([imu.en(x - DESFASE_IMU_S)[2] for x in t])

    out = {"dataset": os.path.basename(os.path.normpath(args.carpeta)), "nota": resumen.get("nota", ""),
           "fotogramas": len(t), "duracion_s": float(t[-1] - t[0]),
           "fps": float((len(t) - 1) / (t[-1] - t[0])), "perdidos_en_camara": resumen.get("perdidos_en_camara"),
           "andando_s": float(and_l.sum() * np.median(np.diff(tl))),
           "linea_vista": float(vista.mean()), "linea_vista_andando": float(vista[anda].mean()) if anda.any() else math.nan,
           "giro_total_deg": float(math.degrees(imu.yaw[-1] - imu.yaw[0])),
           "pitch_de_pie_deg": float(pitch[~and_l].mean()) if (~and_l).any() else math.nan,
           "pitch_andando_deg": float(pitch[and_l].mean()) if and_l.any() else math.nan,
           "roll_de_pie_deg": float(roll[~and_l].mean()) if (~and_l).any() else math.nan,
           "roll_andando_deg": float(roll[and_l].mean()) if and_l.any() else math.nan,
           "geometria_inclinacion_deg": geo["inclinacion_deg"]}
    if and_l.sum() > 300:
        out["cadencia_hz"] = analisis.cadencia(tl[and_l], roll[and_l])

    def rapida(x, m):
        x = analisis.rellenar(x)
        return float(np.std((x - analisis.media_centrada(t, x, 1.0))[m])) if m.sum() > 30 else math.nan

    m = anda & vista
    # al arrancar a andar, los primeros pasos desplazan el torso de lado y cambian el roll: fuera
    arranque = np.zeros(len(t), dtype=bool)
    for i0 in np.flatnonzero(anda[1:] & ~anda[:-1]) + 1:
        arranque |= (t >= t[i0]) & (t < t[i0] + ARRANQUE_S)
    omega = np.gradient(analisis.rellenar(yaw), t)
    out["oscilacion_angulo_deg"] = math.degrees(rapida(theta, m))
    out["oscilacion_angulo_con_yaw_deg"] = math.degrees(rapida(theta + yaw, m))
    out["oscilacion_lateral_cm"] = 100 * rapida(a, m)

    av = analisis.direccion_de_avance(t, a, theta, m & ~arranque, omega=omega, puntos=True)
    if av:
        # con pocos puntos o la linea casi al mismo angulo, v y phi no se separan (2026-10-05: el
        # recorrido "girado a la izquierda" pierde la linea en 4 s y da v = 0.04 m/s)
        rango = math.degrees(float(np.ptp(av["theta"])))
        fiable = av["n"] >= MIN_PUNTOS_AVANCE and rango >= MIN_RANGO_AVANCE_DEG
        out.update(v_real=av["v"], yaw_camara_avance_deg=-math.degrees(av["phi"]), d_giro_m=av["d"],
                   n_avance=av["n"], residuo_avance=av["residuo"], rango_angulo_avance_deg=rango,
                   avance_fiable=fiable)

    print(f"{out['dataset']} ({out['nota']}): {out['fotogramas']} fotogramas, {out['duracion_s']:.1f} s, "
          f"{out['fps']:.1f} fps, {out['perdidos_en_camara']} perdidos; andando {out['andando_s']:.1f} s")
    print(f"  linea vista {100 * out['linea_vista']:.0f} % (andando {100 * out['linea_vista_andando']:.0f} %); "
          f"giro total {out['giro_total_deg']:+.1f} grados")
    print(f"  pitch de pie {out['pitch_de_pie_deg']:+.2f}, andando {out['pitch_andando_deg']:+.2f}; roll de pie "
          f"{out['roll_de_pie_deg']:+.2f}, andando {out['roll_andando_deg']:+.2f} grados; cadencia "
          f"{out.get('cadencia_hz', math.nan):.2f} Hz")
    print(f"  oscilacion rapida andando: angulo {out['oscilacion_angulo_deg']:.2f} grados -> con el yaw de la IMU "
          f"{out['oscilacion_angulo_con_yaw_deg']:.2f}; lateral {out['oscilacion_lateral_cm']:.2f} cm")
    if av and not out["avance_fiable"]:
        print(f"  avance: NO FIABLE ({av['n']} puntos, el angulo de la linea solo recorre {out['rango_angulo_avance_deg']:.1f} "
              f"grados): v {av['v']:.3f} m/s, yaw {out['yaw_camara_avance_deg']:+.1f} grados")
    elif av:
        print(f"  avance: v real {av['v']:.3f} m/s; yaw de la camara respecto del avance "
              f"{out['yaw_camara_avance_deg']:+.2f} grados; centro de giro {100 * av['d']:.0f} cm por detras "
              f"({av['n']} puntos, residuo {100 * av['residuo']:.1f} cm/s)")
    else:
        print("  avance: pocos fotogramas andando con la linea bien vista")
    registro.guardar_json(os.path.join(args.carpeta, "analisis.json"), out)

    if args.figura:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ejes = plt.subplots(3, 1, figsize=(10, 9))
        t0 = t[0]
        for ax in ejes[:2]:
            ax.fill_between(t - t0, 0, 1, where=anda, transform=ax.get_xaxis_transform(), color="0.9", label="andando")
        ejes[0].plot(t - t0, 100 * a, ".", ms=2, label="desplazamiento a (cm)")
        ejes[0].plot(t - t0, np.degrees(theta), ".", ms=2, label="angulo de la linea (grados)")
        ejes[0].legend(loc="upper right", fontsize=8)
        ejes[0].set_xlabel("s")
        ejes[1].plot(tl - t0, np.degrees(np.unwrap(np.radians([float(r["yaw_deg"]) for r in filas]))) -
                     np.degrees(imu.yaw[0]), lw=1, label="yaw (grados)")
        ejes[1].plot(tl - t0, pitch, lw=0.6, label="pitch (grados)")
        ejes[1].legend(loc="upper right", fontsize=8)
        ejes[1].set_xlabel("s")
        if av:
            ejes[2].plot(np.degrees(av["theta"]), 100 * (av["dadt"] + av["d"] * av["omega"]), ".", ms=3, alpha=0.5)
            xs = np.linspace(av["theta"].min(), av["theta"].max(), 10)
            ejes[2].plot(np.degrees(xs), 100 * av["v"] * (xs - av["phi"]), "r-",
                         label=f"v = {av['v']:.3f} m/s, yaw camara {out['yaw_camara_avance_deg']:+.2f} grados")
            ejes[2].axhline(0, color="0.6", lw=0.8)
            ejes[2].axvline(math.degrees(av["phi"]), color="0.6", lw=0.8, ls="--")
            ejes[2].set_xlabel("angulo de la linea (grados)")
            ejes[2].set_ylabel("da/dt + d·omega (cm/s)")
            ejes[2].legend(fontsize=8)
        fig.suptitle(f"{out['dataset']} ({out['nota']})")
        fig.tight_layout()
        fig.savefig(os.path.join(args.carpeta, "analisis.png"), dpi=100)


if __name__ == "__main__":
    main()
