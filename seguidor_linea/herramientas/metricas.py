#!/usr/bin/env python3
"""Metricas de una tirada (seccion 9 del PDF), en la PC, sobre datos/tirada_<fecha>_<...>/.

Las del PDF:
  - error lateral medio y maximo medido por la percepcion, en el tramo con linea visible (medidas usadas
    entre SEGUIMIENTO y FIN): en la vertical de la camara (~ la puntera) y en el centro de giro (~ los
    pies, 14 cm detras, prolongando la linea medida), en perpendicular a la linea
  - porcentaje del tiempo con la linea vista (confianza >= conf_usar y medida de menos de t_aviso_s)
  - tiempo del recorrido (de SEGUIMIENTO a FIN o PARADA)
  - distancia de parada respecto de la barra: la mide el equipo con cinta (--parada-cm); aqui, la prevista
  - intervenciones con el mando: tramos con un eje del joystick fuera de cero durante el recorrido
  - roll y pitch maximos de la IMU durante el recorrido
Ademas: angulo y desplazamiento al salir, velocidad real (acercamiento a la barra) frente a la mandada,
suavidad y saturacion de vyaw, edad de los fotogramas y ms de la percepcion.

    python3 herramientas/metricas.py datos/tirada_<...> [mas carpetas] [--parada-cm 12 8] [--figura]

Escribe metricas.json (y metricas.png con --figura) en cada carpeta.
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import analisis, registro  # noqa: E402

SIGUIENDO = ("SEGUIMIENTO", "LINEA_PERDIDA", "GIRO_ESQUINA")


def leer(ruta):
    with open(ruta) as f:
        return list(csv.DictReader(f))


def num(filas, k):
    return np.array([float(r[k]) if r.get(k, "") not in ("", None) else math.nan for r in filas])


def tramos(mascara):
    """Numero de tramos seguidos de True."""
    m = np.asarray(mascara, dtype=bool)
    return int(np.sum(m[1:] & ~m[:-1]) + (1 if len(m) and m[0] else 0))


def metricas(carpeta, parada_cm=None):
    with open(os.path.join(carpeta, "resumen.json")) as f:
        resumen = json.load(f)
    cfg = resumen["config"]
    sup, est, ctl = cfg["supervisor"], cfg["estimacion"], cfg["control"]
    d_giro = est["centro_giro_m"]
    ordenes = leer(os.path.join(carpeta, "ordenes.csv"))
    medidas = leer(os.path.join(carpeta, "medidas.csv"))
    low = leer(os.path.join(carpeta, "lowstate.csv"))

    to = num(ordenes, "t")
    estado = np.array([r["estado"] for r in ordenes])
    sig = np.isin(estado, SIGUIENDO)
    if not sig.any():
        raise SystemExit(f"{carpeta}: no llego a SEGUIMIENTO")
    t_ini = to[np.argmax(sig)]
    fin = np.isin(estado, ("FIN", "PARADA"))
    t_fin = to[np.argmax(fin)] if fin.any() else to[-1]
    estado_final, motivo = resumen["estado_final"], resumen["motivo"]

    # --- error lateral (percepcion) ------------------------------------------------------------------------
    tm = num(medidas, "t")
    usada = np.array([r["usada"] in ("1", "True") for r in medidas])
    y, th = num(medidas, "y"), np.radians(num(medidas, "theta_deg"))
    m = usada & (tm >= t_ini) & (tm <= t_fin) & np.isfinite(y)
    y_cam = y[m] * np.cos(th[m])                              # perpendicular a la linea
    y_pies = (y[m] - d_giro * np.tan(th[m])) * np.cos(th[m])  # la linea en x = -d_giro
    tras3 = tm[m] >= t_ini + 3.0                             # sin la correccion de la salida

    # --- linea vista, ordenes -------------------------------------------------------------------------------
    conf, edad = num(ordenes, "confianza"), num(ordenes, "edad_s")
    vista = (conf >= est["conf_usar"]) & (edad < sup["t_aviso_s"])
    tramo = (to >= t_ini) & (to < t_fin)
    vx, vyaw = num(ordenes, "vx"), num(ordenes, "vyaw")
    escala = resumen["escala"]
    vyaw_tope = cfg["limites"]["vyaw_max"] * escala
    dvy = np.diff(vyaw[tramo])

    # --- IMU y mando ------------------------------------------------------------------------------------------
    tl = num(low, "t")
    ml = (tl >= t_ini) & (tl <= t_fin)
    roll, pitch, yaw = num(low, "roll_deg"), num(low, "pitch_deg"), num(low, "yaw_deg")
    ejes = np.max(np.abs(np.column_stack([num(low, k) for k in ("lx", "ly", "rx", "ry")])), axis=1)
    toca = ejes[ml] > 0.05
    gz = num(low, "gz")
    anda = analisis.andando(tl, gz)
    despues = tl > t_fin
    t_quieto = math.nan
    if despues.any() and anda[despues].any():
        idx = np.flatnonzero(despues & anda)
        t_quieto = float(tl[idx[-1]] - t_fin)
    yaw_u = np.degrees(np.unwrap(np.radians(yaw[ml])))

    # --- velocidad real: acercamiento a la barra mientras se ve -----------------------------------------------
    barra = num(medidas, "barra")
    mb = np.isfinite(barra) & usada & (tm >= t_ini) & (tm <= t_fin)
    vel = {}
    if mb.sum() >= 10:
        tb, db = tm[mb], barra[mb]
        v_media = -np.polyfit(tb, db, 1)[0]
        lejos = db > ctl["cerca_barra_m"] + 0.1
        cerca = (db < ctl["cerca_barra_m"] - 0.2)
        vel = {"barra_vista_desde_m": float(db[0]), "barra_vista_hasta_m": float(db[-1]),
               "v_media_m_s": float(v_media)}
        for nombre, sel in (("lejos", lejos), ("cerca", cerca)):
            if sel.sum() >= 8 and np.ptp(tb[sel]) > 0.5:
                v = -np.polyfit(tb[sel], db[sel], 1)[0]
                # la vx mandada medio segundo antes (la marcha tarda en responder)
                vx_m = float(np.mean(np.interp(tb[sel] - 0.5, to, vx)))
                vel[f"v_{nombre}_m_s"] = float(v)
                vel[f"vx_mandada_{nombre}_m_s"] = vx_m
                vel[f"factor_{nombre}"] = float(v / vx_m) if vx_m > 0.01 else math.nan

    # --- fin ---------------------------------------------------------------------------------------------------
    k_fin = int(np.argmax(fin)) if fin.any() else len(to) - 1
    dist_fin, v_est = num(ordenes, "dist_fin")[max(0, k_fin - 1)], num(ordenes, "v_est")[max(0, k_fin - 1)]
    prevista = v_est * sup["t_frenado_s"] - dist_fin if np.isfinite(dist_fin) else math.nan

    # salida: la linea al empezar (media del primer medio segundo)
    m0 = usada & (tm >= t_ini) & (tm < t_ini + 0.5)
    out = {
        "tirada": os.path.basename(os.path.normpath(carpeta)), "nivel": resumen["nivel"], "escala": escala,
        "simulacro": resumen["simulacro"], "estado_final": estado_final, "motivo": motivo,
        "tiempo_recorrido_s": round(float(t_fin - t_ini), 2),
        "error_lateral_cm": {
            "puntera_medio": float(100 * np.mean(np.abs(y_cam))), "puntera_max": float(100 * np.max(np.abs(y_cam))),
            "pies_medio": float(100 * np.mean(np.abs(y_pies))), "pies_max": float(100 * np.max(np.abs(y_pies))),
            "pies_medio_tras_3s": float(100 * np.mean(np.abs(y_pies[tras3]))) if tras3.any() else math.nan,
            "pies_max_tras_3s": float(100 * np.max(np.abs(y_pies[tras3]))) if tras3.any() else math.nan,
            "medidas": int(m.sum())},
        "linea_vista_pct": float(100 * np.mean(vista[tramo])) if tramo.any() else math.nan,
        "intervenciones_mando": tramos(toca), "roll_max_deg": float(np.max(np.abs(roll[ml]))),
        "pitch_max_deg": float(np.max(np.abs(pitch[ml]))),
        "salida": {"y_cm": float(100 * np.nanmean(y[m0])), "angulo_deg": float(np.degrees(np.nanmean(th[m0])))},
        "giro_total_deg": float(yaw_u[-1] - yaw_u[0]) if len(yaw_u) else math.nan,
        "ordenes": {"vx_max": float(np.max(vx[tramo])), "vyaw_max_abs": float(np.max(np.abs(vyaw[tramo]))),
                    "vyaw_saturada_pct": float(100 * np.mean(np.abs(vyaw[tramo]) >= vyaw_tope - 1e-3)),
                    "suavidad_vyaw_rad_s": float(np.sqrt(np.mean(dvy ** 2))) if len(dvy) else math.nan,
                    "moves": resumen.get("moves_enviados")},
        "velocidad": vel,
        "fin": {"dist_fin_al_parar_m": float(dist_fin), "v_m_s": float(v_est),
                "puntera_prevista_pasada_cm": float(100 * prevista),
                "parada_medida_cm": parada_cm, "s_hasta_quedarse_quieto": t_quieto},
        "edad_fotograma_ms": {"mediana": float(1000 * np.nanmedian(num(ordenes, "edad_fotograma")[tramo])),
                              "max": float(1000 * np.nanmax(num(ordenes, "edad_fotograma")[tramo]))},
        "percepcion_ms": resumen["percepcion_ms"],
    }
    serie = {"t": to - t_ini, "vx": vx, "vyaw": vyaw, "conf": conf, "dist_fin": num(ordenes, "dist_fin"),
             "tm": tm[m] - t_ini, "y_cam": y_cam, "y_pies": y_pies, "th": np.degrees(th[m]), "t_fin": t_fin - t_ini}
    return out, serie


def main():
    ap = argparse.ArgumentParser(description="Metricas de tiradas (seccion 9 del PDF)")
    ap.add_argument("carpetas", nargs="+")
    ap.add_argument("--parada-cm", type=float, nargs="*", default=[],
                    help="distancia de parada medida con cinta, una por carpeta (+ pasada la barra)")
    ap.add_argument("--figura", action="store_true")
    args = ap.parse_args()
    for k, carpeta in enumerate(args.carpetas):
        parada = args.parada_cm[k] if k < len(args.parada_cm) else None
        out, s = metricas(carpeta, parada)
        e, v, f = out["error_lateral_cm"], out["velocidad"], out["fin"]
        print(f"{out['tirada']}: {out['estado_final']} ({out['motivo']}) en {out['tiempo_recorrido_s']} s")
        print(f"  salida: linea a {out['salida']['y_cm']:+.1f} cm y {out['salida']['angulo_deg']:+.1f} grados; giro total "
              f"{out['giro_total_deg']:+.1f} grados")
        print(f"  error lateral: puntera {e['puntera_medio']:.1f} cm de media, {e['puntera_max']:.1f} max; pies "
              f"{e['pies_medio']:.1f} / {e['pies_max']:.1f} cm (tras 3 s: {e['pies_medio_tras_3s']:.1f} / {e['pies_max_tras_3s']:.1f})")
        print(f"  linea vista {out['linea_vista_pct']:.0f} % del tiempo; intervenciones {out['intervenciones_mando']}; roll max "
              f"{out['roll_max_deg']:.1f}, pitch max {out['pitch_max_deg']:.1f} grados")
        o = out["ordenes"]
        print(f"  ordenes: vx max {o['vx_max']:.2f}, |vyaw| max {o['vyaw_max_abs']:.2f} (saturada {o['vyaw_saturada_pct']:.0f} %), "
              f"suavidad {o['suavidad_vyaw_rad_s']:.3f} rad/s; {o['moves']} Move")
        if v:
            txt = f"  velocidad real (barra de {v['barra_vista_desde_m']:.2f} a {v['barra_vista_hasta_m']:.2f} m): media {v['v_media_m_s']:.3f} m/s"
            for n in ("lejos", "cerca"):
                if f"v_{n}_m_s" in v:
                    txt += f"; {n} {v[f'v_{n}_m_s']:.3f} con vx {v[f'vx_mandada_{n}_m_s']:.3f} (x{v[f'factor_{n}']:.2f})"
            print(txt)
        print(f"  fin: puntera a {100 * f['dist_fin_al_parar_m']:+.1f} cm de la barra a {f['v_m_s']:.2f} m/s -> prevista "
              f"{f['puntera_prevista_pasada_cm']:+.1f} cm pasada" + (f"; medida {f['parada_medida_cm']:+.1f} cm" if f["parada_medida_cm"] is not None else "")
              + f"; quieto a los {f['s_hasta_quedarse_quieto']:.1f} s")
        print(f"  fotograma: edad {out['edad_fotograma_ms']['mediana']:.0f} ms (max {out['edad_fotograma_ms']['max']:.0f}); "
              f"percepcion {out['percepcion_ms']['mediana']:.1f} ms (p95 {out['percepcion_ms']['p95']:.1f})")
        registro.guardar_json(os.path.join(carpeta, "metricas.json"), out)
        if args.figura:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ejes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
            ejes[0].plot(s["tm"], 100 * s["y_cam"], ".", ms=2, label="linea respecto de la puntera (cm)")
            ejes[0].plot(s["tm"], 100 * s["y_pies"], ".", ms=2, label="linea respecto de los pies (cm)")
            ejes[0].plot(s["tm"], s["th"], ".", ms=2, label="angulo de la linea (grados)")
            ejes[0].axhline(0, color="0.6", lw=0.8)
            ejes[0].legend(fontsize=8)
            ejes[1].plot(s["t"], s["vx"], label="vx (m/s)")
            ejes[1].plot(s["t"], s["vyaw"], label="vyaw (rad/s)")
            ejes[1].legend(fontsize=8)
            ejes[2].plot(s["t"], s["conf"], label="confianza")
            ejes[2].plot(s["t"], s["dist_fin"], label="puntera a la barra (m)")
            ejes[2].axhline(0, color="0.6", lw=0.8)
            ejes[2].legend(fontsize=8)
            for ax in ejes:
                ax.axvline(s["t_fin"], color="r", lw=0.8, ls="--")
            ejes[2].set_xlabel("s desde SEGUIMIENTO")
            ejes[2].set_xlim(-0.5, s["t_fin"] + 2)
            fig.suptitle(out["tirada"])
            fig.tight_layout()
            fig.savefig(os.path.join(carpeta, "metricas.png"), dpi=90)


if __name__ == "__main__":
    main()
