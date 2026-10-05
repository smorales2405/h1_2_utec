#!/usr/bin/env python3
"""Hito 2: la percepcion sobre datasets grabados, en la PC. No usa el robot.

Pasa seguidor/percepcion.py por cada fotograma con la postura LENTA del torso (roll/pitch de la IMU
con una media centrada de 1.5 s; en vivo sera un filtro causal) y lo compara con un detector
independiente, el minimo de la calibracion (seguidor/calibracion.py), cuando este ve la linea.

    python3 herramientas/evaluar_percepcion.py datos/dataset_<...> [mas datasets] [--umbral fijo] [--video]

Por dataset escribe medidas_<umbral>.csv y evaluacion_<umbral>.json (y evaluacion_<umbral>.mp4 con
--video: la imagen con la linea, los puntos, el objetivo y la barra encima, y la vista desde arriba).
Al final, una tabla con todos.
"""

import argparse
import csv
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import analisis, calibracion, config, registro  # noqa: E402
from seguidor.fuentes import FuenteDataset  # noqa: E402
from seguidor.geometria import Intrinsecos, ModeloSuelo  # noqa: E402
from seguidor.percepcion import Percepcion  # noqa: E402

CONF_DETECCION = 0.5     # confianza a partir de la que se cuenta como linea detectada
POSTURA_S = 1.5          # media de la postura del torso


def dibujar(ir, m, per, roll, pitch):
    """Imagen con la medida encima | vista desde arriba con los puntos."""
    det = per.detalle
    img = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
    if np.isfinite(m.y):
        a, b, c = det.coef
        x_fin = det.x_fin if np.isfinite(det.x_fin) else 2.5
        xs = np.linspace(per.x_min, x_fin, 40)
        u, v = per.modelo.suelo_a_pixel(xs, a + b * xs + c * xs ** 2, roll, pitch)
        ok = np.isfinite(u)
        cv2.polylines(img, [np.stack([u[ok], v[ok]], 1).astype(np.int32)], False, (255, 0, 255), 2)
        u, v = per.modelo.suelo_a_pixel(det.xs, det.ys, roll, pitch)
        for uu, vv, d in zip(u, v, det.dentro):
            if np.isfinite(uu):
                cv2.circle(img, (int(uu), int(vv)), 2, (0, 220, 0) if d else (0, 0, 255), -1)
        if np.isfinite(m.objetivo[0]):
            uo, vo = per.modelo.suelo_a_pixel(m.objetivo[0], m.objetivo[1], roll, pitch)
            if np.isfinite(uo):
                cv2.circle(img, (int(uo), int(vo)), 7, (0, 255, 255), 2)
        if m.barra_fin is not None:
            yb = a + b * m.barra_fin + c * m.barra_fin ** 2
            ys = np.linspace(yb - 0.3, yb + 0.3, 10)
            u, v = per.modelo.suelo_a_pixel(np.full_like(ys, m.barra_fin), ys, roll, pitch)
            ok = np.isfinite(u)
            cv2.polylines(img, [np.stack([u[ok], v[ok]], 1).astype(np.int32)], False, (0, 165, 255), 3)
    texto = (f"conf {m.confianza:.2f}  y {100 * m.y:+.1f} cm  th {math.degrees(m.theta):+.1f}  k {m.kappa:+.2f}"
             if np.isfinite(m.y) else f"conf {m.confianza:.2f}  sin linea")
    cv2.putText(img, texto, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    extra = f"barra {m.barra_fin:.2f} m" if m.barra_fin is not None else ""
    if m.esquina is not None:
        extra += f"  esquina {m.esquina[0]:.2f} m {'izq' if m.esquina[1] > 0 else 'der'}"
    cv2.putText(img, f"{m.ms:.1f} ms  {extra}", (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    bev = cv2.cvtColor(det.bev, cv2.COLOR_GRAY2BGR)
    for x, y, d in zip(det.xs, det.ys, det.dentro):
        r, c_ = int(round((per.x_max - x) / per.res)), int(round((per.y_max - y) / per.res))
        cv2.circle(bev, (c_, r), 1, (0, 220, 0) if d else (0, 0, 255), -1)
    for xc, _ in det.filas_barra:
        r = int(round((per.x_max - xc) / per.res))
        cv2.line(bev, (0, r), (bev.shape[1] - 1, r), (0, 165, 255), 1)
    bev = cv2.resize(bev, (int(bev.shape[1] * img.shape[0] / bev.shape[0]), img.shape[0]))
    return np.hstack([img, bev])


def evaluar(carpeta, cfg, umbral, video, con_postura):
    with open(os.path.join(carpeta, "resumen.json")) as f:
        resumen = json.load(f)
    geo = resumen.get("geometria") or cfg["geometria"]
    fuente = FuenteDataset(carpeta)
    intr = Intrinsecos.desde_dict(fuente.meta["ir"])
    per = Percepcion(dict(cfg["percepcion"], umbral=umbral), intr, geo)
    ref = ModeloSuelo(intr, geo["altura_m"], math.radians(geo["inclinacion_deg"]), math.radians(geo.get("roll_deg", 0.0)))

    with open(os.path.join(carpeta, "lowstate.csv")) as f:
        filas = list(csv.DictReader(f))
    tl = np.array([float(r["t"]) for r in filas])
    roll_l = analisis.media_centrada(tl, np.radians([float(r["roll_deg"]) for r in filas]), POSTURA_S)
    pitch_l = analisis.media_centrada(tl, np.radians([float(r["pitch_deg"]) for r in filas]), POSTURA_S)
    anda_l = analisis.andando(tl, np.array([float(r["gz"]) for r in filas]))

    escritor = None
    if video:
        ruta_v = os.path.join(carpeta, f"evaluacion_{umbral}.mp4")
    csvm = registro.CsvSimple(os.path.join(carpeta, f"medidas_{umbral}.csv"),
                              ["t", "n", "y", "theta_deg", "kappa", "obj_x", "obj_y", "confianza", "n_franjas",
                               "barra", "esquina_x", "esquina_sentido", "ms", "polaridad", "andando", "y_ref", "theta_ref_deg"])
    R = []
    for f in fuente:
        roll = float(np.interp(f.t_rx, tl, roll_l)) if con_postura else None
        pitch = float(np.interp(f.t_rx, tl, pitch_l)) if con_postura else None
        m = per.procesar(f, roll, pitch)
        pts = calibracion.puntos_de_linea(f.ir, ref, x_min=0.3, x_max=2.5)
        y_ref = th_ref = math.nan
        if len(pts) >= 60:
            a0, b0, k = calibracion.ajustar_recta(pts)
            if k >= 30:
                y_ref, th_ref = a0, math.atan(b0)
        anda = bool(np.interp(f.t_rx, tl, anda_l.astype(float)) > 0.5)
        R.append((f.t_rx, m.confianza, m.y, m.theta, m.ms, m.barra_fin if m.barra_fin is not None else math.nan,
                  anda, y_ref, th_ref, per.detalle.polaridad))
        csvm.escribir({"t": f.t_rx, "n": f.n, "y": m.y, "theta_deg": math.degrees(m.theta), "kappa": m.kappa,
                       "obj_x": m.objetivo[0], "obj_y": m.objetivo[1], "confianza": m.confianza, "n_franjas": m.n_franjas,
                       "barra": m.barra_fin if m.barra_fin is not None else "",
                       "esquina_x": m.esquina[0] if m.esquina else "", "esquina_sentido": m.esquina[1] if m.esquina else "",
                       "ms": m.ms, "polaridad": per.detalle.polaridad, "andando": anda,
                       "y_ref": y_ref, "theta_ref_deg": math.degrees(th_ref)})
        if video:
            cuadro = dibujar(f.ir, m, per, roll, pitch)
            if escritor is None:
                escritor = cv2.VideoWriter(ruta_v, cv2.VideoWriter_fourcc(*"mp4v"), 30, (cuadro.shape[1], cuadro.shape[0]))
            escritor.write(cuadro)
    csvm.cerrar()
    if escritor is not None:
        escritor.release()

    t, conf, y, th, ms, barra, anda, y_ref, th_ref, pol = (np.array(c, dtype=float) for c in zip(*R))
    anda = anda > 0.5
    det = conf >= CONF_DETECCION
    hay_ref = np.isfinite(y_ref)
    ambos = det & hay_ref & np.isfinite(y)
    out = {"dataset": os.path.basename(os.path.normpath(carpeta)), "nota": resumen.get("nota", ""), "umbral": umbral,
           "postura": con_postura, "fotogramas": len(t), "ms_mediana": float(np.median(ms)),
           "ms_p95": float(np.percentile(ms, 95)), "deteccion": float(det.mean()),
           "deteccion_de_pie": float(det[~anda].mean()) if (~anda).any() else math.nan,
           "deteccion_andando": float(det[anda].mean()) if anda.any() else math.nan,
           "referencia_ve_linea": float(hay_ref.mean()),
           "deteccion_donde_la_referencia_ve": float(det[hay_ref].mean()) if hay_ref.any() else math.nan,
           "deteccion_donde_la_referencia_no_ve": float(det[~hay_ref].mean()) if (~hay_ref).any() else math.nan,
           "dif_y_cm_mediana": float(100 * np.median(np.abs(y[ambos] - y_ref[ambos]))) if ambos.any() else math.nan,
           "dif_theta_deg_mediana": float(np.degrees(np.median(np.abs(th[ambos] - th_ref[ambos])))) if ambos.any() else math.nan,
           "confianza_mediana_detectada": float(np.median(conf[det])) if det.any() else math.nan,
           # la que mas se uso con la linea detectada (fuera de la pista puede cambiar sin importar)
           "polaridad": int(np.sign(np.sum(pol[det]))) if det.any() else 0,
           "polaridad_estable": float(np.mean(pol[det] == np.sign(np.sum(pol[det])))) if det.any() else math.nan}
    vb = np.isfinite(barra) & det        # con confianza baja (fuera de la pista) no cuenta
    if vb.any():
        out.update(barra_fotogramas=int(vb.sum()), barra_primera_m=float(barra[vb][0]), barra_ultima_m=float(barra[vb][-1]))
        tramo = vb & anda
        if tramo.sum() > 30 and np.ptp(t[tramo]) > 2.0:
            out["barra_velocidad_acercamiento"] = float(-np.polyfit(t[tramo], barra[tramo], 1)[0])
    registro.guardar_json(os.path.join(carpeta, f"evaluacion_{umbral}.json"), out)
    return out


def main():
    ap = argparse.ArgumentParser(description="Evaluacion de la percepcion sobre datasets (Hito 2)")
    ap.add_argument("carpetas", nargs="+")
    ap.add_argument("--umbral", choices=["adaptativo", "fijo"], default="adaptativo")
    ap.add_argument("--video", action="store_true", help="evaluacion_<umbral>.mp4 con la medida encima")
    ap.add_argument("--sin-postura", action="store_true", help="geometria fija, sin la postura de la IMU")
    args = ap.parse_args()
    cfg = config.cargar()
    filas = []
    for carpeta in args.carpetas:
        o = evaluar(carpeta, cfg, args.umbral, args.video, not args.sin_postura)
        filas.append(o)
        print(f"{o['dataset']} ({o['nota']}): {o['fotogramas']} fotogramas, {o['ms_mediana']:.1f} ms (p95 {o['ms_p95']:.1f}); "
              f"detectada {100 * o['deteccion']:.0f} % (de pie {100 * o['deteccion_de_pie']:.0f}, andando "
              f"{100 * o['deteccion_andando']:.0f}); donde la referencia ve la linea {100 * o['deteccion_donde_la_referencia_ve']:.0f} %, "
              f"donde no {100 * o['deteccion_donde_la_referencia_no_ve']:.0f} %; frente a la referencia: y {o['dif_y_cm_mediana']:.1f} cm, "
              f"angulo {o['dif_theta_deg_mediana']:.2f} grados; polaridad {o['polaridad']:+d} ({100 * o['polaridad_estable']:.0f} %)"
              + (f"; barra de {o['barra_primera_m']:.2f} a {o['barra_ultima_m']:.2f} m"
                 + (f", se acerca a {o['barra_velocidad_acercamiento']:.3f} m/s" if "barra_velocidad_acercamiento" in o else "")
                 if "barra_primera_m" in o else ""))
    if len(filas) > 1:
        det = [o["deteccion_donde_la_referencia_ve"] for o in filas]
        print(f"\nTotal ({args.umbral}): detectada donde la referencia ve la linea {100 * np.nanmean(det):.0f} % de media; "
              f"{np.mean([o['ms_mediana'] for o in filas]):.1f} ms por fotograma")


if __name__ == "__main__":
    main()
