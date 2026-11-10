"""Cuanto se mueve la camara con el robot quieto, solo por el equilibrio (PDF 6.2.5), con los ArUco fijos de la mesa.

Lee la camara (camara_servidor.py) durante --segundos y, en cada fotograma de color, detecta los ArUco de la mesa
(2-5, los que se vean) con esquinas a subpixel y su pose (solvePnP, lado de calibracion.marcador_lado_m). Para
separar el movimiento de la camara del ruido de la deteccion: lo que mueve a la vez a todos los marcadores es la
camara (modo comun); la mitad de la diferencia entre dos marcadores es ruido (la mesa no se deforma). Del modo
comun da la desviacion tipica y el pico a pico en pixeles, en grados (el giro de la camara que lo explica, px / f)
y en mm sobre la mesa (a la distancia de los marcadores y en el centro de la zona de recogida), y la frecuencia
dominante; del ruido, lo mismo. Con arnes.py corriendo, tambien la IMU del torso (la que mide arnes.py en su ciclo
de 500 Hz, sin tomar los brazos) y su relacion con el giro de la camara (la cabeza va fija al torso).

    terminal 1: ./ejecutar.sh herramientas/arnes.py                       (de pie, [n]; nadie tocando el robot)
    terminal 2: ./ejecutar.sh herramientas/vibracion_camara.py [--segundos 60]
    en la PC:   python3 herramientas/vibracion_camara.py --analizar datos/vibracion_<fecha>
                python3 herramientas/vibracion_camara.py --dataset datos/dataset_<...>   (una grabacion)
Se puede correr a la vez que curva_pasos.py (--segundos 700) para ver la camara con los brazos moviendose.
Registro en datos/vibracion_<fecha>/: marcas.csv, imu.csv, primera.png y resumen.json.
"""

import argparse
import csv
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from manip import calibracion as cb  # noqa: E402
from manip import config  # noqa: E402
from manip.geometria import Intrinsecos  # noqa: E402
from manip.registro import CsvSimple, carpeta_tirada, guardar_json  # noqa: E402

IDS_MESA = (2, 3, 4, 5)
COL_MARCAS = ["t", "t_cam", "n", "id", "u", "v", "x", "y", "z", "lado_px"]
COL_IMU = ["t", "roll", "pitch", "yaw", "gx", "gy", "gz", "carga", "int_pitch"]
PRESENCIA_MIN = 0.8          # un marcador entra en el modo comun si se ve en al menos esta fraccion de fotogramas
F_MIN_HZ = 0.1               # por debajo, deriva: no cuenta para la frecuencia dominante


# --- medida -------------------------------------------------------------------------------------------
def medir_fotograma(f, det, intr, lado_m, ids=IDS_MESA):
    """[filas de COL_MARCAS] de los marcadores de la mesa en el color de `f`."""
    if f.color is None:
        return []
    filas = []
    for i, esq in cb.detectar(f.color, det, ids_validos=set(ids), escalas=(1,)).items():
        T = cb.pose_marcador(esq, intr, lado_m)
        c = esq.mean(axis=0)
        lado = float(np.mean([np.linalg.norm(esq[k] - esq[(k + 1) % 4]) for k in range(4)]))
        x, y, z = (T[:3, 3] if T is not None else (np.nan,) * 3)
        filas.append({"t": f.t_rx, "t_cam": f.t_cam, "n": f.n, "id": i, "u": c[0], "v": c[1], "x": x, "y": y, "z": z,
                      "lado_px": lado})
    return filas


# --- analisis -----------------------------------------------------------------------------------------
def _leer_csv(ruta):
    if not os.path.exists(ruta):
        return []
    with open(ruta) as fh:
        return [{k: float(v) if v not in ("", None) else np.nan for k, v in r.items()} for r in csv.DictReader(fh)]


def _estad(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) < 2:
        return {"std": np.nan, "pp": np.nan}
    return {"std": float(np.std(x)), "pp": float(np.ptp(x))}


def frecuencia_dominante(t, x, f_min=F_MIN_HZ):
    """(Hz, amplitud) del pico del espectro de x(t) (remuestreado uniforme), sin lo de por debajo de f_min."""
    t, x = np.asarray(t, float), np.asarray(x, float)
    ok = np.isfinite(x)
    t, x = t[ok], x[ok]
    if len(t) < 16 or np.ptp(t) <= 0:
        return np.nan, np.nan
    fs = (len(t) - 1) / np.ptp(t)
    tu = np.linspace(t[0], t[-1], len(t))
    xu = np.interp(tu, t, x)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    esp = np.abs(np.fft.rfft(xu * np.hanning(len(xu)))) * 4.0 / len(xu)       # ~amplitud de una senoide
    fr = np.fft.rfftfreq(len(xu), 1.0 / fs)
    sel = fr >= f_min
    if not sel.any():
        return np.nan, np.nan
    k = np.argmax(esp[sel])
    return float(fr[sel][k]), float(esp[sel][k])


def medias(t, x, s=1.0):
    """Las medias de x en ventanas de s segundos (quitan el ruido de la deteccion: queda lo lento)."""
    t, x = np.asarray(t, float), np.asarray(x, float)
    if len(t) < 2:
        return np.array([])
    k = np.floor((t - t[0]) / s).astype(int)
    return np.array([x[k == i].mean() for i in range(k.max() + 1) if np.any(k == i)])


def tendencia_por_min(t, x):
    t, x = np.asarray(t, float), np.asarray(x, float)
    if len(t) < 3 or np.ptp(t) <= 0:
        return np.nan
    return float(np.polyfit(t - t[0], x, 1)[0] * 60.0)


def analizar(marcas, imu, intr, distancia_zona_m=None, presencia_min=PRESENCIA_MIN):
    """Del registro (filas de marcas.csv e imu.csv): el modo comun y el ruido de los marcadores, en px, grados y
    mm, su frecuencia dominante, y la IMU en la misma ventana. Devuelve un dict (resumen)."""
    if not marcas:
        return {"error": "ningun marcador de la mesa visto"}
    por_n = {}
    for r in marcas:
        por_n.setdefault(int(r["n"]), {})[int(r["id"])] = r
    ns = sorted(por_n)
    vistos = {}
    for n in ns:
        for i in por_n[n]:
            vistos[i] = vistos.get(i, 0) + 1
    usados = sorted(i for i, k in vistos.items() if k >= presencia_min * len(ns))
    if not usados:
        return {"error": f"ningun marcador en el {presencia_min:.0%} de los fotogramas", "vistos": vistos}
    comunes = [n for n in ns if all(i in por_n[n] for i in usados)]
    t = np.array([np.mean([por_n[n][i]["t"] for i in usados]) for n in comunes])
    U = np.array([[por_n[n][i]["u"] for i in usados] for n in comunes])
    V = np.array([[por_n[n][i]["v"] for i in usados] for n in comunes])
    Z = np.array([[por_n[n][i]["z"] for i in usados] for n in comunes])
    du, dv = U - U.mean(axis=0), V - V.mean(axis=0)
    cu, cv = du.mean(axis=1), dv.mean(axis=1)
    z_med = float(np.nanmedian(Z))
    r = {"fotogramas": len(ns), "fotogramas_comunes": len(comunes), "duracion_s": float(np.ptp(t)) if len(t) > 1 else 0.0,
         "fps": float((len(t) - 1) / np.ptp(t)) if len(t) > 1 and np.ptp(t) > 0 else np.nan,
         "marcadores": usados, "vistos": vistos, "distancia_marcadores_m": z_med,
         "f_px": [intr.fx, intr.fy]}
    m = {}
    for eje, c, f in (("horizontal", cu, intr.fx), ("vertical", cv, intr.fy)):
        e = _estad(c)
        fr, amp = frecuencia_dominante(t, c)
        e1 = _estad(medias(t, c))
        m[eje] = {"px": e, "grados": {k: math.degrees(v / f) for k, v in e.items()},
                  "mm_en_marcadores": {k: 1000 * v * z_med / f for k, v in e.items()},
                  "frecuencia_hz": fr, "amplitud_px": amp,
                  "medias_1s_px": e1, "medias_1s_grados": {k: math.degrees(v / f) for k, v in e1.items()},
                  "tendencia_grados_min": math.degrees(tendencia_por_min(t, c) / f)}
        if distancia_zona_m:
            m[eje]["mm_en_zona"] = {k: 1000 * v * distancia_zona_m / f for k, v in e.items()}
    r["camara"] = m
    if len(usados) >= 2:
        # ruido: con la deteccion de cada marcador independiente (sigma), la mitad de la diferencia entre dos tiene
        # sigma / raiz(2) y el modo comun de N, sigma / raiz(N). Lo neto de la camara: el modo comun sin ese ruido
        ruido = {}
        for eje, d in (("horizontal", du), ("vertical", dv)):
            sigma = math.sqrt(2.0) * float(np.std((d[:, 0] - d[:, 1]) / 2.0))
            en_comun = sigma / math.sqrt(len(usados))
            neto = math.sqrt(max(0.0, m[eje]["px"]["std"] ** 2 - en_comun ** 2))
            ruido[eje] = {"por_marcador": sigma, "en_modo_comun": en_comun}
            m[eje]["neto_px"] = neto
            m[eje]["neto_grados"] = math.degrees(neto / (intr.fx if eje == "horizontal" else intr.fy))
        r["ruido_px"] = ruido
    zc = Z - np.nanmean(Z, axis=0)
    r["profundidad_mm"] = _estad(1000 * np.nanmean(zc, axis=1))
    if imu:
        ti = np.array([x["t"] for x in imu])
        sel = (ti >= t[0] - 0.5) & (ti <= t[-1] + 0.5) if len(t) else np.ones(len(ti), bool)
        if sel.sum() >= 2:
            g = {k: np.array([x[k] for x in imu])[sel] for k in ("roll", "pitch", "yaw", "gx", "gy", "gz", "carga")}
            r["imu"] = {k: {kk: math.degrees(vv) for kk, vv in _estad(g[k]).items()} for k in ("roll", "pitch", "yaw")}
            r["imu"]["giroscopo_grados_s"] = {k: math.degrees(_estad(g[k])["std"]) for k in ("gx", "gy", "gz")}
            r["imu"]["carga"] = _estad(g["carga"])
            r["imu"]["hz"] = float((sel.sum() - 1) / np.ptp(ti[sel])) if np.ptp(ti[sel]) > 0 else np.nan
            yaw = np.unwrap(g["yaw"])
            r["imu"]["tendencia_grados_min"] = {k: math.degrees(tendencia_por_min(ti[sel], v))
                                                for k, v in (("roll", g["roll"]), ("pitch", g["pitch"]), ("yaw", yaw))}
            r["imu"]["medias_1s_grados"] = {k: {kk: math.degrees(vv) for kk, vv in _estad(medias(ti[sel], v)).items()}
                                            for k, v in (("roll", g["roll"]), ("pitch", g["pitch"]))}
            # la camara gira con el torso: el giro vertical de la imagen (pitch) frente al pitch de la IMU
            if len(t) >= 8:
                p_imu = np.interp(t, ti[sel], g["pitch"])
                p_cam = np.arctan2(cv, intr.fy)
                if np.std(p_imu) > 0 and np.std(p_cam) > 0:
                    cc = float(np.corrcoef(p_cam, p_imu - p_imu.mean())[0, 1])
                    pend = float(np.polyfit(p_imu - p_imu.mean(), p_cam, 1)[0])
                    r["imu"]["camara_frente_a_imu"] = {"r_pitch": cc, "pendiente_pitch": pend}
    return r


def distancia_zona(cfg):
    """Profundidad (eje optico) del centro de la zona de recogida en la camara de color (calibracion y montaje), o
    None: la que pasa px a mm (mm = px * z / f), como la z de solvePnP de los marcadores."""
    try:
        from manip.cinematica import Cinematica
        from manip.geometria import Plano, aplicar, inv
        cal = cb.cargar_o_urdf(Cinematica.desde_config(cfg))
        mon = config.cargar_montaje()
        pl = Plano(mon["mesa"]["plano"]["n"], mon["mesa"]["plano"]["d"])
        z = mon["zona_recogida"]
        x, y = float(np.mean(z["x"])), float(np.mean(z["y"]))
        zt = -(pl.n[0] * x + pl.n[1] * y + pl.d) / pl.n[2]
        p = aplicar(inv(cal.T_torso_color), np.array([[x, y, zt]]))[0]
        return float(p[2])
    except Exception:
        return None


def informe(r):
    if "error" in r:
        return [f"  {r['error']}"]
    lin = [f"  {r['fotogramas']} fotogramas ({r['fotogramas_comunes']} con todos), {r['duracion_s']:.1f} s a "
           f"{r['fps']:.1f} fps; marcadores {r['marcadores']} a {r['distancia_marcadores_m']:.2f} m"]
    for eje, m in r["camara"].items():
        z = f", {m['mm_en_zona']['std']:.2f} mm en la zona" if "mm_en_zona" in m else ""
        lin.append(f"  camara, {eje:10s}: {m['px']['std']:.2f} px de desviacion ({m['px']['pp']:.2f} pico a pico) = "
                   f"{m['grados']['std'] * 60:.1f}' ({m['grados']['pp'] * 60:.1f}') = "
                   f"{m['mm_en_marcadores']['std']:.2f} mm en los marcadores{z}; "
                   f"frecuencia dominante {m['frecuencia_hz']:.2f} Hz ({m['amplitud_px']:.2f} px)")
    if "ruido_px" in r:
        rn, c = r["ruido_px"], r["camara"]
        lin.append(f"  ruido de la deteccion: {rn['horizontal']['por_marcador']:.2f} / {rn['vertical']['por_marcador']:.2f} px "
                   f"por marcador, {rn['horizontal']['en_modo_comun']:.2f} / {rn['vertical']['en_modo_comun']:.2f} en el modo "
                   f"comun (horizontal / vertical)")
        lin.append(f"  movimiento neto de la camara (modo comun sin el ruido): {c['horizontal']['neto_px']:.2f} / "
                   f"{c['vertical']['neto_px']:.2f} px = {c['horizontal']['neto_grados'] * 60:.1f}' / "
                   f"{c['vertical']['neto_grados'] * 60:.1f}' de desviacion")
    c = r["camara"]
    lin.append(f"  camara, lo lento (medias de 1 s, casi sin ruido): {c['horizontal']['medias_1s_px']['std']:.3f} / "
               f"{c['vertical']['medias_1s_px']['std']:.3f} px de desviacion ({c['horizontal']['medias_1s_grados']['std'] * 60:.2f}' / "
               f"{c['vertical']['medias_1s_grados']['std'] * 60:.2f}'), {c['horizontal']['medias_1s_px']['pp']:.3f} / "
               f"{c['vertical']['medias_1s_px']['pp']:.3f} px pico a pico; tendencia {c['horizontal']['tendencia_grados_min']:+.3f} / "
               f"{c['vertical']['tendencia_grados_min']:+.3f} grados/min (horizontal / vertical)")
    lin.append(f"  profundidad (solvePnP) de los marcadores: {r['profundidad_mm']['std']:.1f} mm de desviacion")
    if "imu" in r:
        i = r["imu"]
        lin.append(f"  IMU del torso ({i['hz']:.0f} Hz): roll {i['roll']['std'] * 60:.1f}' / pitch {i['pitch']['std'] * 60:.1f}' / "
                   f"yaw {i['yaw']['std'] * 60:.1f}' de desviacion ({i['roll']['pp'] * 60:.1f}' / {i['pitch']['pp'] * 60:.1f}' / "
                   f"{i['yaw']['pp'] * 60:.1f}' pico a pico); carga {100 * i['carga']['std']:.1f} %")
        td = i["tendencia_grados_min"]
        lin.append(f"  IMU, medias de 1 s: roll {i['medias_1s_grados']['roll']['pp'] * 60:.1f}' / pitch "
                   f"{i['medias_1s_grados']['pitch']['pp'] * 60:.1f}' pico a pico; tendencia roll {td['roll']:+.3f} / pitch "
                   f"{td['pitch']:+.3f} / yaw {td['yaw']:+.3f} grados/min")
        if abs(td["yaw"]) > 5 * abs(c["horizontal"]["tendencia_grados_min"]) + 0.05:
            lin.append(f"  OJO: el yaw de la IMU deriva {td['yaw']:+.2f} grados/min y la camara no gira "
                       f"({c['horizontal']['tendencia_grados_min']:+.3f}): es la IMU, no el cuerpo. El vigilante para con 5 "
                       f"grados: a este ritmo, a los {5 / abs(td['yaw']):.0f} min de tirada")
        if "camara_frente_a_imu" in i:
            c = i["camara_frente_a_imu"]
            lin.append(f"  camara frente a la IMU (pitch): r = {c['r_pitch']:.2f}, pendiente {c['pendiente_pitch']:.2f}")
    return lin


# --- programa ----------------------------------------------------------------------------------------
def _imu_hilo(b, imu_csv, fin, cada=0.01):
    while not fin.is_set():
        e = b.medir()
        c = (e or {}).get("cuerpo")
        if c:
            imu_csv.escribir({"t": c["t"], "roll": c["rpy"][0], "pitch": c["rpy"][1], "yaw": c["rpy"][2],
                              "gx": c["gyro"][0], "gy": c["gyro"][1], "gz": c["gyro"][2], "carga": c["carga"],
                              "int_pitch": c["integral"][1]})
        time.sleep(cada)


class _SinRobot:
    def q(self):
        return np.zeros(27)


def grabar(cfg, carpeta, fuente, segundos, con_imu=True, log=print):
    det = cb.detector(cfg["calibracion"]["diccionario"])
    lado = float(cfg["calibracion"]["marcador_lado_m"])
    marcas = CsvSimple(os.path.join(carpeta, "marcas.csv"), COL_MARCAS)
    imu_csv = CsvSimple(os.path.join(carpeta, "imu.csv"), COL_IMU)
    b, fin, hilo = None, threading.Event(), None
    if con_imu:
        try:
            from manip.brazo_arnes import BrazoArnes
            b = BrazoArnes(_SinRobot(), cfg, nombre="vibracion_camara.py")       # solo pregunta: no toma los brazos
            hilo = threading.Thread(target=_imu_hilo, args=(b, imu_csv, fin), daemon=True)
            hilo.start()
        except RuntimeError as ex:
            log(f"  sin la IMU: {ex}")
    intr, n, sin, primera = None, 0, 0, None
    t_fin = time.monotonic() + segundos
    try:
        while time.monotonic() < t_fin:
            f = fuente.siguiente(1.0)
            if f is None:
                sin += 1
                if fuente.__class__.__name__ == "FuenteDataset":
                    break
                continue
            if intr is None:
                intr = Intrinsecos.de_meta(f.meta["color"])
            if primera is None and f.color is not None:
                primera = f.color
                cv2.imwrite(os.path.join(carpeta, "primera.png"), f.color)
            for fila in medir_fotograma(f, det, intr, lado):
                marcas.escribir(fila)
            n += 1
    finally:
        fin.set()
        if hilo is not None:
            hilo.join(timeout=1.0)
        if b is not None:
            b.parar()
        marcas.cerrar()
        imu_csv.cerrar()
    return intr, n, sin


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--segundos", type=float, default=60.0)
    ap.add_argument("--analizar", metavar="CARPETA", help="solo el analisis de un registro ya hecho")
    ap.add_argument("--dataset", metavar="CARPETA", help="leer los fotogramas de una grabacion (sin IMU)")
    ap.add_argument("--sin-imu", action="store_true", help="no preguntar a arnes.py")
    a = ap.parse_args()
    dz = distancia_zona(cfg)
    if a.analizar:
        carpeta = a.analizar
        with open(os.path.join(carpeta, "resumen.json")) as fh:
            intr = Intrinsecos.de_meta(json.load(fh)["intrinsecos_color"])
    else:
        if a.dataset:
            from manip.fuentes import FuenteDataset
            fuente = FuenteDataset(a.dataset)
            nombre = "dataset"
        else:
            from manip.fuentes import FuenteZmq
            fuente = FuenteZmq(cfg["red"]["zmq_fotogramas"], solo_ultimo=True)
            nombre = None
        carpeta = carpeta_tirada(config.ruta_datos(cfg), "vibracion", nombre)
        print(f"\n  Camara quieta: {a.segundos:g} s" + (f" de {a.dataset}" if a.dataset else "") +
              ". Nadie tocando el robot ni la mesa.")
        intr, n, sin = grabar(cfg, carpeta, fuente, a.segundos, con_imu=not (a.sin_imu or a.dataset))
        if intr is None:
            sys.exit("  No llegan fotogramas: ./camara_servidor.sh")
        print(f"  {n} fotogramas" + (f", {sin} esperas sin fotograma" if sin else ""))
    r = analizar(_leer_csv(os.path.join(carpeta, "marcas.csv")), _leer_csv(os.path.join(carpeta, "imu.csv")), intr, dz)
    r.update(intrinsecos_color=intr.a_dict(), distancia_zona_m=dz, carpeta=carpeta,
             fecha=time.strftime("%Y-%m-%d %H:%M:%S"))
    if a.dataset:
        r["dataset"] = a.dataset
    print("\n" + "\n".join(informe(r)))
    guardar_json(os.path.join(carpeta, "resumen.json"), r)
    print(f"\n  Registro en {carpeta}")


if __name__ == "__main__":
    main()
