#!/usr/bin/env python3
"""Camara externa (Logitech BRIO en la laptop, de frente al robot y al tablero): la tirada vista desde fuera, para
revisar los agarres (RESULTADOS §60). Solo en la PC; no hace falta nada del robot salvo para medir el desfase de
reloj y traer la tirada.

    python3 herramientas/camara_externa.py ver                  # configura la camara y guarda una foto (encuadre)
    python3 herramientas/camara_externa.py grabar               # graba hasta Ctrl+C (o SIGTERM)
    python3 herramientas/camara_externa.py fotos --ultima       # trae la ultima tirada y saca las fotos de sus eventos
    python3 herramientas/camara_externa.py fotos datos/tirada_<fecha>_nivel2

Donde ponerla (altura, distancia, inclinacion y giro): herramientas/situar_camara_externa.py (§63).

manipulacion.sh, lanzado desde la PC, graba solo mientras dura la tirada y al acabar saca las fotos (CAMARA_EXTERNA=0
lo apaga). El video va a datos/externa/externa_<fecha>.mp4 con su .json: la hora (epoch de la laptop) del primer
fotograma, cada fotograma con su instante real (sin duplicar) y el desfase del reloj del robot. Las fotos van a
datos/<tirada>/externa/: una por evento (y mas al levantar), un mosaico y el clip de cada agarre.
"""

import argparse
import glob
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

import yaml

AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATOS = os.path.join(AQUI, "datos")
ROBOT_SSH = os.environ.get("ROBOT_SSH", "unitree@192.168.0.143")
DEST = "utec/manipulacion"


def cfg_camara():
    with open(os.path.join(AQUI, "config", "manipulacion.yaml")) as f:
        return yaml.safe_load(f)["camara_externa"]


def dispositivo(nombre):
    """/dev/videoN de captura (indice 0) de la camara cuyo nombre contiene `nombre`, o None."""
    for d in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            if nombre.lower() in open(f"{d}/name").read().lower() and open(f"{d}/index").read().strip() == "0":
                return "/dev/" + os.path.basename(d)
        except OSError:
            continue
    return None


def configurar(dev, controles):
    """Los controles en orden (el zoom antes que pan/tilt: sin zoom no se mueven; el foco automatico antes que el
    manual). Devuelve los que no se pudieron poner."""
    malos = []
    for k, v in controles.items():
        r = subprocess.run(["v4l2-ctl", "-d", dev, "-c", f"{k}={v}"], capture_output=True, text=True)
        if r.returncode != 0:
            malos.append(k)
    return malos


def desfase_robot(rondas=6):
    """reloj del robot - reloj de la laptop (s), por una sola sesion ssh: cada ronda pide la hora y se queda la de
    menor ida y vuelta (la conexion no cuenta). None si el robot no contesta."""
    try:
        p = subprocess.Popen(["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", ROBOT_SSH,
                              f"for i in $(seq {rondas}); do read x || exit; date +%s.%N; done"],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        mejor = None
        for _ in range(rondas):
            t1 = time.time()
            p.stdin.write("\n")
            p.stdin.flush()
            linea = p.stdout.readline()
            t2 = time.time()
            if not linea:
                break
            d = (float(linea) - (t1 + t2) / 2, t2 - t1)
            if mejor is None or d[1] < mejor[1]:
                mejor = d
        p.stdin.close()
        p.wait(timeout=5)
        return None if mejor is None else {"s": round(mejor[0], 4), "ida_y_vuelta_s": round(mejor[1], 4)}
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def entrada_ffmpeg(dev, c):
    return ["ffmpeg", "-hide_banner", "-nostdin", "-f", "v4l2", "-input_format", "mjpeg", "-video_size",
            f"{c['ancho']}x{c['alto']}", "-framerate", str(c["fps"]), "-use_wallclock_as_timestamps", "1", "-i", dev]


# --- ver y grabar ---------------------------------------------------------------------------------------------
def cmd_ver(a):
    c = cfg_camara()
    dev = dispositivo(c["nombre"])
    if dev is None:
        sys.exit(f"No hay camara '{c['nombre']}'")
    malos = configurar(dev, c["controles"])
    salida = a.salida or os.path.join(DATOS, "externa", "encuadre.jpg")
    os.makedirs(os.path.dirname(salida), exist_ok=True)
    # los primeros fotogramas, con la exposicion aun ajustandose: se descartan
    subprocess.run(entrada_ffmpeg(dev, c) + ["-vf", "select=gte(n\\,45)", "-frames:v", "1", "-y", salida],
                   check=True, capture_output=True)
    print(f"{dev}: {salida}" + (f" (sin poner: {malos})" if malos else ""))


def cmd_grabar(a):
    c = cfg_camara()
    dev = dispositivo(c["nombre"])
    if dev is None:
        print(f"[camara externa] no hay camara '{c['nombre']}': no se graba", file=sys.stderr)
        return 1
    malos = configurar(dev, c["controles"])
    carpeta = os.path.join(AQUI, c.get("carpeta", "datos/externa"))
    os.makedirs(carpeta, exist_ok=True)
    base = os.path.join(carpeta, "externa_" + time.strftime("%Y%m%d_%H%M%S"))
    meta = {"dispositivo": dev, "controles": c["controles"], "sin_poner": malos, "video": os.path.basename(base) + ".mp4",
            "desfase_robot": None if a.sin_robot else desfase_robot(), "inicio_epoch": None, "fin_epoch": None}

    def guardar():
        with open(base + ".json", "w") as f:
            json.dump(meta, f, indent=1)
    guardar()
    # cada fotograma con su instante real (passthrough: sin duplicar ni tirar); el instante del primero lo da la
    # entrada ("start:") y el video empieza en 0
    cmd = entrada_ffmpeg(dev, c) + ["-vsync", "passthrough", "-c:v", "libx264", "-preset", "veryfast",
                                    "-crf", str(c.get("crf", 26)), "-pix_fmt", "yuv420p", "-movflags", "+frag_keyframe+empty_moov",
                                    "-y", base + ".mp4"]
    cmd.remove("-nostdin")
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE, text=True, errors="replace")
    parar = []
    signal.signal(signal.SIGTERM, lambda *_: parar.append(1))
    signal.signal(signal.SIGINT, lambda *_: parar.append(1))
    patron = re.compile(r"start: (\d+\.\d+)")
    buf = ""
    os.set_blocking(p.stderr.fileno(), False)
    while p.poll() is None and not parar:
        try:
            trozo = p.stderr.read()
        except (TypeError, OSError):
            trozo = None
        if trozo:
            buf = (buf + trozo)[-4000:]
            if meta["inicio_epoch"] is None:
                m = patron.search(buf)
                if m:
                    meta["inicio_epoch"] = float(m.group(1))
                    guardar()
                    if not a.silencio:
                        print(f"[camara externa] grabando {base}.mp4", file=sys.stderr)
        time.sleep(0.2)
    if p.poll() is not None and not parar:
        print(f"[camara externa] ffmpeg termino ({p.returncode}): {buf.strip()[-600:]}", file=sys.stderr)
    if p.poll() is None:
        try:
            p.stdin.write("q")
            p.stdin.flush()
            p.wait(timeout=15)
        except (OSError, subprocess.TimeoutExpired):
            p.kill()
    meta["fin_epoch"] = round(time.time(), 3)
    guardar()
    return 0


# --- fotos de una tirada ----------------------------------------------------------------------------------------
def traer_ultima():
    """Trae del robot la tirada mas reciente (solo esa carpeta) y devuelve su ruta local."""
    r = subprocess.run(["ssh", "-o", "ConnectTimeout=5", ROBOT_SSH, f"ls -td {DEST}/datos/tirada_* | head -1"],
                       capture_output=True, text=True, timeout=30)
    remota = r.stdout.strip()
    if not remota:
        sys.exit("no encuentro tiradas en el robot")
    subprocess.run(["rsync", "-a", f"{ROBOT_SSH}:{remota}", DATOS + "/"], check=True, timeout=300)
    return os.path.join(DATOS, os.path.basename(remota))


def eventos_epoch(tirada):
    """[(epoch del robot, estado, motivo)] de eventos.csv (t monotonico) con el inicio del resumen."""
    import csv
    res = json.load(open(os.path.join(tirada, "resumen.json")))
    if "inicio_epoch" in res:
        e0, m0 = res["inicio_epoch"], res["t0_monotonic"]
        return [(e0 + float(f["t"]) - m0, f["estado"], f["motivo"]) for f in csv.DictReader(open(os.path.join(tirada, "eventos.csv")))]
    # tiradas de antes de §60: el inicio con resolucion de 1 s (hora local del robot) y los eventos relativos
    import datetime
    print("(tirada sin inicio_epoch: sincronia de +-1 s y la zona horaria del robot supuesta UTC+8)")
    e0 = datetime.datetime.strptime(res["inicio"], "%Y-%m-%d %H:%M:%S").replace(
        tzinfo=datetime.timezone(datetime.timedelta(hours=8))).timestamp()
    return [(e0 + t, est, mot) for t, est, mot in res["eventos"]]


def video_de(epochs):
    """El video (y su json) que cubre esos instantes (epoch de la laptop)."""
    for j in sorted(glob.glob(os.path.join(DATOS, "externa", "externa_*.json")), reverse=True):
        m = json.load(open(j))
        if m.get("inicio_epoch") and m["inicio_epoch"] <= max(epochs) and (m.get("fin_epoch") or time.time()) >= min(epochs):
            return os.path.join(os.path.dirname(j), m["video"]), m
    return None, None


def cuadro(video, t_video, salida, ancho=None):
    vf = ["-vf", f"scale={ancho}:-2"] if ancho else []
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-ss", f"{max(0.0, t_video):.3f}", "-i", video,
                    "-frames:v", "1", *vf, "-q:v", "3", "-y", salida], check=True)


# momentos extra (s despues del evento) para ver el agarre: el cierre, al levantar y en el transporte
EXTRA = {"CERRAR": [0.0], "TRANSPORTE": [1.5, 3.0, 6.0], "DESCENSO": [0.0], "SOLTAR": [0.0, 1.5]}


def cmd_fotos(a):
    import cv2
    import numpy as np
    tirada = traer_ultima() if a.ultima else os.path.abspath(a.tirada)
    eventos = eventos_epoch(tirada)
    mejor_video, m = None, None
    if eventos:
        # el desfase de esa grabacion (robot - laptop): epoch de la laptop = epoch del robot - desfase
        mejor_video, m = video_de([e for e, _, _ in eventos])
    if mejor_video is None:
        sys.exit(f"no hay video de la camara externa que cubra {os.path.basename(tirada)}")
    desf = (m.get("desfase_robot") or {}).get("s", 0.0)
    salida = os.path.join(tirada, "externa")
    os.makedirs(salida, exist_ok=True)
    fotos = []
    for k, (e, est, mot) in enumerate(eventos):
        for dt in EXTRA.get(est, [0.0]):
            t_video = e - desf - m["inicio_epoch"] + dt
            nombre = f"{k:02d}_{est}{'' if dt == 0 else f'_+{dt:.1f}s'}.jpg"
            try:
                cuadro(mejor_video, t_video, os.path.join(salida, nombre))
            except subprocess.CalledProcessError:
                continue
            if not os.path.exists(os.path.join(salida, nombre)):      # pasado el final del video
                continue
            fotos.append((nombre, f"{est} {mot[:60]}" + ("" if dt == 0 else f" +{dt:.1f} s")))
    # el mosaico: cada foto reducida con su rotulo (para verlo de una vez)
    tiles = []
    for nombre, rotulo in fotos:
        im = cv2.imread(os.path.join(salida, nombre))
        if im is None:
            continue
        im = cv2.resize(im, (640, 360))
        cv2.rectangle(im, (0, 0), (640, 22), (0, 0, 0), -1)
        cv2.putText(im, rotulo, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        tiles.append(im)
    if tiles:
        cols = 3
        while len(tiles) % cols:
            tiles.append(np.zeros_like(tiles[0]))
        filas = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
        cv2.imwrite(os.path.join(salida, "mosaico.jpg"), np.vstack(filas), [cv2.IMWRITE_JPEG_QUALITY, 85])
    # el clip de cada agarre: del descenso a 3 s despues de soltar (o al siguiente VOLVER), a 960 px
    descensos = [e for e, est, _ in eventos if est == "DESCENSO"]
    for n, e0 in enumerate(descensos):
        fin = next((e for e, est, _ in eventos if e > e0 and est in ("SOLTAR", "VOLVER")), e0 + 30) + 3.0
        t0 = e0 - desf - m["inicio_epoch"] - 1.0
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-ss", f"{max(0.0, t0):.3f}", "-i", mejor_video,
                        "-t", f"{fin - e0 + 1.0:.2f}", "-vf", "scale=960:-2", "-c:v", "libx264", "-preset", "veryfast",
                        "-crf", "26", "-an", "-y", os.path.join(salida, f"agarre_{n + 1}.mp4")], check=False)
    print(f"{len(fotos)} fotos y {len(descensos)} clips en {salida} (video {os.path.basename(mejor_video)}, "
          f"desfase del robot {desf:+.3f} s)")


def cmd_hay(a):
    return 0 if dispositivo(cfg_camara()["nombre"]) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("ver", help="configura la camara y guarda una foto")
    v.add_argument("--salida", default=None)
    g = sub.add_parser("grabar", help="graba hasta Ctrl+C o SIGTERM")
    g.add_argument("--sin-robot", action="store_true", help="sin medir el desfase del reloj del robot")
    g.add_argument("--silencio", action="store_true")
    f = sub.add_parser("fotos", help="las fotos de los eventos de una tirada")
    f.add_argument("tirada", nargs="?")
    f.add_argument("--ultima", action="store_true", help="trae del robot la tirada mas reciente")
    sub.add_parser("hay", help="sale con 0 si la camara esta conectada")
    a = ap.parse_args()
    if a.cmd == "fotos" and not (a.tirada or a.ultima):
        ap.error("fotos: la carpeta de la tirada o --ultima")
    return {"ver": cmd_ver, "grabar": cmd_grabar, "fotos": cmd_fotos, "hay": cmd_hay}[a.cmd](a) or 0


if __name__ == "__main__":
    sys.exit(main())
