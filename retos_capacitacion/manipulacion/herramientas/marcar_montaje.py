#!/usr/bin/env python3
"""Hito 1: colocar el carrito y las bandejas mirando la pantalla, con el robot de pie en el arnes y los brazos a 0.

    ./ejecutar.sh herramientas/marcar_montaje.py              # en el robot (camara_servidor.sh en marcha)
    python3 herramientas/marcar_montaje.py datos/dataset_<fecha>    # en la PC, sobre una grabacion
    python3 herramientas/marcar_montaje.py --sim               # en la PC, con el gemelo y teclas que mueven el carrito

Solo lee la camara: no mueve el robot. Dos fases:
  MESA      el borde cercano (x0), el desvio lateral y el giro de la mesa en vivo, y lo que falta para llegar a
            objetivo_mesa de montaje.yaml. Las tres dentro de su tolerancia durante 2 s: "MESA EN SU SITIO", frenos.
            [g] con la mesa VACIA: la mide con 15 fotogramas y la guarda en montaje.yaml (como medir_montaje.py)
            y en datos/dataset_<fecha>_mesa_marcar/ (fotogramas y copia del montaje: traerla a la PC con
            ./desplegar.sh --traer, porque desplegar.sh copia config/ de la PC al robot), y pasa a BANDEJAS.
            [b] pasa sin guardar.
            El [g] de la mesa borra lo medido de las bandejas (hay que volver a medirlas).
  BANDEJAS  cada bandeja por su color (montaje.yaml) y su altura, y lo que falta para llevarla a su centro. Con
            los brazos a 0 las manos tapan parte de cada bandeja: basta con ver dos paredes; con una sola lo dice
            y esa se coloca con regla. Colocadas, [g] guarda donde esta cada una (mediana de los ultimos 15
            fotogramas) en montaje.yaml (`medida`) y en datos/dataset_<fecha>_bandejas_marcar/: el robot suelta
            ahi y no en el objetivo. Las que no se ven enteras (la C con los brazos a 0) o estan fuera de
            tolerancia no se guardan: se usa su objetivo. [m] vuelve a MESA.
La imagen de la camara con el objetivo pintado (mesa, zona, cruz y bandejas) se ve en http://<robot>:5001 (en la
PC: --salida guarda las imagenes). [q] o Ctrl+C: salir. En --sim: [w]/[s] alejan/acercan el carrito 5 mm,
[a]/[d] lo llevan a la izquierda/derecha del robot, [z]/[x] lo giran 0.5 grados.
"""

import argparse
import collections
import copy
import datetime
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
from manip import calibracion, config  # noqa: E402
from manip import montaje as mt  # noqa: E402
from manip.geometria import T, aplicar, inv  # noqa: E402

N_SUAVE = 5           # mediana de las ultimas medidas de la mesa
N_BANDEJAS = 15       # [g] de las bandejas: mediana de las ultimas medidas de cada una
MIN_BANDEJA = 8       # ...vista entera en al menos estos fotogramas
T_QUIETA_S = 2.0      # dentro de tolerancia durante esto para dar la mesa por colocada
VERDE, ROJO, AMARILLO, BLANCO, CIAN = (60, 200, 60), (40, 40, 230), (0, 220, 255), (255, 255, 255), (230, 200, 0)
BGR_BANDEJA = {"blanca": (235, 235, 235), "roja": (30, 30, 220), "azul": (220, 80, 20), "naranja": (20, 120, 255)}


def argumentos():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", nargs="?", help="carpeta grabada con grabar_dataset.py (prueba en la PC)")
    ap.add_argument("--sim", action="store_true", help="en la PC: camara virtual del gemelo")
    ap.add_argument("--mesa", default="0,0,0", help="--sim: desvio inicial del carrito dx,dy (m),yaw (grados)")
    ap.add_argument("--fase", choices=["mesa", "bandejas"], default="mesa")
    ap.add_argument("--salida", default=None, help="carpeta para las imagenes (en la PC)")
    ap.add_argument("--puerto", type=int, default=None, help="video en http://<robot>:PUERTO (0 = sin video)")
    ap.add_argument("--una", action="store_true", help="un solo fotograma: imprime la medida en JSON y sale")
    return ap.parse_args()


# --- fuentes ---------------------------------------------------------------------------------------------

class FuenteSim:
    """Camara virtual del gemelo con el montaje de montaje.yaml, el robot de pie con los brazos a 0 y la mesa
    donde dice el objetivo mas un desvio que se cambia con teclas."""

    def __init__(self, cfg, montaje, cin, desvio):
        os.environ.setdefault("MUJOCO_GL", "glfw")
        import mujoco
        from sim import escena
        from sim.camara_virtual import CamaraVirtual
        self.mj = mujoco
        cal = calibracion.cargar_o_urdf(cin, {})
        postura = list(escena.POSTURA_PIE)
        for i in range(13, 27):               # brazos a 0 (13-19 izquierdo, 20-26 derecho)
            postura[i] = 0.0
        m, info = escena.construir(montaje=montaje_en_objetivo(montaje), objetos={"objetos": {}}, cal=cal, cfg=cfg,
                                   postura=postura)
        self.m, self.est = m, escena.Estado(m, info)
        self.cam = CamaraVirtual(m, info, cal)
        self.cal = cal
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "mesa")
        self.b_mesa = b
        self.g_aruco = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"aruco_{i}") for i in (2, 3, 4, 5)]
        self.pos0 = (m.body_pos[b].copy(), m.body_quat[b].copy(),
                     [(m.geom_pos[g].copy(), m.geom_quat[g].copy()) for g in self.g_aruco])
        self.desvio = np.array(desvio, float)
        self.meta = self.cam.meta()
        self.mover(0, 0, 0)

    def _rot(self, q):
        R = np.zeros(9)
        self.mj.mju_quat2Mat(R, q)
        return R.reshape(3, 3)

    def mover(self, dx, dy, dyaw_deg):
        """Mueve el carrito en el torso (el giro, alrededor del centro de su borde cercano)."""
        from sim.escena import _quat
        self.desvio += [dx, dy, dyaw_deg]
        p0, q0, ar0 = self.pos0
        c = math.radians(self.desvio[2])
        Rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1.0]])
        D = T(Rz, p0 + [self.desvio[0], self.desvio[1], 0]) @ T(np.eye(3), -p0)
        M = D @ T(self._rot(q0), p0)
        self.m.body_pos[self.b_mesa], self.m.body_quat[self.b_mesa] = M[:3, 3], _quat(M[:3, :3])
        for g, (pg, qg) in zip(self.g_aruco, ar0):
            Mg = D @ T(self._rot(qg), pg)
            self.m.geom_pos[g], self.m.geom_quat[g] = Mg[:3, 3], _quat(Mg[:3, :3])

    def mover_bandeja(self, nombre, dx, dy, dyaw_deg=0.0):
        """Desplaza una bandeja del gemelo (en el torso; el giro, alrededor de su centro)."""
        from sim.escena import _quat
        b = self.mj.mj_name2id(self.m, self.mj.mjtObj.mjOBJ_BODY, f"bandeja_{nombre}")
        c = math.radians(dyaw_deg)
        Rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1.0]])
        self.m.body_pos[b] += [dx, dy, 0.0]
        self.m.body_quat[b] = _quat(Rz @ self._rot(self.m.body_quat[b]))

    def siguiente(self, timeout_s=None):
        self.est.adelante()
        return self.cam.fotograma(self.est)


def montaje_en_objetivo(montaje):
    """El montaje con la mesa medida llevada a objetivo_mesa (para el gemelo de --sim)."""
    import copy
    m = copy.deepcopy(montaje)
    T_obj = T_objetivo(montaje)
    m["mesa"]["T_torso_mesa"] = T_obj.tolist()
    m["mesa"].pop("aruco_torso", None)
    return m


def T_objetivo(montaje):
    """T_torso_mesa del objetivo: el plano medido, el borde cercano en (x0_m, y_m) y el giro yaw_deg."""
    o = montaje["objetivo_mesa"]
    pl = mt.plano(montaje)
    c = math.radians(o.get("yaw_deg", 0.0))
    x = np.array([math.cos(c), math.sin(c), 0.0])
    x -= (x @ pl.n) * pl.n
    x /= np.linalg.norm(x)
    origen = mt.punto_en_mesa(montaje, (o["x0_m"], o.get("y_m", 0.0)))
    return T(np.column_stack([x, np.cross(pl.n, x), pl.n]), origen)


# --- video -----------------------------------------------------------------------------------------------

class Video:
    """MJPEG en http://<robot>:puerto con la ultima imagen (como el mosaico de camara_servidor.py)."""

    def __init__(self, puerto):
        import logging
        from flask import Flask, Response
        self._jpeg, self._n = None, 0
        self._cond = threading.Condition()
        app = Flask(__name__)
        logging.getLogger("werkzeug").setLevel(logging.ERROR)

        def flujo():
            visto = -1
            while True:
                with self._cond:
                    if not self._cond.wait_for(lambda: self._n != visto and self._jpeg is not None, timeout=10):
                        continue
                    jpeg, visto = self._jpeg, self._n
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"

        app.add_url_rule("/", "video", lambda: Response(flujo(), mimetype="multipart/x-mixed-replace; boundary=frame"))
        threading.Thread(target=lambda: app.run(host="0.0.0.0", port=puerto, debug=False, threaded=True),
                         daemon=True).start()

    def publicar(self, img):
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self._cond:
                self._jpeg, self._n = buf.tobytes(), self._n + 1
                self._cond.notify_all()


# --- dibujo ----------------------------------------------------------------------------------------------

class Dibujo:
    def __init__(self, f, cal):
        from manip.fuentes import intrinsecos
        self.img = f.color.copy()
        self.ic = intrinsecos(f, "color")
        self.M = inv(cal.T_torso_color)

    def uv(self, P):
        return self.ic.proyectar(aplicar(self.M, np.atleast_2d(P)))

    def poligono(self, P, color, grosor=2, cerrado=True):
        uv = self.uv(P)
        if not np.isfinite(uv).all():
            return
        cv2.polylines(self.img, [np.round(uv).astype(np.int32)], cerrado, color, grosor, cv2.LINE_AA)

    def rect_mesa(self, T_mesa, x, y, color, grosor=2):
        esq = [[x[0], y[0], 0], [x[1], y[0], 0], [x[1], y[1], 0], [x[0], y[1], 0]]
        self.poligono(aplicar(T_mesa, np.array(esq, float)), color, grosor)

    def cuadrado(self, montaje, centro, lado, yaw, color, grosor=2, altura=0.0):
        a = lado / 2
        R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        esq = [np.asarray(centro, float) + R @ v for v in ([a, a], [a, -a], [-a, -a], [-a, a])]
        self.poligono(np.array([mt.punto_en_mesa(montaje, p, altura) for p in esq]), color, grosor)

    def texto(self, lineas):
        y = 18
        for txt, color in lineas:
            (w, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(self.img, (4, y - h - 4), (10 + w, y + 5), (0, 0, 0), -1)
            cv2.putText(self.img, txt, (7, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            y += h + 10


def dibujar(f, cal, montaje, pose, fase, bandejas, lineas):
    d = Dibujo(f, cal)
    sup = montaje["mesa"]["superficie"]
    d.rect_mesa(T_objetivo(montaje), sup["x"], sup["y"], AMARILLO, 1)
    if pose is not None:
        d.rect_mesa(pose["T"], sup["x"], sup["y"], VERDE if pose.get("ok") else ROJO, 2)
    z = montaje["zona_recogida"]
    d.poligono(np.array([mt.punto_en_mesa(montaje, p) for p in
                         ((z["x"][0], z["y"][0]), (z["x"][1], z["y"][0]), (z["x"][1], z["y"][1]), (z["x"][0], z["y"][1]))]),
               CIAN, 1)
    if montaje.get("cruz"):
        c = np.asarray(montaje["cruz"], float)
        for dv in ([0.015, 0], [0, 0.015]):
            d.poligono(np.array([mt.punto_en_mesa(montaje, c - dv), mt.punto_en_mesa(montaje, c + dv)]), BLANCO, 1, False)
    for nombre, b in montaje.get("bandejas", {}).items():
        lado = b["hueco_m"] + 2 * b["pared_m"]
        col = BGR_BANDEJA.get(b.get("color"), BLANCO)
        d.cuadrado(montaje, b["centro"], lado, 0.0, col, 1 if fase == "MESA" else 2)
        med = (bandejas or {}).get(nombre)
        if fase == "BANDEJAS" and med is not None and not med["a_medias"]:
            yaw = math.radians(med["yaw_deg"])
            d.cuadrado(montaje, med["centro"], lado, yaw, VERDE if med.get("ok") else ROJO, 2, b["alto_m"])
            uv = d.uv(mt.punto_en_mesa(montaje, med["centro"], b["alto_m"]))[0]
            if np.isfinite(uv).all():
                cv2.putText(d.img, nombre, (int(uv[0]) - 5, int(uv[1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            VERDE if med.get("ok") else ROJO, 2, cv2.LINE_AA)
    d.texto(lineas)
    return d.img


# --- medida ----------------------------------------------------------------------------------------------

def suavizar(historial):
    """Mediana de las ultimas poses (la T, de la ultima)."""
    p = dict(historial[-1])
    for k in ("x0", "y", "yaw_deg"):
        p[k] = float(np.median([h[k] for h in historial]))
    return p


def decir(*lineas):
    """Imprime las lineas y una en blanco detras: los mensajes se separan a la vista."""
    print("\n".join(lineas) + "\n", flush=True)


def lineas_mesa(pose, dv, objetivo):
    if pose is None:
        return [("MESA: sin medida", ROJO)]
    col = lambda ok: VERDE if ok else ROJO   # noqa: E731
    return [(f"borde cercano {pose['x0']:.3f} m (objetivo {objetivo['x0_m']:.3f} +-{objetivo['tol_x_m'] * 100:.0f} cm)", col(dv["ok"]["x"])),
            (f"lateral {pose['y'] * 100:+.1f} cm (+ = a la izquierda del robot)", col(dv["ok"]["y"])),
            (f"giro {pose['yaw_deg']:+.2f} grados (+-{objetivo['tol_yaw_deg']:g})", col(dv["ok"]["yaw"])),
            ("MESA EN SU SITIO: frenos" if dv["todo_ok"] else "falta: " + ", ".join(dv["pasos"]), col(dv["todo_ok"]))]


CABECERA = ("Montaje del reto. mesa: la escriben herramientas/medir_montaje.py y marcar_montaje.py ([g] de la mesa);\n"
            "bandejas.*.medida, el [g] de las bandejas de marcar_montaje.py. zona_recogida, bandejas (centro: el\n"
            "objetivo), cruz y objetivo_mesa: las fija el plan (marco del torso).")


def medir_y_guardar(fuente, cal, montaje, dicc, cfg, n=15, guardar=True):
    """Como medir_montaje.py: n fotogramas, mediana de la profundidad, mt.medir y montaje.yaml. Ademas deja los
    fotogramas y una copia del montaje en datos/dataset_<fecha>_mesa_marcar/: desplegar.sh copia config/ de la PC
    al robot (pisaria lo medido aqui); con ./desplegar.sh --traer la copia llega a la PC."""
    sys.path.insert(0, os.path.join(RAIZ, "herramientas"))
    from medir_montaje import promediar
    from manip.fuentes import EscritorDataset
    from manip.registro import carpeta_tirada
    fotos = [x for x in (fuente.siguiente(2.0) for _ in range(n)) if x is not None and x.prof is not None]
    if not fotos:
        raise ValueError("sin fotogramas con profundidad")
    mesa, informe = mt.medir(promediar(fotos), cal, montaje, dicc)
    nuevo = copy.deepcopy(montaje)
    # mesa nueva: lo medido de las bandejas ya no vale
    informe["bandejas_borradas"] = sorted(k for k, b in nuevo.get("bandejas", {}).items() if b.pop("medida", None))
    nuevo["fecha"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    nuevo["mesa"] = mesa
    carpeta = None
    if guardar:
        carpeta = carpeta_tirada(config.ruta_datos(cfg), "dataset", "mesa_marcar")
        esc = EscritorDataset(carpeta)
        for x in fotos:
            esc.escribir(x)
        esc.cerrar()
        nuevo["origen"] = f"{os.path.basename(carpeta)} ({len(fotos)} fotogramas); calibracion {cal.origen} {cal.fecha}"
        config.guardar_yaml(os.path.join(carpeta, "montaje.yaml"), nuevo, CABECERA)
        config.guardar_yaml(config.RUTA_MONTAJE, nuevo, CABECERA)
    return nuevo, mesa, informe, carpeta


def guardar_bandejas(montaje, historial, fotos, cfg, tol_b, guardar=True, n_min=MIN_BANDEJA, tol_yaw_deg=5.0):
    """[g] en BANDEJAS: la mediana de las ultimas medidas de cada bandeja (centro en el torso y giro respecto de la
    mesa) como `medida` en montaje.yaml, y una copia con los fotogramas en datos/dataset_<fecha>_bandejas_marcar/.
    Solo las vistas enteras (no a medias) en >= n_min fotogramas y dentro de tolerancia; las demas se quedan sin
    `medida` (el robot usa su objetivo). Devuelve (montaje nuevo, [lineas de informe], carpeta o None)."""
    nuevo = copy.deepcopy(montaje)
    fecha = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    informe = []
    for nombre, b in nuevo.get("bandejas", {}).items():
        b.pop("medida", None)
        meds = [h[nombre] for h in historial if h.get(nombre) is not None and not h[nombre]["a_medias"]]
        etiqueta = f"{nombre} ({b.get('color')})"
        if len(meds) < n_min:
            informe.append(f"{etiqueta}: sin guardar, vista entera en {len(meds)} de {len(historial)} fotogramas "
                           f"(se usa su objetivo)")
            continue
        c = np.median([m["centro"] for m in meds], axis=0)
        yaw = float(np.median([m["yaw_deg"] for m in meds]))
        e = c - np.asarray(b["centro"], float)
        if np.linalg.norm(e) > tol_b or abs(yaw) > tol_yaw_deg:
            informe.append(f"{etiqueta}: sin guardar, esta a {np.linalg.norm(e) * 100:.1f} cm y {yaw:+.0f} grados de su "
                           f"sitio (colocala primero)")
            continue
        b["medida"] = {"centro": [round(float(v), 4) for v in c], "yaw_deg": round(yaw, 1), "fotogramas": len(meds),
                       "fecha": fecha}
        informe.append(f"{etiqueta}: guardada en ({c[0]:.3f}, {c[1]:+.3f}) m y {yaw:+.1f} grados "
                       f"({e[0] * 100:+.1f}, {e[1] * 100:+.1f} cm del objetivo)")
    carpeta = None
    if guardar:
        from manip.fuentes import EscritorDataset
        from manip.registro import carpeta_tirada
        carpeta = carpeta_tirada(config.ruta_datos(cfg), "dataset", "bandejas_marcar")
        esc = EscritorDataset(carpeta)
        for x in fotos:
            esc.escribir(x)
        esc.cerrar()
        config.guardar_yaml(os.path.join(carpeta, "montaje.yaml"), nuevo, CABECERA)
        config.guardar_yaml(config.RUTA_MONTAJE, nuevo, CABECERA)
    return nuevo, informe, carpeta


def main():
    a = argumentos()
    cfg = config.cargar()
    montaje = config.cargar_montaje()
    if "objetivo_mesa" not in montaje:
        sys.exit("Falta objetivo_mesa en config/montaje.yaml")
    from manip.cinematica import Cinematica
    cin = Cinematica.desde_config(cfg)
    dicc = cfg["calibracion"]["diccionario"]
    det = calibracion.detector(dicc)
    hsv = cfg["montaje"]["hsv_bandejas"]
    tol_b = float(montaje.get("tol_bandejas_m", 0.015))
    objetivo = montaje["objetivo_mesa"]
    sim = None
    if a.sim:
        sim = FuenteSim(cfg, montaje, cin, [float(v) for v in a.mesa.split(",")])
        fuente, cal = sim, sim.cal
    elif a.dataset:
        from manip.fuentes import FuenteDataset
        fuente = FuenteDataset(a.dataset)
        cal = calibracion.cargar_o_urdf(cin, fuente.meta)
    else:
        from manip.fuentes import FuenteZmq
        fuente = FuenteZmq(cfg["red"]["zmq_fotogramas"])
        if fuente.fresco(timeout_s=5.0) is None:
            sys.exit("No llegan fotogramas: ./camara_servidor.sh")
        cal = calibracion.cargar_o_urdf(cin, fuente.meta)
    puerto = cfg["montaje"]["puerto_video"] if a.puerto is None else a.puerto
    video = None
    if puerto and not (a.una or a.dataset or a.sim):
        try:
            video = Video(puerto)
            decir(f"video en http://<robot>:{puerto}")
        except ImportError:
            decir("sin flask: no hay video")
    if a.salida:
        os.makedirs(a.salida, exist_ok=True)
    decir(f"objetivo: borde cercano {objetivo['x0_m']} m, lateral {objetivo.get('y_m', 0.0)}, giro {objetivo.get('yaw_deg', 0.0)}; "
          f"calibracion {cal.origen}. Robot de pie, brazos a 0, nadie entre la camara y la mesa.")
    teclas = collections.deque()
    from manip.teclado import Teclado
    teclado = Teclado(lambda: None, tecla=b"\x00", al_teclear=lambda ch: teclas.append(ch.lower()))
    if not (a.una or a.dataset):
        teclado.abrir()
    fase = a.fase.upper()
    historial = collections.deque(maxlen=N_SUAVE)
    hist_band, fotos_band = collections.deque(maxlen=N_BANDEJAS), collections.deque(maxlen=N_BANDEJAS)
    t_dentro, avisado, t_linea, k = None, False, 0.0, 0
    ultimo = {}
    try:
        while True:
            while teclas:
                ch = teclas.popleft()
                if ch == "q":
                    raise KeyboardInterrupt
                if ch == "b":
                    fase = "BANDEJAS"
                if ch == "m":
                    fase, t_dentro, avisado = "MESA", None, False
                if ch == "g" and fase == "BANDEJAS":
                    montaje, informe, carpeta = guardar_bandejas(montaje, hist_band, list(fotos_band), cfg, tol_b,
                                                                 guardar=not (a.sim or a.dataset))
                    decir("bandejas guardadas" + (f" en {config.RUTA_MONTAJE} y {carpeta} (./desplegar.sh --traer y "
                                                  f"copiar su montaje.yaml a config/ de la PC):" if carpeta else
                                                  " (sin escribir: --sim o grabacion):"), *("  " + x for x in informe))
                elif ch == "g" and fase == "MESA":
                    decir("midiendo la mesa (15 fotogramas, mesa vacia)...")
                    try:
                        montaje, mesa, informe, carpeta = medir_y_guardar(fuente, cal, montaje, dicc, cfg,
                                                                          guardar=not (a.sim or a.dataset))
                        decir(f"mesa: borde {mesa['T_torso_mesa'][0][3]:.3f} m, lateral {mesa['T_torso_mesa'][1][3] * 100:+.1f} cm, "
                              f"giro {mesa['yaw_deg']:+.2f} grados, ArUco {informe['aruco_vistos']}"
                              + (f"; en {config.RUTA_MONTAJE} y {carpeta} (./desplegar.sh --traer y copiar su "
                                 f"montaje.yaml a config/ de la PC)" if carpeta else " (sin guardar: --sim o grabacion)")
                              + ". Ahora las bandejas; colocadas, [g] las guarda.",
                              *([f"lo medido de las bandejas {', '.join(informe['bandejas_borradas'])} se ha borrado "
                                 "(mesa nueva): vuelve a guardarlas"] if informe.get("bandejas_borradas") else []))
                        fase = "BANDEJAS"
                        hist_band.clear()
                        fotos_band.clear()
                    except ValueError as e:
                        decir(f"no se pudo medir: {e}")
                if sim is not None and ch in "wsadzx":
                    paso = {"w": (0.005, 0, 0), "s": (-0.005, 0, 0), "a": (0, 0.005, 0), "d": (0, -0.005, 0),
                            "z": (0, 0, 0.5), "x": (0, 0, -0.5)}[ch]
                    sim.mover(*paso)
            f = fuente.siguiente(2.0)
            if f is None:
                if a.dataset:
                    break
                continue
            k += 1
            if f.color is None or f.prof is None:
                decir("la camara no da color y profundidad")
                continue
            pose, bandejas, dv = None, None, None
            try:
                historial.append(mt.pose_mesa(f, cal, montaje, det))
                pose = suavizar(historial)
                dv = mt.desvio_mesa(pose, objetivo)
                pose["ok"] = dv["todo_ok"]
                lineas = lineas_mesa(pose, dv, objetivo)
            except ValueError as e:
                lineas = [(f"MESA: {e}", ROJO)]
            n_mesa = len(lineas)
            ahora = time.monotonic()
            if fase == "MESA" and dv is not None:
                t_dentro = (t_dentro or ahora) if dv["todo_ok"] else None
                quieta = t_dentro is not None and ahora - t_dentro >= T_QUIETA_S
                if quieta and not avisado:
                    decir("=" * 70, "  MESA EN SU SITIO: pon los frenos. Con la mesa vacia, [g] la mide y la guarda.", "=" * 70)
                avisado = quieta
            if fase == "BANDEJAS" and pose is not None:
                plano = pose["plano"]
                bandejas = mt.medir_bandejas(f, cal, plano, pose["T"], montaje, hsv)
                hist_band.append(bandejas)
                fotos_band.append(f)
                todas = True
                for nombre, b in montaje["bandejas"].items():
                    med = bandejas.get(nombre)
                    if med is None:
                        lineas.append((f"{nombre} ({b.get('color')}): no la veo", ROJO))
                        todas = False
                        continue
                    if med["a_medias"]:
                        lineas.append((f"{nombre} ({b.get('color')}): solo veo una pared (la tapa la mano): "
                                       f"colocala con regla", ROJO))
                        med["ok"] = todas = False
                        continue
                    db = mt.desvio_bandeja(med, b["centro"], tol_b)
                    med["ok"] = db["ok"]
                    todas &= db["ok"]
                    lineas.append((f"{nombre} ({b.get('color')}): {'bien' if db['ok'] else ', '.join(db['pasos'])} "
                                   f"[{db['ex'] * 100:+.1f}, {db['ey'] * 100:+.1f} cm, {med['yaw_deg']:+.0f} gr]"
                                   + ("  (guardada)" if b.get("medida") else ""), VERDE if db["ok"] else ROJO))
                lineas.append(("BANDEJAS EN SU SITIO: [g] las guarda" if todas else "bandejas: falta", VERDE if todas else ROJO))
            teclas_txt = "[g] guardar mesa  [b] bandejas" if fase == "MESA" else "[g] guardar bandejas  [m] mesa"
            lineas.insert(0, (f"{fase}   {teclas_txt}  [q] salir", BLANCO))
            img = dibujar(f, cal, montaje, pose, fase, bandejas, lineas)
            if video is not None:
                video.publicar(img)
            if a.salida:
                cv2.imwrite(os.path.join(a.salida, f"{k:04d}.jpg"), img)
            ultimo = {"fase": fase, "pose": None if pose is None else {kk: pose[kk] for kk in ("x0", "y", "yaw_deg", "aruco")},
                      "desvio": None if dv is None else {kk: dv[kk] for kk in ("ex", "ey", "eyaw", "todo_ok", "pasos")},
                      "bandejas": None if bandejas is None else
                      {n: None if v is None else {"centro": [round(float(c), 4) for c in v["centro"]],
                                                  "yaw_deg": round(v["yaw_deg"], 1), "ok": v.get("ok"),
                                                  "n_paredes": v["n_paredes"], "rms_mm": round(1000 * v["rms"], 1),
                                                  "lados": v["lados"], "a_medias": v["a_medias"]}
                       for n, v in bandejas.items()}}
            if a.una:
                break
            if ahora - t_linea >= 1.0 or a.dataset:
                t_linea = ahora
                # la mesa en una linea y cada bandeja en la suya, con una en blanco detras
                decir("MESA  " + " | ".join(t for t, _ in lineas[1:1 + n_mesa]), *("  " + t for t, _ in lineas[1 + n_mesa:]))
    except KeyboardInterrupt:
        pass
    finally:
        teclado.cerrar()
    if a.una or a.dataset:
        print(json.dumps(ultimo, indent=1, ensure_ascii=False, default=float))


if __name__ == "__main__":
    main()
