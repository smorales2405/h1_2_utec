#!/usr/bin/env python3
"""Video de la simulacion de la PC: la tirada de nivel 2 en el gemelo, desde de pie en el arnes con los brazos a
0 hasta que los cuatro objetos estan en sus bandejas y los brazos vuelven a 0.

    python3 herramientas/video_simulacion.py                  # -> datos/videos/sim_nivel2_<fecha>.mp4 (x2)
    python3 herramientas/video_simulacion.py --velocidad 1    # a velocidad real (~5 min)

Dos pasadas:
  1. La tirada de `manipulacion.py --nivel 2 --sim --arnes` sin cambios (percepcion sobre la camara virtual,
     planificador, supervisor y manos simuladas) con un reloj de la tirada: los movimientos del brazo cuentan lo
     que duran en el robot (la simulacion acelerada no los espera) y lo demas (percibir, planificar, la mano), lo
     que tarda de verdad en la PC. Se guarda el estado del gemelo hasta 60 veces por segundo de ese reloj.
  2. El render: vista frontal alta; en cada agarre, de cerca desde el lado del brazo que trabaja (el objeto y
     luego su bandeja); la camara de la cabeza en un recuadro; rotulos. ffmpeg (libx264) escribe el MP4.
El gemelo no tiene fisica: al abrir la mano, el objeto pasa al fondo de la bandeja de golpe. En el video cae
desde donde lo tenia la mano, con la gravedad y a velocidad real.
"""

import argparse
import bisect
import functools
import math
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import config  # noqa: E402
from manip.montaje import bandeja_real, punto_en_mesa  # noqa: E402

G = 9.81
HZ_MUESTRAS = 60.0
RETRASO_CAIDA_S = 0.2           # de la orden de abrir a que el objeto se suelta (los dedos ya se han abierto algo)
FUENTES = "/usr/share/fonts/truetype/dejavu/"

# camaras libres de MuJoCo: (mirando a, distancia, azimut, elevacion); azimut 180 = desde delante del robot
GENERAL = (np.array([0.32, 0.0, -0.05]), 2.3, 195.0, -33.0)      # todo el robot y la mesa
AZIMUT_CERCA = {"izq": 215.0, "der": 145.0}     # de cerca, por delante y del lado del brazo que trabaja
T_TRANSICION = 1.2              # s de video de una toma a la siguiente

NOMBRE = {"cilindro": "el cilindro", "prisma": "el prisma", "tenis": "la pelota", "cubo": "el cubo"}
MANO = {"izq": "izquierda", "der": "derecha"}


# --- pasada 1: la tirada con su reloj ----------------------------------------------------------------------
class Reloj:
    """El tiempo de la tirada como en el robot: dentro de un movimiento del brazo avanza lo que este espera entre
    consignas (dormir); fuera, con el reloj de pared."""

    def __init__(self):
        self.t = 0.0
        self._ultimo = time.monotonic()
        self._en_brazo = 0
        self._cerrojo = threading.Lock()

    def _pasar(self):
        ahora = time.monotonic()
        if not self._en_brazo:
            self.t += ahora - self._ultimo
        self._ultimo = ahora

    def ahora(self):
        with self._cerrojo:
            self._pasar()
            return self.t

    def dormir(self, s):
        with self._cerrojo:
            self.t += s

    def brazo(self, f):
        """f (un movimiento del brazo) con el reloj de pared parado."""
        def g(*a, **k):
            with self._cerrojo:
                self._pasar()
                self._en_brazo += 1
            try:
                return f(*a, **k)
            finally:
                with self._cerrojo:
                    self._en_brazo -= 1
                    self._ultimo = time.monotonic()
        return g


class Grabadora:
    """El estado del gemelo (qpos y objetos) cada 1/hz s del reloj, los cambios de estado del supervisor y lo que
    hace falta para animar cada caida."""

    def __init__(self, mundo, reloj, hz=HZ_MUESTRAS):
        self.mundo, self.reloj, self.dt = mundo, reloj, 1.0 / hz
        self.muestras = []          # (t, qpos, mocap_pos, mocap_quat)
        self.eventos = []           # (t, estado, motivo)
        self.caidas = []            # (t, clase, pose en la mano, pose en la bandeja)
        self._sig = -1.0
        self._cerrojo = threading.Lock()
        self._vivo = True
        self._hilo = threading.Thread(target=self._bucle, daemon=True)

    def tomar(self):
        with self._cerrojo:
            t = self.reloj.ahora()
            if t < self._sig:
                return
            with self.mundo.cerrojo:
                self.mundo.actualizar()
                d = self.mundo.est.d
                self.muestras.append((t, d.qpos.copy(), d.mocap_pos.copy(), d.mocap_quat.copy()))
            self._sig = t + self.dt

    def _bucle(self):
        while self._vivo:
            self.tomar()
            time.sleep(self.dt / 2)

    def enganchar(self, t):
        """A una tirada recien armada (manipulacion.armar)."""
        b = t.brazo
        b._dormir = lambda s: (self.reloj.dormir(s), self.tomar())      # la espera entre consignas de brazo.py
        b.ir_a = self.reloj.brazo(b.ir_a)
        b.rampa_peso = self.reloj.brazo(b.rampa_peso)
        t.sup.al_estado.append(lambda e: self.eventos.append((self.reloj.ahora(), e, t.sup.motivo_estado)))
        mundo, al_abrir = self.mundo, self.mundo.al_abrir

        def al_abrir_y_caer(lado):
            with mundo.cerrojo:
                clase = mundo.sujeto[lado]
                if clase is not None:
                    mundo.actualizar()
                    antes = mundo.donde(clase).copy()
                al_abrir(lado)
                if clase is not None:
                    self.caidas.append((self.reloj.ahora(), clase, antes, mundo.donde(clase).copy()))
        mundo.al_abrir = al_abrir_y_caer
        self._hilo.start()

    def parar(self):
        self._vivo = False
        self._hilo.join(timeout=1.0)


def grabar(cfg, objetos, montaje):
    """Corre la tirada de nivel 2 desde los brazos a 0 y devuelve la grabacion (y el motivo de fin)."""
    import manipulacion as mp
    asign = {k: v["bandeja"] for k, v in objetos["objetos"].items() if v.get("bandeja")} or dict(mp.ASIGNACION_SIM)
    a = SimpleNamespace(nivel=2, simulacro=False, sim=True, arnes=True, escala_tiempo=1.0, dianas=None,
                        colocacion=dict(mp.COLOCACION_SIM),
                        asignacion={k: b for k, b in asign.items() if k in mp.COLOCACION_SIM})
    t = mp.armar(a, cfg, montaje=montaje, objetos=objetos)
    grab = Grabadora(t.mundo, Reloj())
    grab.enganchar(t)
    try:
        motivo = mp.correr(t, confirmar=False)
    finally:
        grab.parar()
    finales = {k: t.mundo.bandeja_de(k) for k in t.mundo.poses}
    return SimpleNamespace(m=t.mundo.m, info=t.mundo.info, cal=t.cal, muestras=grab.muestras, eventos=grab.eventos,
                           caidas=grab.caidas, motivo=motivo, finales=finales, asignacion=a.asignacion)


# --- pasada 2: el video ------------------------------------------------------------------------------------
def suave(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def mezclar(a, b, s):
    (la, da, aa, ea), (lb, db, ab, eb) = a, b
    daz = (ab - aa + 180.0) % 360.0 - 180.0
    return (la + (lb - la) * s, da + (db - da) * s, aa + daz * s, ea + (eb - ea) * s)


def camara_en(tomas, v, transicion=T_TRANSICION):
    """La camara en el instante v del video: cada toma (v_i, camara) empieza en v_i una transicion suave desde
    donde estaba la camara (aunque la anterior no hubiera terminado)."""
    desde, (v0, hacia) = tomas[0][1], tomas[0]
    for vi, ci in tomas[1:]:
        if vi > v:
            break
        desde = mezclar(desde, hacia, suave((vi - v0) / transicion))
        v0, hacia = vi, ci
    return mezclar(desde, hacia, suave((v - v0) / transicion))


def slerp(q0, q1, s):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    c = float(q0 @ q1)
    if c < 0:
        q1, c = -q1, -c
    if c > 0.9995:
        q = q0 + s * (q1 - q0)
    else:
        th = math.acos(c)
        q = (math.sin((1 - s) * th) * q0 + math.sin(s * th) * q1) / math.sin(th)
    return q / np.linalg.norm(q)


def quat_de(R):
    import mujoco
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def caida(M0, M1, tau):
    """Pose (p, quat) del objeto tau s despues de soltarlo en M0 (4x4, en la mano) camino de M1 (en la bandeja):
    cae con la gravedad; el giro y el desvio horizontal, repartidos. None si ya ha llegado."""
    h = max(0.0, M0[2, 3] - M1[2, 3])
    T = math.sqrt(2 * h / G)
    if tau >= T:
        return None
    s = tau / T if T > 0 else 1.0
    p = M0[:3, 3] + s * (M1[:3, 3] - M0[:3, 3])
    p[2] = M0[2, 3] - 0.5 * G * tau * tau
    return p, slerp(quat_de(M0[:3, :3]), quat_de(M1[:3, :3]), s)


class Linea:
    """El guion del video: de los eventos de la grabacion, en segundos de video, con la entradilla y el final."""

    def __init__(self, grab, montaje, velocidad, entrada=2.5, salida=4.0):
        self.vel, self.entrada = velocidad, entrada
        ev = grab.eventos
        self.t0 = next(t for t, e, _ in ev if e == "A_OBSERVACION")
        t_fin = next((t for t, e, _ in ev if e == "FIN"), grab.muestras[-1][0])
        self.duracion = self.v(t_fin) + salida
        self.n_obj = len(grab.asignacion)
        self.muestras, self._tm = grab.muestras, [x[0] for x in grab.muestras]
        self.rotulos, self.tomas = [(0.0, "Posición inicial: brazos a 0°")], [(0.0, GENERAL)]
        # (cuando abre la mano, cuando empieza a caer, clase, pose en la mano, pose en la bandeja)
        self.caidas = [(self.v(t), self.v(t) + RETRASO_CAIDA_S / velocidad, clase, M0, M1)
                       for t, clase, M0, M1 in grab.caidas]
        ctx = {}
        for t, e, motivo in ev:
            if t < self.t0:
                continue
            v = self.v(t)
            r = re.match(r"(\w+) -> bandeja (\w+) con la mano (izq|der)$", motivo) if e == "PLANIFICAR" else None
            if r:
                b = montaje["bandejas"][r.group(2)]
                ctx = {"clase": r.group(1), "lado": r.group(3), "color": b.get("color", r.group(2)), "bandeja": b}
            texto = rotulo(e, motivo, ctx, bool(r))
            if texto:
                self.rotulos.append((v, texto))
            if e == "MOVER" and ctx:
                self.tomas.append((v, self._cerca(grab, t, ctx, "objeto", montaje)))
            elif e == "TRANSPORTE" and ctx:
                self.tomas.append((v, self._cerca(grab, t, ctx, "bandeja", montaje)))
            elif e in ("VOLVER", "SALIDA"):
                self.tomas.append((v, GENERAL))
            elif e == "FIN":                    # al final, de cerca las cuatro bandejas
                c = np.mean([bandeja_real(b)[0][:2] for b in montaje["bandejas"].values()], axis=0)
                self.tomas.append((v, (punto_en_mesa(montaje, c, 0.03), 1.35, 180.0, -55.0)))
        self.rotulos.append((self.v(t_fin), f"{len(self.caidas)}/{self.n_obj} objetos en su bandeja"))

    def v(self, t):
        return self.entrada + (t - self.t0) / self.vel

    def t(self, v):
        return self.t0 + max(0.0, v - self.entrada) * self.vel

    def muestra(self, v):
        i = bisect.bisect_right(self._tm, self.t(v)) - 1
        return self.muestras[max(0, i)]

    def _cerca(self, grab, t, ctx, que, montaje):
        if que == "objeto":
            import mujoco
            b = mujoco.mj_name2id(grab.m, mujoco.mjtObj.mjOBJ_BODY, f"obj_{ctx['clase']}")
            i = bisect.bisect_right(self._tm, t) - 1
            mira, dist, el = self.muestras[max(0, i)][2][grab.m.body_mocapid[b]] + [0, 0, 0.02], 0.80, -30.0
        else:
            mira, dist, el = punto_en_mesa(montaje, bandeja_real(ctx["bandeja"])[0][:2], 0.05), 0.95, -36.0
        return (np.asarray(mira, float), dist, AZIMUT_CERCA[ctx["lado"]], el)

    def rotulo_en(self, v):
        return _ultimo(self.rotulos, v)

    def colocados(self, v):
        return sum(1 for _, vc, _, M0, M1 in self.caidas if v >= vc and caida(M0, M1, v - vc) is None)


def _ultimo(lista, v):
    i = bisect.bisect_right([x[0] for x in lista], v) - 1
    return lista[max(0, i)][1]


def rotulo(estado, motivo, ctx, plan):
    obj = NOMBRE.get(ctx.get("clase"), ctx.get("clase", ""))
    lado, color = MANO.get(ctx.get("lado"), ""), ctx.get("color", "")
    if estado == "A_OBSERVACION":
        return "Brazos a la pose de observación"
    if estado == "OBSERVAR":
        n = int(motivo.split()[0]) if motivo[:1].isdigit() else 0
        return f"Cámara de la cabeza: {n} objeto{'s' if n != 1 else ''} en la mesa"
    if estado == "PLANIFICAR":
        return f"Planificando: {obj} → bandeja {color}, mano {lado}" if plan else "Planificando"
    if estado == "MOVER":
        return f"Mano {lado} hacia {obj}"
    if estado == "DESCENSO":
        return f"Mano {lado}: bajando a agarrar {obj}"
    if estado == "CERRAR":
        return "Agarre comprobado" if "comprobado" in motivo else f"Cerrando la mano {lado}"
    if estado == "TRANSPORTE":
        return f"Llevando {obj} a la bandeja {color}"
    if estado == "SOLTAR":
        return f"Soltando {obj} en la bandeja {color}"
    if estado == "VOLVER":
        return "Volviendo a la pose de observación"
    if estado == "SALIDA":
        return "Brazos de vuelta a 0°"
    return None


# --- rotulos sobre la imagen -------------------------------------------------------------------------------
@functools.lru_cache(maxsize=256)
def texto_rgba(texto, tam, negrita=False):
    from PIL import Image, ImageDraw, ImageFont
    try:
        f = ImageFont.truetype(FUENTES + ("DejaVuSans-Bold.ttf" if negrita else "DejaVuSans.ttf"), tam)
    except OSError:
        f = ImageFont.load_default()
    x0, y0, x1, y1 = f.getbbox(texto)
    im = Image.new("RGBA", (x1 - x0 + 2, y1 - y0 + 2), (0, 0, 0, 0))
    ImageDraw.Draw(im).text((-x0 + 1, -y0 + 1), texto, font=f, fill=(255, 255, 255, 255))
    return np.asarray(im)


def pegar(img, rgba, x, y):
    """rgba sobre img con su esquina en (x, y); lo que se sale de la imagen no se pinta."""
    H, W = img.shape[:2]
    x0, y0, x1, y1 = max(x, 0), max(y, 0), min(x + rgba.shape[1], W), min(y + rgba.shape[0], H)
    if x1 <= x0 or y1 <= y0:
        return
    r = rgba[y0 - y:y1 - y, x0 - x:x1 - x]
    a = r[:, :, 3:4].astype(np.float32) / 255.0
    img[y0:y1, x0:x1] = (img[y0:y1, x0:x1].astype(np.float32) * (1 - a) + r[:, :, :3] * a).astype(np.uint8)


def caja(img, x0, y0, x1, y1, alfa=0.55):
    x0, y0 = max(x0, 0), max(y0, 0)
    img[y0:y1, x0:x1] = (img[y0:y1, x0:x1].astype(np.float32) * (1 - alfa) + 15 * alfa).astype(np.uint8)


def etiqueta(img, texto, x, y, tam, negrita=False, margen=14, derecha=False):
    """Texto blanco sobre una caja oscura; (x, y) es la esquina de arriba (izquierda, o derecha con derecha)."""
    r = texto_rgba(texto, tam, negrita)
    h, w = r.shape[:2]
    if derecha:
        x -= w + 2 * margen
    caja(img, x, y, x + w + 2 * margen, y + h + 2 * margen)
    pegar(img, r, x + margen, y + margen)


def componer(img, recuadro, rotulo_txt, colocados, n_obj, velocidad):
    H, W = img.shape[:2]
    e = H / 1080.0
    m, mg = int(30 * e), max(2, int(14 * e))
    etiqueta(img, "Simulación MuJoCo" + (f"  ·  ×{velocidad:g}" if velocidad != 1 else ""), m, m, int(38 * e), True, mg)
    etiqueta(img, f"Objetos en su bandeja: {colocados}/{n_obj}", W - m, m, int(34 * e), margen=mg, derecha=True)
    if recuadro is not None:
        rh, rw = recuadro.shape[:2]
        x, y = W - m - rw, H - m - rh
        img[y - 3:y + rh + 3, x - 3:x + rw + 3] = 235
        img[y:y + rh, x:x + rw] = recuadro
        r = texto_rgba("Cámara de la cabeza", int(24 * e))
        caja(img, x - 3, y - r.shape[0] - int(20 * e), x + r.shape[1] + int(20 * e), y - 3)
        pegar(img, r, x + int(8 * e), y - r.shape[0] - int(12 * e))
    if rotulo_txt:
        etiqueta(img, rotulo_txt, m, H - m - int(40 * e) - 2 * mg, int(40 * e), margen=mg)
    return img


class Video:
    """Los fotogramas del video de una grabacion."""

    def __init__(self, grab, montaje, velocidad=2.0, ancho=1920, alto=1080):
        import mujoco
        from sim.camara_virtual import CamaraVirtual
        from sim.escena import Estado
        self.lin = Linea(grab, montaje, velocidad)
        self.vel = velocidad
        m = self.m = grab.m
        m.vis.global_.offwidth = max(ancho, m.vis.global_.offwidth)
        m.vis.global_.offheight = max(alto, m.vis.global_.offheight)
        self.est = Estado(m, grab.info)
        self.r = mujoco.Renderer(m, alto, ancho)
        self.cabeza = CamaraVirtual(m, grab.info, grab.cal)
        lado = int(round(alto * 0.30))
        self.tam_rec = (int(round(lado * 4 / 3)), lado)
        self.libre = mujoco.MjvCamera()
        self.mocap = {c: m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"obj_{c}")]
                      for _, _, c, _, _ in self.lin.caidas}

    def fotograma(self, v):
        import cv2
        import mujoco
        lin, d = self.lin, self.est.d
        _, qpos, mpos, mquat = lin.muestra(v)
        d.qpos[:], d.mocap_pos[:], d.mocap_quat[:] = qpos, mpos, mquat
        for v_abre, v_cae, clase, M0, M1 in lin.caidas:
            i = self.mocap[clase]
            if v_abre <= v < v_cae:                   # los dedos abriendose: aun lo sostienen
                d.mocap_pos[i], d.mocap_quat[i] = M0[:3, 3], quat_de(M0[:3, :3])
            elif v >= v_cae:
                c = caida(M0, M1, v - v_cae)
                if c is not None:
                    d.mocap_pos[i], d.mocap_quat[i] = c
        mujoco.mj_forward(self.m, d)
        self.libre.lookat[:], self.libre.distance, self.libre.azimuth, self.libre.elevation = camara_en(lin.tomas, v)
        self.r.update_scene(d, camera=self.libre)
        img = self.r.render()
        rec = cv2.resize(self.cabeza.color_rgb(self.est), self.tam_rec, interpolation=cv2.INTER_AREA)
        return componer(img, rec, lin.rotulo_en(v), lin.colocados(v), lin.n_obj, self.vel)

    def cerrar(self):
        self.r.close()
        self.cabeza.cerrar()


def renderizar(grab, montaje, salida, velocidad=2.0, fps=30, ancho=1920, alto=1080, log=print):
    vid = Video(grab, montaje, velocidad, ancho, alto)
    os.makedirs(os.path.dirname(os.path.abspath(salida)), exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", f"{ancho}x{alto}", "-r", str(fps), "-i", "-", "-c:v", "libx264", "-preset", "medium",
                           "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", salida], stdin=subprocess.PIPE)
    n = int(vid.lin.duracion * fps)
    t_ini = time.monotonic()
    try:
        for k in range(n):
            ff.stdin.write(vid.fotograma(k / fps).tobytes())
            if k % (fps * 15) == 0:
                log(f"  {k / fps:5.1f} / {vid.lin.duracion:.1f} s de video ({time.monotonic() - t_ini:.0f} s)")
    finally:
        ff.stdin.close()
        ff.wait()
        vid.cerrar()
    return vid.lin


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--velocidad", type=float, default=2.0, help="el video va x esto de la velocidad real")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--ancho", type=int, default=1920)
    ap.add_argument("--alto", type=int, default=1080)
    ap.add_argument("--salida", default=None, help="por defecto datos/videos/sim_nivel2_<fecha>.mp4")
    a = ap.parse_args()
    os.environ.setdefault("MUJOCO_GL", "glfw")
    salida = a.salida or os.path.join(config.ruta_datos(cfg), "videos",
                                      f"sim_nivel2_{datetime.now():%Y%m%d_%H%M}.mp4")
    montaje, objetos = config.cargar_montaje(), config.cargar_objetos()
    print("1/2 la tirada simulada (unos 30 s)", flush=True)
    grab = grabar(cfg, objetos, montaje)
    if grab.motivo != "completado":
        print(f"ATENCION: la tirada acabo en '{grab.motivo}'; el video lo muestra igual")
    print(f"2/2 el video: {len(grab.muestras)} muestras, x{a.velocidad:g}, {a.ancho}x{a.alto} a {a.fps} fps", flush=True)
    lin = renderizar(grab, montaje, salida, a.velocidad, a.fps, a.ancho, a.alto)
    print(f"-> {salida} ({lin.duracion:.0f} s; objetos al final: {grab.finales})")


if __name__ == "__main__":
    main()
