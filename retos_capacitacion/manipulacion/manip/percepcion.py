"""Percepcion (6.2 del PDF): Fotograma -> Escena con los objetos en el marco del torso.

1. Solo se mira la profundidad que cae sobre la zona de recogida (ampliada): se proyecta la zona a la imagen
   de profundidad y se recorta, sin submuestrear.
2. Los puntos pasan al torso con la calibracion; la altura es la distancia al plano de la mesa, que se
   reajusta en cada escena alrededor del calibrado (si el cuerpo se balancea, la mesa "se mueve").
3. Se quita lo que sobresale pero no es objeto: el propio brazo (cilindros alrededor del esqueleto que da la
   FK con la q medida) y lo que no llega a altura_min (alfombrillas, ArUco, cinta de las dianas).
4. Lo que queda se rasteriza en una vista desde arriba y se agrupa en manchas; las pequenas son ruido (los
   reflejos del tablero negro dan manchas de 15-25 mm de alto pero de pocos cm2).
5. Identidad: color (los puntos de la mancha proyectados al color, por el extrinseco de fabrica) y tamano.
   Pose segun la forma:
     esfera    ajuste de una esfera de radio conocido apoyada en la mesa (la profundidad ve la pelota
               ~2 cm mas baja: con el radio conocido no importa)
     cilindro  centro de la cara de arriba (de pie)
     caja      rectangulo de la cara de arriba: centro y yaw (modulo la simetria: 90 el cubo, 180 el prisma)
6. Movido: cada clase solo aparece una vez; se compara con la escena anterior.
Debe funcionar igual sobre datos grabados que en vivo (seccion 5): solo usa el Fotograma y sus metadatos.
"""

import math
import time

import cv2
import numpy as np

from .fuentes import T_color_prof, intrinsecos, prof_en_metros
from .geometria import Intrinsecos, Plano, aplicar, envolver_simetria, inv
from .mensajes import Escena, Objeto


def _altura_clase(o):
    m = o["medidas_m"]
    return m[0] if o["forma"] == "esfera" else (m[1] if o["forma"] == "cilindro" else m[2])


def _planta_clase(o):
    """(lado largo, lado corto) en planta."""
    m = o["medidas_m"]
    if o["forma"] in ("esfera", "cilindro"):
        return m[0], m[0]
    return max(m[0], m[1]), min(m[0], m[1])


def _dist_segmentos(P, A, B):
    """Distancia de cada punto de P (N, 3) al segmento AB."""
    AB = B - A
    t = np.clip(((P - A) @ AB) / max(float(AB @ AB), 1e-12), 0, 1)
    return np.linalg.norm(P - (A + t[:, None] * AB), axis=1)


def ajustar_esfera_apoyada(P, plano: Plano, radio, c0):
    """Centro de una esfera de radio conocido apoyada en el plano: minimiza sum(|p - c| - r)^2 con c en
    el plano paralelo a radio de altura. Gauss-Newton en las dos coordenadas del plano."""
    o, ex, ey = plano.base()
    c = np.asarray(c0, float)
    uv = np.array([(c - o) @ ex, (c - o) @ ey])
    for _ in range(20):
        c = o + uv[0] * ex + uv[1] * ey + radio * plano.n
        d = P - c
        n = np.linalg.norm(d, axis=1) + 1e-9
        r = n - radio
        J = -np.stack([d @ ex, d @ ey], 1) / n[:, None]
        paso = np.linalg.lstsq(J, -r, rcond=None)[0]
        uv += paso
        if np.linalg.norm(paso) < 1e-5:
            break
    c = o + uv[0] * ex + uv[1] * ey + radio * plano.n
    rms = float(np.sqrt(np.mean((np.linalg.norm(P - c, axis=1) - radio) ** 2)))
    return c, rms


class Percepcion:
    def __init__(self, cfg, montaje, objetos, cal, cin=None):
        self.p = cfg["percepcion"]
        self.montaje = montaje
        self.objetos = objetos["objetos"]
        self.cal = cal
        self.cin = cin
        pl = montaje["mesa"]["plano"]
        self.plano_cal = Plano(pl["n"], pl["d"])
        self.zona = montaje["zona_recogida"]
        self._previa = None

    # --- utilidades -------------------------------------------------------------------------------
    def _recorte(self, intr: Intrinsecos, T_torso_prof):
        """(v0, v1, u0, u1) de la imagen de profundidad que cubre la zona ampliada hasta altura_max."""
        mg = self.p["margen_zona_m"]
        x0, x1 = self.zona["x"][0] - mg, self.zona["x"][1] + mg
        y0, y1 = self.zona["y"][0] - mg, self.zona["y"][1] + mg
        esquinas = []
        for x in (x0, x1):
            for y in (y0, y1):
                z = -(self.plano_cal.n[0] * x + self.plano_cal.n[1] * y + self.plano_cal.d) / self.plano_cal.n[2]
                for h in (0.0, self.p["altura_max_m"]):
                    esquinas.append([x, y, z + h])
        uv = intr.proyectar(aplicar(inv(T_torso_prof), np.array(esquinas)))
        u0, v0 = np.floor(np.nanmin(uv, 0)).astype(int) - 2
        u1, v1 = np.ceil(np.nanmax(uv, 0)).astype(int) + 2
        return max(v0, 0), min(v1, intr.alto), max(u0, 0), min(u1, intr.ancho)

    def _nube_torso(self, f):
        prof = prof_en_metros(f)
        intr = intrinsecos(f, "prof")
        v0, v1, u0, u1 = self._recorte(intr, self.cal.T_torso_prof)
        z = prof[v0:v1, u0:u1]
        v, u = np.mgrid[v0:v1, u0:u1]
        ok = z > 0
        z, u, v = z[ok], u[ok], v[ok]
        P = np.stack([(u - intr.cx) / intr.fx * z, (v - intr.cy) / intr.fy * z, z], 1)
        return P, aplicar(self.cal.T_torso_prof, P)

    def _plano(self, Pt):
        if not self.p.get("plano_vivo", True):
            return self.plano_cal
        h = self.plano_cal.altura(Pt)
        cerca = np.abs(h) < 0.015
        if cerca.sum() < 500:
            return self.plano_cal
        pl, frac, _ = Plano.ajustar(Pt[cerca], umbral=0.004, iteraciones=120, cerca_de=self.plano_cal, tol_ang_deg=3.0)
        if pl is None or frac < 0.5 or abs(pl.d - self.plano_cal.d) > 0.02:
            return self.plano_cal
        return pl

    def _fuera_de_bandejas(self, Pt, margen=0.01):
        """Fuera de las bandejas de montaje.yaml (con `margen`): sus paredes de 4 cm caen dentro de la zona
        ampliada y, pegadas a un objeto, lo deformaban (o salian como objeto desconocido)."""
        from .montaje import bandeja_real
        fuera = np.ones(len(Pt), bool)
        for b in self.montaje.get("bandejas", {}).values():
            a = b["hueco_m"] / 2 + b["pared_m"] + margen
            fuera &= ~np.all(np.abs(Pt[:, :2] - bandeja_real(b)[0]) <= a, axis=1)
        return fuera

    def _mascara_robot(self, Pt, q_brazos):
        if q_brazos is None or self.cin is None:
            return np.ones(len(Pt), bool)
        esq = self.cin.esqueleto(q_brazos.get("izq"), q_brazos.get("der"))
        fuera = np.ones(len(Pt), bool)
        for pts in esq.values():
            for (a, b), r in zip(((0, 1), (1, 2), (2, 3), (3, 4)), (0.07, 0.065, 0.075, 0.06)):
                fuera &= _dist_segmentos(Pt, pts[a], pts[b]) > r
        return fuera

    # --- clasificacion ------------------------------------------------------------------------------
    def _color(self, f, P_prof):
        """HSV (N, 3) de los puntos en la imagen de color (None si no hay color)."""
        if f.color is None:
            return None
        ic = intrinsecos(f, "color")
        uv = ic.proyectar(aplicar(T_color_prof(f.meta), P_prof))
        ok = np.isfinite(uv).all(1)
        uv = np.round(uv[ok]).astype(int)
        dentro = (uv[:, 0] >= 0) & (uv[:, 0] < ic.ancho) & (uv[:, 1] >= 0) & (uv[:, 1] < ic.alto)
        uv = uv[dentro]
        if len(uv) == 0:
            return None
        hsv = cv2.cvtColor(f.color, cv2.COLOR_BGR2HSV)
        return hsv[uv[:, 1], uv[:, 0]]

    def _puntuar(self, clave, o, hsv, h_max, largo, corto):
        if hsv is not None and len(hsv):
            lo, hi = np.array(o["color"]["hsv_min"]), np.array(o["color"]["hsv_max"])
            fr_color = float(np.mean(np.all((hsv >= lo) & (hsv <= hi), axis=1)))
        else:
            fr_color = 0.5
        alto = _altura_clase(o)
        pl, pc = _planta_clase(o)
        # la profundidad ve las esferas y los bordes mas bajos y pequenos: tolerancias asimetricas
        s_alto = math.exp(-((max(0.0, h_max - alto) / 0.015) ** 2 + (max(0.0, 0.55 * alto - h_max) / 0.015) ** 2))
        s_planta = math.exp(-(((largo - pl) / 0.03) ** 2 + ((corto - pc) / 0.03) ** 2) / 2)
        return fr_color * (0.3 + 0.7 * s_alto * s_planta), fr_color, s_alto * s_planta

    # --- principal -----------------------------------------------------------------------------------
    def procesar(self, f, q_brazos=None):
        """q_brazos: {"izq": q7, "der": q7} medidas, para quitar el brazo de la escena."""
        return self.procesar_varios([f], q_brazos)

    def procesar_varios(self, fotogramas, q_brazos=None):
        """Varios fotogramas seguidos con la camara y la mesa quietas -> una Escena: cada clase vista en al menos la
        mitad, con la mediana de su posicion y el yaw del fotograma mas central (modulo la simetria). Con uno solo,
        el yaw de la tapa del cubo iba de -12 a +21 grados con el cubo quieto (RESULTADOS §57)."""
        t0 = time.monotonic()
        detectados = [self._detectar(f, q_brazos) for f in fotogramas]
        objetos = detectados[0][0] if len(detectados) == 1 else self._fusionar([d[0] for d in detectados])
        plano = detectados[-1][1]
        self._marcar_movidos(objetos)
        for i, o in enumerate(objetos):
            o.id = i
        esc = Escena(t=fotogramas[-1].t_rx, objetos=objetos, ms=1000 * (time.monotonic() - t0), plano=(plano.n, plano.d))
        self._previa = esc
        return esc

    def _fusionar(self, listas):
        n, sal = len(listas), []
        for clase in sorted({o.clase for lista in listas for o in lista if o.clase != "desconocido"}):
            vistos = [o for lista in listas for o in lista if o.clase == clase]
            if 2 * len(vistos) < n:
                continue
            # si la mayoria tiene la tapa en color, solo esos (la profundidad sola cae hasta 2 cm al lado)
            con_color = [o for o in vistos if o.por_color]
            base = con_color if 2 * len(con_color) > len(vistos) else vistos
            p = np.median([o.p for o in base], axis=0)
            sim = math.radians(self.objetos.get(clase, {}).get("simetria_deg", 0))
            yaws = [o.yaw for o in base]
            yaw = min(yaws, key=lambda y: sum(abs(envolver_simetria(z - y, sim)) for z in yaws)) if sim > 0 else 0.0
            ref = min(base, key=lambda o: float(np.linalg.norm(o.p - p)))
            sal.append(Objeto(id=-1, clase=clase, p=p, yaw=float(yaw), medidas=np.median([o.medidas for o in vistos], axis=0),
                              confianza=float(np.median([o.confianza for o in vistos])), n_puntos=ref.n_puntos,
                              pixel=ref.pixel, por_color=base is con_color))
        # lo desconocido no se puede emparejar entre fotogramas: el del ultimo
        return sal + [o for o in listas[-1] if o.clase == "desconocido"]

    def _detectar(self, f, q_brazos):
        """Un fotograma -> (objetos, plano de la mesa), sin comparar con la escena anterior."""
        P_prof, Pt = self._nube_torso(f)
        plano = self._plano(Pt)
        h = plano.altura(Pt)
        mg = self.p["margen_zona_m"]
        sel = ((h > self.p["altura_min_m"]) & (h < self.p["altura_max_m"])
               & (Pt[:, 0] > self.zona["x"][0] - mg) & (Pt[:, 0] < self.zona["x"][1] + mg)
               & (Pt[:, 1] > self.zona["y"][0] - mg) & (Pt[:, 1] < self.zona["y"][1] + mg))
        sel &= self._mascara_robot(Pt, q_brazos)
        sel &= self._fuera_de_bandejas(Pt)
        Pt_todo, h_todo = Pt, h
        P_prof, Pt, h = P_prof[sel], Pt[sel], h[sel]
        objetos = []
        if len(Pt):
            res = self.p["resolucion_m"]
            x0, y0 = self.zona["x"][0] - mg, self.zona["y"][0] - mg
            nx = int(math.ceil((self.zona["x"][1] - self.zona["x"][0] + 2 * mg) / res)) + 1
            ny = int(math.ceil((self.zona["y"][1] - self.zona["y"][0] + 2 * mg) / res)) + 1
            ix = np.clip(((Pt[:, 0] - x0) / res).astype(int), 0, nx - 1)
            iy = np.clip(((Pt[:, 1] - y0) / res).astype(int), 0, ny - 1)
            # se agrupa por encima de altura_nucleo: los bordes de profundidad dejan puntos a media altura
            # alrededor de cada objeto (+2 cm por lado) que unian objetos casi en contacto (2026-10-09)
            nucleo = h > self.p.get("altura_nucleo_m", 0.02)
            ocup = np.zeros((nx, ny), np.uint8)
            ocup[ix[nucleo], iy[nucleo]] = 1
            ocup = cv2.morphologyEx(ocup, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            n, lab = cv2.connectedComponents(ocup, connectivity=8)
            etiqueta = lab[ix, iy]
            for k in range(1, n):
                celdas = np.argwhere(lab == k)
                if len(celdas) * res * res < self.p["area_min_m2"]:
                    continue
                m = (etiqueta == k) & nucleo
                o = self._objeto(f, P_prof[m], Pt[m], h[m], plano)
                if o is not None:
                    objetos.append(o)
        # las cajas con tapa en color que la profundidad no agrupo (§75)
        por_tapa = self._tapas_sin_profundidad(f, Pt_todo, h_todo, plano, {o.clase for o in objetos if o.clase != "desconocido"})
        if por_tapa:
            objetos = [o for o in objetos if o.clase != "desconocido"
                       or min(np.linalg.norm(o.p[:2] - t.p[:2]) for t in por_tapa) > 0.05] + por_tapa
        return self._una_por_clase(objetos), plano

    def _tapa_en_ir(self, f, P):
        """Fraccion de los puntos P (torso) de una tapa que salen saturados (>= 200) en el IR: la tapa naranja del prisma,
        muy clara con el emisor (la profundidad, ahi, falla). 0 sin IR."""
        if getattr(f, "ir", None) is None:
            return 0.0
        ii = intrinsecos(f, "ir")
        uv = ii.proyectar(aplicar(inv(self.cal.T_torso_prof), P))
        uv = uv[np.isfinite(uv).all(1)]
        uv = np.round(uv).astype(int)
        uv = uv[(uv[:, 0] >= 0) & (uv[:, 0] < ii.ancho) & (uv[:, 1] >= 0) & (uv[:, 1] < ii.alto)]
        if len(uv) == 0:
            return 0.0
        return float(np.mean(f.ir[uv[:, 1], uv[:, 0]] >= 200))

    def _tapas_sin_profundidad(self, f, Pt, h, plano, vistas):
        """Las cajas con `tapa_color` que la profundidad no agrupo: el prisma naranja (su tapa, muy clara en el IR, da
        de 0 a 84 mm en vez de 40 y a veces no pasa de altura_nucleo_m, §75). Sus manchas en la imagen de color,
        retroproyectadas a la altura de su tapa: del tamano de la tapa (+-25 %), dentro de la zona ampliada, fuera de
        las bandejas y con la profundidad por encima de altura_min_m en parte de su huella (algo hay). -> [Objeto]"""
        if f.color is None:
            return []
        sal = []
        ic = intrinsecos(f, "color")
        T_t_c = self.cal.T_torso_color
        o_cam = T_t_c[:3, 3]
        hsv = None
        mg = self.p["margen_zona_m"]
        for clave, o in self.objetos.items():
            t = o.get("tapa_color")
            if not t or o["forma"] != "caja" or clave in vistas:
                continue
            if hsv is None:
                hsv = cv2.cvtColor(f.color, cv2.COLOR_BGR2HSV)
            if "hsv_min" in t:
                mascara = cv2.inRange(hsv, np.array(t["hsv_min"]), np.array(t["hsv_max"]))
            else:
                mascara = ((hsv[..., 2] >= t.get("v_min", 235)) & (hsv[..., 1] <= t.get("s_max", 60))).astype(np.uint8)
            n, lab, st, _ = cv2.connectedComponentsWithStats((mascara > 0).astype(np.uint8), connectivity=8)
            alto = _altura_clase(o)
            largo, corto = _planta_clase(o)
            lo, hi = t.get("tamano_rel", [0.75, 1.25])
            for k in range(1, n):
                if st[k, cv2.CC_STAT_AREA] < t.get("px_min", 150):
                    continue
                vv, uu = np.nonzero(lab == k)
                d = ic.rayos(np.stack([uu, vv], 1).astype(float)) @ T_t_c[:3, :3].T
                esc_ = (alto - plano.altura(o_cam[None])[0]) / (d @ plano.n)
                P = o_cam + esc_[:, None] * d
                (cx, cy), (w, l), ang = cv2.minAreaRect(P[:, :2].astype(np.float32))
                if not (lo * largo <= max(w, l) <= hi * largo and lo * corto <= min(w, l) <= hi * corto):
                    continue
                if not (self.zona["x"][0] - mg < cx < self.zona["x"][1] + mg and self.zona["y"][0] - mg < cy < self.zona["y"][1] + mg):
                    continue
                c = plano.proyectar(np.array([cx, cy, float(np.mean(P[:, 2]))])) + alto / 2 * plano.n
                if not self._fuera_de_bandejas(c[None])[0]:
                    continue
                yaw = math.radians(ang) if w >= l else math.radians(ang + 90)
                # algo hay: la huella de la tapa (con 1 cm alrededor), en la profundidad, por encima de la mesa; o, si la
                # profundidad no la ve (en una tirada la tapa del prisma salio a la altura de la mesa: 7 % por encima de
                # 12 mm, como la mesa sola, hasta 7.9 %), la tapa saturada en el IR (92-93 %; la mesa, brillo 36), §75
                dxy = Pt[:, :2] - [cx, cy]
                u = np.array([math.cos(yaw), math.sin(yaw)])
                dentro = (np.abs(dxy @ u) < largo / 2 + 0.01) & (np.abs(dxy @ [-u[1], u[0]]) < corto / 2 + 0.01)
                encima = dentro & (h > self.p["altura_min_m"]) & (h < self.p["altura_max_m"])
                if encima.sum() >= max(30, 0.1 * dentro.sum()):
                    h_max = float(np.percentile(h[encima], 97))
                    s_bajo = math.exp(-(max(0.0, 0.55 * alto - h_max) / 0.015) ** 2)
                elif self._tapa_en_ir(f, P) >= t.get("ir_saturado_min", 0.6):
                    h_max, s_bajo = alto, 4.0 / 7.0          # sin la altura: confianza 0.7
                else:
                    continue
                pix = ic.proyectar(aplicar(inv(T_t_c), c[None]))[0]
                sal.append(Objeto(id=-1, clase=clave, p=c, yaw=float(envolver_simetria(yaw, math.radians(o.get("simetria_deg", 0)))),
                                  medidas=np.array([max(w, l), min(w, l), h_max]), confianza=float(min(1.0, 0.3 + 0.7 * s_bajo)),
                                  n_puntos=int(encima.sum()),
                                  pixel=(int(round(pix[0])), int(round(pix[1]))) if np.isfinite(pix).all() else None,
                                  por_color=True))
        return sal

    def _tapa_en_color(self, f, p, o, plano):
        """La tapa de una caja en la imagen de color (objetos.yaml `tapa_color`): la mancha clara mas cercana a donde
        la puso la profundidad, retroproyectada al plano de la tapa (la mesa subida el alto del objeto), y su
        rectangulo minimo -> (centro del objeto en el torso, yaw del lado largo) o None. La profundidad del D435i en
        el cubo blanco tiene un hueco en la tapa y alturas de 2 a 6 cm: el centro de sus puntos mas altos caia hasta
        2.2 cm al lado del real; el color esta calibrado con el brazo (RESULTADOS §59)."""
        t = o.get("tapa_color")
        if t is None or f.color is None:
            return None
        ic = intrinsecos(f, "color")
        alto = _altura_clase(o)
        largo, corto = _planta_clase(o)
        T_t_c = self.cal.T_torso_color
        centro_tapa = plano.proyectar(p) + alto * plano.n
        Pc = aplicar(inv(T_t_c), centro_tapa[None])[0]
        if Pc[2] <= 0:
            return None
        u0, v0 = ic.proyectar(Pc[None])[0]
        radio = int(math.ceil((math.hypot(largo, corto) / 2 + t.get("busqueda_m", 0.04)) * ic.fx / Pc[2]))
        a0, a1 = max(0, int(u0) - radio), min(ic.ancho, int(u0) + radio + 1)
        b0, b1 = max(0, int(v0) - radio), min(ic.alto, int(v0) + radio + 1)
        if a1 - a0 < 3 or b1 - b0 < 3:
            return None
        hsv = cv2.cvtColor(f.color[b0:b1, a0:a1], cv2.COLOR_BGR2HSV)
        if "hsv_min" in t:      # una tapa de color: su rango, con el brillo de la cara iluminada (el prisma, §74)
            mascara = (cv2.inRange(hsv, np.array(t["hsv_min"]), np.array(t["hsv_max"])) > 0).astype(np.uint8)
        else:                   # blanca: muy clara y sin color (el cubo, §59)
            mascara = ((hsv[..., 2] >= t.get("v_min", 235)) & (hsv[..., 1] <= t.get("s_max", 60))).astype(np.uint8)
        n, lab, st, cen = cv2.connectedComponentsWithStats(mascara, connectivity=8)
        cands = [k for k in range(1, n) if st[k, cv2.CC_STAT_AREA] >= t.get("px_min", 150)]
        if not cands:
            return None
        k = min(cands, key=lambda k: math.hypot(cen[k][0] + a0 - u0, cen[k][1] + b0 - v0))
        vv, uu = np.nonzero(lab == k)
        uv = np.stack([uu + a0, vv + b0], 1).astype(float)
        d = ic.rayos(uv) @ T_t_c[:3, :3].T
        o_cam = T_t_c[:3, 3]
        s = (alto - plano.altura(o_cam[None])[0]) / (d @ plano.n)
        P = o_cam + s[:, None] * d
        (cx, cy), (w, l), ang = cv2.minAreaRect(P[:, :2].astype(np.float32))
        # una tapa entera y del tamano del objeto, y cerca de la profundidad (si no, es otra cosa o esta cortada)
        lo, hi = t.get("tamano_rel", [0.75, 1.25])
        if not (lo * largo <= max(w, l) <= hi * largo and lo * corto <= min(w, l) <= hi * corto):
            return None
        c = plano.proyectar(np.array([cx, cy, float(np.mean(P[:, 2]))])) + alto / 2 * plano.n
        if np.linalg.norm(c[:2] - p[:2]) > t.get("desvio_max_m", 0.04):
            return None
        return c, (math.radians(ang) if w >= l else math.radians(ang + 90))

    def _objeto(self, f, P_prof, Pt, h, plano):
        h_max = float(np.percentile(h, 97))
        # planta con la mitad de arriba: sin el halo de los bordes de profundidad
        alto = Pt[h > 0.5 * h_max]
        (cx, cy), (w, l), ang = cv2.minAreaRect((alto if len(alto) >= 5 else Pt)[:, :2].astype(np.float32))
        largo, corto = max(w, l), min(w, l)
        hsv = self._color(f, P_prof[h > 0.4 * h_max])
        mejor = None
        for clave, o in self.objetos.items():
            s, fc, sg = self._puntuar(clave, o, hsv, h_max, largo, corto)
            if mejor is None or s > mejor[1]:
                mejor = (clave, s, fc, sg)
        clave, conf, fr_color, s_geo = mejor
        tapa_previa = None
        # color Y tamano: la bola roja de espuma (fuera del reto) daba "prisma" por color y el borde de una
        # bandeja, "cilindro" (2026-10-09); ninguno de los dos tiene el tamano
        if fr_color < self.p.get("color_min", 0.35) or s_geo < self.p.get("geometria_min", 0.3):
            clave = "desconocido"
            # una caja con su tapa en color puede salir mal por la profundidad (el prisma naranja, §74): si alguna
            # tiene su tapa ahi (color y tamano, a menos de desvio_max_m), es ella
            c0 = plano.proyectar(np.array([cx, cy, Pt[:, 2].mean()]))
            for k, ob in self.objetos.items():
                if ob.get("tapa_color") and ob["forma"] == "caja":
                    tapa_previa = self._tapa_en_color(f, c0 + _altura_clase(ob) / 2 * plano.n, ob, plano)
                    if tapa_previa is not None:
                        clave = k
                        break
        o = self.objetos.get(clave)
        p, yaw, por_color = None, 0.0, False
        if o is None:
            p = np.array([cx, cy, 0.0])
            p = plano.proyectar(np.array([cx, cy, Pt[:, 2].mean()])) + h_max / 2 * plano.n
        elif o["forma"] == "esfera":
            r = o["medidas_m"][0] / 2
            c0 = plano.proyectar(Pt.mean(0)) + r * plano.n
            p, _ = ajustar_esfera_apoyada(Pt, plano, r, c0)
        elif o["forma"] == "cilindro":
            arriba = Pt[h > h_max - 0.012]
            c = arriba.mean(0) if len(arriba) >= 10 else np.array([cx, cy, 0.0])
            p = plano.proyectar(c) + _altura_clase(o) / 2 * plano.n
        else:
            arriba = Pt[h > h_max - 0.012]
            if len(arriba) >= 10:
                (ax, ay), (aw, al), aang = cv2.minAreaRect(arriba[:, :2].astype(np.float32))
                lado_largo = math.radians(aang) if aw >= al else math.radians(aang + 90)
                c = np.array([ax, ay, arriba[:, 2].mean()])
            else:
                lado_largo = math.radians(ang) if w >= l else math.radians(ang + 90)
                c = np.array([cx, cy, Pt[:, 2].mean()])
            p = plano.proyectar(c) + _altura_clase(o) / 2 * plano.n
            yaw = envolver_simetria(lado_largo, math.radians(o.get("simetria_deg", 0)))
            tapa = tapa_previa if tapa_previa is not None else self._tapa_en_color(f, p, o, plano)
            if tapa is not None:
                p, yaw = tapa[0], envolver_simetria(tapa[1], math.radians(o.get("simetria_deg", 0)))
                por_color = True
                # la tapa, en su color y de su tamano (+-25 %), confirma la clase: la confianza, con el color y la
                # planta de la tapa, y de la profundidad solo que no sea mas bajo. En el prisma naranja (tapa muy clara
                # en el IR) el p97 de la altura va de 34 a 84 mm y la mediana es ~20 (el real, 40): su planta, su
                # color y su alto salian mal (§74)
                s_bajo = math.exp(-(max(0.0, 0.55 * _altura_clase(o) - h_max) / 0.015) ** 2)
                conf = max(conf, 0.3 + 0.7 * s_bajo)
        pix = None
        if f.color is not None:
            ic = intrinsecos(f, "color")
            uv = ic.proyectar(aplicar(inv(self.cal.T_torso_color), p))[0]
            if np.isfinite(uv).all():
                pix = (int(round(uv[0])), int(round(uv[1])))
        return Objeto(id=-1, clase=clave, p=np.asarray(p, float), yaw=float(yaw),
                      medidas=np.array([largo, corto, h_max]), confianza=float(min(1.0, conf)),
                      n_puntos=int(len(Pt)), pixel=pix, por_color=por_color)

    def _una_por_clase(self, objetos):
        """Cada objeto conocido esta una sola vez: si dos manchas dicen ser lo mismo, gana la de mas confianza
        y la otra pasa a desconocido (o se descarta si es poca cosa)."""
        por = {}
        sal = []
        for o in sorted(objetos, key=lambda o: -o.confianza):
            if o.clase != "desconocido" and o.clase in por:
                o.clase, o.confianza = "desconocido", 0.0
            if o.clase != "desconocido":
                por[o.clase] = o
            sal.append(o)
        return sal

    def _marcar_movidos(self, objetos):
        if self._previa is None:
            return
        prev = {o.clase: o for o in self._previa.objetos if o.clase != "desconocido"}
        for o in objetos:
            q = prev.get(o.clase)
            if q is None:
                continue
            sim = math.radians(self.objetos.get(o.clase, {}).get("simetria_deg", 0))
            dyaw = abs(envolver_simetria(o.yaw - q.yaw, sim)) if sim > 0 else 0.0
            o.movido = (np.linalg.norm(o.p[:2] - q.p[:2]) > self.p["movido_m"]
                        or math.degrees(dyaw) > self.p["movido_deg"])


def dibujar(img, esc: Escena, montaje=None, cal=None, intr=None):
    """Copia de la imagen de color con los objetos (clase, confianza) y, si se da, la zona de recogida."""
    out = img.copy()
    if montaje is not None and cal is not None and intr is not None:
        z = montaje["zona_recogida"]
        pl = Plano(montaje["mesa"]["plano"]["n"], montaje["mesa"]["plano"]["d"])
        pts = []
        for x, y in ((z["x"][0], z["y"][0]), (z["x"][1], z["y"][0]), (z["x"][1], z["y"][1]), (z["x"][0], z["y"][1])):
            zz = -(pl.n[0] * x + pl.n[1] * y + pl.d) / pl.n[2]
            pts.append([x, y, zz])
        uv = intr.proyectar(aplicar(inv(cal.T_torso_color), np.array(pts)))
        cv2.polylines(out, [np.round(uv).astype(np.int32)], True, (0, 255, 0), 1)
    for o in esc.objetos:
        if o.pixel is None:
            continue
        cv2.circle(out, o.pixel, 4, (0, 0, 255), -1)
        txt = f"{o.clase} {o.confianza:.2f}" + (" MOVIDO" if o.movido else "")
        cv2.putText(out, txt, (o.pixel[0] + 6, o.pixel[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    return out
