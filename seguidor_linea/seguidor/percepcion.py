"""Percepcion (Hito 2): Fotograma -> MedidaLinea.

1. Vista desde arriba del suelo a `resolucion_m` por pixel con la geometria calibrada y la postura
   LENTA del torso (roll/pitch de la IMU suavizados, nunca los del paso: RESULTADOS.md 4). Ahi la
   cinta mide siempre `ancho_cinta_m`, este cerca o lejos. Antes, mediana 3x3: quita los puntos del
   emisor si estuviera encendido.
2. Filtro de franja del ancho de la cinta con contraste a los dos lados (un borde no cuenta). La polaridad (cinta oscura o clara) se fija o, con 'auto', se prueban las dos en cada
   fotograma y se queda la de mas confianza.
3. En cada fila, la columna de mas respuesta; vale si pasa el umbral: adaptativo (k veces el ruido
   robusto de su franja) o fijo (niveles de gris), para comparar (6.2.2).
4. Ajuste robusto de y = a + b x (+ c x^2 si hay recorrido): desplazamiento a, angulo atan(b),
   curvatura y el punto adelantado. Confianza = filas con linea x contraste x residuo.
5. Barra de fin y esquina: filas cubiertas por cinta transversal alrededor del final de la linea, a
   los dos lados (barra) o a uno solo (esquina, nivel 4).

Todo en el marco de la camara (x adelante, y a la izquierda, origen en su vertical): el robot avanza
~7.5 grados a la izquierda de ese eje y eso lo corrige el control, no la percepcion.
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .geometria import Intrinsecos, ModeloSuelo
from .mensajes import Fotograma, MedidaLinea

POSTURA_TOL_RAD = math.radians(0.1)   # recalcular la vista desde arriba si la postura cambia mas


@dataclass
class Detalle:
    """Lo que hay debajo de la ultima medida, para dibujarla y depurar."""
    bev: Optional[np.ndarray] = None          # vista desde arriba (uint8)
    xs: np.ndarray = field(default_factory=lambda: np.zeros(0))   # puntos candidatos (m)
    ys: np.ndarray = field(default_factory=lambda: np.zeros(0))
    dentro: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    coef: Tuple[float, float, float] = (math.nan, math.nan, 0.0)   # a, b, c
    filas_barra: List[Tuple[float, float]] = field(default_factory=list)   # (x_cerca, x_lejos)
    polaridad: int = 0
    sigma: List[float] = field(default_factory=list)
    x_fin: float = math.nan                    # x del ultimo punto de linea


def _franja(img, ancho, horizontal):
    """Respuesta a una franja de `ancho` px con contraste a LOS DOS LADOS, para cada polaridad:
    (clara, oscura) = (min(C - F1, C - F2), min(F1 - C, F2 - C)), con C la media en la franja y F1,
    F2 las de los flancos a +-`ancho`. Una cinta da el contraste entero; un borde (puerta, mueble,
    sombra) solo tiene contraste a un lado y da ~0 (2026-10-05: el canto de una puerta pasaba por
    linea con el filtro de flancos promediados)."""
    k = (ancho, 1) if horizontal else (1, ancho)
    eje = 1 if horizontal else 0
    c = cv2.blur(img, k)
    f1 = np.roll(c, ancho, axis=eje)
    f2 = np.roll(c, -ancho, axis=eje)
    d1, d2 = c - f1, c - f2
    return np.minimum(d1, d2), np.minimum(-d1, -d2), d1


def _robusta(x):
    """Desviacion robusta (1.4826 MAD)."""
    if x.size == 0:
        return math.nan
    return 1.4826 * float(np.median(np.abs(x - np.median(x))))


class Percepcion:
    def __init__(self, cfg_percepcion: dict, intr: Intrinsecos, geometria: dict):
        p = cfg_percepcion
        self.p = p
        self.res = p["resolucion_m"]
        self.x_min, self.x_max, self.y_max = p["x_min_m"], p["x_max_m"], p["y_max_m"]
        r = math.radians
        # yaw 0: el marco es el de la camara (el yaw estatico no es fiable, RESULTADOS.md 1)
        self.modelo = ModeloSuelo(intr, geometria["altura_m"], r(geometria["inclinacion_deg"]),
                                  r(geometria.get("roll_deg", 0.0)), 0.0,
                                  r(geometria.get("imu_roll_ref_deg", 0.0)), r(geometria.get("imu_pitch_ref_deg", 0.0)))
        self.ancho_px = max(3, int(round(p["ancho_cinta_m"] / self.res)) | 1)
        nx = int(round((self.x_max - self.x_min) / self.res))
        ny = int(round(2 * self.y_max / self.res))
        self.x_filas = self.x_max - self.res * np.arange(nx)          # x de cada fila (arriba = lejos)
        self.y_cols = self.y_max - self.res * np.arange(ny)           # y de cada columna (izquierda = +y)
        self._mapas = None
        self._postura = (None, None)
        self.polaridad = {"oscura": -1, "clara": +1}.get(p.get("polaridad", "auto"), 0)
        self._auto = self.polaridad == 0
        self.detalle = Detalle()

    # --- vista desde arriba ------------------------------------------------------------------
    def _mapas_para(self, roll, pitch):
        """Mapas de la vista desde arriba; se recalculan solo si la postura cambio mas de 0.1 grados."""
        if self._mapas is not None:
            r0, p0 = self._postura
            if roll is None and r0 is None:
                return self._mapas
            if roll is not None and r0 is not None and abs(roll - r0) < POSTURA_TOL_RAD \
                    and abs(pitch - p0) < POSTURA_TOL_RAD:
                return self._mapas
        mu, mv = self.modelo.mapa_vista_superior(self.x_min, self.x_max, self.y_max, self.res, roll, pitch)
        mu, mv = mu[:len(self.x_filas), :len(self.y_cols)], mv[:len(self.x_filas), :len(self.y_cols)]
        w, h = self.modelo.intr.ancho, self.modelo.intr.alto
        valido = (mu >= 0) & (mu <= w - 1) & (mv >= 0) & (mv <= h - 1)
        self._mapas = (mu, mv, valido)
        self._postura = (roll, pitch)
        return self._mapas

    def vista_superior(self, ir, imu_roll=None, imu_pitch=None):
        mu, mv, valido = self._mapas_para(imu_roll, imu_pitch)
        bev = cv2.remap(cv2.medianBlur(ir, 3), mu, mv, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        return bev, valido

    # --- medida ----------------------------------------------------------------------------
    def procesar(self, f: Fotograma, imu_roll=None, imu_pitch=None) -> MedidaLinea:
        t0 = time.perf_counter()
        p = self.p
        bev, valido = self.vista_superior(f.ir, imu_roll, imu_pitch)
        img = bev.astype(np.float32)
        clara, oscura, ruido = _franja(img, self.ancho_px, horizontal=True)
        # los bordes de la zona valida dan respuestas falsas: fuera; y suavizado a lo largo de x
        borde = cv2.erode(valido.astype(np.uint8), np.ones((3, 3 * self.ancho_px), np.uint8)) > 0
        resp = {+1: np.where(borde, cv2.blur(clara, (1, 3)), 0.0), -1: np.where(borde, cv2.blur(oscura, (1, 3)), 0.0)}
        filas_ok = borde.any(axis=1)

        # umbral por franja (el ruido robusto es el mismo para las dos polaridades)
        umbral_fila = np.full(len(self.x_filas), np.inf)
        sigmas = []
        for x0, x1 in p["franjas_m"]:
            filas = (self.x_filas >= x0) & (self.x_filas < x1)
            sig = _robusta(ruido[filas][borde[filas]])
            sigmas.append(sig)
            if p.get("umbral", "adaptativo") == "fijo":
                u = p["umbral_fijo"]
            else:
                u = max(p["umbral_sigmas"] * sig, p["umbral_minimo"]) if np.isfinite(sig) else np.inf
            umbral_fila[filas] = u

        # polaridad: con 'auto' se prueban las dos y se queda la de mas confianza, con una pequena
        # preferencia por la anterior. Asi no depende de una evidencia acumulada que se puede dar la
        # vuelta mientras no se ve la linea (2026-10-05, fuera de la pista)
        mejor = None
        for signo in ((self.polaridad,) if not self._auto else (-1, +1)):
            r = self._medir_signo(signo, resp, umbral_fila, filas_ok, x_hasta=None)
            nota = r["conf"] + (0.05 if signo == self.polaridad else 0.0)
            if mejor is None or nota > mejor[0]:
                mejor = (nota, signo, r)
        _, signo, r = mejor
        if self._auto and r["conf"] >= 0.3:
            self.polaridad = signo
        a, b, c = r["coef"]
        detalle = Detalle(bev=bev, xs=r["xs"], ys=r["ys"], dentro=r["dentro"], coef=(a, b, c), polaridad=signo,
                          sigma=sigmas, x_fin=r["x_fin"])
        barra, esquina = self._barra(img, valido, signo, (a, b, c), detalle)
        if barra is not None:
            # con barra de fin la linea solo puede llegar hasta ella: no penalizar la que no queda
            r = self._medir_signo(signo, resp, umbral_fila, filas_ok, x_hasta=barra + 0.05, previo=r)
        conf, n_franjas = r["conf"], r["n_franjas"]
        self.detalle = detalle

        if np.isfinite(a):
            L = min(p["distancia_objetivo_m"], detalle.x_fin if np.isfinite(detalle.x_fin) else p["distancia_objetivo_m"])
            objetivo = (L, a + b * L + c * L ** 2)
            kappa = 2 * c / (1 + b ** 2) ** 1.5
            theta = math.atan(b)
        else:
            objetivo, kappa, theta = (math.nan, math.nan), math.nan, math.nan
        ms = 1000 * (time.perf_counter() - t0)
        return MedidaLinea(t=f.t_rx, y=float(a), theta=float(theta), kappa=float(kappa), objetivo=objetivo,
                           confianza=float(conf), n_franjas=n_franjas, barra_fin=barra, esquina=esquina, ms=ms)

    def _medir_signo(self, signo, resp, umbral_fila, filas_ok, x_hasta=None, previo=None):
        """Candidatos, ajuste y confianza para una polaridad. Con `previo` reutiliza su ajuste y solo
        recalcula la confianza con otro alcance (`x_hasta`)."""
        p = self.p
        if previo is None:
            s = resp[signo]
            cols = np.argmax(s, axis=1)
            smax = s[np.arange(len(cols)), cols]
            cand = (smax > umbral_fila) & filas_ok
            xs, ys = self.x_filas[cand], self.y_cols[cols[cand]]
            pesos = smax[cand] / np.maximum(umbral_fila[cand], 1e-6)
            a = b = c = math.nan
            dentro = np.zeros(len(xs), dtype=bool)
            if len(xs) >= 8:
                a, b, c, dentro = self._ajustar(xs, ys)
        else:
            xs, ys, pesos, dentro = previo["xs"], previo["ys"], previo["pesos"], previo["dentro"]
            a, b, c = previo["coef"]
        x_fin = float(xs[dentro].max()) if (np.isfinite(a) and dentro.sum() >= 8) else math.nan
        x_lim = min(2.5, self.x_max) if x_hasta is None else min(2.5, self.x_max, x_hasta)
        rango = (self.x_filas <= x_lim) & filas_ok
        n_filas = max(1, int(rango.sum()))
        conf, n_franjas = 0.0, 0
        if np.isfinite(a) and dentro.sum() >= 8:
            xin, yin = xs[dentro], ys[dentro]
            for x0, x1 in p["franjas_m"]:
                filas = rango & (self.x_filas >= x0) & (self.x_filas < x1)
                if filas.sum() and ((xin >= x0) & (xin < x1)).sum() >= p["franja_vista"] * filas.sum():
                    n_franjas += 1
            res = yin - (a + b * xin + c * xin ** 2)
            c_filas = min(1.0, dentro.sum() / n_filas / 0.6)    # 60 % de las filas ya es una linea clara
            # la cinta da 6-7 veces el umbral; los bordes del fondo ~1.7 (2026-10-05)
            c_contraste = float(np.clip(np.median(pesos[dentro]) / 4.0, 0.0, 1.0))
            c_residuo = float(np.clip(1.0 - np.sqrt(np.mean(res ** 2)) / p["tolerancia_ajuste_m"], 0.0, 1.0))
            conf = c_filas * c_contraste * c_residuo
        return {"xs": xs, "ys": ys, "pesos": pesos, "dentro": dentro, "coef": (a, b, c), "x_fin": x_fin,
                "conf": conf, "n_franjas": n_franjas}

    def _ajustar(self, xs, ys):
        """Recta y luego parabola si hay recorrido, empezando por la mediana y cerrando la tolerancia."""
        a, b, c = float(np.median(ys)), 0.0, 0.0
        dentro = np.ones(len(xs), dtype=bool)
        for tol in (0.30, 0.10, self.p["tolerancia_ajuste_m"]):
            dentro = np.abs(ys - (a + b * xs + c * xs ** 2)) < tol
            if dentro.sum() < 8:
                return math.nan, math.nan, math.nan, dentro
            b, a = np.polyfit(xs[dentro], ys[dentro], 1)
            c = 0.0
        if np.ptp(xs[dentro]) >= 1.0 and dentro.sum() >= 30:
            c2, b2, a2 = np.polyfit(xs[dentro], ys[dentro], 2)
            dentro2 = np.abs(ys - (a2 + b2 * xs + c2 * xs ** 2)) < self.p["tolerancia_ajuste_m"]
            if dentro2.sum() >= dentro.sum():
                a, b, c, dentro = a2, b2, c2, dentro2
        # continuidad: la linea es la cadena desde el punto mas cercano hasta el primer hueco de mas de
        # `hueco_max_m` (la interrupcion del nivel 3 es de 0.4 m); lo de mas alla (bordes de puertas y
        # muebles del fondo, 2026-10-05) no es la linea
        xin = np.sort(xs[dentro])
        saltos = np.flatnonzero(np.diff(xin) > self.p["hueco_max_m"])
        if len(saltos):
            corte = xin[saltos[0]]
            dentro = dentro & (xs <= corte + 1e-9)
            if dentro.sum() < 8:
                return math.nan, math.nan, math.nan, dentro
            grado = 2 if (np.ptp(xs[dentro]) >= 1.0 and dentro.sum() >= 30) else 1
            coefs = np.polyfit(xs[dentro], ys[dentro], grado)
            c = float(coefs[0]) if grado == 2 else 0.0
            b, a = float(coefs[-2]), float(coefs[-1])
        return float(a), float(b), float(c), dentro

    def _barra(self, img, valido, signo, coef, detalle):
        """Barra de fin o esquina: filas con cinta transversal alrededor de la linea, cerca de su final."""
        a, b, c = coef
        if not np.isfinite(a):
            return None, None
        p = self.p
        clara_v, oscura_v, ruido_v = _franja(img, self.ancho_px, horizontal=False)
        sv = clara_v if signo > 0 else oscura_v
        sv = np.where(cv2.erode(valido.astype(np.uint8), np.ones((3 * self.ancho_px, 3), np.uint8)) > 0, sv, 0.0)
        sig = _robusta(ruido_v[valido])
        if not np.isfinite(sig):
            return None, None
        umbral = max(p["barra_sigmas"] * sig, p["umbral_minimo"])
        marcado = sv > umbral
        # cobertura en perpendicular a la linea (la barra es perpendicular a ella; vista desde arriba
        # sale inclinada si el robot va girado): muestras a d = -35..35 cm sobre la normal en cada fila
        x = self.x_filas[:, None]
        yl = a + b * x + c * x ** 2
        m = b + 2 * c * x
        norma = np.sqrt(1 + m ** 2)
        d = np.arange(-0.35, 0.35 + 1e-9, self.res)[None, :]
        X = x - d * m / norma
        Yp = yl + d / norma
        fi = np.rint((self.x_max - X) / self.res).astype(int)
        co = np.rint((self.y_max - Yp) / self.res).astype(int)
        dentro_img = (fi >= 0) & (fi < marcado.shape[0]) & (co >= 0) & (co < marcado.shape[1])
        fi, co = np.clip(fi, 0, marcado.shape[0] - 1), np.clip(co, 0, marcado.shape[1] - 1)
        ok = dentro_img & valido[fi, co]
        hay = marcado[fi, co] & ok
        lado_izq, lado_der = (d > 0.05) & ok, (d < -0.05) & ok
        n_izq, n_der = lado_izq.sum(axis=1), lado_der.sum(axis=1)
        # un lado fuera de la vista (robot muy girado) queda sin dato (NaN): no se decide con el
        izq = np.where(n_izq >= 20, (hay & lado_izq).sum(axis=1) / np.maximum(n_izq, 1), np.nan)
        der = np.where(n_der >= 20, (hay & lado_der).sum(axis=1) / np.maximum(n_der, 1), np.nan)
        filas = np.flatnonzero(np.fmax(izq, der) >= p["barra_cobertura"])
        if not len(filas):
            return None, None
        grupos = np.split(filas, np.flatnonzero(np.diff(filas) > 2) + 1)
        x_fin = detalle.x_fin if np.isfinite(detalle.x_fin) else math.inf
        barra, esquina = None, None
        for g in grupos:
            x_lejos, x_cerca = float(self.x_filas[g[0]]), float(self.x_filas[g[-1]])
            if x_lejos - x_cerca > p["barra_grosor_max_m"]:
                continue
            detalle.filas_barra.append((x_cerca, x_lejos))
            # tiene que estar donde acaba la linea: si la linea sigue mas alla es un cruce, y si acaba
            # mucho antes no tiene que ver con ella
            if x_fin > x_lejos + 0.15 or x_fin < x_cerca - 0.25:
                continue
            li, ld = float(np.mean(izq[g])), float(np.mean(der[g]))
            if not (np.isfinite(li) and np.isfinite(ld)):
                continue                    # un lado no se ve: ni barra ni esquina todavia
            if li >= p["barra_cobertura"] and ld >= p["barra_cobertura"]:
                barra = round(x_cerca, 3) if barra is None else min(barra, round(x_cerca, 3))
            elif max(li, ld) >= p["barra_cobertura"] and min(li, ld) < 0.2:
                esquina = (round(x_cerca, 3), +1 if li > ld else -1)
        return barra, esquina
