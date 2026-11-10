"""El giro del cuerpo para el vigilante (seccion 10 del PDF: salida segura con el yaw a mas de 5 grados): el yaw de
la IMU, como pide el PDF, sin su deriva.

La IMU del H1-2 deriva en yaw sin que el cuerpo gire (RESULTADOS §32–33): con el robot de pie en el arnes y la
camara quieta (vibracion_camara.py, 0.000 grados/min), -0.45 grados/min, y a lo largo de 40 min de la misma sesion
entre +0.4 y -0.6 grados/min (unos 6 grados en total): en una tirada de 10 min, varios grados, y el vigilante
paraba con 5. Los ArUco de la mesa no derivan: el giro del cuerpo respecto de la mesa sale de la recta entre el 2
y el 3 (el cruce de sus diagonales en el color, llevado al plano de la mesa del [g] con la calibracion
camara-torso), con
~0.02 grados de ruido por fotograma.

    deriva medida  = lo que gira la IMU desde el inicio - lo que gira el cuerpo respecto de la mesa vista
    deriva(t)      = la recta de las medidas de los ultimos `ventana_s` en t (si un brazo tapa los marcadores, se
                     sigue con su ritmo, como mucho `ritmo_max` grados/min)
    giro vigilado  = lo que gira la IMU desde el inicio - deriva(t)
Un giro de verdad lo ven a la vez la IMU y la camara y no se resta. Si se mueve la mesa (un golpe al carrito), la
camara ve un giro que la IMU no: sale como giro del cuerpo, y pararse entonces tambien es lo que toca.
Con la fuente de la camara en vivo (fuentes.LectorContinuo, su ultimo fotograma) mide en un hilo a `hz`.
"""

import math
import threading
import time

import numpy as np

from . import calibracion as cb
from .geometria import Intrinsecos, Plano

IDS = (2, 3)            # los ArUco lejanos de la mesa: con el carrito cerca son los que se ven


def ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def centro(esq):
    """El centro del marcador en la imagen: el cruce de sus diagonales (la media de las esquinas se sesga con la
    perspectiva y el sesgo cambia al girar: 1.39 grados por 1.5 en el gemelo)."""
    p1, p2, p3, p4 = (np.append(np.asarray(e, float), 1.0) for e in esq)
    x = np.cross(np.cross(p1, p3), np.cross(p2, p4))
    return x[:2] / x[2] if abs(x[2]) > 1e-12 else np.asarray(esq, float).mean(axis=0)


def yaw_mesa(color, intr, T_torso_color, plano, det, ids=IDS):
    """El angulo en planta (rad, en el torso) de la recta del ArUco ids[0] al ids[1]: sus centros en el color
    llevados al plano de la mesa. None si no se ven los dos."""
    esq = cb.detectar(color, det, ids_validos=set(ids), escalas=(1,))
    if not all(i in esq for i in ids):
        return None
    o, R = T_torso_color[:3, 3], T_torso_color[:3, :3]
    P = []
    for i in ids:
        p = plano.corte_rayo(o, R @ intr.rayos(centro(esq[i]))[0])
        if p is None:
            return None
        P.append(p)
    v = P[1] - P[0]
    return math.atan2(v[1], v[0])


class Giro:
    def __init__(self, robot, fuente, cal, plano, diccionario="DICT_4X4_50", hz=2.0, ventana_s=30.0, ritmo_max=1.0,
                 reloj=time.monotonic, log=None):
        """robot: con estado().imu.yaw; fuente: con `ultimo` (el ultimo Fotograma) o None; plano: el de la mesa
        del [g] (Plano o {"n", "d"}); ritmo_max: grados/min, el mayor ritmo de deriva que se extrapola."""
        self.robot, self.fuente, self.T = robot, fuente, np.asarray(cal.T_torso_color, float)
        self.plano = plano if isinstance(plano, Plano) else Plano(plano["n"], plano["d"])
        self.det = cb.detector(diccionario)
        self.cada, self.ventana = 1.0 / float(hz), float(ventana_s)
        self.ritmo_max = math.radians(float(ritmo_max)) / 60.0
        self.reloj, self.log = reloj, log
        self.cerrojo = threading.Lock()
        self.imu0, self.mesa0 = None, None
        self.hist = []                    # (t, deriva medida en rad)
        self.medidas, self.sin_ver = 0, 0
        self._vivo, self._hilo, self._n_ult = False, None, None

    # --- medidas -----------------------------------------------------------------------------------------
    def iniciar(self, yaw_imu):
        """Al empezar la tirada (lo llama el vigilante): el yaw de la IMU de referencia; la mesa, en la primera
        medida."""
        with self.cerrojo:
            self.imu0, self.mesa0, self.hist = float(yaw_imu), None, []

    def medir(self, f, yaw_imu, t=None):
        """Una medida con el fotograma f (color y meta) y el yaw de la IMU de ese momento. Devuelve la deriva
        medida (rad) o None (sin referencia o sin los dos marcadores)."""
        if self.imu0 is None or f is None or getattr(f, "color", None) is None:
            return None
        a = yaw_mesa(f.color, Intrinsecos.de_meta(f.meta["color"]), self.T, self.plano, self.det)
        t = self.reloj() if t is None else t
        with self.cerrojo:
            if a is None:
                self.sin_ver += 1
                return None
            if self.mesa0 is None:
                self.mesa0 = a
            giro_camara = -ang(a - self.mesa0)            # la mesa gira al reves que el cuerpo
            d = ang(yaw_imu - self.imu0) - giro_camara
            self.hist = [h for h in self.hist if t - h[0] <= self.ventana] + [(t, d)]
            self.medidas += 1
            return d

    def deriva(self, t=None):
        """La deriva del yaw de la IMU (rad) en t, desde el inicio; 0 sin medidas."""
        with self.cerrojo:
            h = list(self.hist)
        if not h:
            return 0.0
        if len(h) < 4 or h[-1][0] - h[0][0] < 2.0:
            return h[-1][1]
        ts, ds = np.array([x[0] for x in h]), np.array([x[1] for x in h])
        ritmo, d_ult = np.polyfit(ts - ts[-1], ds, 1)
        ritmo = float(np.clip(ritmo, -self.ritmo_max, self.ritmo_max))
        t = self.reloj() if t is None else t
        return float(d_ult + ritmo * max(0.0, t - ts[-1]))

    def corregido(self, yaw_imu, t=None):
        """El giro del cuerpo desde el inicio (rad): el de la IMU sin la deriva."""
        if self.imu0 is None:
            return 0.0
        return ang(ang(yaw_imu - self.imu0) - self.deriva(t))

    def ultima_vista_s(self):
        """s desde la ultima medida con los dos marcadores (inf si ninguna)."""
        with self.cerrojo:
            return self.reloj() - self.hist[-1][0] if self.hist else math.inf

    # --- hilo con la camara en vivo ---------------------------------------------------------------------------
    def arrancar(self):
        if self.fuente is None or not hasattr(self.fuente, "ultimo"):
            return False
        self._vivo = True
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()
        return True

    def _bucle(self):
        while self._vivo:
            t0 = self.reloj()
            f = self.fuente.ultimo
            if f is not None and f.n != self._n_ult:
                self._n_ult = f.n
                try:
                    self.medir(f, self.robot.estado().imu.yaw)
                except Exception as ex:                   # el vigilante sigue con la IMU sola
                    if self.log:
                        self.log(f"giro con la mesa: {ex!r}")
            time.sleep(max(0.0, self.cada - (self.reloj() - t0)))

    def parar(self):
        self._vivo = False
        if self._hilo is not None:
            self._hilo.join(timeout=1.0)
