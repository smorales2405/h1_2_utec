"""Geometria camara-suelo: plano del suelo, pose de la camara y proyeccion pixel <-> suelo.

Marcos:
  suelo  G: X adelante, Y a la izquierda, Z arriba; origen en la vertical de la camara.
  camara C: el optico de librealsense, x a la derecha, y abajo, z hacia delante.
Pose: R_GC = Rz(yaw) * Ry(inclinacion) * Rx(roll) * R0, con inclinacion > 0 mirando abajo y
R0 la camara mirando al frente. El plano del suelo en el marco de la camara, n.p + d = 0 con n
hacia arriba, queda d = altura y n = (-cos(i) sin(r), -cos(i) cos(r), -sin(i)): el yaw no se
puede sacar del suelo, se mide aparte con una linea alineada con los pies.

Correccion por la IMU (imu_roll / imu_pitch): gira la camara alrededor de su centro con el roll y
el pitch del torso respecto de los de la calibracion, R_GC(ahora) = Rimu(ahora) Rimu(cal)^T R_GC(cal).
NO USARLA ANDANDO: medido en los datasets del 2026-10-03, triplica la oscilacion lateral de la
linea (~1 cm -> ~3 cm) con cualquier signo o eje. El torso oscila como un pendulo sobre los
tobillos: la camara gira y a la vez se desplaza (1.65 m x 1.5 grados ~ 4 cm) y las dos cosas casi
se cancelan; visto desde el cuerpo el suelo apenas se mueve, que es lo que quiere el control.
Lo que si oscila es el yaw del torso (~1 grado por paso): eso se compensa fuera, sumando el yaw
de la IMU (tomado ~10 ms antes de que llegue el fotograma) al angulo de la linea.
"""

import math
from dataclasses import dataclass

import numpy as np

R0 = np.array([[0.0, 0.0, 1.0],
               [-1.0, 0.0, 0.0],
               [0.0, -1.0, 0.0]])   # columnas: ejes x, y, z de la camara en G


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def yaw_de(q):
    """Yaw (rad) del cuaternion (w, x, y, z) de la IMU."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rot_imu(roll, pitch):
    """Orientacion del torso respecto de la vertical, sin yaw (convencion ZYX de la IMU)."""
    return rot_y(pitch) @ rot_x(roll)


@dataclass
class Intrinsecos:
    fx: float
    fy: float
    cx: float
    cy: float
    ancho: int = 640
    alto: int = 480

    @classmethod
    def desde_dict(cls, d):
        return cls(fx=d["fx"], fy=d["fy"], cx=d["cx"], cy=d["cy"],
                   ancho=d.get("ancho", 640), alto=d.get("alto", 480))

    def fov_deg(self):
        """(horizontal, vertical) en grados."""
        return (2 * math.degrees(math.atan(self.ancho / 2 / self.fx)),
                2 * math.degrees(math.atan(self.alto / 2 / self.fy)))


def nube(prof_m, intr: Intrinsecos, paso=4, zmin=0.2, zmax=6.0):
    """Puntos (N x 3) en el marco de la camara a partir de una imagen de profundidad en metros."""
    z = prof_m[::paso, ::paso]
    v, u = np.mgrid[0:prof_m.shape[0]:paso, 0:prof_m.shape[1]:paso]
    ok = (z > zmin) & (z < zmax)
    z = z[ok]
    return np.stack([(u[ok] - intr.cx) * z / intr.fx, (v[ok] - intr.cy) * z / intr.fy, z], axis=1)


def ajustar_plano(puntos, umbral=0.02, iteraciones=300, semilla=0):
    """RANSAC + minimos cuadrados. Devuelve (n, d, fraccion de puntos en el plano), con n
    unitaria orientada hacia la camara (d > 0, d = distancia de la camara al plano)."""
    if len(puntos) < 100:
        raise ValueError(f"solo {len(puntos)} puntos de profundidad validos")
    rng = np.random.default_rng(semilla)
    mejor, mejor_n, mejor_d = -1, None, None
    for _ in range(iteraciones):
        a, b, c = puntos[rng.choice(len(puntos), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norma = np.linalg.norm(n)
        if norma < 1e-9:
            continue
        n /= norma
        d = -n @ a
        cuenta = np.count_nonzero(np.abs(puntos @ n + d) < umbral)
        if cuenta > mejor:
            mejor, mejor_n, mejor_d = cuenta, n, d
    n, d = mejor_n, mejor_d
    for _ in range(2):   # refinar con los puntos del plano
        dentro = puntos[np.abs(puntos @ n + d) < umbral]
        centro = dentro.mean(axis=0)
        _, vectores = np.linalg.eigh((dentro - centro).T @ (dentro - centro))
        n = vectores[:, 0]
        d = -n @ centro
    if d < 0:
        n, d = -n, -d
    frac = np.count_nonzero(np.abs(puntos @ n + d) < umbral) / len(puntos)
    return n, float(d), float(frac)


def pose_de_plano(n, d):
    """(altura m, inclinacion rad, roll rad) de la camara a partir del plano del suelo."""
    inclinacion = math.asin(max(-1.0, min(1.0, -n[2])))
    roll = math.atan2(-n[0], -n[1])
    return d, inclinacion, roll


def normal_de_pose(inclinacion, roll):
    ci = math.cos(inclinacion)
    return np.array([-ci * math.sin(roll), -ci * math.cos(roll), -math.sin(inclinacion)])


class ModeloSuelo:
    """Proyeccion pixel <-> suelo para una camara a `altura` con la pose dada (angulos en rad).
    Si se pasan imu_roll / imu_pitch, la pose se corrige con el giro del torso desde la calibracion."""

    def __init__(self, intr: Intrinsecos, altura, inclinacion, roll=0.0, yaw=0.0,
                 imu_roll_ref=0.0, imu_pitch_ref=0.0):
        self.intr = intr
        self.altura = altura
        self.inclinacion = inclinacion
        self.roll = roll
        self.yaw = yaw
        self._r_cal = rot_z(yaw) @ rot_y(inclinacion) @ rot_x(roll) @ R0
        self._imu_ref_t = rot_imu(imu_roll_ref, imu_pitch_ref).T

    @classmethod
    def desde_config(cls, geometria, intr: Intrinsecos):
        r = math.radians
        return cls(intr, geometria["altura_m"], r(geometria["inclinacion_deg"]),
                   r(geometria.get("roll_deg", 0.0)), r(geometria.get("yaw_deg", 0.0)),
                   r(geometria.get("imu_roll_ref_deg", 0.0)), r(geometria.get("imu_pitch_ref_deg", 0.0)))

    def rotacion(self, imu_roll=None, imu_pitch=None):
        if imu_roll is None or imu_pitch is None:
            return self._r_cal
        return rot_imu(imu_roll, imu_pitch) @ self._imu_ref_t @ self._r_cal

    def pixel_a_suelo(self, u, v, imu_roll=None, imu_pitch=None):
        """(X, Y) en m de los pixeles (u, v); NaN por encima del horizonte."""
        u = np.asarray(u, dtype=float)
        v = np.asarray(v, dtype=float)
        rc = np.stack([(u - self.intr.cx) / self.intr.fx, (v - self.intr.cy) / self.intr.fy, np.ones_like(u)])
        rg = np.tensordot(self.rotacion(imu_roll, imu_pitch), rc, axes=1)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(rg[2] < -1e-6, -self.altura / rg[2], np.nan)
        return t * rg[0], t * rg[1]

    def suelo_a_pixel(self, x, y, imu_roll=None, imu_pitch=None):
        """(u, v) de los puntos del suelo (x, y); NaN si quedan detras de la camara."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        pg = np.stack([x, y, np.full_like(x, -self.altura)])
        pc = np.tensordot(self.rotacion(imu_roll, imu_pitch).T, pg, axes=1)
        with np.errstate(divide="ignore", invalid="ignore"):
            z = np.where(pc[2] > 1e-6, pc[2], np.nan)
        return self.intr.fx * pc[0] / z + self.intr.cx, self.intr.fy * pc[1] / z + self.intr.cy

    def alcance(self):
        """Distancia en el suelo (m, desde la vertical de la camara) del borde inferior y del
        superior de la imagen en la columna central. inf si el borde superior mira sobre el horizonte."""
        x, _ = self.pixel_a_suelo([self.intr.cx, self.intr.cx], [self.intr.alto - 1, 0])
        return float(x[0]), (float(x[1]) if np.isfinite(x[1]) else math.inf)

    def mapa_vista_superior(self, x_min, x_max, y_max, resolucion, imu_roll=None, imu_pitch=None):
        """Mapas para cv2.remap de una vista desde arriba: filas de x_max (arriba) a x_min (abajo),
        columnas de +y_max (izquierda) a -y_max (derecha), `resolucion` m por pixel."""
        xs = np.arange(x_max, x_min, -resolucion)
        ys = np.arange(y_max, -y_max, -resolucion)
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        u, v = self.suelo_a_pixel(gx, gy, imu_roll, imu_pitch)
        return np.nan_to_num(u, nan=-1).astype(np.float32), np.nan_to_num(v, nan=-1).astype(np.float32)
