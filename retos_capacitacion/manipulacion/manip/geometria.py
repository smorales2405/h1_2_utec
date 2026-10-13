"""Geometria comun: transformaciones homogeneas, camara pinhole, plano de la mesa y nube de puntos.

Notacion: T_a_b es la matriz 4x4 que lleva puntos del marco b al marco a (p_a = T_a_b @ p_b). Los
marcos opticos de la camara siguen a OpenCV y librealsense: z adelante, x a la derecha, y abajo.
"""

import math
from dataclasses import dataclass

import numpy as np

# camera_link del URDF (x adelante, y izquierda, z arriba) -> marco optico (z adelante, x derecha, y abajo)
R_LINK_OPTICO = np.array([[0.0, 0.0, 1.0],
                          [-1.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0]])


def T(R=None, t=None):
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    if t is not None:
        M[:3, 3] = t
    return M


def inv(M):
    R, t = M[:3, :3], M[:3, 3]
    return T(R.T, -R.T @ t)


def aplicar(M, P):
    """Puntos (N, 3) o (3,) del marco de origen al de destino de M."""
    P = np.asarray(P, dtype=float)
    return P @ M[:3, :3].T + M[:3, 3]


def rpy_a_R(r, p, y):
    """Como el URDF: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_a_rpy(R):
    p = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])


def rotvec_a_R(v):
    from scipy.spatial.transform import Rotation
    return Rotation.from_rotvec(np.asarray(v, dtype=float)).as_matrix()


def R_a_rotvec(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(np.asarray(R, dtype=float)).as_rotvec()


def T_a_vector(M):
    """4x4 -> (rotvec, t): 6 parametros para los ajustes por minimos cuadrados."""
    return np.concatenate([R_a_rotvec(M[:3, :3]), M[:3, 3]])


def vector_a_T(x):
    return T(rotvec_a_R(x[:3]), x[3:6])


def angulo_entre(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.acos(max(-1.0, min(1.0, c)))


def envolver(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def envolver_simetria(a, simetria):
    """Yaw modulo la simetria del objeto (rad), en [-s/2, s/2). simetria 0 = sin orientacion: 0."""
    if simetria <= 0:
        return 0.0
    return (a + simetria / 2) % simetria - simetria / 2


@dataclass
class Intrinsecos:
    fx: float
    fy: float
    cx: float
    cy: float
    ancho: int = 640
    alto: int = 480

    @classmethod
    def de_meta(cls, d):
        return cls(d["fx"], d["fy"], d["cx"], d["cy"], int(d.get("ancho", 640)), int(d.get("alto", 480)))

    def a_dict(self):
        return {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy, "ancho": self.ancho, "alto": self.alto}

    @property
    def K(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def proyectar(self, P):
        """Puntos (N, 3) en el marco optico -> pixeles (N, 2). Detras de la camara: nan."""
        P = np.atleast_2d(np.asarray(P, float))
        z = np.where(P[:, 2] > 1e-6, P[:, 2], np.nan)
        return np.stack([self.fx * P[:, 0] / z + self.cx, self.fy * P[:, 1] / z + self.cy], 1)

    def rayos(self, uv):
        """Pixeles (N, 2) -> direcciones (N, 3) en el marco optico, con z = 1."""
        uv = np.atleast_2d(np.asarray(uv, float))
        return np.stack([(uv[:, 0] - self.cx) / self.fx, (uv[:, 1] - self.cy) / self.fy, np.ones(len(uv))], 1)

    def fov_v_deg(self):
        return math.degrees(2 * math.atan(self.alto / 2 / self.fy))


def nube(prof_m, intr: Intrinsecos, paso=1, mascara=None):
    """Profundidad en metros (alto x ancho, 0 = sin dato) -> (puntos (N, 3) en el marco optico, (v, u))."""
    v, u = np.mgrid[0:prof_m.shape[0]:paso, 0:prof_m.shape[1]:paso]
    z = prof_m[::paso, ::paso]
    ok = z > 0
    if mascara is not None:
        ok &= mascara[::paso, ::paso]
    z, u, v = z[ok], u[ok], v[ok]
    P = np.stack([(u - intr.cx) / intr.fx * z, (v - intr.cy) / intr.fy * z, z], 1)
    return P, (v, u)


class Plano:
    """Plano n.p + d = 0 con n unitario; altura(p) = n.p + d (positiva del lado de n)."""

    def __init__(self, n, d):
        n = np.asarray(n, float)
        s = np.linalg.norm(n)
        self.n, self.d = n / s, float(d) / s

    def altura(self, P):
        return np.asarray(P, float) @ self.n + self.d

    def proyectar(self, P):
        P = np.asarray(P, float)
        return P - np.outer(self.altura(P), self.n) if P.ndim == 2 else P - self.altura(P) * self.n

    def corte_rayo(self, o, direccion):
        """Punto donde el rayo o + s*dir corta el plano (None si es paralelo o queda detras)."""
        den = float(self.n @ direccion)
        if abs(den) < 1e-9:
            return None
        s = -(self.n @ o + self.d) / den
        return None if s <= 0 else o + s * np.asarray(direccion, float)

    def transformar(self, M):
        """El mismo plano expresado en el marco destino de M."""
        n = M[:3, :3] @ self.n
        p0 = aplicar(M, -self.d * self.n)
        return Plano(n, -float(n @ p0))

    def base(self, x_aprox=(1.0, 0.0, 0.0)):
        """(origen, ex, ey) en el plano: ex es la proyeccion de x_aprox; ey = n x ex."""
        ex = np.asarray(x_aprox, float) - (self.n @ x_aprox) * self.n
        ex /= np.linalg.norm(ex)
        return -self.d * self.n, ex, np.cross(self.n, ex)

    @classmethod
    def ajustar(cls, P, umbral=0.006, iteraciones=300, semilla=0, cerca_de=None, tol_ang_deg=10.0):
        """RANSAC + minimos cuadrados. Con `cerca_de` (Plano) solo vale un plano de normal parecida.
        Devuelve (plano, fraccion de inliers, residuo rms) o (None, 0, nan)."""
        P = np.asarray(P, float)
        if len(P) < 3:
            return None, 0.0, float("nan")
        rng = np.random.default_rng(semilla)
        sub = P if len(P) <= 30000 else P[rng.choice(len(P), 30000, replace=False)]
        mejor = None
        cos_min = math.cos(math.radians(tol_ang_deg))
        for _ in range(iteraciones):
            s = sub[rng.choice(len(sub), 3, replace=False)]
            n = np.cross(s[1] - s[0], s[2] - s[0])
            nn = np.linalg.norm(n)
            if nn < 1e-9:
                continue
            n /= nn
            if cerca_de is not None:
                if n @ cerca_de.n < 0:
                    n = -n
                if n @ cerca_de.n < cos_min:
                    continue
            d = -n @ s[0]
            k = int((np.abs(sub @ n + d) < umbral).sum())
            if mejor is None or k > mejor[0]:
                mejor = (k, n, d)
        if mejor is None:
            return None, 0.0, float("nan")
        _, n, d = mejor
        dentro = np.abs(P @ n + d) < umbral
        c = P[dentro].mean(0)
        _, _, Vt = np.linalg.svd(P[dentro] - c, full_matrices=False)
        n = Vt[2]
        if cerca_de is not None and n @ cerca_de.n < 0:
            n = -n
        d = -n @ c
        res = P[dentro] @ n + d
        return cls(n, d), float(dentro.mean()), float(np.sqrt(np.mean(res ** 2)))


def kabsch(A, B, pesos=None):
    """R, t que minimizan sum w |R a + t - b|^2 (A, B: (N, 3) correspondientes)."""
    A, B = np.asarray(A, float), np.asarray(B, float)
    w = np.ones(len(A)) if pesos is None else np.asarray(pesos, float)
    w = w / w.sum()
    ca, cb = w @ A, w @ B
    H = (A - ca).T @ ((B - cb) * w[:, None])
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca
