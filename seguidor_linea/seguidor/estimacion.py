"""Estimacion (Hito 3): MedidaLinea + IMU -> EstadoLinea.

La linea medida se guarda en un marco fijo al suelo (girado con el yaw de la IMU) y en cada ciclo de
control se vuelve a expresar en el marco actual de la camara. Asi:
  - la direccion de la linea no lleva el balanceo de yaw de cada paso: con el yaw de la IMU tomado
    10 ms antes del fotograma, su oscilacion cae de ~1 a ~0.1 grados (RESULTADOS.md 4);
  - mientras no hay medida (interrupcion, camara tapada, la barra en la zona ciega) la linea sigue
    donde estaba, desplazada con la estima de avance: el rumbo por la IMU y el avance por la vx
    mandada por el factor medido, sin odometria (6.4.2 y 6.4.4);
  - la barra de fin y la esquina se siguen contando cuando ya no se ven. La barra, con la velocidad
    real de acercamiento medida mientras se veia (no con factor x vx).

Estima de avance (RESULTADOS.md 3): el robot avanza a `factor_velocidad` x vx en una direccion
`direccion_avance_deg` a la izquierda del eje de la camara y gira alrededor de un punto `centro_giro_m`
por detras de ella. La velocidad real sigue a la mandada con una constante `tau_velocidad_s`.

El marco del suelo es el del primer instante; el yaw es el de la IMU, desenrollado. Todo recibe el
tiempo como argumento (time.monotonic() en el robot, el reloj del simulador en las pruebas).
"""

import math
import threading
from collections import deque

import numpy as np

from .mensajes import EstadoLinea, MedidaLinea

EXTENSION_M = 3.0       # la linea se prolonga en recta por delante de lo medido...
EXTENSION_ATRAS_M = 1.0  # ...y por detras, para tener la distancia a la camara


def envolver(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class HistorialImu:
    """roll, pitch y yaw (desenrollado) de rt/lowstate con su hora. Lo escribe el hilo del DDS y lo
    leen la percepcion (postura lenta) y la estimacion (yaw en la hora de cada fotograma)."""

    def __init__(self, capacidad=16384):
        self._d = np.zeros((capacidad, 4))
        self._n = 0
        self._cerrojo = threading.Lock()
        self._yaw_prev = None
        self._vueltas = 0.0

    def agregar(self, t, roll, pitch, yaw):
        if self._yaw_prev is not None:
            salto = yaw - self._yaw_prev
            if salto > math.pi:
                self._vueltas -= 2 * math.pi
            elif salto < -math.pi:
                self._vueltas += 2 * math.pi
        self._yaw_prev = yaw
        with self._cerrojo:
            if self._n == len(self._d):          # lleno: se queda con la mitad mas reciente
                mitad = self._n // 2
                self._d[:self._n - mitad] = self._d[mitad:self._n]
                self._n -= mitad
            self._d[self._n] = (t, roll, pitch, yaw + self._vueltas)
            self._n += 1

    def al_lowstate(self, msg, t):
        """Para robot.al_lowstate(): toma la IMU del torso de cada rt/lowstate."""
        from .geometria import yaw_de
        s = msg.imu_state
        self.agregar(t, s.rpy[0], s.rpy[1], yaw_de(s.quaternion))

    def __len__(self):
        return self._n

    def ultimo(self):
        with self._cerrojo:
            if not self._n:
                return None
            return tuple(self._d[self._n - 1])

    def en(self, t):
        """(roll, pitch, yaw) interpolados en `t`; fuera del registro, el extremo mas cercano."""
        with self._cerrojo:
            if not self._n:
                return 0.0, 0.0, 0.0
            d = self._d[:self._n]
            i = int(np.searchsorted(d[:, 0], t))
            if i <= 0:
                return tuple(d[0, 1:])
            if i >= self._n:
                return tuple(d[-1, 1:])
            t0, t1 = d[i - 1, 0], d[i, 0]
            w = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return tuple(d[i - 1, 1:] * (1 - w) + d[i, 1:] * w)

    def media(self, t0, t1):
        """(roll, pitch) medios entre t0 y t1: la postura lenta del torso para la percepcion."""
        with self._cerrojo:
            if not self._n:
                return None, None
            d = self._d[:self._n]
            i0, i1 = np.searchsorted(d[:, 0], [t0, t1 + 1e-9])
            if i1 <= i0:
                i0, i1 = max(0, i1 - 1), max(1, i1)
            return float(d[i0:i1, 1].mean()), float(d[i0:i1, 2].mean())


class _Marca:
    """Barra de fin o esquina vista en el suelo: posicion (marco del suelo), direccion de la linea en
    ella y cuantos avistamientos seguidos lleva."""

    def __init__(self, p, u, sentido=0):
        self.p, self.u, self.sentido, self.n = p, u, sentido, 1


class Estimador:
    def __init__(self, cfg, imu: HistorialImu, x_puntera=0.0, velocidad_externa=None):
        """velocidad_externa(t) -> vx: para reproducir un dataset, donde el robot lo llevo el operador
        (la vx que manda el supervisor en simulacro no es la que anduvo)."""
        e = cfg["estimacion"]
        self.imu = imu
        self.velocidad_externa = velocidad_externa
        self.x_puntera = 0.0 if x_puntera is None else float(x_puntera)
        self.desfase = e["desfase_imu_s"]
        self.postura_s = e["postura_s"]
        self.factor = e["factor_velocidad"]
        self.phi = math.radians(e["direccion_avance_deg"])
        self.d_giro = e["centro_giro_m"]
        self.tau_v = e["tau_velocidad_s"]
        self.conf_usar = e["conf_usar"]
        self.paso = e["paso_linea_m"]
        self.confirmar = e["confirmar"]
        self.tolerancia = e["tolerancia_m"]
        self.puerta_m = e.get("puerta_m", math.inf)
        self.puerta = math.radians(e.get("puerta_deg", 180.0))
        self.puerta_edad = e.get("puerta_edad_s", 0.0)
        self.readquirir = e.get("readquirir", 3)
        self.rechazadas = 0
        # estima de avance: centro de giro C en el marco del suelo
        self._t = None
        self._c = np.zeros(2)
        self._v = 0.0
        self._vx_cmd = 0.0
        self._poses = deque(maxlen=400)      # (t, cx, cy)
        self.reiniciar()

    def reiniciar(self):
        """Olvida la linea, la barra y la esquina (tras el giro de la esquina, o al empezar)."""
        self._linea = None          # (N, 2) en el marco del suelo
        self._t_buena = None
        self._conf = 0.0
        self._kappa = math.nan
        self._fin_medido = 0        # indice del ultimo punto medido de _linea
        self.racha = 0
        self._apoyo = 0             # medidas coherentes seguidas que respaldan la linea estimada
        self._candidata = None      # (linea en el suelo, t, medidas coherentes seguidas)
        self._avist_barra = deque(maxlen=90)   # (t, m de la puntera a la barra) de cada avistamiento
        self.barra = None
        self.esquina = None

    # --- estima de avance ----------------------------------------------------------------------
    def orden(self, t, vx):
        """La vx que se acaba de mandar (en simulacro, la que se habria mandado)."""
        self.avanzar(t)
        self._vx_cmd = float(vx if self.velocidad_externa is None else self.velocidad_externa(t))

    def avanzar(self, t):
        if self._t is None:
            self._t = t
            self._poses.append((t, *self._c))
            return
        if t <= self._t:
            return
        while t > self._t:
            dt = min(0.02, t - self._t)
            tm = self._t + dt / 2
            yaw = self.imu.en(tm)[2]
            objetivo = self.factor * self._vx_cmd
            self._v += (objetivo - self._v) * min(1.0, dt / self.tau_v)
            self._c = self._c + self._v * dt * np.array([math.cos(yaw + self.phi), math.sin(yaw + self.phi)])
            self._t += dt
        self._poses.append((self._t, *self._c))

    def pose(self, t):
        """(x, y, yaw) de la camara en `t` en el marco del suelo."""
        if self._t is None or t > self._t:
            self.avanzar(t)
        ts = [p[0] for p in self._poses]
        i = int(np.searchsorted(ts, t))
        if i <= 0:
            c = np.array(self._poses[0][1:])
        elif i >= len(ts):
            c = np.array(self._poses[-1][1:])
        else:
            t0, t1 = ts[i - 1], ts[i]
            w = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            c = np.array(self._poses[i - 1][1:]) * (1 - w) + np.array(self._poses[i][1:]) * w
        yaw = self.imu.en(t)[2]
        return c[0] + self.d_giro * math.cos(yaw), c[1] + self.d_giro * math.sin(yaw), yaw

    @property
    def v(self):
        return self._v

    def postura(self, t_fotograma):
        """roll y pitch lentos del torso para proyectar un fotograma (media de `postura_s` hasta el)."""
        t = t_fotograma - self.desfase
        return self.imu.media(t - self.postura_s, t)

    # --- medidas ---------------------------------------------------------------------------------
    @staticmethod
    def _a_suelo(pose, pts):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]])

    @staticmethod
    def _a_camara(pose, pts):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        return np.column_stack([c * dx + s * dy, -s * dx + c * dy])

    def medida(self, m: MedidaLinea):
        """Incorpora una medida. Devuelve True si se uso (confianza suficiente)."""
        if not (m.confianza >= self.conf_usar and np.isfinite(m.y) and np.all(np.isfinite(m.alcance))):
            self.racha = 0
            return False
        pose = self.pose(m.t - self.desfase)
        a, b = m.y, math.tan(m.theta)
        c = (m.kappa * (1 + b * b) ** 1.5 / 2) if np.isfinite(m.kappa) else 0.0
        x0, x1 = m.alcance
        xs = np.append(np.arange(x0, x1, self.paso), x1)
        cam = np.column_stack([xs, a + b * xs + c * xs ** 2])
        nueva = self._a_suelo(pose, cam)
        # puerta: una medida lejos de la linea predicha no se usa. Si esa linea es vieja o poco respaldada,
        # la medida empieza una linea CANDIDATA, que solo se adopta con `readquirir` medidas seguidas
        # coherentes entre si: un falso suelto no puede refrescar la edad (con la camara tapada por una
        # caja, falsos sueltos retrasaron la PARADA de 3 a 13 s, 2026-10-06) ni bloquear la buena
        if self._linea is None or self._coherente(self._linea, pose, a, b, c, x0, x1):
            self._apoyo += 1
            self._candidata = None
        else:
            self.racha = 0
            if self._apoyo >= 3 and m.t - self._t_buena <= self.puerta_edad:
                self.rechazadas += 1
                return False
            if self._candidata is not None and m.t - self._candidata[1] < 0.3 and \
                    self._coherente(self._candidata[0], pose, a, b, c, x0, x1):
                self._candidata = (nueva, m.t, self._candidata[2] + 1)
            else:
                self._candidata = (nueva, m.t, 1)
            if self._candidata[2] < self.readquirir:
                self.rechazadas += 1
                return False
            self._apoyo = self._candidata[2]
            self._candidata = None
        self._linea = nueva
        self._fin_medido = len(xs) - 1
        self._t_buena = m.t
        self._conf = m.confianza
        self._kappa = m.kappa
        self.racha += 1

        def en_linea(x):
            p = np.array([[x, a + b * x + c * x ** 2]])
            pend = b + 2 * c * x
            u = np.array([[1.0, pend]]) / math.hypot(1.0, pend)
            return self._a_suelo(pose, p)[0], self._a_suelo((0.0, 0.0, pose[2]), u)[0]

        barra_antes = self.barra
        self.barra = self._actualizar(self.barra, m.barra_fin, 0, en_linea, pose, x1)
        if self.barra is not barra_antes:
            self._avist_barra.clear()          # barra nueva (o descartada): otra serie
        if m.barra_fin is not None and self.barra is not None:
            toe = self._a_suelo(pose, np.array([[self.x_puntera, 0.0]]))[0]
            self._avist_barra.append((m.t, float((self.barra.p - toe) @ self.barra.u)))
        esq = m.esquina
        self.esquina = self._actualizar(self.esquina, None if esq is None else esq[0],
                                        0 if esq is None else esq[1], en_linea, pose, x1)
        return True

    def _coherente(self, linea, pose, a, b, c, x0, x1):
        """La medida cae cerca de `linea` (marco del suelo) vista desde `pose`."""
        prev = self._a_camara(pose, linea)
        xr = min(max(0.5, x0), x1)
        orden = np.argsort(prev[:, 0])
        px, py = prev[orden, 0], prev[orden, 1]
        if not (px[0] <= xr <= px[-1]) or len(px) < 2:
            return True
        y_prev = float(np.interp(xr, px, py))
        i = int(np.clip(np.searchsorted(px, xr), 1, len(px) - 1))
        ang_prev = math.atan2(py[i] - py[i - 1], px[i] - px[i - 1])
        y_med = a + b * xr + c * xr * xr
        ang_med = math.atan(b + 2 * c * xr)
        return abs(y_med - y_prev) <= self.puerta_m and abs(envolver(ang_med - ang_prev)) <= self.puerta

    def _actualizar(self, marca, x, sentido, en_linea, pose, x_fin):
        if x is None:
            # si la linea medida sigue claramente mas alla de la marca, la marca era falsa
            if marca is not None and marca.n < self.confirmar:
                xm = self._a_camara(pose, marca.p[None])[0, 0]
                if x_fin > xm + 0.3:
                    return None
            return marca
        p, u = en_linea(x)
        if marca is not None and np.linalg.norm(p - marca.p) < self.tolerancia and marca.sentido == sentido:
            marca.p, marca.u, marca.n = p, u, marca.n + 1
            return marca
        return _Marca(p, u, sentido)

    # --- estado ------------------------------------------------------------------------------------
    def estado(self, t, distancia_objetivo, ref_x=None) -> EstadoLinea:
        """`distancia_objetivo` se mide desde el punto (ref_x, 0) del marco de la camara: por defecto
        el centro de giro, que es el que sigue la curva (y donde estan los pies)."""
        ref = np.array([-self.d_giro if ref_x is None else ref_x, 0.0])
        pose = self.pose(t)
        edad = math.inf if self._t_buena is None else t - self._t_buena
        if self._linea is None:
            return EstadoLinea(t=t, y=math.nan, theta=math.nan, kappa=math.nan, confianza=0.0, edad_s=edad,
                               rumbo_ref=math.nan, v=self._v, racha=self.racha, hay_linea=False)
        pts = self._a_camara(pose, self._linea)
        # prolongar en recta por los dos extremos
        if len(pts) >= 2:
            u_ini = pts[1] - pts[0]
            u_fin = pts[-1] - pts[-2]
        else:
            u_ini = u_fin = np.array([1.0, 0.0])
        u_ini = u_ini / max(1e-9, np.linalg.norm(u_ini))
        u_fin = u_fin / max(1e-9, np.linalg.norm(u_fin))
        atras = pts[0] - EXTENSION_ATRAS_M * u_ini
        delante = pts[-1] + EXTENSION_M * u_fin
        poli = np.vstack([atras, pts, delante])

        # punto mas cercano a la camara: distancia con signo y direccion
        a, b = poli[:-1], poli[1:]
        seg = b - a
        largo2 = np.maximum((seg ** 2).sum(axis=1), 1e-12)
        w = np.clip(-(a * seg).sum(axis=1) / largo2, 0.0, 1.0)
        cerca = a + w[:, None] * seg
        k = int(np.argmin((cerca ** 2).sum(axis=1)))
        u = seg[k] / math.sqrt(largo2[k])
        q = cerca[k]
        y = float(q[1] * u[0] - q[0] * u[1])     # sobre la normal izquierda de la linea: izquierda > 0
        theta = math.atan2(u[1], u[0])

        # punto adelantado: primer cruce de la circunferencia de radio L alrededor de `ref`, desde el
        # punto mas cercano
        L = distancia_objetivo
        objetivo, i_obj = poli[-1], len(poli) - 1
        for j in range(k, len(poli) - 1):
            p0 = (cerca[k] if j == k else poli[j]) - ref
            p1 = poli[j + 1] - ref
            if np.hypot(*p1) >= L:
                d = p1 - p0
                aa, bb, cc = d @ d, 2 * p0 @ d, p0 @ p0 - L * L
                disc = max(0.0, bb * bb - 4 * aa * cc)
                s = (-bb + math.sqrt(disc)) / (2 * aa) if aa > 0 else 0.0
                objetivo, i_obj = ref + p0 + np.clip(s, 0.0, 1.0) * d, j
                break
        extrapolado = i_obj >= 1 + self._fin_medido   # en poli, el medido va de 1 a 1 + _fin_medido

        dist_fin = esquina = None
        v = self._v
        toe = np.array([self.x_puntera, 0.0])
        if self.barra is not None and self.barra.n >= self.confirmar:
            v_barra = self.v_barra()
            if v_barra is not None:
                # la distancia a la barra baja a la velocidad real del robot: se sigue con la medida
                # mientras se veia (2026-10-06: con factor x vx el final a ciegas iba segundos tarde
                # cuando la velocidad real no era la mandada)
                t_b, d_b = self._avist_barra[-1]
                dist_fin, v = d_b - v_barra * max(0.0, t - t_b), v_barra
            else:
                pb = self._a_camara(pose, self.barra.p[None])[0]
                ub = self._a_camara((0.0, 0.0, pose[2]), self.barra.u[None])[0]
                dist_fin = float((pb - toe) @ ub)
        if self.esquina is not None and self.esquina.n >= self.confirmar:
            pe = self._a_camara(pose, self.esquina.p[None])[0]
            ue = self._a_camara((0.0, 0.0, pose[2]), self.esquina.u[None])[0]
            esquina = (float((pe - toe) @ ue), self.esquina.sentido)
        return EstadoLinea(t=t, y=y, theta=theta, kappa=self._kappa, confianza=self._conf, edad_s=edad,
                           rumbo_ref=envolver(pose[2] + theta), dist_fin=dist_fin,
                           objetivo=(float(objetivo[0]), float(objetivo[1])), extrapolado=bool(extrapolado),
                           esquina=esquina, v=v, racha=self.racha, hay_linea=True)

    def v_barra(self, ventana_s=1.2):
        """Velocidad real de acercamiento a la barra: recta sobre los avistamientos del ultimo
        `ventana_s` (al menos 8 en 0.5 s). None si no hay bastantes o sale fuera de lo razonable."""
        if len(self._avist_barra) < 8:
            return None
        a = np.array(self._avist_barra)
        a = a[a[:, 0] >= a[-1, 0] - ventana_s]
        if len(a) < 8 or np.ptp(a[:, 0]) < 0.5:
            return None
        v = -float(np.polyfit(a[:, 0] - a[-1, 0], a[:, 1], 1)[0])
        return v if 0.03 <= v <= 0.8 else None
