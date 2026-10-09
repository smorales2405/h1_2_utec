"""Simulador del lazo cerrado, sin robot ni camara: para ajustar y probar estimacion, control y supervisor
(el mismo codigo que corre en el robot) sobre las pistas de los cuatro niveles del PDF.

- Marcha (RobotSimulado, con la interfaz de robot.Robot): lo medido en RESULTADOS.md. Avanza a 1.14 x vx
  7.5 grados a la izquierda del eje de la camara, gira alrededor de un punto 14 cm por detras de ella,
  responde a vyaw con un tiempo muerto mas un primer orden (retardo efectivo ~0.45 s, t90 ~0.85 s),
  deriva a la izquierda 0.026 rad/s andando y balancea el yaw ~1 grado con cada zancada. Cada Move dura
  1 s; StopMove frena con la misma constante que el arranque.
- Camara (SensorSimulado): la percepcion sin la imagen. Ve la cinta entre 0.2 y 3 m por delante y
  +-1 m a los lados, una deteccion por fila con ruido, y pasa por el mismo ajuste (recta, parabola que
  crece, cadena continua) y la misma confianza que la percepcion; da la barra (de 0.28 a 2.95 m) y la
  esquina donde acaba la linea.

No sustituye a las pruebas en el robot: el modelo de la marcha es lineal y la camara no tiene sombras
ni falsos positivos (eso lo prueba la percepcion sobre los datasets).
"""

import math
from collections import deque

import numpy as np

from .control import Control
from .estimacion import Estimador, HistorialImu, envolver
from .mensajes import Imu, Mando, MedidaLinea
from .supervisor import FIN, Supervisor
from .vigilante import Vigilante

# --- pistas ---------------------------------------------------------------------------------------------


class Pista:
    """Cinta como polilinea a 1 cm, con huecos (intervalos de longitud de arco sin cinta), esquinas y
    barra de fin perpendicular en el ultimo punto."""

    def __init__(self, puntos, huecos=(), barra=True, nombre=""):
        self.p = np.asarray(puntos, dtype=float)
        seg = np.diff(self.p, axis=0)
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
        u = seg / np.maximum(np.hypot(seg[:, 0], seg[:, 1]), 1e-9)[:, None]
        self.u = np.vstack([u, u[-1:]])
        self.huecos = list(huecos)
        self.barra = barra
        self.nombre = nombre
        ang = np.unwrap(np.arctan2(self.u[:, 1], self.u[:, 0]))
        salto = np.diff(ang)
        self.esquinas = [(float(self.s[i + 1]), int(np.sign(salto[i])), i + 1)
                         for i in np.flatnonzero(np.abs(salto) > math.radians(45))]
        self.cinta = np.ones(len(self.p), dtype=bool)
        for s0, s1 in self.huecos:
            self.cinta &= ~((self.s >= s0) & (self.s <= s1))

    @property
    def largo(self):
        return float(self.s[-1])

    def cercano(self, q):
        """(distancia con signo, s) del punto q a la pista (izquierda de la pista > 0)."""
        d = self.p - q
        i = int(np.argmin((d ** 2).sum(axis=1)))
        u = self.u[i]
        lado = u[0] * (q[1] - self.p[i, 1]) - u[1] * (q[0] - self.p[i, 0])
        return float(math.copysign(math.sqrt((d[i] ** 2).sum()), lado)), float(self.s[i])


def _trazar(tramos, inicio=(0.0, 0.0), rumbo=0.0, paso=0.01):
    """tramos: ("recta", largo) | ("arco", radio, grados con signo, + izquierda) | ("esquina", grados)."""
    p = [np.array(inicio, dtype=float)]
    th = rumbo
    for t in tramos:
        if t[0] == "recta":
            n = max(1, int(round(t[1] / paso)))
            for _ in range(n):
                p.append(p[-1] + paso * np.array([math.cos(th), math.sin(th)]))
        elif t[0] == "arco":
            r, ang = t[1], math.radians(t[2])
            n = max(1, int(round(abs(ang) * r / paso)))
            for _ in range(n):
                th += ang / n
                p.append(p[-1] + (abs(ang) * r / n) * np.array([math.cos(th), math.sin(th)]))
        elif t[0] == "esquina":
            th += math.radians(t[1])
    return np.array(p)


def pista_nivel(nivel, atras=0.3, sentido=+1):
    """Las pistas del PDF (seccion 4). El robot sale con la puntera en s = `atras`; la recta del nivel 1
    mide 4 m desde la puntera hasta la barra. `sentido` +1 gira a la izquierda, -1 a la derecha."""
    g = sentido
    if nivel == 1:
        return Pista(_trazar([("recta", atras + 4.0)]), nombre="nivel 1: recta de 4 m")
    if nivel == 2:
        return Pista(_trazar([("recta", atras + 2.0), ("arco", 2.0, 90 * g), ("recta", 2.0)]),
                     nombre="nivel 2: recta, curva de 90 grados con radio 2 m, recta")
    if nivel == 3:
        p = _trazar([("recta", atras + 1.0), ("arco", 1.2, 90 * g), ("arco", 1.2, -90 * g), ("recta", 1.0)])
        s_hueco = atras + 1.0 + 1.2 * math.pi / 2 + 0.3     # al principio de la segunda curva
        return Pista(p, huecos=[(s_hueco, s_hueco + 0.4)],
                     nombre="nivel 3: S con radios de 1.2 m e interrupcion de 40 cm")
    if nivel == 4:
        return Pista(_trazar([("recta", atras + 2.0), ("esquina", 90 * g), ("recta", 2.0)]),
                     nombre="nivel 4: esquina de 90 grados y 2 m de recta")
    raise ValueError(nivel)


# --- marcha ---------------------------------------------------------------------------------------------

MARCHA = {
    "factor": 1.14, "direccion_avance_deg": 7.5, "centro_giro_m": 0.14, "x_puntera_m": -0.019,
    "retardo_s": 0.15, "tau_giro_s": 0.30, "tau_v_s": 0.40, "ganancia_giro": 0.95,
    "deriva": 0.026, "balanceo_yaw_deg": 1.4, "zancada_hz": 0.7, "zona_muerta_sitio": 0.12,
    "v_minima": 0.0,      # andando no baja de esto (en el robot ~0.18 m/s: con vx 0.116 va a 0.17-0.20)
}


class RobotSimulado:
    """Interfaz de robot.Robot (mover, parar, imu, mando, edad_lowstate, motores_en_fallo)."""

    def __init__(self, x=0.0, y=0.0, rumbo=0.0, params=None, t0=0.0):
        self.p = dict(MARCHA, **(params or {}))
        self.t = t0
        self.psi = rumbo                       # rumbo del eje de la camara, sin el balanceo
        self.phi = math.radians(self.p["direccion_avance_deg"])
        d = self.p["centro_giro_m"]
        self.c = np.array([x, y]) - d * np.array([math.cos(rumbo), math.sin(rumbo)])   # centro de giro
        self.v = 0.0
        self.w = 0.0
        self._ordenes = deque()                # (t, vx, vy, vyaw)
        self._orden = (-10.0, 0.0, 0.0, 0.0)
        self.marcha_habilitada = True
        self.moves = 0
        self.stops = 0

    # interfaz de robot.Robot
    def habilitar_marcha(self):
        pass

    def mover(self, vx, vy, vyaw):
        self._ordenes.append((self.t, min(0.4, max(-0.4, vx)), vy, min(0.5, max(-0.5, vyaw))))
        self.moves += 1

    def parar(self):
        self._ordenes.append((self.t, 0.0, 0.0, 0.0))
        self.stops += 1

    def edad_lowstate(self):
        return 0.0

    def motores_en_fallo(self):
        return []

    def mando(self):
        return Mando(botones=0, lx=0.0, ly=0.0, rx=0.0, ry=0.0)

    def balanceo(self):
        if self.v < 0.03:
            return 0.0
        return math.radians(self.p["balanceo_yaw_deg"]) * math.sin(2 * math.pi * self.p["zancada_hz"] * self.t)

    def imu(self):
        yaw = envolver(self.psi + self.balanceo())
        return Imu(t=self.t, tick=int(self.t * 1000), roll=0.0, pitch=0.0, yaw=yaw, gx=0.0, gy=0.0, gz=self.w)

    # dinamica
    def avanzar(self, dt):
        p = self.p
        while self._ordenes and self._ordenes[0][0] <= self.t - p["retardo_s"]:
            self._orden = self._ordenes.popleft()
        t_o, vx, _vy, vyaw = self._orden
        if self.t - p["retardo_s"] - t_o > 1.0:        # Move dura 1 s
            vx, vyaw = 0.0, 0.0
        v_obj = max(p["v_minima"], p["factor"] * vx) if vx > 0 else 0.0
        anda = self.v > 0.03 or vx > 0
        w_obj = p["ganancia_giro"] * vyaw
        if not anda and abs(vyaw) < p["zona_muerta_sitio"]:
            w_obj = 0.0
        if anda and vx > 0:
            w_obj += p["deriva"]
        self.v += (v_obj - self.v) * min(1.0, dt / p["tau_v_s"])
        self.w += (w_obj - self.w) * min(1.0, dt / p["tau_giro_s"])
        self.psi += self.w * dt
        self.c = self.c + self.v * dt * np.array([math.cos(self.psi + self.phi), math.sin(self.psi + self.phi)])
        self.t += dt

    def pose_camara(self):
        d = self.p["centro_giro_m"]
        psi = self.psi + self.balanceo()
        x, y = self.c + d * np.array([math.cos(self.psi), math.sin(self.psi)])
        return float(x), float(y), psi

    def puntera(self):
        x, y, _ = self.pose_camara()
        return np.array([x, y]) + self.p["x_puntera_m"] * np.array([math.cos(self.psi), math.sin(self.psi)])


# --- camara ---------------------------------------------------------------------------------------------


class SensorSimulado:
    """La percepcion sin la imagen: por cada fila de la vista desde arriba (1 cm), el punto de cinta
    que la cruza, con ruido, y despues el MISMO ajuste y la misma regla de confianza que
    seguidor/percepcion.py (Percepcion._ajustar y _x_salida). Asi el lazo simulado hereda sus limites
    en las curvas cerradas."""

    PENDIENTE_MAX_DEG = 70.0     # mas torcida, la franja de 5 cm ya no responde en la fila

    def __init__(self, pista, cfg, rng=None, ruido_y=0.003, ruido_barra=0.01):
        from .geometria import Intrinsecos
        from .percepcion import Percepcion
        p = cfg["percepcion"]
        self.pista = pista
        self.rng = rng or np.random.default_rng(0)
        self.per = Percepcion(p, Intrinsecos(fx=378.8, fy=378.8, cx=321.5, cy=236.0), cfg["geometria"])
        self.p = p
        self.x_min, self.x_max, self.y_max = p["x_min_m"], p["x_max_m"], p["y_max_m"]
        self.L = p["distancia_objetivo_m"]
        self.ruido_y, self.ruido_barra = ruido_y, ruido_barra
        self.tapada = []          # intervalos de tiempo con la camara tapada

    def _camara(self, pose, pts):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        return np.column_stack([c * dx + s * dy, -s * dx + c * dy])

    def medir(self, pose, t):
        nada = MedidaLinea(t=t, y=math.nan, theta=math.nan, kappa=math.nan, objetivo=(math.nan, math.nan),
                           confianza=0.02, n_franjas=0)
        if any(t0 <= t <= t1 for t0, t1 in self.tapada):
            return nada
        pi, per = self.pista, self.per
        q = self._camara(pose, pi.p)
        u = self._camara((0.0, 0.0, pose[2]), pi.u)
        pend = np.abs(np.degrees(np.arctan2(u[:, 1], np.abs(u[:, 0]) + 1e-9)))
        vis = pi.cinta & (q[:, 0] >= self.x_min) & (q[:, 0] <= self.x_max) & \
            (np.abs(q[:, 1]) <= self.y_max - 0.05) & (pend < self.PENDIENTE_MAX_DEG)
        idx = np.flatnonzero(vis)
        if len(idx) < 8:
            return nada
        # una deteccion por fila: el punto de cinta mas cercano a la fila (la primera vez que la cruza)
        filas = np.round(q[idx, 0] / per.res).astype(int)
        _, primero = np.unique(filas, return_index=True)
        sel = idx[primero]
        xs = q[sel, 0]
        ys = q[sel, 1] + self.rng.normal(0, self.ruido_y, len(sel))
        a, b, c, dentro = per._ajustar(xs, ys)
        if not np.isfinite(a) or dentro.sum() < 8:
            return nada
        x_ini, x_fin = float(xs[dentro].min()), float(xs[dentro].max())
        s_fin = float(pi.s[sel[dentro]].max())

        barra = esquina = None
        if pi.barra and pi.largo - s_fin < 0.1:
            xb = q[-1, 0]
            if 0.28 <= xb <= 2.95:
                barra = round(float(xb + self.rng.normal(0, self.ruido_barra)), 3)
        for se, sentido, ie in pi.esquinas:
            if abs(se - s_fin) < 0.15 and 0.28 <= q[ie, 0] <= 2.95:
                lado = pi.cinta & (pi.s > se) & (pi.s < se + 0.6) & (np.abs(q[:, 1]) <= self.y_max) & (q[:, 0] >= self.x_min)
                if lado.sum() >= 30:
                    esquina = (round(float(q[ie, 0] + self.rng.normal(0, self.ruido_barra)), 3), sentido)
        # confianza: la regla de Percepcion._medir_signo con contraste pleno
        fin_linea = barra if barra is not None else (esquina[0] if esquina is not None else math.inf)
        x_lim = min(2.5, self.x_max, fin_linea + 0.05, per._x_salida(a, b, c))
        n_filas = max(1, int(np.sum((per.x_filas <= x_lim) & (per.x_filas >= self.x_min))))
        res = ys[dentro] - (a + b * xs[dentro] + c * xs[dentro] ** 2)
        c_filas = min(1.0, dentro.sum() / n_filas / 0.6)
        c_res = float(np.clip(1.0 - np.sqrt(np.mean(res ** 2)) / self.p["tolerancia_ajuste_m"], 0.0, 1.0))
        conf = c_filas * c_res
        L = min(self.L, x_fin)
        return MedidaLinea(t=t, y=float(a), theta=math.atan(b), kappa=2 * c / (1 + b * b) ** 1.5,
                           objetivo=(L, a + b * L + c * L * L), confianza=conf, n_franjas=4, barra_fin=barra,
                           esquina=esquina, ms=4.0, alcance=(x_ini, x_fin))


# --- lazo -----------------------------------------------------------------------------------------------


def salida(pista, desalineado_deg=10.0, lateral=0.0):
    """Pose de la camara para salir con la puntera en s = 0.3 m, girada `desalineado_deg` (+ izquierda)
    alrededor de los pies y desplazada `lateral` m (+ izquierda)."""
    i = int(np.searchsorted(pista.s, 0.3))
    u = pista.u[i]
    n = np.array([-u[1], u[0]])
    rumbo = math.atan2(u[1], u[0]) + math.radians(desalineado_deg)
    pies = pista.p[i] + lateral * n
    centro = pies - 0.10 * np.array([math.cos(rumbo), math.sin(rumbo)])   # centro de los pies, 10 cm detras de la puntera
    cam = centro + (0.10 - MARCHA["x_puntera_m"]) * np.array([math.cos(rumbo), math.sin(rumbo)])
    return float(cam[0]), float(cam[1]), rumbo


def simular(cfg, nivel, escala=0.5, desalineado_deg=10.0, lateral=0.0, sentido=+1, marcha=None, semilla=0,
            tapar=(), duracion=None, hz_fisica=500):
    """Una tirada simulada. Devuelve un dict con el resultado y las series para dibujarlo."""
    rng = np.random.default_rng(semilla)
    pista = pista_nivel(nivel, sentido=sentido)
    x, y, rumbo = salida(pista, desalineado_deg, lateral)
    robot = RobotSimulado(x, y, rumbo, marcha)
    imu = HistorialImu()
    estimador = Estimador(cfg, imu, cfg["geometria"].get("x_puntera_m") or 0.0)
    control = Control(cfg, escala)
    vigilante = Vigilante(robot, cfg)
    sup = Supervisor(cfg, robot, vigilante, control, estimador, nivel=nivel, simulacro=False)
    sensor = SensorSimulado(pista, cfg, rng)
    sensor.tapada = list(tapar)
    dur = duracion or cfg["supervisor"]["duracion_max_s"]

    dt = 1.0 / hz_fisica
    periodo = 1.0 / cfg["supervisor"]["hz_ordenes"]
    t_foto, t_orden, t_imu = 0.0, 0.0, 0.0
    pendientes = deque()
    ultimo_foto = 0.0
    serie = {k: [] for k in ("t", "estado", "vx", "vyaw", "error", "s", "conf", "edad", "dist_fin", "alfa")}
    camino = []
    sup.empezar(0.0)
    t_fin = None
    while robot.t < dur:
        robot.avanzar(dt)
        t = robot.t
        if t >= t_imu:
            s = robot.imu()
            imu.agregar(t, s.roll, s.pitch, s.yaw)
            t_imu += 0.002
        if t >= t_foto:                      # captura; llega 10 ms despues y se procesa en ~5 ms
            m = sensor.medir(robot.pose_camara(), t + 0.010)
            pendientes.append((t + 0.015, m))
            t_foto += 1.0 / 30
        while pendientes and pendientes[0][0] <= t:
            _, m = pendientes.popleft()
            estimador.medida(m)
            ultimo_foto = m.t
        if t >= t_orden:
            t_orden += periodo
            if not sup.terminado:
                dec = sup.paso(t, t - ultimo_foto)
                est = dec.detalle.get("est")
                err, s_pista = pista.cercano(robot.c)
                for k, v in (("t", t), ("estado", dec.estado), ("vx", dec.orden_enviada.vx),
                             ("vyaw", dec.orden_enviada.vyaw), ("error", err), ("s", s_pista),
                             ("conf", est.confianza if est else math.nan), ("edad", est.edad_s if est else math.nan),
                             ("dist_fin", est.dist_fin if (est and est.dist_fin is not None) else math.nan),
                             ("alfa", dec.detalle.get("alfa", math.nan))):
                    serie[k].append(v)
                if sup.terminado and t_fin is None:
                    t_fin = t
            camino.append((t, *robot.c, *robot.puntera()))
        if t_fin is not None and t - t_fin > 2.0 and robot.v < 0.005:
            break

    serie = {k: np.array(v) for k, v in serie.items()}
    camino = np.array(camino)
    res = {"nivel": nivel, "pista": pista.nombre, "escala": escala, "desalineado_deg": desalineado_deg,
           "lateral_m": lateral, "sentido": sentido, "estado_final": sup.estado, "motivo": sup.motivo,
           "transiciones": sup.transiciones, "tiempo_s": round(t_fin if t_fin is not None else robot.t, 2),
           "moves": robot.moves}
    # error lateral del centro de los pies, solo mientras se sigue
    sig = np.isin(serie["estado"], ["SEGUIMIENTO", "LINEA_PERDIDA"])
    err = np.abs(serie["error"][sig]) if sig.any() else np.array([math.nan])
    res["error_medio_m"] = float(np.mean(err))
    res["error_max_m"] = float(np.max(err))
    # pasada de la puntera respecto del borde cercano de la barra, ya parado
    toe = robot.puntera()
    pb, ub = pista.p[-1], pista.u[-1]
    res["pasada_m"] = float((toe - pb) @ ub)
    res["lateral_final_m"] = float(abs(pista.cercano(toe)[0]))
    dvy = np.diff(serie["vyaw"][sig]) if sig.sum() > 2 else np.array([0.0])
    res["suavidad_vyaw"] = float(np.sqrt(np.mean(dvy ** 2)))        # rad/s entre ordenes seguidas
    res["exito"] = bool(sup.estado == FIN and 0.0 <= res["pasada_m"] <= 0.30 and res["error_max_m"] < 0.20)
    res["serie"] = serie
    res["camino"] = camino
    res["pista_xy"] = pista.p
    return res
