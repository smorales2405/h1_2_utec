"""Control (Hito 3): EstadoLinea -> Orden saturada.

Persecucion de un punto adelantado (pure pursuit, 6.3.3) desde el CENTRO DE GIRO y sobre la DIRECCION
REAL DE AVANCE: el robot avanza `direccion_avance_deg` a la izquierda del eje de la camara y gira
alrededor de un punto `centro_giro_m` por detras de ella (RESULTADOS.md 3). Ese punto es el que
recorre el arco del pure pursuit; la camara, 14 cm por delante, se desplaza ademas w*d hacia dentro
en las curvas (~6 grados en la S del nivel 3), y aplicar la ley en la camara recortaba ~20 cm en el
simulador. Con alfa el angulo entre la direccion de avance y el punto adelantado, a distancia L del
centro de giro:

    vyaw = ganancia * 2 v sin(alfa) / L + sesgo (+ ki * integral de alfa)

con v = factor x vx la velocidad real. El sesgo compensa la deriva a la izquierda que se mide andando
(~0.026 rad/s); sin el, el proporcional se quedaria ~4 cm fuera de la linea. Con el retardo medido
(0.3-0.45 s) y L = 1 m a 0.23 m/s, la frecuencia natural del lazo es ~0.3 rad/s y el retardo le
quita menos de 10 grados de fase.

vx (6.3.5) = vx_max x escala x 1/(1 + k|curvatura pedida|) x h(confianza) x cos(alfa), con h lineal entre
conf_minima (0) y conf_plena (1), mas lenta cerca de la barra, y en rampa. Si el giro pedido no cabe
en vyaw_max, se baja vx para mantener la curvatura. vy queda a 0 (6.3.4: se corrige con vyaw).

La escala (primeras tiradas, a la mitad) se aplica a vx y a vyaw_max: a mitad de velocidad la misma
curva pide la mitad de giro.
"""

import math

from .mensajes import EstadoLinea, Orden


def _recortar(v, tope):
    return max(-tope, min(tope, v))


class Control:
    def __init__(self, cfg, escala=1.0):
        c, lim, e = cfg["control"], cfg["limites"], cfg["estimacion"]
        if not 0.0 < escala <= 1.0:
            raise ValueError("la escala tiene que estar en (0, 1]")
        self.escala = escala
        self.vx_max = min(c["vx"], lim["vx_max"]) * escala
        self.vyaw_max = lim["vyaw_max"] * escala
        self.vy = _recortar(c.get("vy", 0.0), lim["vy_max"] * escala)
        self.L = c["distancia_objetivo_m"]
        self.ganancia = c["ganancia_pp"]
        self.sesgo = c["vyaw_sesgo"]
        self.ki = c["ki"]
        self.k_curv = c["k_curvatura"]
        self.conf_min, self.conf_plena = c["conf_minima"], c["conf_plena"]
        self.frenar_alfa = c.get("frenar_alfa", True)
        self.girar_primero = math.radians(c.get("girar_primero_deg", 0.0))   # 0 = no
        self.pausa_alfa = math.radians(c.get("pausa_giro_deg", 0.0))          # 0 = no
        self.pausa_perdida = math.radians(c.get("pausa_perdida_deg", 0.0))    # 0 = no
        self.pausa_rumbo = math.radians(c.get("pausa_perdida_rumbo_deg", 0.0))  # 0 = no
        self.pausa_rumbo_edad = c.get("pausa_perdida_rumbo_edad_s", 0.0)
        self.vyaw_busqueda = c.get("vyaw_busqueda", 0.0)                      # 0 = no
        self.vx_busqueda = c.get("vx_busqueda", 0.0)                          # 0 = en el sitio
        self.vx_tras_pausa = c.get("vx_tras_pausa", 0.0)                      # 0 = no
        self.t_tras_pausa = c.get("t_tras_pausa_s", 0.0)
        self.dvx, self.dvyaw = c["dvx_max"], c["dvyaw_max"]
        self.cerca_barra, self.vx_cerca_barra = c["cerca_barra_m"], c["vx_cerca_barra"]
        self.phi = math.radians(e["direccion_avance_deg"])
        self.factor = e["factor_velocidad"]
        self.v_minima = e.get("v_minima", 0.0)
        self.d_giro = e["centro_giro_m"]
        self.reiniciar()

    def reiniciar(self):
        self._arranque = True        # primera orden tras reiniciar: decide si se gira en el sitio
        self._girando = False
        self._pausa = False          # girando en el sitio a mitad de recorrido (pausa_giro_deg, pausa_perdida_deg)
        self._pausa_umbral = 0.0
        self._pausa_rumbo = False    # la pausa mira tambien el rumbo de la linea (perdida y de lado)
        self._pausa_perdida = False  # la pausa empezo con la linea perdida
        self._pausa_signo = 1.0
        self._t_fin_pausa = -math.inf
        self._t = None
        self._vx = 0.0
        self._vyaw = 0.0
        self._integral = 0.0

    def alfa(self, est: EstadoLinea):
        ox, oy = est.objetivo
        a = math.atan2(oy, ox + self.d_giro) - self.phi
        return math.atan2(math.sin(a), math.cos(a))

    def calcular(self, t, est: EstadoLinea, factor_vx=1.0, perdida=False):
        """Orden para este ciclo y lo que la explica (para el registro). `perdida`: el supervisor esta en
        LINEA_PERDIDA (la linea es la estimada, sin medida reciente)."""
        dt = 0.05 if self._t is None else min(0.2, max(0.0, t - self._t))
        self._t = t
        ox, oy = est.objetivo
        if not (est.hay_linea and math.isfinite(ox) and math.isfinite(oy)):
            return self._rampa(dt, 0.0, 0.0), {"alfa": math.nan, "vx_deseada": 0.0}
        Ld = max(0.3, math.hypot(ox + self.d_giro, oy))
        alfa = self.alfa(est)
        # rumbo de la linea frente a la direccion de avance (0 si no se sabe)
        rumbo = math.atan2(math.sin(est.theta - self.phi), math.cos(est.theta - self.phi)) \
            if math.isfinite(est.theta) else 0.0
        if ox + self.d_giro < 0.05:
            # objetivo detras del centro de giro (linea estimada agotada): alfa no dice nada, vale el rumbo
            alfa = rumbo
        if self._arranque:
            # salida muy girada (|alfa| > girar_primero al empezar): girar en el sitio hasta la mitad antes de
            # andar; andando se abre hasta que el giro llega (16.7 cm saliendo girado a la izquierda, 2026-10-10)
            self._girando = self.girar_primero > 0 and abs(alfa) > self.girar_primero
            self._arranque = False
        if self._girando:
            if abs(alfa) > self.girar_primero / 2:
                vyaw = _recortar(1.2 * alfa, self.vyaw_max)
                if abs(vyaw) < 0.15:                 # en el sitio, por debajo de ~0.15 rad/s apenas gira
                    vyaw = math.copysign(0.15, alfa)
                return self._rampa(dt, 0.0, vyaw), {"alfa": alfa, "vx_deseada": 0.0, "girando": True}
            self._girando = False

        curv = self.ganancia * 2.0 * math.sin(alfa) / Ld
        # si el giro no cabe ni a la velocidad minima de la marcha (no anda mas despacio de v_minima), andar solo
        # abre la curva: se para y gira en el sitio hasta la mitad de pausa_giro_deg (S de radio 1 m a media
        # escala, 2026-11-07: vyaw al tope 13 s, el robot 12-22 cm por fuera y la linea perdida al cambiar de curva)
        # y con la linea perdida y la estimada hacia un lado, andar solo aleja de ella: tambien se para y gira hacia
        # ella (S de 1 m, 2026-11-07: perdida al cambiar de curva, seguia andando recto mientras la cinta se curvaba)
        # La linea perdida con alfa pequeno pero la cinta de lado (> pausa_perdida_rumbo_deg de la marcha) tambien:
        # el objetivo cae en la prolongacion recta de una curva (S de 1 m, 2026-11-07: alfa 4-12 grados, cinta a
        # +21-26, anduvo a ciegas 2-4 s fuera de la curva). Entonces gira hasta alinearse tambien con la cinta.
        no_cabe = self.v_minima > 0 and abs(curv) * self.v_minima > self.vyaw_max - abs(self.sesgo)
        if not self._pausa:
            if self.pausa_alfa > 0 and no_cabe and abs(alfa) > self.pausa_alfa:
                self._empezar_pausa(self.pausa_alfa, False, False, alfa)
            elif self.pausa_perdida > 0 and perdida and abs(alfa) > self.pausa_perdida:
                self._empezar_pausa(self.pausa_perdida, False, True, alfa)
            elif self.pausa_rumbo > 0 and perdida and abs(rumbo) > self.pausa_rumbo and rumbo * alfa > 0 \
                    and est.edad_s >= self.pausa_rumbo_edad:
                self._empezar_pausa(self.pausa_perdida or self.pausa_rumbo, True, True, alfa)
        buscando = False
        if self._pausa:
            error = alfa
            if self._pausa_rumbo and rumbo * alfa > 0 and abs(rumbo) > abs(alfa):
                error = rumbo
            if abs(error) < self._pausa_umbral / 2:
                # alineado con la linea ESTIMADA (la prolongacion recta de lo ultimo visto). Si sigue perdida, en una
                # curva cerrada la cinta sigue doblando hacia el mismo lado: seguir girando despacio hacia el hasta
                # verla, no andar recto (S de 1 m, 2026-11-07: anduvo a ciegas 3 s alejandose; la cinta, a los pies,
                # iba casi de lado). Lo acota t_perdida_s del supervisor.
                if perdida and self._pausa_perdida and self.vyaw_busqueda > 0:
                    buscando = True
                else:
                    self._pausa = False
                    self._t_fin_pausa = t
        if self._pausa:
            if buscando:
                # en arco hacia el mismo lado, como si la curva siguiera (en el sitio no cruza un hueco en curva)
                vyaw = math.copysign(self.vyaw_busqueda, self._pausa_signo)
                return self._rampa(dt, self.vx_busqueda, vyaw + (self.sesgo if self.vx_busqueda > 0 else 0.0)), \
                    {"alfa": alfa, "vx_deseada": self.vx_busqueda, "girando": True, "pausa": True, "busqueda": True}
            else:
                vyaw = _recortar(1.2 * error, self.vyaw_max)
                if abs(vyaw) < 0.15:
                    vyaw = math.copysign(0.15, error)
            return self._rampa(dt, 0.0, vyaw), {"alfa": alfa, "vx_deseada": 0.0, "girando": True, "pausa": True,
                                                "busqueda": buscando}
        h = min(1.0, max(0.0, (est.confianza - self.conf_min) / (self.conf_plena - self.conf_min)))
        if est.dist_fin is not None and est.dist_fin < self.cerca_barra:
            # barra confirmada delante: la confianza baja solo porque queda poca linea antes de ella, y en
            # el tramo a ciegas es la de una medida vieja; no debe frenar (2026-10-06: 0.04 m/s al final)
            h = 1.0
        # la curvatura que se va a pedir, no la del ajuste: en las curvas cerradas la parabola la
        # sobreestima (1.5-2.5 frente a 0.83 en el nivel 3 simulado) y frenaba hasta 0.06 m/s
        g = 1.0 / (1.0 + self.k_curv * abs(curv))
        vx = self.vx_max * g * h * factor_vx
        if self.frenar_alfa:
            vx *= max(0.3, math.cos(alfa))
        if est.dist_fin is not None and est.dist_fin < self.cerca_barra:
            vx *= self.vx_cerca_barra
        if self.vx_tras_pausa > 0 and t - self._t_fin_pausa < self.t_tras_pausa:
            # recien girado en el sitio en una curva cerrada: sin acelerar (2026-11-07: tras la pausa, la linea
            # cercana recta, alfa ~0 y vx 0.35; a los 0.6 s la cinta otra vez de lado)
            vx = min(vx, self.vx_tras_pausa)
        sesgo = self.sesgo if vx > 0 else 0.0
        hueco = self.vyaw_max - abs(sesgo)
        if abs(curv) * self.v_real(vx) > hueco:      # el giro no cabe: mas despacio, misma curvatura
            vx = hueco / (abs(curv) * self.factor)

        vx_d = vx
        if self._vx > 0 and vx_d > 0:
            self._integral = _recortar(self._integral + alfa * dt, 0.5)
        vyaw = _recortar(curv * self.v_real(vx_d), hueco) + sesgo + self.ki * self._integral
        orden = self._rampa(dt, vx_d, vyaw)
        return orden, {"alfa": alfa, "vx_deseada": vx_d, "curvatura": curv, "h_conf": h, "g_curv": g}

    def _empezar_pausa(self, umbral, por_rumbo, con_perdida, alfa):
        self._pausa, self._pausa_umbral, self._pausa_rumbo = True, umbral, por_rumbo
        self._pausa_perdida, self._pausa_signo = con_perdida, math.copysign(1.0, alfa)

    def v_real(self, vx):
        """Velocidad real del robot con vx mandada: factor x vx, pero andando no baja de v_minima."""
        return max(self.v_minima, self.factor * vx) if vx > 0 else 0.0

    def _rampa(self, dt, vx, vyaw):
        dvx = vx - self._vx
        self._vx += _recortar(dvx, (2.0 if dvx < 0 else 1.0) * self.dvx * dt)
        self._vx = min(self.vx_max, max(0.0, self._vx))
        self._vyaw += _recortar(vyaw - self._vyaw, self.dvyaw * dt)
        self._vyaw = _recortar(self._vyaw, self.vyaw_max)
        return Orden(vx=self._vx, vy=self.vy if self._vx > 0 else 0.0, vyaw=self._vyaw)

    def girar(self, t, error, kp, minimo):
        """Giro en el sitio hacia un rumbo (esquina del nivel 4), como cuadrado.py."""
        dt = 0.05 if self._t is None else min(0.2, max(0.0, t - self._t))
        self._t = t
        vyaw = kp * error
        if abs(vyaw) < minimo:
            vyaw = math.copysign(minimo, error)
        self._vx = 0.0
        self._vyaw += _recortar(_recortar(vyaw, self.vyaw_max) - self._vyaw, 2 * self.dvyaw * dt)
        return Orden(vx=0.0, vy=0.0, vyaw=_recortar(self._vyaw, self.vyaw_max))
