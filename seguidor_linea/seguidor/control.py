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
        self.dvx, self.dvyaw = c["dvx_max"], c["dvyaw_max"]
        self.cerca_barra, self.vx_cerca_barra = c["cerca_barra_m"], c["vx_cerca_barra"]
        self.phi = math.radians(e["direccion_avance_deg"])
        self.factor = e["factor_velocidad"]
        self.d_giro = e["centro_giro_m"]
        self.reiniciar()

    def reiniciar(self):
        self._t = None
        self._vx = 0.0
        self._vyaw = 0.0
        self._integral = 0.0

    def alfa(self, est: EstadoLinea):
        ox, oy = est.objetivo
        a = math.atan2(oy, ox + self.d_giro) - self.phi
        return math.atan2(math.sin(a), math.cos(a))

    def calcular(self, t, est: EstadoLinea, factor_vx=1.0):
        """Orden para este ciclo y lo que la explica (para el registro)."""
        dt = 0.05 if self._t is None else min(0.2, max(0.0, t - self._t))
        self._t = t
        ox, oy = est.objetivo
        if not (est.hay_linea and math.isfinite(ox) and math.isfinite(oy)):
            return self._rampa(dt, 0.0, 0.0), {"alfa": math.nan, "vx_deseada": 0.0}
        Ld = max(0.3, math.hypot(ox + self.d_giro, oy))
        alfa = self.alfa(est)

        curv = self.ganancia * 2.0 * math.sin(alfa) / Ld
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
        sesgo = self.sesgo if vx > 0 else 0.0
        hueco = self.vyaw_max - abs(sesgo)
        if abs(curv) * self.factor * vx > hueco:      # el giro no cabe: mas despacio, misma curvatura
            vx = hueco / (abs(curv) * self.factor)

        vx_d = vx
        if self._vx > 0 and vx_d > 0:
            self._integral = _recortar(self._integral + alfa * dt, 0.5)
        vyaw = curv * self.factor * vx_d + sesgo + self.ki * self._integral
        orden = self._rampa(dt, vx_d, vyaw)
        return orden, {"alfa": alfa, "vx_deseada": vx_d, "curvatura": curv, "h_conf": h, "g_curv": g}

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
