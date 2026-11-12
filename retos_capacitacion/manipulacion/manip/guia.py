"""La bajada del toque (nivel 1) en lazo cerrado sobre lo medido.

El brazo no va donde se le manda: en el robot, 1-2 cm corrido de su consigna, y ese error cambia con la postura y con
hacia donde se mueve (RESULTADOS §43-48). En la tirada del 2026-11-12 la yema bajo 13 mm mas al fondo y 18 mm mas a la
izquierda que su consigna. Aqui la consigna es la postura de la recta del plan (q cada cm, alturas decrecientes:
Cinematica.recta) a la altura que toca mas una correccion que integra lo que la postura medida se aparta de ella
(en articulares, junta a junta): con el error del brazo compensado, la yema medida va por la recta sobre la diana.
El supervisor la manda con fijar (sin trayectoria, pasos pequenos) cada `cada` s.

Con el brazo como un primer orden de constante tau y la ganancia k (1/s), el lazo tiene zeta = 1 / (2 sqrt(k tau)):
con k = 1.5 y tau hasta 0.5 s (lo que tarda en asentarse al parar), zeta >= 0.58.

El brazo se pega y se suelta (rozamiento estatico: baja a saltos de 4-6 mm a ~30 mm/s, guiado o no, RESULTADOS §50).
La correccion se integra tambien en marcha: con la de reposo congelada al bajar, la yema se iba hasta 2 cm de la
vertical en el ultimo tramo (§51). Cada consigna se aparta de la anterior como mucho `salto_max` (fijar rechaza mas de
0.05 rad): al parar con el brazo pegado 1 cm por encima de su consigna, la consigna a la altura medida saltaba 0.09 rad
en el codo (§51).
"""

import numpy as np


class Guia:
    def __init__(self, Q, H, ganancia=1.5, paso_max=0.005, corr_max=0.15, salto_max=0.04):
        """Q: [q7] de la recta, de arriba abajo; H: la altura de la yema de cada uno (estrictamente decreciente).
        ganancia: 1/s; paso_max: rad por ciclo que puede cambiar la correccion; corr_max: rad, su tope por junta;
        salto_max: rad, lo que una consigna se aparta como mucho de la anterior (la primera, de Q[0]: el brazo llega
        a aproximar con ir_a)."""
        self.Q, self.H = np.asarray(Q, float), np.asarray(H, float)
        if len(self.H) < 2 or np.any(np.diff(self.H) >= 0):
            raise ValueError("la recta tiene que bajar")
        self.k, self.paso_max, self.corr_max = float(ganancia), float(paso_max), float(corr_max)
        self.corr = np.zeros(self.Q.shape[1])
        self.salto_max = float(salto_max)
        self.ultima = self.Q[0].copy()

    @property
    def arriba(self):
        return float(self.H[0])

    @property
    def abajo(self):
        return float(self.H[-1])

    def q_en(self, h):
        """La postura de la recta con la yema a la altura h (interpolada; fuera, el extremo)."""
        h = float(np.clip(h, self.H[-1], self.H[0]))
        return np.array([np.interp(-h, -self.H, self.Q[:, i]) for i in range(self.Q.shape[1])])

    def consigna(self, h, q_medido, dt, integrar=True):
        """La consigna para llevar la yema a la altura h de la recta, con la correccion actualizada por q_medido
        (integrar=False: la correccion se queda como esta)."""
        q_d = self.q_en(h)
        if integrar:
            paso = np.clip(self.k * float(dt) * (q_d - np.asarray(q_medido, float)), -self.paso_max, self.paso_max)
            self.corr = np.clip(self.corr + paso, -self.corr_max, self.corr_max)
        q = self.ultima + np.clip(q_d + self.corr - self.ultima, -self.salto_max, self.salto_max)
        self.ultima = q
        return q
