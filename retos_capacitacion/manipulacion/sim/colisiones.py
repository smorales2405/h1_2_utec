"""Colisiones en el gemelo: el robot contra la mesa (tablero, labio, asas, carrito), las bandejas y los objetos,
y contra si mismo. Devuelve la distancia minima por pareja de cuerpos, con un margen (la holgura del plan).

Como modelo_h1_2_manos.Colisiones: los pares del robot que ya se tocan con los brazos en reposo se
excluyen, y los que ya estan cerca solo avisan si chocan. Dentro de una misma mano los dedos quedan siempre
cerca: solo cuentan si chocan. MuJoCo choca con la envolvente CONVEXA de cada malla: puede avisar de mas
en las zonas concavas (palma, torso), nunca de menos.
"""

import mujoco
import numpy as np

from h1_2_mujoco import modelo_h1_2_manos as mm


def _nombre(m, b):
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)


def es_entorno(nombre):
    return nombre in ("world", "mesa") or nombre.startswith(("bandeja_", "obj_"))


def grupo(nombre):
    if nombre == "world":
        return "suelo"
    if es_entorno(nombre):
        return nombre.split("_")[0] if nombre.startswith("bandeja_") else nombre
    return mm.grupo(nombre)


class Colisiones:
    def __init__(self, m, estado, margen=0.02):
        """`estado`: un sim.escena.Estado con el robot en su postura de pie y los brazos en reposo."""
        self.m, self.e, self.margen = m, estado, float(margen)
        for g in range(m.ngeom):
            if m.geom_contype[g] or m.geom_conaffinity[g]:
                m.geom_margin[g] = self.margen
        base = self._pares()
        robot = {p: d for p, d in base.items() if not (es_entorno(p[0]) or es_entorno(p[1]))}
        self.excluidos = {p for p, d in robot.items() if d < 0.0}
        self.solo_choque = {p for p, d in robot.items() if d >= 0.0}

    def como_robot(self, cuerpo, si):
        """El objeto que lleva la mano choca como parte del robot (contra mesa y bandejas); si no, es entorno."""
        b = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, cuerpo)
        for g in range(self.m.ngeom):
            if self.m.geom_bodyid[g] == b:
                self.m.geom_contype[g], self.m.geom_conaffinity[g] = (1, 3) if si else (2, 1)

    def _pares(self, ignorar=()):
        self.e.adelante()
        d, m = self.e.d, self.m
        pares = {}
        for c in d.contact[:d.ncon]:
            a, b = _nombre(m, m.geom_bodyid[c.geom1]), _nombre(m, m.geom_bodyid[c.geom2])
            if a in ignorar or b in ignorar:
                continue
            par = tuple(sorted((a, b)))
            pares[par] = min(pares.get(par, np.inf), float(c.dist))
        return pares

    def revisar(self, ignorar=(), mano_con=None):
        """[(a, b, dist)] de choques (< 0) y acercamientos (< margen) con el estado actual, de peor a mejor.
        `ignorar`: cuerpos que no cuentan (p. ej. el objeto que se va a coger, en el descenso final).
        `mano_con`: el objeto que lleva una mano: su contacto con esa mano (y la muneca) no cuenta."""
        sal = []
        for (a, b), dist in self._pares(ignorar).items():
            if (a, b) in self.excluidos:
                continue
            if mano_con is not None and mano_con in (a, b):
                otro = b if a == mano_con else a
                if grupo(otro).startswith("mano") or "wrist" in otro:
                    continue
            entorno = es_entorno(a) or es_entorno(b)
            if entorno and (a == "world" or b == "world") and not (grupo(a).startswith("brazo") or
                                                                    grupo(a).startswith("mano") or
                                                                    grupo(b).startswith("brazo") or
                                                                    grupo(b).startswith("mano")):
                continue                    # pies y suelo: no es asunto del brazo
            misma_mano = grupo(a) == grupo(b) and grupo(a).startswith("mano")
            cerca = dist < self.margen and (a, b) not in self.solo_choque and not misma_mano
            if dist < 0.0 or cerca:
                sal.append((a, b, dist))
        return sorted(sal, key=lambda x: x[2])

    def libre(self, ignorar=()):
        return not self.revisar(ignorar)

    def distancia_minima(self, ignorar=()):
        h = self.revisar(ignorar)
        return h[0][2] if h else self.margen


def informe(hallazgos):
    lineas = []
    for a, b, dist in hallazgos[:6]:
        lineas.append(f"{'[CHOQUE]' if dist < 0 else '[CERCA] '} {grupo(a)} <-> {grupo(b)}: {a} / {b} {dist * 1000:+.0f} mm")
    return lineas
