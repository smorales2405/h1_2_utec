"""Comprobación de autocolisión sobre el modelo geométrico del URDF.

Por qué existe, y por qué solo se usa aquí. Los topes de `config/gains.yaml`
—`soft_limits_deg` y `shoulder_roll_vs_elbow_deg`— describen bien la envolvente
segura **cuando se mueve una articulación cada vez**, que es como se hicieron
todos los ensayos hasta F5. Pero un tope por articulación **no puede** expresar
una autocolisión, que por definición depende de la configuración completa: dos
posturas seguras por separado pueden tener entre ellas una trayectoria que no lo
sea.

F6 mueve las siete a la vez. Ahí hace falta comprobar la trayectoria entera,
punto a punto, ANTES de ejecutarla.

Dos cautelas, las dos medidas:

* La geometría de colisión del URDF puede estar simplificada respecto a la
  pieza real. El modelo dijo que el primer contacto en la postura de ensayo es
  a +4° de `shoulder_roll` y el operador midió que ±10° va bien: coinciden en
  el orden pero no son la misma superficie. **Este módulo no sustituye a los
  topes**, los complementa.
* Los pares que ya chocan en la postura neutra son eslabones adyacentes, no
  colisiones: se descartan al construir. Sin eso, `pelvis` contra `torso_link`
  aparece en todas las configuraciones.
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from .joints import BY_INDEX, NUM_CMD_MOTOR


def _urdf() -> Path:
    v = os.environ.get("H12_URDF")
    if v:
        return Path(v)
    for c in (Path.home() / "humanoid_ws/src/h1_2_utec/h1_2_description/urdf/h1_2.urdf",
              Path.home() / "h1_2_utec/h1_2_description/urdf/h1_2.urdf",
              Path.home() / "ros2_ws/src/h1_2_utec/h1_2_description/urdf/h1_2.urdf"):
        if c.exists():
            return c
    raise FileNotFoundError(
        "no encuentro el URDF del H1-2 con manos Inspire.\n"
        "  Viene de github.com/oscar-ramos/h1_2_utec (h1_2_description).\n"
        "  Clónalo, o indica la ruta con:  export H12_URDF=/ruta/a/h1_2.urdf")


class CollisionModel:
    """Distancia mínima entre pares de geometrías, para una postura de 27."""

    def __init__(self, verbose: bool = True, adyacencia: float = 0.010):
        import pinocchio as pin
        self._pin = pin
        urdf = _urdf()
        self.model = pin.buildModelFromUrdf(str(urdf))
        # `package_dirs` POR NOMBRE: en posicional cae en `geometry_model` y
        # pinocchio avisa pero sigue, dejando las mallas sin encontrar.
        self.geom = pin.buildGeomFromUrdf(
            self.model, str(urdf), pin.GeometryType.COLLISION,
            package_dirs=str(urdf.parent.parent.parent))
        self.geom.addAllCollisionPairs()
        self.data = self.model.createData()
        gd = self.geom.createData()

        # Se descartan los pares que en la postura NEUTRA ya están pegados.
        #
        # Filtrar por `isCollision()` no basta, y el fallo es sutil: dos
        # falanges contiguas del pulgar se tocan sin penetrar, así que
        # `isCollision()` es False pero `min_distance` es exactamente 0.0. Como
        # el chequeo devuelve el mínimo sobre todos los pares, esas falanges
        # ganaban siempre y tapaban lo único que interesa, que es el brazo
        # contra el cuerpo. Medido: las tres posturas de F3 daban 0.0 mm.
        #
        # Con el umbral por distancia se van todas las adyacencias reales, las
        # que penetran y las que solo se rozan.
        pin.computeDistances(self.model, self.data, self.geom, gd,
                             pin.neutral(self.model))
        malos = [k for k in range(len(self.geom.collisionPairs))
                 if gd.distanceResults[k].min_distance < adyacencia]
        for k in sorted(malos, reverse=True):
            self.geom.removeCollisionPair(self.geom.collisionPairs[k])
        self.gdata = self.geom.createData()
        if verbose:
            print(f"  colisión: {len(self.geom.collisionPairs)} pares vigilados "
                  f"({len(malos)} descartados por estar a menos de "
                  f"{adyacencia*1000:.0f} mm en la postura neutra)")

    def _q(self, q27):
        q = self._pin.neutral(self.model)
        for i in range(NUM_CMD_MOTOR):
            n = BY_INDEX[i].urdf
            if self.model.existJointName(n):
                q[self.model.joints[self.model.getJointId(n)].idx_q] = float(q27[i])
        return q

    def clearance(self, q27) -> tuple[float, str, str]:
        """(distancia mínima en m, geometría A, geometría B). Negativa = choque."""
        self._pin.computeDistances(self.model, self.data, self.geom,
                                   self.gdata, self._q(q27))
        d = [(self.gdata.distanceResults[k].min_distance, k)
             for k in range(len(self.geom.collisionPairs))]
        mn, k = min(d)
        cp = self.geom.collisionPairs[k]
        return (float(mn), self.geom.geometryObjects[cp.first].name,
                self.geom.geometryObjects[cp.second].name)

    def check_path(self, puntos, margen: float = 0.02):
        """Comprueba una lista de posturas de 27. Devuelve (ok, peor, info).

        `margen` en metros: no basta con «no choca», hace falta holgura. 20 mm
        por defecto, que es del orden de lo que el URDF puede estar
        simplificando.
        """
        peor = (1e9, "", "", -1)
        for n, q in enumerate(puntos):
            d, a, b = self.clearance(q)
            if d < peor[0]:
                peor = (d, a, b, n)
        return peor[0] >= margen, peor[0], peor

    def sample_path(self, q_de_t, duracion: float, n: int = 200):
        """Muestrea `q_de_t(t) -> q27` en `n` instantes del intervalo."""
        return [np.asarray(q_de_t(t), dtype=float)
                for t in np.linspace(0.0, duracion, n)]
