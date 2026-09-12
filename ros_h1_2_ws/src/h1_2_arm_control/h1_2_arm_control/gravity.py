"""Compensación de gravedad por modelo identificado.

Predice el par que hace falta para sostener cada articulación contra la
gravedad, para pasarlo como `tau_ff`. Sin él, el PD solo puede generar ese par
a costa de un error permanente `tau_g/kp` que no se va nunca — que es lo que
tiene a los hombros con 1° de caída y a kp pegado a su techo de saturación.

Los parámetros NO son los del URDF. Se identifican con `12_gravity_map.py` +
`13_gravity_fit.py`, porque los del URDF dan 2.57 Nm de error rms contra los
0.34 del ajuste. La cinemática sí es la del URDF: son longitudes y ejes, y eso
un URDF suele tenerlo bien.
"""
from __future__ import annotations

import glob
import json
from pathlib import Path

import numpy as np

from .gains import _config_path
from .joints import BY_INDEX, NUM_CMD_MOTOR

def _urdf() -> Path:
    """Ruta del URDF con las manos Inspire.

    Viene del repositorio `oscar-ramos/h1_2_utec` (el paquete de descripción),
    que NO es este. En otra máquina la ruta cambia, así que se puede fijar con
    la variable de entorno `H12_URDF`.
    """
    import os
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


URDF = None   # se resuelve al construir el modelo


class GravityModel:
    """`g(q27) -> {idx: tau}` con los parámetros de masa identificados.

    Si no hay fichero de parámetros para un brazo, ese brazo usa los del URDF y
    se avisa: es mejor una compensación imperfecta y anunciada que ninguna, pero
    conviene saber cuál se está usando.
    """

    def __init__(self, params_files=None, verbose=True):
        import pinocchio as pin
        self._pin = pin
        self.urdf = _urdf()
        self.model = pin.buildModelFromUrdf(str(self.urdf))
        self.data = self.model.createData()
        self.fuentes: dict[str, str] = {}

        ficheros = ([Path(f) for f in params_files] if params_files
                    else [Path(f) for f in sorted(glob.glob(
                        str(_config_path().parent / "gravity_params_*.json")))])
        # el más reciente de cada brazo gana
        elegido: dict[str, Path] = {}
        for f in ficheros:
            try:
                d = json.loads(f.read_text())
            except Exception:
                continue
            elegido[d.get("arm", "left")] = f

        for brazo, f in sorted(elegido.items()):
            d = json.loads(f.read_text())
            pi = np.asarray(d["params"], dtype=float)
            for k in d["bodies"]:
                m_, mcx, mcy, mcz = pi[4 * k:4 * k + 4]
                if m_ <= 1e-9:
                    continue
                I = self.model.inertias[k + 1]
                self.model.inertias[k + 1] = pin.Inertia(
                    float(m_), np.array([mcx, mcy, mcz]) / m_, I.inertia)
            self.fuentes[brazo] = f.name
            if verbose:
                print(f"  gravedad: brazo {brazo} con parámetros de {f.name} "
                      f"(rms {d['rms_prior']:.2f} -> {d['rms_fit']:.2f} Nm)")
        self.data = self.model.createData()

        faltan = {"left", "right"} - set(self.fuentes)
        if faltan and verbose:
            print(f"  ⚠ gravedad: sin parámetros identificados para "
                  f"{', '.join(sorted(faltan))}; ese brazo usa los del URDF, "
                  f"que dan ~2.6 Nm de error rms")

        self._idx_q, self._idx_v = {}, {}
        for i in range(NUM_CMD_MOTOR):
            n = BY_INDEX[i].urdf
            if self.model.existJointName(n):
                j = self.model.joints[self.model.getJointId(n)]
                self._idx_q[i] = j.idx_q
                self._idx_v[i] = j.idx_v
        self._q = pin.neutral(self.model)

    def tau(self, q27) -> dict[int, float]:
        """Par de gravedad por motor, en Nm, para esa configuración."""
        for i, k in self._idx_q.items():
            self._q[k] = float(q27[i])
        t = self._pin.computeGeneralizedGravity(self.model, self.data, self._q)
        return {i: float(t[v]) for i, v in self._idx_v.items()}
