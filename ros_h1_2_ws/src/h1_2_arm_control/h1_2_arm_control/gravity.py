"""Compensación de gravedad por modelo identificado.

Predice el par que hace falta para sostener cada articulación contra la
gravedad, para pasarlo como `tau_ff`. Sin él, el PD solo puede generar ese par
a costa de un error permanente `tau_g/kp` que no se va nunca — que es lo que
tiene a los hombros con 1° de caída y a kp pegado a su techo de saturación.

Los parámetros NO son los del URDF: se identificaron sobre el robot, porque
los del URDF dan 2.57 Nm de error rms contra los 0.34 del ajuste. Y no es solo
cuestión de precisión —el robot carga además el conector y el cable de la mano,
que ningún URDF modela: la masa identificada, 1.14 kg, supera a la de cualquier
URDF disponible—.

La cinemática sí es la del URDF: son longitudes y ejes, y eso un URDF suele
tenerlo bien. Por eso cambiar de modelo mueve el par 0.47 Nm como mucho.
"""
from __future__ import annotations

import glob
import json
from pathlib import Path

import numpy as np

from .gains import _config_path
from .joints import BY_INDEX, NUM_CMD_MOTOR

# Paquete y fichero del modelo. Viven en `h1_2_inspire_description`, que es
# parte de este workspace: así todo lo necesario para trabajar con el robot
# está en un solo sitio y no depende de dónde tenga cada uno sus repos.
URDF_PKG = "h1_2_inspire_description"
URDF_FILE = "h1_2_with_RH56DFTP_hands.urdf"


def _urdf() -> Path:
    """Ruta del URDF del H1-2 con las manos Inspire.

    Se resuelve por el índice de ament, o sea desde el `share/` del paquete
    instalado. `H12_URDF` lo fuerza, que es lo que hace falta para probar otro
    modelo sin tocar nada.

    OJO al cambiar de modelo: los parámetros de masa identificados se cargan
    POR ÍNDICE DE CUERPO, y ese índice vale para el URDF con el que se
    identificaron. `GravityModel` lo comprueba antes de cargar nada; ver
    `_indices_cuadran`.
    """
    import os
    v = os.environ.get("H12_URDF")
    if v:
        return Path(v)
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory(URDF_PKG)) / "urdf" / URDF_FILE
        if p.exists():
            return p
    except Exception:
        pass
    # sin instalar: el árbol fuente, que está al lado
    p = (Path(__file__).resolve().parents[3] / URDF_PKG / "urdf" / URDF_FILE)
    if p.exists():
        return p
    raise FileNotFoundError(
        f"no encuentro {URDF_FILE}.\n"
        f"  Debería venir del paquete `{URDF_PKG}` de este mismo workspace.\n"
        f"  ¿Has hecho `colcon build` y `source install/setup.bash`?\n"
        f"  Se puede forzar con:  export H12_URDF=/ruta/a/{URDF_FILE}")


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
            if not self._indices_cuadran(brazo, d, verbose):
                continue
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

    def _indices_cuadran(self, brazo: str, d: dict, verbose: bool) -> bool:
        """¿Los índices del fichero apuntan a los cuerpos que se ajustaron?

        Esto no es paranoia: los parámetros se cargan en
        `model.inertias[k + 1]`, **por índice**, y ese índice vale para el URDF
        con el que se identificaron. Si se cambia de URDF, los índices siguen
        existiendo y siguen aceptando valores, así que una masa puede acabar en
        el eslabón equivocado **sin que nada falle**.

        Comprobado el 2026-09-14 entre `h1_2.urdf` y
        `h1_2_with_RH56DFTP_hands.urdf`: los dos tienen 52 articulaciones y
        coinciden en los cuerpos 13 a 19 —el brazo— pero divergen a partir del
        20, que son los dedos. Ahí el daño resultó pequeño (0.47 Nm) porque la
        mano es un bulto compacto al final del brazo, pero en otro URDF podría
        no serlo.

        Se comprueban las siete articulaciones de brazo, que son las que llevan
        el par. Si no cuadran, ese brazo se queda sin compensación en vez de
        compensar con masas puestas donde no van.
        """
        prefijo = "left" if brazo == "left" else "right"
        esperado = [f"{prefijo}_{n}_joint" for n in
                    ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                     "wrist_roll", "wrist_pitch", "wrist_yaw")]
        cuerpos = sorted(d.get("bodies", []))[:len(esperado)]
        malos = [(k, self.model.names[k + 1], e)
                 for k, e in zip(cuerpos, esperado)
                 if k + 1 < self.model.njoints and self.model.names[k + 1] != e]
        if malos:
            k, hay, deb = malos[0]
            print(f"  ⚠ gravedad: el brazo {brazo} NO se compensa.\n"
                  f"    Los parámetros se identificaron sobre\n"
                  f"      {d.get('urdf', '(desconocido)')}\n"
                  f"    y este modelo es\n"
                  f"      {self.urdf}\n"
                  f"    En el cuerpo {k} debería estar '{deb}' y está '{hay}':\n"
                  f"    cargarlos pondría masa en el eslabón equivocado.")
            return False
        if verbose and d.get("urdf") and Path(d["urdf"]) != self.urdf:
            print(f"  gravedad: URDF distinto del de la identificación, pero "
                  f"las 7 articulaciones del brazo {brazo} cuadran.")
        return True

    def tau(self, q27) -> dict[int, float]:
        """Par de gravedad por motor, en Nm, para esa configuración."""
        for i, k in self._idx_q.items():
            self._q[k] = float(q27[i])
        t = self._pin.computeGeneralizedGravity(self.model, self.data, self._q)
        return {i: float(t[v]) for i, v in self._idx_v.items()}
