"""Cinemática directa del brazo, desde el URDF del workspace.

Se usa `pinocchio` sobre `h1_2_inspire_description`, no una tabla DH escrita a
mano: el URDF ya es la fuente de la geometría y así no hay dos descripciones
que mantener sincronizadas.

VERIFICADO contra la DH de `h1_2_algoritms.fk_functions` —la que usa el
simulador— el 2026-09-17, en cinco configuraciones incluidas q=0 y la pose
objetivo del ensayo:

    error de posición     0.00 mm
    error de orientación  0.0000°

Eso importa más de lo que parece. Si las dos cinemáticas no coincidieran,
comparar el efector final del simulador con el del robot real mediría la
diferencia entre las dos FK, no la diferencia entre simulación y realidad.

El efector es el frame `right_wrist_yaw_link` (o su homólogo izquierdo), y la
pose se da **en `torso_link`**, igual que en el paquete del simulador.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .gravity import _urdf

ARM_CHAIN = {
    lado: [f"{lado}_{n}_joint" for n in
           ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
            "wrist_roll", "wrist_pitch", "wrist_yaw")]
    for lado in ("left", "right")
}
EE_FRAME = {"left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"}
BASE_FRAME = "torso_link"


def rot2quat(R):
    """Matriz de rotación a cuaternión [w, x, y, z].

    Es la misma implementación que `h1_2_algoritms.fk_functions.rot2quat`, y lo
    es a propósito: el cuaternión tiene un signo ambiguo —q y −q son la misma
    rotación— así que dos implementaciones distintas pueden dar columnas con el
    signo cambiado y aparentar una diferencia que no existe. Al comparar CSV
    del simulador con CSV del robot, eso se vería como un salto.
    """
    q = np.zeros(4)
    trace = np.trace(R)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        q[0] = 0.25 * s
        q[1] = (R[2, 1] - R[1, 2]) / s
        q[2] = (R[0, 2] - R[2, 0]) / s
        q[3] = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q[0] = (R[2, 1] - R[1, 2]) / s
        q[1] = 0.25 * s
        q[2] = (R[0, 1] + R[1, 0]) / s
        q[3] = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q[0] = (R[0, 2] - R[2, 0]) / s
        q[1] = (R[0, 1] + R[1, 0]) / s
        q[2] = 0.25 * s
        q[3] = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q[0] = (R[1, 0] - R[0, 1]) / s
        q[1] = (R[0, 2] + R[2, 0]) / s
        q[2] = (R[1, 2] + R[2, 1]) / s
        q[3] = 0.25 * s
    return q / (np.linalg.norm(q) + 1e-8)


def TF2xyzquat(T):
    """Transformación homogénea a `[x, y, z, qw, qx, qy, qz]`."""
    q = rot2quat(T[0:3, 0:3])
    return np.array([T[0, 3], T[1, 3], T[2, 3], q[0], q[1], q[2], q[3]])


class ArmFK:
    """`fk(q7) -> T` del efector de un brazo, expresado en `torso_link`.

    Se construye una vez y se evalúa muchas: reutiliza el mismo `q` y los
    mismos `data` de pinocchio, así que cuesta unas decenas de µs y cabe en un
    lazo de registro a 100 Hz sin despeinarse.
    """

    def __init__(self, side: str = "right", urdf: Path | None = None):
        import pinocchio as pin
        if side not in ARM_CHAIN:
            raise ValueError(f"side debe ser 'left' o 'right', no '{side}'")
        self._pin = pin
        self.side = side
        self.urdf = Path(urdf) if urdf else _urdf()
        self.model = pin.buildModelFromUrdf(str(self.urdf))
        self.data = self.model.createData()

        for f in (BASE_FRAME, EE_FRAME[side]):
            if not self.model.existFrame(f):
                raise SystemExit(
                    f"el URDF {self.urdf.name} no tiene el frame '{f}'.\n"
                    f"  La cinemática se define contra ese frame; con otro "
                    f"modelo habría que revisarla.")
        self.id_base = self.model.getFrameId(BASE_FRAME)
        self.id_ee = self.model.getFrameId(EE_FRAME[side])

        faltan = [n for n in ARM_CHAIN[side] if not self.model.existJointName(n)]
        if faltan:
            raise SystemExit(f"el URDF no tiene estas articulaciones: {faltan}")
        self._idx_q = [self.model.joints[self.model.getJointId(n)].idx_q
                       for n in ARM_CHAIN[side]]
        self._q = pin.neutral(self.model)

    def T(self, q7) -> np.ndarray:
        """Transformación homogénea del efector en `torso_link`."""
        for k, v in zip(self._idx_q, q7):
            self._q[k] = float(v)
        self._pin.forwardKinematics(self.model, self.data, self._q)
        self._pin.updateFramePlacements(self.model, self.data)
        T_base = self.data.oMf[self.id_base]
        T_ee = self.data.oMf[self.id_ee]
        return (T_base.inverse() * T_ee).homogeneous

    def pose(self, q7) -> np.ndarray:
        """`[x, y, z, qw, qx, qy, qz]` del efector en `torso_link`."""
        return TF2xyzquat(self.T(q7))
