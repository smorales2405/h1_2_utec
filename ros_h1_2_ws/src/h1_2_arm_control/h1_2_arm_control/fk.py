"""Cinemática directa del brazo del H1-2, con la tabla DH del simulador.

Es **la misma** que `h1_2_algoritms.fk_functions`: los mismos parámetros DH,
la misma base respecto de `torso_link` y la misma conversión a cuaternión.
Está copiada y no importada porque los dos paquetes viven en workspaces
distintos, y el objetivo es justamente que el robot real y el simulador
calculen el efector con la misma aritmética. Si un día cambia la del
simulador, hay que traer el cambio aquí.

Cotejada contra el URDF de `h1_2_inspire_description` con `pinocchio`, en cinco
configuraciones incluidas q=0 y la pose objetivo del ensayo:

    error de posición     0.00 mm
    error de orientación  0.0000°

y contra las columnas `ee_*` que el propio simulador dejó escritas en sus CSV
(1200 filas): 0.0002 mm y 0.0229°, y eso último es la precisión con que el CSV
guarda decimales, no error de cálculo.

Esa comprobación no es decorativa. Si la cinemática del robot real no fuera la
misma que la del simulador, comparar los efectores mediría la diferencia entre
las dos fórmulas y no entre simulación y realidad, que es lo que se quiere ver.
`verifica_contra_urdf()` la repite cuando haga falta.

El efector es la punta de la cadena tras `wrist_yaw` —el frame
`right_wrist_yaw_link` del URDF— y la pose se da **en `torso_link`**.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

ARM_CHAIN = {
    lado: [f"{lado}_{n}_joint" for n in
           ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
            "wrist_roll", "wrist_pitch", "wrist_yaw")]
    for lado in ("left", "right")
}
EE_FRAME = {"left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"}
BASE_FRAME = "torso_link"

# Base de la cadena respecto de `torso_link`. El ±0.2618 rad (15°) del segundo
# parámetro DH y el signo de la segunda fila son lo único que distingue un
# brazo del otro.
_T_BASE = {
    "right": np.array([
        [0.0, -1.0, 0.0, 0.0],
        [-0.25881905, 0.0, 0.96592583, -0.20794643],
        [-0.96592583, 0.0, -0.25881905, 0.43937656],
        [0.0, 0.0, 0.0, 1.0]]),
    "left": np.array([
        [0.0, -1.0, 0.0, 0.0],
        [0.25881905, 0.0, 0.96592583, 0.20794643],
        [-0.96592583, 0.0, 0.25881905, 0.43937656],
        [0.0, 0.0, 0.0, 1.0]]),
}
# El término del hombro es `q[1] - (pi/2 - off)`: en el derecho off = +0.2618
# y en el izquierdo −0.2618. Tenerlos cambiados cuesta 2×15° = 30° exactos de
# orientación y 253 mm de posición, que es justo lo que salió la primera vez
# que se porto esto. De ahí que `verifica_contra_urdf` exista.
_OFFSET_HOMBRO = {"right": +0.2618, "left": -0.2618}


def dh(d, theta, a, alpha):
    """Transformación homogénea de Denavit-Hartenberg."""
    sth, cth = np.sin(theta), np.cos(theta)
    sa, ca = np.sin(alpha), np.cos(alpha)
    return np.array([[cth, -ca * sth,  sa * sth, a * cth],
                     [sth,  ca * cth, -sa * cth, a * sth],
                     [0.0,        sa,        ca,       d],
                     [0.0,       0.0,       0.0,     1.0]])


def fkine_arm(q, side: str = "right") -> np.ndarray:
    """Cadena 7R del brazo, del `torso_link` a la punta tras `wrist_yaw`.

    `q` son los 7 ángulos en el orden shoulder_pitch, shoulder_roll,
    shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw.
    """
    if side not in _T_BASE:
        raise ValueError(f"side debe ser 'left' o 'right', no '{side}'")
    off = _OFFSET_HOMBRO[side]
    T = _T_BASE[side]
    T = T.dot(dh(0.0,     q[0],                        0.0060011,  np.pi / 2))
    T = T.dot(dh(0.0,     q[1] - (np.pi / 2 - off),    0.0,        np.pi / 2))
    T = T.dot(dh(-0.3276, q[2] + np.pi / 2,            0.006,     -np.pi / 2))
    T = T.dot(dh(0.0,     q[3] + np.pi / 2,            0.011,      np.pi / 2))
    T = T.dot(dh(0.208,   q[4] + np.pi,                0.0,        np.pi / 2))
    T = T.dot(dh(0.0,     q[5] + np.pi / 2,            0.020,      np.pi / 2))
    T = T.dot(dh(0.0,     q[6],                        0.0,        0.0))
    return T

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

    No necesita URDF ni `pinocchio`: la geometría está en la tabla DH de
    arriba. Cuesta unos 120 µs por evaluación contando la conversión a
    cuaternión, o sea el 1.2 % de un periodo de registro a 100 Hz.
    """

    def __init__(self, side: str = "right", urdf: Path | None = None):
        if side not in ARM_CHAIN:
            raise ValueError(f"side debe ser 'left' o 'right', no '{side}'")
        self.side = side
        self.urdf = Path(urdf) if urdf else None   # solo para `verifica_*`

    def T(self, q7) -> np.ndarray:
        """Transformación homogénea del efector en `torso_link`."""
        return fkine_arm(q7, self.side)

    def pose(self, q7) -> np.ndarray:
        """`[x, y, z, qw, qx, qy, qz]` del efector en `torso_link`."""
        return TF2xyzquat(self.T(q7))


def verifica_contra_urdf(side: str = "right", urdf=None, n: int = 5,
                         semilla: int = 0, verbose: bool = True):
    """Compara esta DH con la cinemática del URDF. Devuelve (mm, grados).

    Necesita `pinocchio`, así que se llama a mano y no al importar: la DH es la
    fuente y el URDF el contraste, no al revés.
    """
    import pinocchio as pin
    from .gravity import _urdf as _urdf_por_defecto
    ruta = Path(urdf) if urdf else _urdf_por_defecto()
    m = pin.buildModelFromUrdf(str(ruta))
    d = m.createData()
    idx_q = [m.joints[m.getJointId(n)].idx_q for n in ARM_CHAIN[side]]
    id_base, id_ee = m.getFrameId(BASE_FRAME), m.getFrameId(EE_FRAME[side])

    rng = np.random.default_rng(semilla)
    Q = [np.zeros(7), np.array([-1.40, -0.25, 0.30, 0.70, 0.35, 0.20, 0.40])]
    Q += [rng.uniform(-0.6, 0.6, 7) for _ in range(max(0, n - 2))]

    peor_p = peor_a = 0.0
    for q7 in Q:
        q = pin.neutral(m)
        for k, v in zip(idx_q, q7):
            q[k] = v
        pin.forwardKinematics(m, d, q)
        pin.updateFramePlacements(m, d)
        B = (d.oMf[id_base].inverse() * d.oMf[id_ee]).homogeneous
        A = fkine_arm(q7, side)
        peor_p = max(peor_p, float(np.linalg.norm(A[:3, 3] - B[:3, 3])) * 1000)
        dR = A[:3, :3].T @ B[:3, :3]
        peor_a = max(peor_a, float(np.degrees(
            np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))))
    if verbose:
        print(f"  DH contra {ruta.name}, brazo {side}, {len(Q)} configuraciones:"
              f"\n    posición    {peor_p:.4f} mm"
              f"\n    orientación {peor_a:.4f}°")
    return peor_p, peor_a
