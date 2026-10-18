"""Dinamica del cuerpo entero para rt/lowcmd (pinocchio, base libre, URDF del reto).

- gravedad(q): el par que sostiene cada junta con el robot colgado (la base sujeta): piernas colgando, cintura
  y brazos. Es la compensacion de los brazos de tuned_gff.
- de_pie(q, carga): el par que sostiene la postura quieta con los pies llevando `carga` (0..1) del peso y la
  cuerda del arnes el resto, en el gancho del torso. Con carga 1, de pie sin cuerda.
- carga_pies(q, tau): la fuerza vertical de cada pie (y la parte del peso que llevan) a partir del par medido
  de las piernas: el par de una pierna es su propio peso menos la reaccion del suelo en su pie.

Las masas de los brazos son las identificadas (config/gravedad_brazos.yaml): la mano entera agrupada en
wrist_yaw y los dedos sin masa. El resto, las del URDF. La orientacion del cuerpo (roll, pitch) sale de la IMU
del torso, con la cintura a 0.
"""

import os

import numpy as np

from . import config

# Juntas del cuerpo en el orden DDS (indice i de motor_cmd)
JUNTAS_URDF = [f"{l}_{j}_joint" for l in ("left", "right")
               for j in ("hip_yaw", "hip_pitch", "hip_roll", "knee", "ankle_pitch", "ankle_roll")] + ["torso_joint"] + \
              [f"{l}_{j}_joint" for l in ("left", "right")
               for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch",
                         "wrist_yaw")]
PIERNA = {"izq": list(range(0, 6)), "der": list(range(6, 12))}
PIE = {"izq": "left_ankle_roll_link", "der": "right_ankle_roll_link"}
SUELA = np.array([0.044, 0.0, -0.045])     # centro de la planta en el marco del pie (malla de colision)
GANCHO = np.array([0.0, 0.0, 0.47])        # donde tiran las cintas del arnes: placa superior del torso (torso_link)
G = 9.81
RUTA_MASAS = os.path.join(config.CONFIG, "gravedad_brazos.yaml")


def _desciende(model, k, raiz):
    while k > 0:
        if k == raiz:
            return True
        k = model.parents[k]
    return False


def aplicar_masas_brazos(model, masas):
    """masas: el contenido de gravedad_brazos.yaml. Cambia el modelo en el sitio."""
    import pinocchio as pin
    for lado, pref in (("izq", "left"), ("der", "right")):
        for n, c in masas[lado]["cuerpos"].items():
            k = model.getJointId(f"{pref}_{n}_joint")
            model.inertias[k] = pin.Inertia(float(c["masa_kg"]), np.array(c["cdm_m"], float), model.inertias[k].inertia)
        w = model.getJointId(f"{pref}_wrist_yaw_joint")
        for k in range(1, model.njoints):
            if k != w and _desciende(model, k, w):
                model.inertias[k] = pin.Inertia(0.0, np.zeros(3), np.zeros((3, 3)))


def _quat_rp(roll, pitch):
    """(x, y, z, w) de pinocchio con yaw 0."""
    cr, sr, cp, sp = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2)
    return np.array([sr * cp, cr * sp, -sr * sp, cr * cp])


class Dinamica:
    def __init__(self, urdf=None, masas=None):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf or config.ruta_urdf(), pin.JointModelFreeFlyer())
        if masas is None and os.path.exists(RUTA_MASAS):
            masas = config._leer(RUTA_MASAS)
        self.masas_identificadas = masas is not None
        if masas is not None:
            aplicar_masas_brazos(self.model, masas)
        self.data = self.model.createData()
        self.iq = np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in JUNTAS_URDF])
        self.iv = np.array([self.model.joints[self.model.getJointId(n)].idx_v for n in JUNTAS_URDF])
        self.masa = float(sum(I.mass for I in self.model.inertias))
        fr = self.model.frames
        self.marco_pie = {}
        for l, n in PIE.items():
            fid = self.model.getFrameId(n)
            self.marco_pie[l] = self.model.addFrame(pin.Frame(f"suela_{l}", fr[fid].parentJoint, fid,
                                                              fr[fid].placement * pin.SE3(np.eye(3), SUELA),
                                                              pin.FrameType.OP_FRAME))
        tid = self.model.getFrameId("torso_link")
        self.marco_gancho = self.model.addFrame(pin.Frame("gancho", fr[tid].parentJoint, tid,
                                                          fr[tid].placement * pin.SE3(np.eye(3), GANCHO),
                                                          pin.FrameType.OP_FRAME))
        self.data = self.model.createData()

    def _q(self, q27, roll=0.0, pitch=0.0):
        q = self.pin.neutral(self.model)
        q[3:7] = _quat_rp(roll, pitch)
        q[self.iq] = np.asarray(q27, float)
        return q

    def gravedad(self, q27, roll=0.0, pitch=0.0):
        """Par de gravedad de las 27 juntas con la base sujeta (robot colgado)."""
        g = self.pin.computeGeneralizedGravity(self.model, self.data, self._q(q27, roll, pitch))
        return g[self.iv].copy()

    def _jacobianos(self, q):
        pin = self.pin
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = {l: pin.getFrameJacobian(self.model, self.data, f, pin.LOCAL_WORLD_ALIGNED)
             for l, f in self.marco_pie.items()}
        Jg = pin.getFrameJacobian(self.model, self.data, self.marco_gancho, pin.LOCAL_WORLD_ALIGNED)[:3]
        return J, Jg

    def de_pie(self, q27, carga=1.0, roll=0.0, pitch=0.0):
        """(par de las 27 juntas, fuerza vertical de cada pie {lado: N}) quieto con los pies llevando `carga` del
        peso y la cuerda (vertical, en el gancho) el resto. La reaccion de los pies (fuerza y momento en el centro
        de cada planta) es la de norma minima que equilibra la base."""
        q = self._q(q27, roll, pitch)
        g = self.pin.computeGeneralizedGravity(self.model, self.data, q).copy()
        J, Jg = self._jacobianos(q)
        f_cuerda = np.array([0.0, 0.0, (1.0 - float(np.clip(carga, 0.0, 1.0))) * self.masa * G])
        resto = g - Jg.T @ f_cuerda
        Jp = np.vstack([J["izq"], J["der"]])                 # 12 x nv
        F = np.linalg.lstsq(Jp[:, :6].T, resto[:6], rcond=None)[0]
        tau = resto - Jp.T @ F
        return tau[self.iv].copy(), {"izq": float(F[2]), "der": float(F[8])}

    def sosten(self, q27, carga, roll=0.0, pitch=0.0):
        """El par de feedforward: colgado (carga 0) y de pie (carga 1) mezclados linealmente."""
        carga = float(np.clip(carga, 0.0, 1.0))
        colgado = self.gravedad(q27, roll, pitch)
        if carga <= 0.0:
            return colgado
        pie, _ = self.de_pie(q27, 1.0, roll, pitch)
        return (1.0 - carga) * colgado + carga * pie

    def carga_pies(self, q27, tau27, roll=0.0, pitch=0.0):
        """({lado: fuerza vertical del suelo en N}, parte del peso que llevan los pies). Para cada pierna,
        tau = g - J^T F con las 6 juntas de la pierna: F (6) por minimos cuadrados."""
        q = self._q(q27, roll, pitch)
        g = self.pin.computeGeneralizedGravity(self.model, self.data, q).copy()
        J, _ = self._jacobianos(q)
        tau27 = np.asarray(tau27, float)
        fz = {}
        for l, idx in PIERNA.items():
            cols = self.iv[idx]
            A = J[l][:, cols].T                              # 6 x 6
            F = np.linalg.lstsq(A, g[cols] - tau27[idx], rcond=None)[0]
            fz[l] = float(F[2])
        return fz, (fz["izq"] + fz["der"]) / (self.masa * G)
