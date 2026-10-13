"""Cinematica de los brazos sobre el URDF del repo (pinocchio 3 en el PC2 y 4 en la PC): FK e IK propia.

Modelo reducido a las 14 juntas de los brazos: piernas, cintura y dedos quedan fijos en la configuracion
de referencia (la cintura en la q que se sujeta, los dedos en el gesto que se de, con sus juntas mimic).
Todo sale en el marco del torso (torso_link): los brazos cuelgan de el y la camara tambien, asi que nada
depende de las piernas.

Marcos (por lado, "izq" / "der"):
  mano   *_wrist_yaw_link: dedos en +x, indice en +z; palma hacia -y en la izquierda y +y en la derecha
  yema   punta del indice, medida en la malla de *_index_2 (con el indice extendido: x 0.295 en mano)
El punto de agarre (objetos.yaml) y el marcador del dorso (calibracion) son transformaciones en `mano`.

IK: minimos cuadrados amortiguados con limites (web n URDF, con margen) desde varias semillas. La tarea
es una posicion y bandas sobre productos escalares de ejes de la mano con direcciones del torso
(palma abajo, palma vertical, dedos inclinados, guinada de la mano), que es como pide el plan las
orientaciones: con tolerancia, porque la muneca solo cabecea +-27 grados.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import pinocchio as pin

from . import config
from .geometria import aplicar, inv
from .mensajes import JUNTAS_BRAZO, LADOS

PREFIJO = {"izq": "left", "der": "right"}
NOMBRES_BRAZO = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
DOF_MANO = ("little_1", "ring_1", "middle_1", "index_1", "thumb_1", "thumb_swing")
MIMIC = {"little_2": ("little_1", 0.85), "ring_2": ("ring_1", 0.85), "middle_2": ("middle_1", 0.85),
         "index_2": ("index_1", 0.85), "thumb_2": ("thumb_1", 0.40), "thumb_3": ("thumb_1", 0.60)}
# Punta del indice en *_index_2 (malla; el dedo va por +y en la izquierda y por -y en la derecha)
YEMA_LOCAL = {"izq": np.array([0.0079, 0.0517, -0.0075]), "der": np.array([0.0079, -0.0517, -0.0075])}
# Ejes de la mano en `mano` (*_wrist_yaw_link)
EJE_DEDOS = np.array([1.0, 0.0, 0.0])
EJE_ANCHO = np.array([0.0, 0.0, 1.0])          # del menique al indice
EJE_PALMA = {"izq": np.array([0.0, -1.0, 0.0]), "der": np.array([0.0, 1.0, 0.0])}
# El indice en el marco de la yema (*_index_2): +y en la izquierda, -y en la derecha (no x: ese es transversal)
EJE_DEDO_YEMA = {"izq": np.array([0.0, 1.0, 0.0]), "der": np.array([0.0, -1.0, 0.0])}
# Brazo derecho en espejo del izquierdo (caja_cuadrado.py): mismo cabeceo, alabeo y guinada cambiados
SIGNO_ESPEJO = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
# Semillas del brazo izquierdo que llegan a la mesa (mapa de alcance del 2026-10-09); la derecha, en espejo
SEMILLAS_IZQ = [
    [-0.60, 0.20, 0.00, 0.60, 0.00, 0.00, 0.00],
    [-0.90, 0.30, -0.30, 0.20, -1.50, 0.00, 0.00],
    [-0.57, 0.22, -1.52, 0.85, 0.37, 0.24, 0.90],
    [-0.27, -0.30, 0.11, 0.92, 1.79, 0.23, 0.68],
    [-0.20, 0.00, 0.03, 1.20, 0.00, -0.41, 0.31],
    [-0.40, 0.10, 0.30, 1.00, -1.50, 0.00, 0.00],
]


def espejo(q7):
    return SIGNO_ESPEJO * np.asarray(q7, float)


def nombre_junta(lado, i):
    return f"{PREFIJO[lado]}_{NOMBRES_BRAZO[i]}_joint"


@dataclass
class Banda:
    """lo <= (R eje_local) . ref <= hi, con eje_local en el marco de la tarea y ref en el torso."""
    eje_local: np.ndarray
    ref: np.ndarray
    lo: float = -1.0
    hi: float = 1.0
    peso: float = 0.3


@dataclass
class TareaIK:
    posicion: np.ndarray                    # objetivo del punto (torso)
    marco: str = "mano"                     # mano | yema
    punto: np.ndarray = field(default_factory=lambda: np.zeros(3))   # punto en el marco (p. ej. el de agarre)
    bandas: List[Banda] = field(default_factory=list)
    tol_pos: float = 0.003
    peso_postura: float = 0.0               # hacia q_pref en el espacio nulo (0: la continuidad la da la semilla q0)


@dataclass
class ResultadoIK:
    q: np.ndarray
    ok: bool
    err_pos: float                          # m
    violacion: float                        # la mayor de las bandas (en producto escalar)
    iteraciones: int
    semilla: int


def palma_abajo(lado, tol_deg):
    """La palma mira hacia abajo con `tol_deg` de margen."""
    return Banda(EJE_PALMA[lado], np.array([0.0, 0.0, -1.0]), math.cos(math.radians(tol_deg)), 1.0)


def palma_vertical(lado, tol_deg):
    s = math.sin(math.radians(tol_deg))
    return Banda(EJE_PALMA[lado], np.array([0.0, 0.0, 1.0]), -s, s)


def dedos_inclinados(deg_min, deg_max, eje=EJE_DEDOS):
    """Los dedos (o el eje dado, en el marco de la tarea) entre deg_min y deg_max bajo la horizontal."""
    return Banda(eje, np.array([0.0, 0.0, 1.0]), -math.sin(math.radians(deg_max)), -math.sin(math.radians(deg_min)))


def guinada(direccion_xy, tol_deg, eje=EJE_ANCHO):
    """El eje de la mano (por defecto el ancho, menique -> indice) apunta en planta hacia direccion_xy."""
    d = np.array([direccion_xy[0], direccion_xy[1], 0.0])
    return Banda(eje, d / np.linalg.norm(d), math.cos(math.radians(tol_deg)), 1.0, peso=0.3)


class Cinematica:
    def __init__(self, urdf=None, cintura=0.0, dedos=None, limites=None, margen=0.05):
        """dedos: {lado: {clave de DOF_MANO: rad}} (por defecto, la mano abierta). limites: {indice DDS:
        [lo, hi]} de manipulacion.yaml (por defecto los del URDF). Los limites usados se recortan `margen`."""
        self.urdf = urdf or config.ruta_urdf()
        completo = pin.buildModelFromUrdf(self.urdf)
        q_ref = pin.neutral(completo)

        def poner(nombre, valor):
            jid = completo.getJointId(nombre)
            if jid < completo.njoints:
                q_ref[completo.joints[jid].idx_q] = valor

        poner("torso_joint", cintura)
        for lado in LADOS:
            d = dict.fromkeys(DOF_MANO, 0.0)
            d.update((dedos or {}).get(lado, {}))
            for k, v in d.items():
                poner(f"{PREFIJO[lado]}_{k}_joint", v)
            for k, (origen, mult) in MIMIC.items():
                poner(f"{PREFIJO[lado]}_{k}_joint", mult * d[origen])
        mantener = {nombre_junta(lado, i) for lado in LADOS for i in range(7)}
        bloquear = [j for j in range(1, completo.njoints) if completo.names[j] not in mantener]
        self.model = pin.buildReducedModel(completo, bloquear, q_ref)
        m = self.model
        for lado in LADOS:
            fi = m.getFrameId(f"{PREFIJO[lado]}_index_2")
            f = m.frames[fi]
            m.addFrame(pin.Frame(f"yema_{lado}", f.parentJoint, fi,
                                 f.placement * pin.SE3(np.eye(3), YEMA_LOCAL[lado]), pin.FrameType.OP_FRAME))
        self.data = m.createData()
        self.id_torso = m.getFrameId("torso_link")
        self.id_marco = {}
        for lado in LADOS:
            self.id_marco[("mano", lado)] = m.getFrameId(f"{PREFIJO[lado]}_wrist_yaw_link")
            self.id_marco[("yema", lado)] = m.getFrameId(f"yema_{lado}")
            self.id_marco[("codo", lado)] = m.getFrameId(f"{PREFIJO[lado]}_elbow_link")
            self.id_marco[("hombro", lado)] = m.getFrameId(f"{PREFIJO[lado]}_shoulder_pitch_link")
        self.id_camara = m.getFrameId("camera_link")
        # indices de q y v de cada brazo en el modelo reducido, en el orden de arm_sdk
        self.iq, self.iv = {}, {}
        for lado in LADOS:
            ids = [m.getJointId(nombre_junta(lado, i)) for i in range(7)]
            self.iq[lado] = np.array([m.joints[j].idx_q for j in ids])
            self.iv[lado] = np.array([m.joints[j].idx_v for j in ids])
        self.lo, self.hi = {}, {}
        for lado in LADOS:
            lo = np.array([m.lowerPositionLimit[i] for i in self.iq[lado]])
            hi = np.array([m.upperPositionLimit[i] for i in self.iq[lado]])
            if limites:
                for k, j in enumerate(JUNTAS_BRAZO[lado]):
                    a, b = limites[j] if j in limites else limites[str(j)]
                    lo[k], hi[k] = max(lo[k], a), min(hi[k], b)
            self.lo[lado], self.hi[lado] = lo + margen, hi - margen
        self.q = pin.neutral(m)
        self._calcular(self.q)

    @classmethod
    def desde_config(cls, cfg, **kw):
        b = cfg["brazo"]
        return cls(limites=b["limites"], margen=b["margen_limites_rad"], **kw)

    # --- estado -----------------------------------------------------------------------------
    def poner(self, lado, q7):
        self.q[self.iq[lado]] = q7

    def q_brazos(self, q_izq=None, q_der=None):
        q = self.q.copy()
        if q_izq is not None:
            q[self.iq["izq"]] = q_izq
        if q_der is not None:
            q[self.iq["der"]] = q_der
        return q

    def _calcular(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        self._T_torso_inv = self.data.oMf[self.id_torso].inverse()

    def _T(self, fid):
        M = self._T_torso_inv * self.data.oMf[fid]
        return M.homogeneous

    # --- cinematica directa -------------------------------------------------------------------
    def fk(self, lado, q7, marco="mano", q_otro=None):
        """4x4 del marco del brazo `lado` en el torso, con ese brazo en q7."""
        q = self.q.copy()
        q[self.iq[lado]] = q7
        if q_otro is not None:
            q[self.iq["der" if lado == "izq" else "izq"]] = q_otro
        self._calcular(q)
        return self._T(self.id_marco[(marco, lado)])

    def T_camara_link(self):
        """camera_link del URDF en el torso: la semilla de la calibracion."""
        self._calcular(self.q)
        return self._T(self.id_camara)

    def esqueleto(self, q_izq, q_der):
        """{lado: [hombro, codo, muneca, centro de la mano, yema]} en el torso, para la mascara del propio
        robot en la imagen."""
        self._calcular(self.q_brazos(q_izq, q_der))
        sal = {}
        for lado in LADOS:
            M = self._T(self.id_marco[("mano", lado)])
            sal[lado] = np.array([self._T(self.id_marco[("hombro", lado)])[:3, 3],
                                  self._T(self.id_marco[("codo", lado)])[:3, 3],
                                  M[:3, 3], aplicar(M, [0.14, 0.0, 0.0]),
                                  self._T(self.id_marco[("yema", lado)])[:3, 3]])
        return sal

    def dentro_de_limites(self, lado, q7, holgura=0.0):
        q7 = np.asarray(q7, float)
        return bool(np.all(q7 >= self.lo[lado] - holgura) and np.all(q7 <= self.hi[lado] + holgura))

    # --- cinematica inversa -------------------------------------------------------------------
    def _jacobianos(self, lado, fid, q):
        """(Jv, Jw) 3x7 en los ejes del torso, para el origen del marco fid."""
        J = pin.computeFrameJacobian(self.model, self.data, q, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        R = self._T_torso_inv.rotation
        return R @ J[:3, self.iv[lado]], R @ J[3:, self.iv[lado]]

    def semillas(self, lado, q0=None, n_azar=6, semilla=0):
        base = [np.array(s, float) for s in SEMILLAS_IZQ]
        if lado == "der":
            base = [espejo(s) for s in base]
        rng = np.random.default_rng(semilla)
        azar = [self.lo[lado] + (self.hi[lado] - self.lo[lado]) * rng.random(7) for _ in range(n_azar)]
        lista = ([np.asarray(q0, float)] if q0 is not None else []) + base + azar
        return [np.clip(s, self.lo[lado], self.hi[lado]) for s in lista]

    def ik(self, lado, tarea: TareaIK, q0=None, semillas=None, iteraciones=200, q_pref=None, todas=False):
        """Resuelve la tarea con el brazo `lado`. Con q0, se prueba primero desde ahi y, entre las
        soluciones validas, gana la mas cercana a q0. Devuelve ResultadoIK (o la lista con todas=True)."""
        lista = semillas if semillas is not None else self.semillas(lado, q0)
        q_pref = np.asarray(q_pref if q_pref is not None else (q0 if q0 is not None else lista[0]), float)
        fid = self.id_marco[(tarea.marco, lado)]
        resultados = []
        for k, s in enumerate(lista):
            r = self._ik_desde(lado, tarea, fid, np.asarray(s, float), q_pref, iteraciones, k)
            resultados.append(r)
            if r.ok and q0 is not None and k == 0 and not todas:
                return r                     # desde donde esta: la mas continua
        if todas:
            return resultados
        validos = [r for r in resultados if r.ok]
        if validos:
            ref = np.asarray(q0 if q0 is not None else q_pref, float)
            return min(validos, key=lambda r: float(np.linalg.norm(r.q - ref)))
        return min(resultados, key=lambda r: r.err_pos + r.violacion)

    def _residuo(self, lado, tarea, fid, q7, holgura_banda=0.0):
        """Residuo de la tarea. holgura_banda estrecha las bandas al resolver (dentro del borde: si no, la
        solucion queda justo en el y oscila); la violacion que se devuelve es contra la banda pedida."""
        q = self.q.copy()
        q[self.iq[lado]] = q7
        self._calcular(q)
        M = self._T(fid)
        R = M[:3, :3]
        p = aplicar(M, tarea.punto)
        Jv, Jw = self._jacobianos(lado, fid, q)
        # el punto no es el origen del marco: v = v_o + w x (p - o)
        r_op = p - M[:3, 3]
        Jp = Jv - np.cross(r_op, Jw.T).T
        e_pos = tarea.posicion - p
        filas_e, filas_J, viol = [e_pos], [Jp], 0.0
        for b in tarea.bandas:
            a = R @ b.eje_local
            c = float(a @ b.ref)
            lo = b.lo + (holgura_banda if b.lo > -1.0 else 0.0)
            hi = b.hi - (holgura_banda if b.hi < 1.0 else 0.0)
            if lo > hi:
                lo = hi = (b.lo + b.hi) / 2
            r = (lo - c) if c < lo else ((hi - c) if c > hi else 0.0)
            viol = max(viol, (b.lo - c) if c < b.lo else ((c - b.hi) if c > b.hi else 0.0))
            filas_e.append(np.array([b.peso * r]))
            filas_J.append(b.peso * (np.cross(a, b.ref) @ Jw)[None, :])
        return np.concatenate(filas_e), np.vstack(filas_J), float(np.linalg.norm(e_pos)), viol

    def _ik_desde(self, lado, tarea, fid, q7, q_pref, iteraciones, k):
        """Levenberg-Marquardt sobre la tarea; la postura solo en el espacio nulo de la tarea (si no,
        compite con ella y el error no baja de unos mm)."""
        lo, hi = self.lo[lado], self.hi[lado]
        q7 = np.clip(q7, lo, hi)
        lam = 1e-2
        hb = 0.02
        e, J, err_pos, viol = self._residuo(lado, tarea, fid, q7, hb)
        coste = float(e @ e)
        it = 0
        for it in range(iteraciones):
            if err_pos < tarea.tol_pos and viol < 2e-3:
                return ResultadoIK(q7, True, err_pos, viol, it, k)
            A = J @ J.T + lam * np.eye(len(e))
            dq = J.T @ np.linalg.solve(A, e)
            if tarea.peso_postura > 0:
                nulo = np.eye(7) - J.T @ np.linalg.solve(A, J)
                dq = dq + nulo @ (tarea.peso_postura * (q_pref - q7))
            q_n = np.clip(q7 + np.clip(dq, -0.2, 0.2), lo, hi)
            e_n, J_n, ep_n, v_n = self._residuo(lado, tarea, fid, q_n, hb)
            c_n = float(e_n @ e_n)
            if c_n < coste:
                q7, e, J, err_pos, viol, coste = q_n, e_n, J_n, ep_n, v_n, c_n
                lam = max(lam * 0.3, 1e-6)
            else:
                lam = min(lam * 5.0, 1e3)
                if lam >= 1e3:
                    break
        return ResultadoIK(q7, err_pos < tarea.tol_pos and viol < 2e-3, err_pos, viol, it, k)


def ejes_mano(T_mano, lado):
    """(dedos, ancho, palma) de la mano en el torso."""
    R = T_mano[:3, :3]
    return R @ EJE_DEDOS, R @ EJE_ANCHO, R @ EJE_PALMA[lado]


def T_punto(punto, lado):
    """El punto de agarre de objetos.yaml (dado para la izquierda) en la mano `lado`."""
    p = np.array(punto, float)
    if lado == "der":
        p[1] = -p[1]
    return p


def T_mano_desde_camara(T_torso_cam, T_cam_marcador, T_dorso):
    """Donde esta la mano segun la camara: T_torso_mano = T_torso_cam T_cam_marcador T_dorso^-1."""
    return T_torso_cam @ T_cam_marcador @ inv(T_dorso)


def opcional(x: Optional[np.ndarray]):
    return None if x is None else np.asarray(x, float)
