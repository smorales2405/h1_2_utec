"""Mensajes entre bloques (tabla "Interfaces entre bloques" de docs/PLAN.md).

Convenciones: marco del torso (torso_link con la cintura a 0, que coincide con la pelvis), con x
adelante, y a la izquierda, z arriba. Metros y radianes; el yaw en planta es positivo antihorario.
Tiempos en time.monotonic() del PC2. Juntas en el orden de arm_sdk: 12 cintura, 13-19 brazo
izquierdo, 20-26 brazo derecho. Manos: DOF 0-5 (menique, anular, medio, indice, flexion del pulgar,
rotacion del pulgar), con 0 cerrado y 1000 abierto. Lados: "izq" y "der".
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

class Parada(Exception):
    """Motivo de salida segura (seccion 10): la lanza el vigilante y la atrapa el supervisor."""


LADOS = ("izq", "der")
JUNTAS_BRAZO = {"izq": list(range(13, 20)), "der": list(range(20, 27))}
CINTURA = 12
PIERNAS = list(range(12))


@dataclass
class Fotograma:
    n: int                              # numero de fotograma de la camara
    t_cam: float                        # s, reloj de la camara (librealsense)
    t_rx: float                         # s, time.monotonic() al llegar al servidor
    ir: Optional[np.ndarray] = None     # uint8 alto x ancho, IR izquierdo
    color: Optional[np.ndarray] = None  # uint8 alto x ancho x 3, BGR
    prof: Optional[np.ndarray] = None   # uint16 alto x ancho, en el marco del IR izquierdo; x escala_prof = m
    emisor: bool = False
    meta: dict = field(default_factory=dict)  # intrinsecos (ir, color, prof), escala_prof, prof_a_color (4x4)


@dataclass
class Imu:
    t: float
    tick: int
    roll: float     # rad, IMU del torso
    pitch: float
    yaw: float
    gx: float       # rad/s
    gy: float
    gz: float


@dataclass
class Mando:
    botones: int    # mascara de bits (mando.BOTONES)
    lx: float       # ejes en [-1, 1]
    ly: float
    rx: float
    ry: float

    def pulsado(self, nombre: str) -> bool:
        from .mando import BOTONES
        return bool(self.botones & (1 << BOTONES.index(nombre)))

    def ejes_activos(self, umbral: float = 0.05) -> bool:
        """Un eje fuera de cero hace andar al robot en FSM 201: cuenta como paso."""
        return max(abs(self.lx), abs(self.ly), abs(self.rx), abs(self.ry)) > umbral


@dataclass
class EstadoRobot:
    t: float                    # s, monotonic al recibir el rt/lowstate
    q: np.ndarray               # 27 juntas del cuerpo (rad)
    dq: np.ndarray
    tau: np.ndarray             # par estimado (N m)
    imu: Imu
    mando: Mando
    motores_en_fallo: List[int]
    mode_machine: int


@dataclass
class EstadoMano:
    t: float
    lado: str
    angulo: np.ndarray          # ANGLE_ACT de los 6 DOF (0-1000)
    fuerza: np.ndarray          # FORCE_ACT (gf), tal cual
    base: np.ndarray            # gf de la linea base (mano abierta y sin carga)
    force_set: np.ndarray       # el ultimo escrito
    error: np.ndarray
    estado: np.ndarray
    tactil_yema: Optional[float] = None   # maximo sobre la base de la yema del indice (unidades del sensor)

    @property
    def fuerza_neta(self) -> np.ndarray:
        return self.fuerza - self.base


@dataclass
class Calibracion:
    """La escribe herramientas/calibrar_camara_torso.py en config/calibracion_<fecha>.yaml."""
    fecha: str
    T_torso_color: np.ndarray           # 4x4, marco optico del color (z adelante, x derecha, y abajo)
    T_color_prof: np.ndarray            # 4x4, del IR izquierdo (profundidad) al color: el de fabrica
    T_dorso: Dict[str, np.ndarray]      # lado -> 4x4 del marcador en *_wrist_yaw_link
    plano_mesa: Tuple[np.ndarray, float]  # (n, d) en el torso: altura = n.p + d
    residuo_px: float = float("nan")    # medio de reproyeccion
    residuo_mm: float = float("nan")    # medio en 3D, en el punto del marcador
    n_poses: int = 0
    origen: str = ""                    # "calibrada" o "urdf" (sin calibrar)

    @property
    def T_torso_prof(self) -> np.ndarray:
        return self.T_torso_color @ self.T_color_prof


@dataclass
class Objeto:
    id: int
    clase: str                  # clave de objetos.yaml, o "desconocido"
    p: np.ndarray               # centro (x, y, z) en el torso (m)
    yaw: float                  # rad, modulo la simetria de la clase
    medidas: np.ndarray         # m, las medidas vistas (en planta y alto)
    confianza: float            # 0-1
    visible: bool = True
    movido: bool = False
    n_puntos: int = 0
    pixel: Optional[Tuple[int, int]] = None   # centro en la imagen de color (para dibujar)


@dataclass
class Escena:
    t: float                    # el del fotograma
    objetos: List[Objeto]
    ms: float = 0.0             # tiempo de proceso
    plano: Optional[Tuple[np.ndarray, float]] = None   # plano de la mesa usado (torso)


@dataclass
class PuntoPaso:
    q: np.ndarray               # 7 juntas del brazo (rad)
    duracion: float             # s desde el punto anterior
    nombre: str = ""


@dataclass
class Agarre:
    preforma: List[int]         # ANGLE_SET de los 6 DOF antes de bajar
    force_set: List[int]        # gf por DOF; -1 = ese DOF no cierra
    speed: int
    T_mano: np.ndarray          # 4x4 de *_wrist_yaw_link en el torso en el agarre


@dataclass
class Plan:
    objeto: Optional[Objeto]
    mano: str
    tarea: str                  # "tocar" | "coger"
    puntos_de_paso: List[PuntoPaso]
    agarre: Optional[Agarre] = None
    bandeja: Optional[str] = None
    comprobado: bool = False
    motivo: str = ""            # por que no es valido, si no lo es
    detalle: dict = field(default_factory=dict)


@dataclass
class OrdenBrazo:
    lado: str
    q_obj: np.ndarray
    duracion: float
    perfil: str = "minima_sacudida"


@dataclass
class OrdenMano:
    lado: str
    accion: str                 # preforma | cerrar | sujetar | soltar | abrir
    force_set: Optional[List[int]] = None
    speed: Optional[int] = None
    angulos: Optional[List[int]] = None


@dataclass
class Salud:
    fsm: Optional[int]
    motores_en_fallo: List[int]
    edad_lowstate: float
    edad_fotograma: float
    mano_ok: Dict[str, bool]
    paso: Optional[str]         # la junta de pierna que lo delata, o None
    yaw_desde_inicio: float     # rad
    roll: float
    pitch: float
    error_seguimiento: float    # rad, el mayor de los brazos
    fuerza_max: float           # gf, la mayor de las manos
    parada: Optional[str]       # tecla o boton, o None


@dataclass
class Decision:
    estado: str
    motivo: str
    objeto: Optional[str] = None
    peso: float = 0.0
    simulacro: bool = False
    detalle: dict = field(default_factory=dict)
