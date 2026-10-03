"""Mensajes entre bloques (tabla "Interfaces entre bloques" del plan).

Convenciones: marco del robot con x adelante, y a la izquierda, z arriba, y origen en la
vertical de la camara sobre el suelo. Angulos en rad, positivos antihorario. Tiempos en
time.monotonic() del PC2 (el mismo reloj para todos los procesos de la maquina).
Una linea a la izquierda da y > 0 y debe producir vyaw > 0.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Fotograma:
    n: int                              # numero de fotograma de la camara
    t_cam: float                        # s, reloj de la camara (librealsense)
    t_rx: float                         # s, time.monotonic() al llegar al servidor
    ir: Optional[np.ndarray] = None     # uint8 alto x ancho, IR izquierdo
    color: Optional[np.ndarray] = None  # uint8 alto x ancho x 3, BGR
    prof: Optional[np.ndarray] = None   # uint16 alto x ancho, unidades de `escala_prof` (m)
    emisor: bool = False
    meta: dict = field(default_factory=dict)  # intrinsecos y escala de profundidad


@dataclass
class Imu:
    t: float        # s, monotonic al recibir el rt/lowstate
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
        """El joystick manda sobre Move: cualquier eje fuera de cero es una intervencion."""
        return max(abs(self.lx), abs(self.ly), abs(self.rx), abs(self.ry)) > umbral


@dataclass
class MedidaLinea:
    t: float                    # s, el del fotograma
    y: float                    # m, desplazamiento lateral de la linea
    theta: float                # rad, angulo de la linea
    kappa: float                # 1/m, curvatura
    objetivo: Tuple[float, float]  # (x, y) en m del punto adelantado
    confianza: float            # 0-1
    n_franjas: int
    barra_fin: Optional[float] = None       # m hasta la barra, si se ve
    esquina: Optional[Tuple[float, int]] = None  # (m hasta la esquina, sentido +1 izq / -1 der)
    ms: float = 0.0             # tiempo de proceso del fotograma


@dataclass
class EstadoLinea:
    t: float
    y: float
    theta: float
    kappa: float
    confianza: float
    edad_s: float               # desde la ultima medida buena
    rumbo_ref: float            # rad, yaw de la IMU que sigue la linea
    dist_fin: Optional[float] = None  # m restantes hasta la barra, estimados


@dataclass
class Orden:
    vx: float = 0.0             # m/s
    vy: float = 0.0             # m/s
    vyaw: float = 0.0           # rad/s


@dataclass
class Salud:
    fsm: Optional[int]
    motores_en_fallo: List[int]
    edad_lowstate: float        # s
    edad_fotograma: float       # s
    boton_parada: bool
    roll: float                 # rad
    pitch: float


@dataclass
class Decision:
    estado: str
    motivo: str
    orden_enviada: Orden
    simulacro: bool
