"""Vigilante: las causas de salida segura de la seccion 10 del PDF.

Se consulta en cada ciclo del brazo (brazo.ir_a(..., comprobar=vigilante.comprobar)) y del supervisor:
  - un paso: una junta de pierna que se aparta mas de paso_pierna_rad de la postura al empezar (lo que usa
    prueba_brazos_armsdk.py) o un eje del joystick fuera de cero (en FSM 201 hace andar al robot)
  - yaw de la IMU > 5 grados desde el inicio de la tirada; roll o pitch > 10 grados
  - error de seguimiento del brazo por encima del fijado (choque o arm_sdk sin efecto)
  - fuerza de una mano por encima del corte; mano sin responder
  - camara sin fotogramas; rt/lowstate sin llegar; motor en fallo; FSM distinta de 201
  - tecla ESPACIO, boton de parada del mando o Ctrl+C
Trabaja con cualquier objeto con la interfaz de robot.Robot (tambien sim/robot_simulado.py): se prueba sin
robot. Devuelve el motivo (o None) y comprobar() lanza Parada con el.
"""

import math
import time

import numpy as np

from .mensajes import PIERNAS, Parada, Salud
from .robot import FSM_VALIDA


class Vigilante:
    def __init__(self, robot, cfg, brazo=None, manos=None, lector_fsm=None, edad_fotograma=None):
        """manos: {lado: Mano}; edad_fotograma(): s desde el ultimo fotograma (None = no se vigila)."""
        s = cfg["supervisor"]
        self.robot, self.brazo, self.manos = robot, brazo, manos or {}
        self.lector_fsm = lector_fsm
        self.edad_fotograma = edad_fotograma
        self.yaw_max = math.radians(s["yaw_max_deg"])
        self.incl_max = math.radians(s["inclinacion_max_deg"])
        self.paso_rad = s["paso_pierna_rad"]
        self.lowstate_max_s = s["lowstate_max_s"]
        self.fotograma_max_s = s["fotograma_max_s"]
        self.mano_max_s = s["mano_max_s"]
        self.boton = s.get("boton_parada")
        self.joystick_para = s.get("joystick_para", True)
        self.err_max = cfg["brazo"]["err_seguimiento_max_rad"]
        self.corte = cfg["mano"]["corte_gf"]
        self.parada_pedida = None           # la pone la tecla ESPACIO o Ctrl+C
        self.manos_vigiladas = set()        # los lados en uso: solo esas se leen a menudo (las lecturas tienen edad)
        self.vigilar_camara = True
        self.yaw0, self.q0, self.t0 = None, None, None

    def iniciar(self):
        """Toma la postura y el rumbo de referencia (al empezar la tirada, antes de la rampa de peso)."""
        e = self.robot.estado()
        self.yaw0, self.q0, self.t0 = e.imu.yaw, e.q.copy(), time.monotonic()

    def pedir_parada(self, motivo):
        self.parada_pedida = motivo

    def salud(self) -> Salud:
        e = self.robot.estado()
        fsm = self.lector_fsm.fsm if self.lector_fsm is not None else None
        paso = None
        if self.q0 is not None:
            d = np.abs(e.q[PIERNAS] - self.q0[PIERNAS])
            if d.max() > self.paso_rad:
                paso = f"junta {int(np.argmax(d))} {math.degrees(d.max()):.1f} grados"
        dyaw = 0.0 if self.yaw0 is None else (e.imu.yaw - self.yaw0 + math.pi) % (2 * math.pi) - math.pi
        fuerza = max((float((m.ultimo.fuerza - m.ultimo.base)[:5].max()) for m in self.manos.values()
                      if m.ultimo is not None), default=0.0)
        return Salud(fsm=fsm, motores_en_fallo=e.motores_en_fallo, edad_lowstate=self.robot.edad_lowstate(),
                     edad_fotograma=self.edad_fotograma() if self.edad_fotograma else 0.0,
                     mano_ok={l: (time.monotonic() - m.t_ultima_lectura) < self.mano_max_s
                              for l, m in self.manos.items() if l in self.manos_vigiladas},
                     paso=paso, yaw_desde_inicio=dyaw, roll=e.imu.roll, pitch=e.imu.pitch,
                     error_seguimiento=self.brazo.error() if self.brazo is not None else 0.0,
                     fuerza_max=fuerza, parada=self.parada_pedida)

    def motivo(self, s: Salud = None):
        if self.parada_pedida:
            return self.parada_pedida
        if self.robot.edad_lowstate() > self.lowstate_max_s:
            return f"rt/lowstate lleva {self.robot.edad_lowstate():.2f} s sin llegar"
        s = s or self.salud()
        if s.motores_en_fallo:
            return f"motor en fallo {s.motores_en_fallo}"
        if self.lector_fsm is not None and s.fsm != FSM_VALIDA:
            return f"FSM {s.fsm}, no {FSM_VALIDA}"
        if s.paso:
            return f"paso: pierna fuera de su postura ({s.paso})"
        e = self.robot.estado()
        if self.joystick_para and e.mando.ejes_activos():
            return "joystick fuera de cero: en FSM 201 hace andar al robot"
        if self.boton and e.mando.pulsado(self.boton):
            return f"boton de parada {self.boton}"
        if abs(s.yaw_desde_inicio) > self.yaw_max:
            return f"giro del cuerpo {math.degrees(s.yaw_desde_inicio):+.1f} grados"
        if abs(s.roll) > self.incl_max or abs(s.pitch) > self.incl_max:
            return f"inclinacion: roll {math.degrees(s.roll):+.1f}, pitch {math.degrees(s.pitch):+.1f} grados"
        if s.error_seguimiento > self.err_max:
            return f"error de seguimiento del brazo {math.degrees(s.error_seguimiento):.1f} grados (choque?)"
        if s.fuerza_max > self.corte:
            return f"fuerza de la mano {s.fuerza_max:.0f} gf > {self.corte}"
        if not all(s.mano_ok.values()):
            return f"mano sin respuesta: {[l for l, ok in s.mano_ok.items() if not ok]}"
        if self.vigilar_camara and self.edad_fotograma is not None and s.edad_fotograma > self.fotograma_max_s:
            return f"camara sin fotogramas desde hace {s.edad_fotograma:.2f} s"
        return None

    def comprobar(self):
        m = self.motivo()
        if m:
            raise Parada(m)
