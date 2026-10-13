"""Vigilante: las paradas de la seccion 10 del PDF que no dependen de la linea.

Se consulta en cada ciclo de ordenes, antes de cada Move. Devuelve el motivo de parada o None.
Lo usan escalon_vyaw.py y el supervisor; la linea perdida la decide el supervisor aparte.
Trabaja con cualquier objeto con la interfaz de robot.Robot (edad_lowstate, motores_en_fallo,
imu, mando), asi se prueba sin robot.
"""

import math


class Vigilante:
    def __init__(self, robot, cfg, joystick_para=True):
        """joystick_para=False solo en el simulacro, donde el operador lleva el robot con el joystick."""
        sup = cfg["supervisor"]
        self.robot = robot
        self.joystick_para = joystick_para
        self.inclinacion_max = math.radians(sup["inclinacion_max_deg"])
        self.lowstate_max_s = sup["lowstate_max_s"]
        self.fotograma_max_s = sup["fotograma_max_s"]
        self.boton = sup.get("boton_parada")

    def motivo_parada(self, edad_fotograma=None):
        r = self.robot
        edad = r.edad_lowstate()
        if edad > self.lowstate_max_s:
            return f"rt/lowstate lleva {edad:.2f} s sin llegar"
        fallos = r.motores_en_fallo()
        if fallos:
            return f"motor en fallo {fallos}"
        imu = r.imu()
        if abs(imu.roll) > self.inclinacion_max or abs(imu.pitch) > self.inclinacion_max:
            return f"inclinacion: roll {math.degrees(imu.roll):+.1f}, pitch {math.degrees(imu.pitch):+.1f} grados"
        m = r.mando()
        if self.joystick_para and m.ejes_activos():
            return "joystick del mando fuera de cero: intervencion del operador"
        if self.boton and m.pulsado(self.boton):
            return f"boton de parada {self.boton}"
        if edad_fotograma is not None and edad_fotograma > self.fotograma_max_s:
            return f"camara sin fotogramas desde hace {edad_fotograma:.2f} s"
        return None
