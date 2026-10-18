"""Mundo simulado para probar el supervisor de punta a punta en la PC: robot simulado, manos simuladas, gemelo y
camara virtual, unidos.

  - el brazo va a su consigna de arm_sdk (en tiempo acelerado, al instante; con la caida de ~5 grados)
  - la camara virtual renderiza el gemelo con las q del robot, los dedos de las manos y los objetos
  - una mano que cierra con un objeto entre los dedos lo nota (fuerza) y se lo lleva; al abrir, el objeto cae
    en la vertical (dentro de la bandeja si esta encima); la yema contra la mesa da fuerza en el indice
No es fisica: sirve para comprobar estados, ordenes, registro y salida segura, no agarres.
"""

import math
import threading
import time

import numpy as np

from manip import mano as mm
from manip.cinematica import T_punto
from manip.geometria import Plano, T, aplicar, inv, rpy_a_R
from manip.mensajes import JUNTAS_BRAZO, LADOS
from manip.montaje import bandeja_real
from sim.camara_virtual import CamaraVirtual
from sim.escena import Estado, angle_set_a_rad, construir
from sim.mano_simulada import ManoSimulada
from sim.robot_simulado import RobotSimulado


class ManoDelMundo(ManoSimulada):
    def __init__(self, mundo, lado):
        super().__init__()
        self.mundo, self.lado = mundo, lado

    def _fuerza(self):
        f = super()._fuerza()
        f[3] += self.mundo.fuerza_yema(self.lado)
        return f

    def write_registers(self, a, valores, slave=1):
        r = super().write_registers(a, valores, slave)
        if a == mm.ANGLE_SET:
            v = [x - 0x10000 if x >= 0x8000 else x for x in valores]
            cierra = any(0 <= x < 500 for x in v[:5])
            abre = all(x < 0 or x >= 900 for x in v[:5]) and any(x >= 900 for x in v[:5])
            if cierra:
                self.mundo.al_cerrar(self.lado)
            elif abre:
                self.mundo.al_abrir(self.lado)
        return r


class FuenteMundo:
    def __init__(self, mundo, hz=10.0):
        self.mundo, self.hz = mundo, hz
        self.meta = mundo.cam.meta()
        self.t_ultimo = time.monotonic()
        self.tapada = False

    def siguiente(self, timeout_s=1.0):
        if self.tapada:
            time.sleep(min(timeout_s, 0.05))
            return None
        f = self.mundo.fotograma()
        self.t_ultimo = f.t_rx
        return f

    def fresco(self, edad_max_s=0.2, timeout_s=3.0):
        return self.siguiente(timeout_s)

    def edad(self):
        return 0.0 if not self.tapada else time.monotonic() - self.t_ultimo


class MundoSimulado:
    def __init__(self, cfg, montaje, objetos, cal, cin, colocacion):
        """colocacion: {clase: ((x, y) en el torso, yaw)} de los objetos en la mesa al empezar."""
        self.cfg, self.montaje, self.objetos, self.cal, self.cin = cfg, montaje, objetos, cal, cin
        self.cerrojo = threading.RLock()        # el visor (sim/visor.py) lee el mundo desde otro hilo
        self.robot = RobotSimulado(caida_rad=0.0, instantaneo=True)
        self.m, self.info = construir(montaje=montaje, objetos=objetos, cal=cal, cfg=cfg)
        self.est = Estado(self.m, self.info)
        self.cam = CamaraVirtual(self.m, self.info, cal, ruido_mm=1.0)
        pl = montaje["mesa"]["plano"]
        self.plano = Plano(pl["n"], pl["d"])
        self.manos = {l: ManoDelMundo(self, l) for l in LADOS}
        self.sujeto = {l: None for l in LADOS}
        self.T_rel = {}
        self.poses = {}
        for clase, (xy, yaw) in colocacion.items():
            self.poner(clase, xy, yaw)

    # --- objetos ------------------------------------------------------------------------------------
    def _alto(self, clase):
        o = self.objetos["objetos"][clase]
        return o["medidas_m"][0] if o["forma"] == "esfera" else (o["medidas_m"][1] if o["forma"] == "cilindro" else o["medidas_m"][2])

    def poner(self, clase, xy, yaw, base=0.0):
        p = np.array([xy[0], xy[1], 0.0])
        z = -(self.plano.n[0] * p[0] + self.plano.n[1] * p[1] + self.plano.d) / self.plano.n[2]
        p = np.array([p[0], p[1], z]) + (base + self._alto(clase) / 2) * self.plano.n
        Tm = np.array(self.montaje["mesa"]["T_torso_mesa"], float)
        M = T(Tm[:3, :3] @ rpy_a_R(0, 0, yaw - math.atan2(Tm[1, 0], Tm[0, 0])), p)
        self.poses[clase] = M
        self.est.poner_objeto(clase, M[:3, 3], M[:3, :3])

    def donde(self, clase):
        return self.poses.get(clase)

    def bandeja_de(self, clase):
        """La bandeja en la que esta el objeto (por su centro), o None."""
        p = self.poses[clase][:3, 3]
        for k, b in self.montaje["bandejas"].items():
            c, _ = bandeja_real(b)
            if abs(p[0] - c[0]) < b["hueco_m"] / 2 and abs(p[1] - c[1]) < b["hueco_m"] / 2:
                return k
        return None

    def _T_mano(self, lado):
        return self.cin.fk(lado, self.robot.q()[JUNTAS_BRAZO[lado]], "mano")

    def al_cerrar(self, lado):
        with self.cerrojo:
            self._al_cerrar(lado)

    def _al_cerrar(self, lado):
        if self.sujeto[lado] is not None:
            return
        M = self._T_mano(lado)
        for tipo in ("arriba", "lado"):
            g = aplicar(M, T_punto(self.objetos["agarres"][tipo]["centro"], lado))
            for clase, Mo in self.poses.items():
                if np.linalg.norm(Mo[:3, 3] - g) < 0.03:
                    self.sujeto[lado] = clase
                    self.T_rel[lado] = inv(M) @ Mo
                    # solo los dedos que estan abiertos (los que la preforma recogio no tocan el objeto)
                    ang = self.manos[lado].ang
                    self.manos[lado].poner_objeto({k: a for k, a in ((0, 650), (1, 650), (2, 650), (3, 650), (4, 700)) if ang[k] > a + 50})
                    return

    def al_abrir(self, lado):
        with self.cerrojo:
            self._al_abrir(lado)

    def _al_abrir(self, lado):
        clase = self.sujeto[lado]
        if clase is None:
            return
        self.sujeto[lado] = None
        self.manos[lado].poner_objeto({})
        M = self._T_mano(lado) @ self.T_rel[lado]
        xy = M[:3, 3][:2]
        base = 0.003 if any(np.all(np.abs(xy - bandeja_real(b)[0]) < b["hueco_m"] / 2)
                            for b in self.montaje["bandejas"].values()) else 0.0
        self.poner(clase, xy, math.atan2(M[1, 0], M[0, 0]), base)

    def fuerza_yema(self, lado):
        """gf en el indice si la yema esta hundida en el tablero (40 gf por mm). Fuera del tablero (el brazo en
        reposo cuelga por debajo de su plano) o mas de 5 cm por debajo, no lo toca."""
        Y = self.cin.fk(lado, self.robot.q()[JUNTAS_BRAZO[lado]], "yema")[:3, 3]
        h = float(self.plano.altura(Y))
        if h >= 0.0 or h < -0.05:
            return 0.0
        me = self.montaje["mesa"]
        pm = aplicar(inv(np.array(me["T_torso_mesa"], float)), Y)
        (x0, x1), (y0, y1) = me["superficie"]["x"], me["superficie"]["y"]
        if not (x0 <= pm[0] <= x1 and y0 <= pm[1] <= y1):
            return 0.0
        return 40.0 * -h * 1000.0

    # --- render ------------------------------------------------------------------------------------
    def actualizar(self):
        with self.cerrojo:
            self._actualizar()

    def _actualizar(self):
        q = self.robot.q()
        self.est.poner_cuerpo(q)
        for lado in LADOS:
            self.est.poner_mano(lado, angle_set_a_rad(self.manos[lado].ang))
            if self.sujeto[lado] is not None:
                M = self._T_mano(lado) @ self.T_rel[lado]
                self.poses[self.sujeto[lado]] = M
                self.est.poner_objeto(self.sujeto[lado], M[:3, 3], M[:3, :3])
        self.est.adelante()

    def fotograma(self):
        with self.cerrojo:
            self._actualizar()
            return self.cam.fotograma(self.est, t=time.monotonic())

    def parar(self):
        self.robot.parar()
        self.cam.cerrar()
