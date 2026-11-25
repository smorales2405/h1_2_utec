"""Mano Inspire simulada para el regulador: la del reto (sim/mano_simulada.py) con lo que el lazo necesita ver.

- POS_ACT (cuentas, crece al cerrar): (1000 - angulo) x 1.75 los dedos y x 1.00 el pulgar (A4, medido).
- Cada DOF publica cada 30.7 ms, escalonados 1.9 ms (E3.6a): entre publicaciones se lee el mismo valor.
- Velocidad por DOF: 1.45 cuentas/s por unidad de SPEED los dedos y 3.1 el pulgar (las tiradas: 145 y 310 a 100).
- Cuanto minimo (E3.2): un ANGLE_SET a menos de 5 (cerrando) o 3 (abriendo) del angulo de un dedo parado no lo mueve.
- El objeto flota entre el pulgar y los dedos de enfrente (sujeto al brazo y a la mesa por un muelle `k_ancla`): cada
  dedo que toca es un muelle (gf por cuenta cerrada pasado el contacto) y el objeto se desplaza hasta equilibrarlos.
  Sale el acoplamiento que midio el banco: cerrar el indice sube la fuerza del pulgar y BAJA la del medio (B1), y un
  dedo solo empuja el objeto en vez de apretarlo. Sobre el residual de flexion de la del reto. El firmware para el
  dedo al llegar a FORCE_SET y lo deja enclavado (como la del reto).
  El cubo, con la planta medida en el robot (PLANTA): sus escalones valen lo que alli (el indice ~155 gf).
- Relajacion (E3.4): la fuerza de contacto cae `relajar` (8 %) con tau 1.5 s tras cada movimiento y se agota.
- Ruido de +-ruido gf en cada publicacion; `vaiven` (gf, periodo): una fuerza que empuja el objeto entre los dedos,
  como el brazo moviendose en el transporte.
- resbalar(du) y quitar_objeto(): el objeto se escurre (menos penetracion) o se va (las pruebas de los detectores);
  empujar(dof, du, sostenido): un tiron echa atras un dedo (vuelve solo, o se queda si el actuador cede).
No es la fisica del agarre (el objeto no se mueve ni gira): es la planta que midio el banco, para probar el lazo, sus
fases y sus detectores sin el robot.
"""

import math
import threading
import time

import numpy as np

from hand_modbus import CURRENT, POS_ACT
from manip import mano as m
from sim.mano_simulada import ManoSimulada

TEMP = 1618
VEL = np.array([1.45, 1.45, 1.45, 1.45, 3.1, 1.45])       # cuentas/s por unidad de SPEED_SET
POS_U = np.array([1.75, 1.75, 1.75, 1.75, 1.0, 1.0])
PERIODO = 0.0307
# rigidez de cada dedo contra el objeto (gf por cuenta) y del objeto contra el brazo y la mesa. Con estas, la
# ganancia de un dedo solo (lo que mide el banco) sale ~7 gf/u el pulgar y ~9 el indice
K_DEDO = np.array([12.0, 12.0, 13.0, 14.0, 9.0, 0.0])
K_ANCLA = 5.0

# donde toca cada dedo a cada objeto en el robot (las tiradas de nivel 2 del 15 al 19-nov; README) y cuanto mas rigido
# es que la bola de espuma del banco (el PLA, 1.2-1.5x)
CONTACTO = {"cubo": {2: 715, 3: 743, 4: 457}, "esfera": {1: 568, 2: 639, 3: 643, 4: 642},
            "cilindro": {3: 720, 4: 389}, "prisma": {2: 676, 3: 706, 4: 334}, "prisma_de_pie": {3: 702, 4: 342}}
RIGIDEZ = {"cubo": 1.5, "esfera": 1.3, "cilindro": 1.3, "prisma": 1.5, "prisma_de_pie": 1.5}
# el cubo, MEDIDO en el robot (mano derecha, 2026-11-25: los escalones aislados del lazo en los bancos 095047 y
# 104925): fuerza de un dedo por unidad de su comando, pulgar 10.8, indice 31, medio 25, e indice<->medio -5.6. Lo
# dan estas rigideces con el objeto anclado a 121 (pulgar<-indice sale +2.2; medido ~0.5)
PLANTA = {"cubo": {"k": {4: 11.5, 3: 38.3, 2: 29.3}, "k_ancla": 121.3}}


def planta(clase):
    """Los argumentos de poner_objeto para una clase: la planta medida si la hay; si no, la rigidez relativa."""
    return dict(PLANTA[clase]) if clase in PLANTA else {"rigidez": RIGIDEZ.get(clase, 1.3)}


class ManoFisica(ManoSimulada):
    def __init__(self, relajar=0.08, tau=1.5, ruido=2.0, semilla=0, cuanto=True):
        super().__init__()
        self.k = K_DEDO.copy()             # gf por cuenta hundida, por dedo (ya con la rigidez del objeto)
        self.k_ancla = K_ANCLA
        self.vaiven = (0.0, 1.0)           # (gf, periodo s): el brazo moviendose empuja el objeto entre los dedos
        self.relajar, self.tau, self.ruido, self.cuanto = relajar, tau, ruido, cuanto
        self.rng = np.random.default_rng(semilla)
        self.k_rel = np.ones(6)
        self.t_mov = time.monotonic()
        self._fase = np.arange(6) * 0.0019
        self._prox = time.monotonic() + self._fase
        self.pub_f = self._fuerza_real()
        self.pub_ang = self.ang.copy()
        self._cerrojo_sim = threading.RLock()

    # --- el objeto -------------------------------------------------------------------------------------
    def poner_objeto(self, angulos_contacto, rigidez=1.0, k=None, k_ancla=None):
        """rigidez: todo x rigidez (los muelles de K_DEDO y el ancla); k ({dof: gf/u}) y k_ancla: los de una planta
        medida (PLANTA)."""
        with self._cerrojo_sim:
            self.contacto = {int(j): float(a) for j, a in dict(angulos_contacto).items()}
            self.k = K_DEDO * float(rigidez)
            self.k_ancla = K_ANCLA * float(rigidez) if k_ancla is None else float(k_ancla)
            for j, v in (k or {}).items():
                self.k[int(j)] = float(v)
            self.k_rel[:] = 1.0

    def empujar(self, dof, du, sostenido=False):
        """Un tiron echa el dedo `du` unidades atras (abre). Suelto, su control lo devuelve al comando a su velocidad
        (el vaiven del transporte); sostenido, se queda alli (el actuador cede: un resbalon de verdad)."""
        with self._cerrojo_sim:
            self.ang[dof] = min(1000.0, self.ang[dof] + du)
            if sostenido:
                self.atascado.add(dof)

    def resbalar(self, du):
        """El objeto se escurre entre los dedos: cada contacto `du` cuentas mas cerrado (menos penetracion)."""
        with self._cerrojo_sim:
            self.contacto = {k: a - du for k, a in self.contacto.items()}

    def quitar_objeto(self):
        with self._cerrojo_sim:
            self.contacto = {}

    # --- la planta --------------------------------------------------------------------------------------
    def penetraciones(self):
        """{dof: cuentas hundidas en el objeto} con el objeto en equilibrio entre el pulgar (empuja en +x) y los
        dedos de enfrente (en -x): x = (k_T p_T - suma k_g p_g) / (suma k + k_ancla), solo con los que tocan."""
        p = {j: a - self.ang[j] for j, a in self.contacto.items()}           # cerrado pasado el contacto
        activos = {j for j, v in p.items() if v > 0}
        amp, per = self.vaiven
        ext = amp * math.sin(2 * math.pi * time.monotonic() / per) if amp else 0.0
        x = 0.0
        for _ in range(4):
            num = sum((self.k[j] if j == 4 else -self.k[j]) * p[j] for j in activos) + (ext if activos else 0.0)
            x = num / (sum(self.k[j] for j in activos) + self.k_ancla) if activos else 0.0
            pen = {j: (p[j] - x if j == 4 else p[j] + x) for j in p}
            nuevos = {j for j, v in pen.items() if v > 0}
            if nuevos == activos:
                break
            activos = nuevos
        return {j: max(0.0, v) for j, v in pen.items()} if p else {}

    def _fuerza_real(self):
        f = 70 + 160 * (1000 - self.ang) / 1000            # el residual de flexion (el de la mano del reto)
        for j, v in self.penetraciones().items():
            f[j] += self.k[j] * v * self.k_rel[j]
        return f

    def _fuerza(self):
        return self._fuerza_real()

    def _avanzar(self):
        with self._cerrojo_sim:
            ahora = time.monotonic()
            dt = ahora - self._t
            self._t = ahora
            antes = self.ang.copy()
            n = max(1, int(math.ceil(dt / 0.002)))
            h = dt / n
            for _ in range(n):
                for k in range(6):
                    if k in self.atascado:
                        continue
                    d = self.obj[k] - self.ang[k]
                    if self.enclavado[k] and d < 0:
                        continue
                    paso = VEL[k] * self.speed[k] * h
                    self.ang[k] += max(-paso, min(paso, d))
                f = self._fuerza_real()
                for k in range(6):
                    # el firmware: cerrando, al llegar a FORCE_SET se para ahi y se enclava, y asi se queda (sin aceptar
                    # ANGLE_SET) hasta que se escriba un FORCE_SET por encima de su fuerza, aunque el objeto se vaya
                    # (en el robot, §84: los dedos se quedan bloqueados donde cerraron)
                    if self.obj[k] < self.ang[k] - 1e-6 and f[k] >= self.fset[k] - 1:
                        self.enclavado[k] = True
            if any(abs(self.ang[k] - antes[k]) > 0.3 for k in self.contacto):
                self.k_rel[:] = 1.0
                self.t_mov = ahora
            else:
                self.k_rel[:] = 1.0 - self.relajar * (1.0 - math.exp(-(ahora - self.t_mov) / self.tau))
            f = self._fuerza_real()
            for k in range(6):
                if ahora >= self._prox[k]:
                    self.pub_f[k] = f[k] + self.rng.uniform(-self.ruido, self.ruido)
                    self.pub_ang[k] = self.ang[k]
                    self._prox[k] += PERIODO * max(1, math.ceil((ahora - self._prox[k]) / PERIODO + 1e-9))

    # --- la interfaz de pymodbus ------------------------------------------------------------------------
    def read_holding_registers(self, a, n, slave=1):
        from types import SimpleNamespace
        self._avanzar()
        with self._cerrojo_sim:
            if a == m.ANGLE_ACT:
                v = np.round(self.pub_ang)
            elif a == m.FORCE_ACT:
                v = np.round(self.pub_f)
            elif a == POS_ACT:
                v = np.round((1000 - self.pub_ang) * POS_U)
            elif a == CURRENT:
                v = np.where(np.abs(self.obj - self.ang) > 1, 100, 0)
            elif a == TEMP:
                v = [0x1E1E] * 3 + [0xFFFF] * 3
            else:
                return super().read_holding_registers(a, n, slave)
        regs = [int(round(x)) & 0xFFFF for x in np.asarray(v)[:n]]
        return SimpleNamespace(registers=regs, isError=lambda: False)

    def write_registers(self, a, valores, slave=1):
        if a == m.ANGLE_SET and self.cuanto:
            self._avanzar()
            with self._cerrojo_sim:
                v = [x - 0x10000 if x >= 0x8000 else x for x in valores]
                for k, x in enumerate(v):
                    # el cuanto (E3.2): por debajo, un dedo parado no se mueve
                    if x >= 0 and abs(self.obj[k] - self.ang[k]) < 0.5:
                        minimo = 5 if x < self.ang[k] else 3
                        if abs(x - self.ang[k]) < minimo:
                            v[k] = -1
                valores = [x & 0xFFFF for x in v]
        return super().write_registers(a, valores, slave)


class ManoFisicaDelMundo(ManoFisica):
    """La de arriba en el mundo simulado del reto (sim/simulacion.py, como ManoDelMundo): al cerrar sobre un objeto lo
    coge (con los contactos de CONTACTO para su clase) y al abrir lo suelta."""

    def __init__(self, mundo, lado, **kw):
        super().__init__(**kw)
        self.mundo, self.lado = mundo, lado

    def _fuerza_real(self):
        f = super()._fuerza_real()
        # la yema contra la mesa (el nivel 1) solo desde el hilo principal: la cinematica del mundo es la del
        # planificador y no se puede usar a la vez desde el hilo del regulador (que solo corre agarrando)
        if getattr(self, "mundo", None) is not None and threading.current_thread() is threading.main_thread():
            f[3] += self.mundo.fuerza_yema(self.lado)
        return f

    def write_registers(self, a, valores, slave=1):
        r = super().write_registers(a, valores, slave)
        if a == m.ANGLE_SET:
            v = [x - 0x10000 if x >= 0x8000 else x for x in valores]
            if any(0 <= x < 500 for x in v[:5]):
                self.mundo.al_cerrar(self.lado)
            elif all(x < 0 or x >= 900 for x in v[:5]) and any(x >= 900 for x in v[:5]):
                self.mundo.al_abrir(self.lado)
        return r


def mundo_fisico():
    """La clase del mundo simulado del reto con estas manos (se importa tarde: sim.simulacion carga MuJoCo)."""
    from sim.simulacion import MundoSimulado

    class MundoFisico(MundoSimulado):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.manos = {l: ManoFisicaDelMundo(self, l) for l in self.manos}

        def _al_cerrar(self, lado):
            antes = self.sujeto[lado]
            super()._al_cerrar(lado)
            clase = self.sujeto[lado]
            if clase is not None and antes is None and clase in CONTACTO:
                mano = self.manos[lado]
                mano.poner_objeto({k: a for k, a in CONTACTO[clase].items() if mano.ang[k] > a + 30}, **planta(clase))

    return MundoFisico
