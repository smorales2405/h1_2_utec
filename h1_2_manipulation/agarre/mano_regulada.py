"""ManoRegulada: la Mano del reto (manip/mano.py) con el regulador de fuerza en el agarre.

Solo cambia lo que hace la mano al cerrar sobre un objeto, al sujetarlo y al decir si se escapo; todo lo demas
(preforma, abrir, leer, la comprobacion del agarre, el corte de fuerza del vigilante, la salida segura) es el de la
Mano del reto, sin tocarlo.

- `regular = True`: Mano.cerrar es el agarre regulado (agarre/lazo.py): aproximacion hibrida con cada dedo congelado
  al tocar, y el lazo en un hilo hasta abrir. sujetar() no hace nada (sujeta el lazo) y escapado() es lo que dice el
  lazo (perdido).
- `regular = False`: la Mano del reto tal cual (el brazo "firmware" del banco). Con observar_firmware, el mismo lazo
  en modo observador mira y apunta, sin mandar nada.
El objeto sale del plan activo del supervisor (`sup._plan_activo`); sin config del regulador para su clase, o si sus
dedos regulados no cierran en objetos.yaml, ese agarre es el del reto.
"""

import os
import time

import numpy as np
import yaml

from manip.mano import ANGLE_ACT, FORCE_ACT, FORCE_SET, SPEED_SET, Mano
from manip.mensajes import Parada

from .lazo import DEDOS_MODO, NOMBRE, NOMBRE_DEDO, LazoAgarre
from .reto import AQUI

RUTA_CONFIG = os.path.join(AQUI, "config", "regulador.yaml")


def cargar_config(ruta=RUTA_CONFIG):
    with open(ruta) as fh:
        return yaml.safe_load(fh)


class ManoRegulada(Mano):
    def __init__(self, lado, cfg, cliente=None, dormir=time.sleep, calibracion="auto", regulador=None, log=print):
        super().__init__(lado, cfg, cliente=cliente, dormir=dormir, calibracion=calibracion)
        self.rc = regulador if regulador is not None else cargar_config()
        self.regular = True
        self.sup = None             # el supervisor: su plan activo dice que objeto se coge
        self.carpeta = None         # donde dejar el CSV de cada agarre (la carpeta de la tirada)
        self.log = log
        self.lazo = None            # el del agarre en curso (regulando u observando)
        self.ultimo_lazo = None
        self.agarres = []           # el resumen de cada agarre (LazoAgarre.resumen)
        self.cero = np.zeros(6)
        self.t_cero = None
        self._medir_cero(forzar=True)

    # --- el cero (la tara por software) ---------------------------------------------------------------
    def _medir_cero(self, forzar=False):
        """La fuerza con la mano abierta y sin carga, >= t_base_s desde la ultima suelta (E3.5: tras sujetar, el cero
        se corre 21-46 gf y tarda ~8 s en volver). Sin escribir FORCE_CLB: no cambia nada de la mano (el brazo sin
        regulador sigue con su cero de siempre). Si no se cumple, se queda el anterior."""
        if not forzar and time.monotonic() - self.t_ultima_suelta < float(self.p["t_base_s"]):
            return False
        ang = self.regs(ANGLE_ACT, 6)
        if min(ang[k] for k in (2, 3, 4)) < 900:        # el pulgar, el indice y el medio, abiertos
            return False
        self.cero = np.median([self.regs(FORCE_ACT, 6) for _ in range(5)], 0)
        self.t_cero = time.monotonic()
        return True

    # --- utilidades ---------------------------------------------------------------------------------
    def _clase(self):
        plan = getattr(self.sup, "_plan_activo", None)
        return getattr(getattr(plan, "objeto", None), "clase", None)

    def _fase(self):
        return getattr(self.sup, "estado", "") or ""

    def _regulando(self):
        return self.lazo is not None and not self.lazo.observador

    def escribir_speed(self, v):
        self.escribir(SPEED_SET, [int(min(max(x, 0), 1000)) for x in v])

    def _config_objeto(self, clase, force_set):
        """(config del regulador de esa clase, dedos regulados) o (None, motivo)."""
        oc = (self.rc.get("objetos") or {}).get(clase) if clase else None
        if oc is None:
            return None, f"sin config del regulador para '{clase}'"
        dedos = DEDOS_MODO.get(int(oc["modo"]))
        if dedos is None:
            return None, f"'{clase}': modo {oc['modo']} desconocido"
        if not all(force_set[d] is not None and force_set[d] > 0 for d in dedos):
            return None, f"'{clase}': sus dedos regulados ({', '.join(NOMBRE[d] for d in dedos)}) no cierran en objetos.yaml"
        return oc, dedos

    def _aprox(self, oc):
        a = oc.get("aprox") or {}
        if self.lado in a:                                   # una tabla por mano, si la hay
            a = a[self.lado]
        return {NOMBRE_DEDO[k] if isinstance(k, str) else int(k): int(v) for k, v in a.items()}

    def _nuevo_lazo(self, clase, oc, observador):
        n = len(self.agarres) + 1
        ruta = None
        if self.carpeta:
            ruta = os.path.join(self.carpeta, f"regulador_{n:02d}_{clase}_{self.lado}"
                                + ("_firmware" if observador else "") + ".csv")
        lazo = LazoAgarre(self, self.rc, oc["modo"], oc["fuerza_gf"], self._aprox(oc), self.cierre, ruta_csv=ruta,
                          fase=self._fase, log=self.log, observador=observador)
        lazo.clase, lazo.n = clase, n
        return lazo

    def _force_set_crudo(self, force_set, dedos):
        """FORCE_SET en bruto de los 6 DOF: los regulados, el respaldo sobre su cero (el paro del firmware, por encima
        del lazo: si FORCE_ACT lo alcanza, el dedo deja de aceptar ANGLE_SET); los demas que cierran, el suyo como
        Mano.escribir_force_set (con la base y el residuo); el resto, el de la mano abierta."""
        respaldo = float(self.rc["seguridad"]["fset_respaldo_gf"])
        v = []
        for k, x in enumerate(force_set):
            if k in dedos:
                v.append(int(min(max(round(respaldo + float(self.cero[k])), self.fs_min), self.fs_max)))
            elif x is None or x < 0:
                v.append(self.fs_abierta)
            else:
                b = float(self.base[k]) if self.compensar_base else 0.0
                b += float(self.residuo[k]) if self.compensar_residuo else 0.0
                v.append(int(min(max(round(x + b), self.fs_min), self.fs_max)))
        return v

    def _parar_lazo(self):
        if self.lazo is None:
            return
        lazo, self.lazo = self.lazo, None
        lazo.parar()
        r = lazo.resumen()
        r.update({"n": lazo.n, "clase": lazo.clase, "lado": self.lado})
        self.agarres.append(r)
        self.ultimo_lazo = lazo

    # --- lo que cambia respecto a la Mano del reto -----------------------------------------------------
    def preforma(self, angulos, vigilar=None):
        self._parar_lazo()
        e = super().preforma(angulos, vigilar=vigilar)
        self._medir_cero()
        return e

    def cerrar(self, force_set, vigilar=None, speed=None):
        self._parar_lazo()
        clase = self._clase()
        oc, dedos = self._config_objeto(clase, force_set)
        if not self.regular or oc is None:
            if self.regular:
                self.log(f"  [regulador {self.lado}] {dedos}: el agarre del reto")
            elif oc is not None and self.rc.get("observar_firmware", True):
                self.lazo = self._nuevo_lazo(clase, oc, observador=True).arrancar()
            return super().cerrar(force_set, vigilar=vigilar, speed=speed)
        lazo = self.lazo = self._nuevo_lazo(clase, oc, observador=False)
        v = self.speed_cierre if speed is None else speed
        v = [int(x) for x in v] if isinstance(v, (list, tuple, np.ndarray)) else [int(v)] * 6
        vel = [min(x, self.speed_cierre) for x in v]
        fs = self._force_set_crudo(force_set, dedos)
        self.escribir(FORCE_SET, fs)
        self.force_set = np.array(fs, float)
        # los que cierran por FORCE_SET (el anular de la esfera), como en el reto: su velocidad y cerrado del todo
        firmware = [k for k in range(5) if force_set[k] is not None and force_set[k] > 0 and k not in dedos]
        if firmware:
            self.escribir_speed(vel)
            self.mover([self.cierre[k] if k in firmware else -1 for k in range(6)])
        lazo.aproximar(firmware, vel, self._vigilar_leyendo(vigilar))
        lazo.arrancar()
        lazo.esperar_establecido(self._vigilar_leyendo(vigilar))
        return self.leer()

    def _vigilar_leyendo(self, vigilar):
        """vigilar() y, cada 0.1 s, una lectura de la mano (manos.csv, y el vigilante la ve viva)."""
        t = [0.0]

        def f():
            if time.monotonic() - t[0] >= 0.1:
                t[0] = time.monotonic()
                self.leer()
            if vigilar is not None:
                vigilar()
        return f

    def sujetar(self, force_set):
        if self._regulando():
            return                           # sujeta el lazo
        return super().sujetar(force_set)

    def escapado(self, e, force_set):
        if self._regulando():
            return bool(self.lazo.perdido)
        return super().escapado(e, force_set)

    def leer(self):
        if self._regulando() and self.lazo.fallo:
            fallo = self.lazo.fallo
            self._parar_lazo()
            raise Parada(f"regulador de la mano {self.lado}: {fallo}")
        return super().leer()

    def abrir(self, vigilar=None, dedos=None):
        self._parar_lazo()
        return super().abrir(vigilar=vigilar, dedos=dedos)

    def cerrar_conexion(self):
        self._parar_lazo()
        super().cerrar_conexion()
