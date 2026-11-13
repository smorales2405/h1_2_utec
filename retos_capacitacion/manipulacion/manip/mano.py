"""Mano (seccion 5 del PDF): ordenes de alto nivel (preforma, cerrar, sujetar, abrir) -> Modbus TCP, y estado.

Inspire RH56DFTP, izquierda 192.168.124.211 y derecha .210 (al reves que la documentacion de Unitree).
DOF 0-5: menique, anular, medio, indice, flexion del pulgar, rotacion del pulgar; 0 cerrado, 1000 abierto.
Lo aprendido (memoria del proyecto, caja_cuadrado.py e inspire_hand_interface/Caracterizacion):
  - cada bloque de registros se lee por separado (1546..1620 de una vez da basura)
  - FORCE_SET (<= 800 gf) se escribe ANTES de cada cierre y la consigna queda en cerrado del todo: el
    firmware aprieta hasta esa fuerza y la mantiene (fijarla en el angulo actual suelta el objeto)
  - SPEED <= 100 al cerrar: el sobreimpulso lo manda la velocidad (+202 gf a 100, +637 a 250 con FS 500)
  - si FORCE_ACT llega a FORCE_SET el dedo deja de aceptar ANGLE_SET (medido en banco): para abrir se sube
    FORCE_SET primero y se comprueba con ANGLE_ACT que abrio; si no, se borra el error (1004) y se reintenta
  - la linea base de fuerza, con la mano abierta y >= 10 s desde la ultima suelta (el cero se corre 21-46 gf)
  - 1005 (SAVE) graba en la memoria de la mano: nunca se escribe
  - hay lecturas sueltas corruptas (3187 gf, 1873 gf, una POS fuera de sitio): un umbral necesita
    `persistencia_corte` lecturas seguidas (inspire_hand_interface exige 3 ciclos)
En el robot (probar_manos.py, 2026-11-11): con SPEED 100 los dedos cierran a ~150 cuentas/s (1000 -> 0 en ~7 s;
`t_cerrar_s`); el firmware compara FORCE_SET con FORCE_ACT en bruto, y con la mano abierta hay dedos a +210 gf
(`compensar_base`: se escribe FORCE_SET + esa base); cerrando en vacio con el pulgar enfrente, el indice choca con
la punta del pulgar (~250 gf con FORCE_SET 300): en vacio los dedos no llegan a 0.
"""

import os
import struct
import threading
import time

import numpy as np

from .mensajes import EstadoMano, Parada

ANGLE_SET, FORCE_SET, SPEED_SET = 1486, 1498, 1522
ANGLE_ACT, FORCE_ACT, ERROR, ESTADO = 1546, 1582, 1606, 1612
CLEAR, SAVE = 1004, 1005
BLOQUES_BYTE = (ERROR, ESTADO)                # un byte por DOF: 3 registros (inspire_sdkpy los lee asi)
TACTIL_INDICE = ((4110, 9), (4128, 96))     # punta y yema del indice (zonas de caja_cuadrado.py)
# las 17 zonas del tactil (inspire_hand_interface, Interfaz/core/tactile_zones.py): (nombre, registro, taxeles)
ZONAS_TACTIL = (("menique punta", 3000, 9), ("menique distal", 3018, 96), ("menique palmar", 3210, 80),
                ("anular punta", 3370, 9), ("anular distal", 3388, 96), ("anular palmar", 3580, 80),
                ("medio punta", 3740, 9), ("medio distal", 3758, 96), ("medio palmar", 3950, 80),
                ("indice punta", 4110, 9), ("indice distal", 4128, 96), ("indice palmar", 4320, 80),
                ("pulgar punta", 4480, 9), ("pulgar distal", 4498, 96), ("pulgar medio", 4690, 9),
                ("pulgar palmar", 4708, 96), ("palma", 4900, 112))
DEDOS = (0, 1, 2, 3)
PULGAR, ROTACION = 4, 5


class ManoSinRespuesta(Parada):
    pass


def bytes_de(registros):
    """ERROR, STATUS y TEMP van en bytes, dos DOF por registro y el primero en el byte alto (como inspire_sdkpy).
    Son 3 registros, no 6: los 3 siguientes quedan fuera del bloque y la mano los da como 0xFFFF (-1)."""
    return [(int(r) >> s) & 0xFF for r in registros for s in (8, 0)]


RUTA_CALIBRACION = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "manos.yaml")


def cargar_calibracion(lado, ruta=RUTA_CALIBRACION):
    """{tope_vacio, residuo_gf} de config/manos.yaml para `lado` (probar_manos.py --guardar-cierre), o None."""
    if not os.path.exists(ruta):
        return None
    import yaml
    with open(ruta) as fh:
        return (yaml.safe_load(fh) or {}).get(lado)


class Mano:
    def __init__(self, lado, cfg, cliente=None, dormir=time.sleep, calibracion="auto"):
        """cliente: un ModbusTcpClient (o el simulado de sim/mano_simulada.py en las pruebas). calibracion: la de
        config/manos.yaml de esta mano ("auto": solo con la mano real, sin cliente), un dict o None.
          tope_vacio[k]: donde se para el DOF k cerrando en vacio. Si un dedo se para lejos de 0 (el indice, contra
            la punta del pulgar), su consigna de cierre pasa a tope + margen_tope: en vacio no choca y llega a ella
            ("vacio"); con un objeto se para antes, con fuerza ("lleno").
          residuo_gf[k]: la fuerza que marca el dedo cerrado en vacio (su propia flexion: 100-390 gf en el robot).
            Con compensar_residuo se suma a FORCE_SET: si no, un dedo duro no cierra (anular derecho, 392 gf)."""
        m = cfg["mano"]
        self.lado = lado
        self.p = m
        self.fs_max = min(int(m["force_set_max"]), 800)
        self.fs_abierta = min(int(m["force_set_abierta"]), self.fs_max)
        self.corte = int(m["corte_gf"])
        self.persistencia = max(1, int(m.get("persistencia_corte", 3)))
        self.speed_cierre = min(int(m["speed_cierre"]), 100)
        self.speed_abrir = int(m["speed_abrir"])
        self.t_cerrar = float(m.get("t_cerrar_s", 10.0))
        self.compensar_base = bool(m.get("compensar_base", False))
        self.fs_min = int(m.get("force_set_min", 100))
        self.cierre = [int(x) for x in m.get("cierre", [0, 0, 0, 0, 300, -1])]
        self.compensar_residuo = bool(m.get("compensar_residuo", False))
        self.residuo = np.zeros(6)
        cal = cargar_calibracion(lado) if calibracion == "auto" and cliente is None else \
            (None if calibracion == "auto" else calibracion)
        self.calibracion = cal
        if cal:
            margen = int(m.get("margen_tope", 40))
            for k, tope in enumerate(cal.get("tope_vacio") or []):
                if k in DEDOS and tope is not None and tope > 60:
                    self.cierre[k] = max(self.cierre[k], int(tope) + margen)
            r = [float(x or 0.0) for x in (cal.get("residuo_gf") or [])][:6]
            self.residuo = np.array(r + [0.0] * (6 - len(r)))
        # con el residuo medido, un dedo en su consigna de cierre se distingue por la fuerza: vacio (~su residuo) o
        # apretando algo. Sin el, un dedo cerrado en vacio marca 70-390 gf (su flexion) y solo vale el angulo
        self.residuo_medido = bool(cal and cal.get("residuo_gf")) and self.compensar_residuo
        self._dormir = dormir
        if cliente is None:
            from pymodbus.client import ModbusTcpClient
            cliente = ModbusTcpClient(cfg["red"]["manos"][lado], port=cfg["red"]["puerto_manos"], timeout=1)
        self.c = cliente
        if not self.c.connect():
            raise ManoSinRespuesta(f"no conecta con la mano {lado}")
        self.cerrojo = threading.Lock()
        self.base = np.zeros(6)
        self.base_tactil = None
        self.force_set = np.full(6, -1.0)
        self.t_ultima_suelta = -1e9
        self.t_ultima_lectura = 0.0
        self.ultimo = None
        self.escribir(SPEED_SET, [self.speed_cierre] * 6)
        self.escribir_force_set([self.fs_abierta] * 6)

    # --- registros -----------------------------------------------------------------------------------
    def _crudos(self, a, n):
        with self.cerrojo:
            r = self.c.read_holding_registers(a, n, 1)
        if r is None or (hasattr(r, "isError") and r.isError()):
            raise ManoSinRespuesta(f"mano {self.lado}: fallo al leer {a}")
        return r

    def regs(self, a, n, signo=True):
        if a in BLOQUES_BYTE:
            return np.array(bytes_de(self._crudos(a, 3).registers), dtype=float)
        r = self._crudos(a, n)
        f = "h" if signo else "H"
        return np.array(struct.unpack(f">{n}{f}", struct.pack(f">{n}H", *r.registers)), dtype=float)

    def escribir(self, a, v):
        if a == SAVE or a <= SAVE < a + len(v):
            raise ValueError("1005 (SAVE) graba en la mano: nunca")
        with self.cerrojo:
            r = self.c.write_registers(a, [int(x) & 0xFFFF for x in v], 1)
        if r is not None and hasattr(r, "isError") and r.isError():
            raise ManoSinRespuesta(f"mano {self.lado}: fallo al escribir {a}")

    def escribir_force_set(self, fs):
        """gf por DOF (netos: sobre la linea base si compensar_base), recortados a [force_set_min, tope]; -1 (no
        cierra) deja el de mano abierta. El firmware compara con FORCE_ACT en bruto: se le suma la base."""
        v = []
        for k, x in enumerate(fs):
            if x is None or x < 0:
                v.append(self.fs_abierta)
                continue
            b = float(self.base[k]) if self.compensar_base else 0.0
            b += float(self.residuo[k]) if self.compensar_residuo else 0.0
            v.append(int(min(max(round(x + b), self.fs_min), self.fs_max)))
        self.escribir(FORCE_SET, v)
        self.force_set = np.array(v, float)

    def mover(self, angulos):
        """ANGLE_SET de los 6 DOF; -1 = no tocar ese DOF."""
        self.escribir(ANGLE_SET, [-1 if a is None or a < 0 else int(min(max(a, 0), 1000)) for a in angulos])

    def borrar_error(self):
        self.escribir(CLEAR, [1])

    def leer(self) -> EstadoMano:
        t = time.monotonic()
        e = EstadoMano(t=t, lado=self.lado, angulo=self.regs(ANGLE_ACT, 6), fuerza=self.regs(FORCE_ACT, 6),
                       base=self.base.copy(), force_set=self.force_set.copy(), error=self.regs(ERROR, 6),
                       estado=self.regs(ESTADO, 6))
        self.t_ultima_lectura, self.ultimo = t, e
        return e

    def tactil_yema(self):
        """Lo que sube el tactil de la punta y la yema del indice sobre su base (maximo, unidades del sensor)."""
        v = np.concatenate([self.regs(a, n, signo=False) for a, n in TACTIL_INDICE])
        if self.base_tactil is None:
            return 0.0
        return float((v - self.base_tactil).max())

    # --- ordenes ------------------------------------------------------------------------------------
    def _esperar_quieta(self, timeout_s, tol=6, quieta_s=0.3, vigilar=None):
        """Espera a que ANGLE_ACT se quede `quieta_s` sin apartarse mas de `tol` de donde estaba al empezar ese rato
        (no entre lecturas seguidas: despacio, cada lectura cambia menos que tol aunque el dedo se mueva). Vigila
        la fuerza: por encima del corte, Parada."""
        t0 = time.monotonic()
        ref, desde, sobre = None, None, 0
        while True:
            e = self.leer()
            if vigilar:
                vigilar()
            sobre = sobre + 1 if (e.fuerza - e.base).max() > self.corte else 0
            if sobre >= self.persistencia:
                self.mover([1000, 1000, 1000, 1000, 1000, -1])
                raise Parada(f"mano {self.lado}: {int(e.fuerza.max())} gf > corte de {self.corte} "
                             f"en {sobre} lecturas seguidas")
            ahora = time.monotonic()
            if ref is None or np.abs(e.angulo - ref).max() > tol:
                ref, desde = e.angulo.copy(), ahora
            elif ahora - desde >= quieta_s:
                return e
            if ahora - t0 > timeout_s:
                return e
            self._dormir(0.05)

    def preforma(self, angulos, vigilar=None):
        """Lleva la mano a una preforma. Si cambia la rotacion del pulgar, primero abre su flexion (si gira
        con los dedos cerrados, choca con ellos)."""
        e = self.leer()
        if angulos[ROTACION] >= 0 and abs(e.angulo[ROTACION] - angulos[ROTACION]) > 50:
            self.escribir_force_set([self.fs_abierta] * 6)
            self.escribir(SPEED_SET, [self.speed_abrir] * 6)
            self.mover([-1, -1, -1, -1, 1000, -1])
            self._esperar_quieta(2.0, vigilar=vigilar)
            self.mover([-1, -1, -1, -1, -1, angulos[ROTACION]])
            self._esperar_quieta(2.0, vigilar=vigilar)
        self.escribir(SPEED_SET, [self.speed_abrir] * 6)
        self.mover(angulos)
        return self._esperar_quieta(3.0, vigilar=vigilar)

    def cerrar(self, force_set, vigilar=None):
        """Cierra los DOF con force_set > 0: FORCE_SET primero, velocidad lenta, consigna cerrada del todo."""
        self.escribir_force_set(force_set)
        self.escribir(SPEED_SET, [self.speed_cierre] * 6)
        self.mover([self.cierre[k] if force_set[k] is not None and force_set[k] > 0 else -1 for k in range(6)])
        return self._esperar_quieta(self.t_cerrar, vigilar=vigilar)

    def sujetar(self, force_set):
        """Reafirma FORCE_SET y la consigna cerrada (cada ~1 s mientras se lleva el objeto)."""
        self.escribir_force_set(force_set)
        self.mover([self.cierre[k] if force_set[k] is not None and force_set[k] > 0 else -1 for k in range(6)])

    def abrir(self, vigilar=None, dedos=None):
        """Abre la flexion del pulgar y los dedos, sin girar el pulgar, y comprueba que abrio. `dedos`: solo esos DOF
        (los que sujetan); los demas se quedan como estan (al soltar, los recogidos de la preforma barrian el objeto
        al estirarse, RESULTADOS §70)."""
        abrir = [k for k in range(5) if dedos is None or k in dedos]
        for intento in range(2):
            self.escribir_force_set([self.fs_abierta] * 6)      # desenclava los dedos que llegaron a FORCE_SET
            self.escribir(SPEED_SET, [self.speed_abrir] * 6)
            self.mover([1000 if k in abrir else -1 for k in range(5)] + [-1])
            e = self._esperar_quieta(self.p["t_abrir_s"], vigilar=vigilar)
            if min(e.angulo[k] for k in abrir) >= self.p["abierta_min"]:
                self.t_ultima_suelta = time.monotonic()
                return e
            self.borrar_error()
            self._dormir(0.2)
        raise ManoSinRespuesta(f"mano {self.lado}: no abre (angulos {e.angulo.astype(int).tolist()})")

    def linea_base(self, n=10):
        """Fuerza (y tactil del indice) con la mano abierta y sin carga. Exige >= t_base_s desde la ultima
        suelta: tras sujetar, el cero se corre 21-46 gf y tarda ~8 s en volver."""
        espera = self.p["t_base_s"] - (time.monotonic() - self.t_ultima_suelta)
        if espera > 0:
            self._dormir(espera)
        f = np.mean([self.leer().fuerza for _ in range(n)], 0)
        self.base = f
        self.base_tactil = np.mean([np.concatenate([self.regs(a, k, signo=False) for a, k in TACTIL_INDICE])
                                    for _ in range(3)], 0)
        return f

    def agarre(self, e: EstadoMano, force_set, vacio_max=60, fuerza_min=80):
        """La pista del PDF: un dedo que se detiene lejos de 0 con fuerza tiene algo delante; uno que llega a
        0 esta vacio. (ok, {dof: "lleno" | "vacio" | "flojo"}). Hacen falta el pulgar y al menos un dedo."""
        detalle = {}
        # sin su residuo de flexion: lo que marca cerrado en vacio (el pulgar izquierdo, 47 gf)
        sin_residuo = e.fuerza - e.base - (self.residuo if self.compensar_residuo else 0.0)
        for k in range(5):
            if force_set[k] is None or force_set[k] <= 0:
                continue
            neta = e.fuerza[k] - e.base[k]
            if (e.angulo[k] <= max(vacio_max, self.cierre[k] + vacio_max if self.cierre[k] > 0 else 0)
                    and not (self.residuo_medido and sin_residuo[k] >= 0.5 * float(force_set[k]))):
                # llego (o casi) a su consigna de cierre (con tope_vacio, la del indice queda antes del pulgar) y sin
                # apenas fuerza: en vacio. Con fuerza aprieta algo: el pulgar cierra el cilindro de 5 cm en 302-331
                # (su consigna, 300) con 290-460 gf, y el agarre se daba por fallido (RESULTADOS §67)
                detalle[k] = "vacio"
            elif neta >= fuerza_min:
                detalle[k] = "lleno"
            else:
                detalle[k] = "flojo"
        llenos = [k for k, v in detalle.items() if v == "lleno"]
        return (PULGAR in llenos and any(k in llenos for k in DEDOS)), detalle

    def escapado(self, e: EstadoMano, force_set):
        """El objeto ya no esta entre los dedos: el pulgar, o todos los dedos que agarran, han llegado a su consigna
        de cierre ("vacio" de agarre). Un dedo "flojo" (parado lejos de 0 con poca fuerza) aun lo sujeta: el indice
        marco 58 gf con el cubo en la mano. La suma de las fuerzas no servia: en tres_dedos el menique y el anular
        recogidos marcan ~460 gf sin tocar nada y el cubo se escapo sin que bajara del 30 % (RESULTADOS §57)."""
        _, det = self.agarre(e, force_set)
        # y sin apenas fuerza (menos de la mitad de su FORCE_SET, sin su residuo de flexion): el pulgar llego a 312 (su
        # tope, 300) apretando el cubo con 1526 gf y daba un escape falso (§61). Con el residuo medido, agarre() ya lo
        # pide; sin el, tambien aqui
        neta = e.fuerza - e.base - (self.residuo if self.compensar_residuo else 0.0)
        vacio = {k: v == "vacio" and neta[k] < 0.5 * float(force_set[k]) for k, v in det.items()}
        dedos = [v for k, v in vacio.items() if k in DEDOS]
        return vacio.get(PULGAR, False) or (bool(dedos) and all(dedos))

    def cerrar_conexion(self):
        with self.cerrojo:
            self.c.close()
