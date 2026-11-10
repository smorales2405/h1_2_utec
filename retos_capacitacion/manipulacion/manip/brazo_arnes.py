"""Brazo de manipulacion.py con el robot de pie en el arnes (PLAN, opcion B, 2026-11-10): la interfaz de
brazo.Brazo que usan el supervisor y el vigilante, pero sin publicar nada. Pide los movimientos a
herramientas/arnes.py (manip/servidor_brazos.py), que es el unico que publica en rt/lowcmd y sigue sosteniendo
las piernas pase lo que pase aqui.

  - "peso": no hay arm_sdk. rampa_peso(1) toma el control de los brazos (arnes.py solo lo da de pie, nivelado y
    con los brazos a 0) y rampa_peso(0) lo devuelve. peso es 1 con el control y 0 sin el.
  - ir_a: el movimiento lo hace el hilo de 500 Hz de arnes.py (minima sacudida, >= 8 s, <= 0.5 rad/s); aqui se
    espera mirando su estado y llamando a `comprobar`. Si `comprobar` lanza algo (Parada, el contacto de la yema)
    o arnes.py deja de contestar, se congelan los brazos ANTES de dejar salir la excepcion: con arm_sdk el brazo
    se paraba solo porque los pasos los daba este bucle; aqui seguiria hasta el final.
  - motivo_parada(): lo de arnes.py que obliga a la salida segura (parada alli, amortiguado, control perdido o sin
    respuesta). Lo consulta el vigilante.
  - en_salida_segura(): desde ahi los movimientos van marcados de salida (arnes.py los acepta tras su parada).
  - Un hilo manda el latido cada `latido_s` con el control dado: sin el, arnes.py congela y quita el control.
"""

import itertools
import json
import os
import socket
import threading
import time

import numpy as np

from .brazo import BRAZOS, T_MIN_TOPE, VEL_TOPE, duracion_movimiento
from .mensajes import CINTURA, JUNTAS_BRAZO, Parada

_N = itertools.count(1)


class BrazoArnes:
    def __init__(self, robot, cfg, escala_tiempo=1.0, puerto=None, recuperar=False, dormir=time.sleep,
                 reloj=time.monotonic, cada_s=0.02, latido_s=0.25, timeout_s=1.0, sin_respuesta_s=1.5,
                 nombre="manipulacion.py"):
        b = cfg["brazo"]
        self.robot, self.cfg = robot, cfg
        self.hz = float(b["hz"])
        self.t_min = max(float(b["t_min_mov_s"]), T_MIN_TOPE)
        self.vel_max = min(float(b["vel_max_rad_s"]), VEL_TOPE)
        self.escala_tiempo = max(1.0, float(escala_tiempo))
        m = float(b["margen_limites_rad"])
        self.limites = {int(j): (lo + m, hi - m) for j, (lo, hi) in b["limites"].items()}
        self.t_entrada, self.t_salida = 0.0, 0.0
        self.texto_entrada = "arnes.py da el control de los brazos (de pie, nivelado, brazos a 0)"
        self.puerto = int(puerto or cfg["red"]["puerto_brazos"])
        self.recuperar = recuperar
        n = next(_N)
        self.id = f"{nombre} (pid {os.getpid()}" + (f", {n})" if n > 1 else ")")
        self._dormir, self.reloj, self.cada, self.latido = dormir, reloj, cada_s, latido_s
        self.timeout, self.sin_respuesta = timeout_s, sin_respuesta_s
        self.peso = 0.0
        self._salida = False
        self._estado, self._t_estado, self._t_ok = {}, -1e9, reloj()
        self._cerrojo = threading.RLock()
        self._sock, self._rf = None, None
        e = self._pedir({"op": "estado"})
        if e is None:
            raise RuntimeError(f"arnes.py no contesta en 127.0.0.1:{self.puerto}: ¿esta corriendo con el robot de pie?")
        self.q_inicial = robot.q()
        self._vivo = True
        self._hilo = threading.Thread(target=self._latir, daemon=True)
        self._hilo.start()

    # --- conexion -------------------------------------------------------------------------------------
    def _conectar(self):
        self._cerrar_socket()
        self._sock = socket.create_connection(("127.0.0.1", self.puerto), timeout=self.timeout)
        self._rf = self._sock.makefile("rb")

    def _cerrar_socket(self):
        for x in (self._rf, self._sock):
            try:
                if x is not None:
                    x.close()
            except OSError:
                pass
        self._sock, self._rf = None, None

    def _pedir(self, req):
        """La respuesta (dict) o None si arnes.py no contesta (se reconecta en la siguiente)."""
        with self._cerrojo:
            try:
                if self._sock is None:
                    self._conectar()
                self._sock.sendall((json.dumps(dict(req, id=self.id)) + "\n").encode())
                linea = self._rf.readline()
                if not linea:
                    raise OSError("conexion cerrada")
                r = json.loads(linea)
            except (OSError, ValueError):
                self._cerrar_socket()
                return None
            self._t_ok = self.reloj()
            if "consigna" in r:
                self._estado, self._t_estado = r, self.reloj()
            return r

    def _latir(self):
        while self._vivo:
            if self.peso > 0.5 and self.reloj() - self._t_ok >= self.latido:
                self._pedir({"op": "estado"})
            time.sleep(self.latido / 2)

    def estado(self, max_edad=0.1):
        """El ultimo estado de arnes.py (se pide si tiene mas de max_edad s)."""
        if self.reloj() - self._t_estado > max_edad:
            self._pedir({"op": "estado"})
        return self._estado

    def medir(self):
        """El estado de arnes.py con lo medido del cuerpo (`cuerpo`: IMU, pies, q y par de las 27), o None si no
        contesta. Del que tiene el control cuenta como latido."""
        return self._pedir({"op": "estado", "cuerpo": True})

    # --- lo que obliga a parar ---------------------------------------------------------------------------
    def motivo_parada(self):
        if self.peso < 0.5:
            return None
        e = self.estado()
        if self.reloj() - self._t_ok > self.sin_respuesta:
            return f"arnes.py no contesta desde hace {self.reloj() - self._t_ok:.1f} s"
        return self._motivo(e)

    def _motivo(self, e):
        if e.get("amortiguado"):
            return f"arnes.py amortiguado ({e.get('motivo_amortiguado')})"
        if not e.get("tuyo"):
            return "arnes.py ha quitado el control de los brazos"
        if e.get("parada") and not self._salida:
            return f"parada en arnes.py: {e['parada']}"
        return None

    def en_salida_segura(self):
        self._salida = True

    # --- interfaz de brazo.Brazo ------------------------------------------------------------------------
    def recortar(self, j, q):
        lo, hi = self.limites[j]
        return min(max(float(q), lo), hi)

    def duracion(self, ini, fin, pedida=None):
        dq = max((abs(fin[j] - ini[j]) for j in fin), default=0.0)
        return duracion_movimiento(dq, self.t_min, self.vel_max, self.escala_tiempo, pedida)

    def pose_brazo(self, lado, q7):
        return {j: float(v) for j, v in zip(JUNTAS_BRAZO[lado], q7)}

    def consigna(self):
        """{junta 12-26: q} de lo que manda arnes.py (lo ultimo sabido: no pregunta)."""
        c = self._estado.get("consigna")
        q = np.asarray(c, float) if c is not None else self.robot.q()
        return {j: float(q[j]) for j in [CINTURA] + BRAZOS}

    def error(self, lados=("izq", "der")):
        q = self.robot.q()
        c = self.estado().get("consigna")
        if c is None:
            return 0.0
        return max(abs(q[j] - c[j]) for l in lados for j in JUNTAS_BRAZO[l])

    def rampa_peso(self, hasta, segundos=None, comprobar=None):
        """Con arnes.py no hay peso: tomar (hasta > 0.5) o soltar el control de los brazos."""
        if hasta > 0.5 and self.peso < 0.5:
            r = self._pedir({"op": "tomar", "recuperar": self.recuperar})
            if r is None or not r.get("ok"):
                raise Parada(f"arnes.py no da los brazos: {'no contesta' if r is None else r.get('error')}")
            self.peso = 1.0
        elif hasta <= 0.5 and self.peso > 0.5:
            self.peso = 0.0
            self._pedir({"op": "soltar"})
        return 0.0

    def ir_a(self, pose, segundos=None, comprobar=None):
        """Minima sacudida (en arnes.py) desde su consigna hasta `pose` {junta: q} (recortada a los limites; la
        cintura no se mueve). Espera a que acabe llamando a `comprobar`. Devuelve la duracion."""
        if self.peso < 0.5:
            raise Parada("sin el control de los brazos de arnes.py")
        ini = self.consigna()
        fin = dict(ini)
        for j, q in pose.items():
            if j != CINTURA:
                fin[j] = self.recortar(j, q)
        T = self.duracion(ini, fin, segundos)
        r = self._pedir({"op": "mover", "q": [fin[j] for j in BRAZOS], "segundos": T, "salida": self._salida})
        if r is None:
            raise Parada("arnes.py no contesta al mover")
        if not r.get("ok"):
            # una parada o amortiguacion de alli, con su nombre; si no, lo que diga
            raise Parada(self._motivo(self.estado(max_edad=0.0)) or f"arnes.py no mueve: {r.get('error')}")
        T = float(r["T"])
        t_tope = self.reloj() + T + 5.0
        try:
            while True:
                e = self.estado(max_edad=0.0)
                if self.reloj() - self._t_ok > self.sin_respuesta:
                    raise Parada(f"arnes.py no contesta desde hace {self.reloj() - self._t_ok:.1f} s")
                m = self._motivo(e)
                if m:
                    raise Parada(m)
                if not e.get("moviendo"):
                    break
                if comprobar:
                    comprobar()
                if self.reloj() > t_tope:
                    raise Parada(f"el movimiento de {T:.1f} s no acaba")
                self._dormir(self.cada)
        except BaseException:
            self.congelar()
            raise
        return T

    def seguir(self, lado, puntos, comprobar=None, al_llegar=None):
        for i, p in enumerate(puntos):
            self.ir_a(self.pose_brazo(lado, p.q), p.duracion, comprobar)
            if al_llegar:
                al_llegar(i)

    def congelar(self):
        """Los brazos se quedan donde estan (arnes.py para el movimiento en curso)."""
        if self.peso > 0.5:
            self._pedir({"op": "congelar"})

    def parar(self):
        """Fin: deja de latir y cierra. Con el control aun dado (la tirada no acabo bien), congela y lo devuelve:
        los brazos se quedan donde esten, sostenidos por arnes.py, y se avisa."""
        aviso = None
        if self.peso > 0.5:
            self.congelar()
            self.rampa_peso(0.0)
            aviso = ("se devolvio el control de los brazos sin terminar: arnes.py los sostiene donde quedaron. "
                     "Si estan sobre la mesa: manipulacion.py --arnes --recuperar")
        self._vivo = False
        self._cerrar_socket()
        if aviso:
            raise RuntimeError(aviso)
