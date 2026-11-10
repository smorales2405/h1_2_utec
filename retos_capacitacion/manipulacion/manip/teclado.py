"""Tecla de parada en el terminal (ssh -t): ESPACIO lanza la salida segura.

Pone el terminal en modo cbreak (cada tecla llega sin Enter y sin eco; Ctrl+C sigue funcionando) y lo
lee en un hilo. Al pulsar la tecla llama a `al_pulsar()`; el lazo principal lo ve en su siguiente
vuelta (menos de 50 ms) y el supervisor sale de forma segura. No sustituye a L2+B del mando.

    teclado = Teclado(lambda: ...)
    teclado.abrir()          # despues de escribir MOVER (input() necesita el modo normal)
    try: ...
    finally: teclado.cerrar()   # devuelve el terminal a como estaba, salga como salga

A `al_teclear` no llegan las secuencias de escape del terminal (flechas, Inicio/Fin, F1-F12, AvPag...): la flecha
derecha es ESC [ C y su 'C' era la [c] de colgar en arnes.py (2026-11-11, el robot de pie pasó a COLGAR).
"""

import os
import select
import sys
import termios
import threading
import time
import tty


class FiltroEscape:
    """Quita de lo tecleado las secuencias de escape: CSI (ESC [ ... final 0x40-0x7E), SS3 (ESC O x), las F1-F5
    de la consola de Linux (ESC [ [ x) y Alt+tecla (ESC x). Una secuencia partida entre dos lecturas se sigue si
    la segunda llega antes de `junto_s`; pasado ese tiempo, un ESC suelto (la tecla Esc) ya no se come la siguiente."""

    def __init__(self, junto_s=0.1, reloj=time.monotonic):
        self.junto, self.reloj = float(junto_s), reloj
        self.estado, self.t = None, 0.0

    def __call__(self, texto):
        ahora = self.reloj()
        if self.estado is not None and ahora - self.t > self.junto:
            self.estado = None
        self.t = ahora
        sal = []
        for ch in texto:
            e = self.estado
            if e is None:
                if ch == "\x1b":
                    self.estado = "esc"
                else:
                    sal.append(ch)
            elif e == "esc":                     # ESC + otra (Alt+tecla): fuera las dos
                self.estado = {"[": "csi0", "O": "uno", "\x1b": "esc"}.get(ch)
            elif e == "csi0" and ch == "[":      # consola de Linux: F1-F5 son ESC [ [ A-E
                self.estado = "uno"
            elif e in ("csi0", "csi"):
                self.estado = None if "\x40" <= ch <= "\x7e" else "csi"
            else:                                # "uno": el ultimo caracter de la secuencia
                self.estado = None
        return "".join(sal)


class Teclado:
    def __init__(self, al_pulsar, tecla=b" ", fd=None, al_teclear=None):
        """al_teclear(caracter): ademas, cada tecla (herramientas/arnes.py)."""
        self.al_pulsar = al_pulsar
        self.al_teclear = al_teclear
        self.tecla = tecla
        try:
            self.fd = sys.stdin.fileno() if fd is None else fd
            self.activo = os.isatty(self.fd)      # sin terminal (lanzado sin ssh -t) no hay tecla
        except (OSError, ValueError, AttributeError):  # p. ej. bajo pytest: la entrada no es un fichero
            self.fd, self.activo = None, False
        self.pulsada = False
        self._viejo = None
        self._fin = threading.Event()
        self._hilo = None
        self._filtro = FiltroEscape()

    def abrir(self):
        if not self.activo:
            return self
        self._viejo = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self._hilo = threading.Thread(target=self._leer, daemon=True)
        self._hilo.start()
        return self

    def _leer(self):
        while not self._fin.is_set():
            listo, _, _ = select.select([self.fd], [], [], 0.1)
            if not listo:
                continue
            try:
                datos = os.read(self.fd, 64)
            except OSError:
                return
            if not datos:                    # se cerro el terminal
                return
            if self.tecla in datos and not self.pulsada:
                self.pulsada = True
                self.al_pulsar()
            if self.al_teclear is not None:
                for ch in self._filtro(datos.decode(errors="ignore")):
                    self.al_teclear(ch)

    def cerrar(self):
        self._fin.set()
        if self._hilo is not None:
            self._hilo.join(timeout=0.5)
        if self._viejo is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self._viejo)
            except (termios.error, OSError):     # el terminal ya no existe (SSH cortado)
                pass
            self._viejo = None

    def __enter__(self):
        return self.abrir()

    def __exit__(self, *exc):
        self.cerrar()
