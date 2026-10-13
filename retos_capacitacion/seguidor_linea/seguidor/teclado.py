"""Tecla de parada en el terminal del seguidor (ssh -t): ESPACIO para el robot.

Pone el terminal en modo cbreak (cada tecla llega sin Enter y sin eco; Ctrl+C sigue funcionando) y lo
lee en un hilo. Al pulsar la tecla llama a `al_pulsar()`; el lazo principal lo ve en su siguiente
vuelta (menos de 50 ms) y el supervisor manda velocidad 0. No sustituye a L2+B del mando.

    teclado = Teclado(lambda: ...)
    teclado.abrir()          # despues de escribir SEGUIR (input() necesita el modo normal)
    try: ...
    finally: teclado.cerrar()   # devuelve el terminal a como estaba, salga como salga
"""

import os
import select
import sys
import termios
import threading
import tty


class Teclado:
    def __init__(self, al_pulsar, tecla=b" ", fd=None):
        self.al_pulsar = al_pulsar
        self.tecla = tecla
        self.fd = sys.stdin.fileno() if fd is None else fd
        self.activo = os.isatty(self.fd)      # sin terminal (lanzado sin ssh -t) no hay tecla
        self.pulsada = False
        self._viejo = None
        self._fin = threading.Event()
        self._hilo = None

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

    def cerrar(self):
        self._fin.set()
        if self._hilo is not None:
            self._hilo.join(timeout=0.5)
        if self._viejo is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._viejo)
            self._viejo = None

    def __enter__(self):
        return self.abrir()

    def __exit__(self, *exc):
        self.cerrar()
