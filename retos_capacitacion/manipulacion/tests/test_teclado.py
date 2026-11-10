"""Teclas: las secuencias de escape del terminal no llegan como letras (2026-11-11: la flecha derecha, ESC [ C,
llego a arnes.py como [c] y el robot de pie paso a COLGAR). Y los eventos van al disco al escribirlos."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from manip.registro import CsvSimple
from manip.teclado import FiltroEscape


class Reloj:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_flechas_y_teclas_de_funcion_no_son_letras():
    f = FiltroEscape(reloj=Reloj())
    secuencias = ["\x1b[A", "\x1b[B", "\x1b[C", "\x1b[D",        # flechas
                  "\x1bOA", "\x1bOC",                            # flechas en modo aplicacion
                  "\x1b[H", "\x1b[F", "\x1bOH", "\x1bOF",        # Inicio, Fin
                  "\x1b[1~", "\x1b[4~", "\x1b[5~", "\x1b[6~", "\x1b[3~", "\x1b[2~",
                  "\x1bOP", "\x1bOS", "\x1b[15~", "\x1b[24~",    # F1, F4, F5, F12
                  "\x1b[1;5C", "\x1b[1;2D",                      # Ctrl+flecha, Mayus+flecha
                  "\x1b[[A", "\x1b[[E",                          # F1, F5 en la consola de Linux
                  "\x1bc", "\x1bf"]                              # Alt+c, Alt+f
    for s in secuencias:
        assert f(s) == "", repr(s)
    assert f("".join(secuencias)) == ""


def test_las_teclas_normales_pasan():
    f = FiltroEscape(reloj=Reloj())
    assert f("c") == "c"
    assert f("F") == "F"                       # Bloq Mayus: arnes.py la pasa a minuscula
    assert f("n e m [ ]") == "n e m [ ]"
    assert f("a\x1b[Cb\x1bOFc") == "abc"       # letras alrededor de flechas en la misma lectura


def test_secuencia_partida_entre_lecturas():
    r = Reloj()
    f = FiltroEscape(junto_s=0.1, reloj=r)
    assert f("\x1b") == ""
    r.t += 0.01
    assert f("[") == ""
    r.t += 0.01
    assert f("C") == ""
    assert f("\x1b[1;") == ""
    r.t += 0.02
    assert f("5Cx") == "x"


def test_esc_suelto_no_se_come_la_tecla_siguiente():
    r = Reloj()
    f = FiltroEscape(junto_s=0.1, reloj=r)
    assert f("\x1b") == ""                     # la tecla Esc
    r.t += 1.0
    assert f("c") == "c"


def test_eventos_en_el_disco_al_momento(tmp_path):
    ruta = tmp_path / "eventos.csv"
    ev = CsvSimple(str(ruta), ["t", "fase", "texto"], al_momento=True)
    assert ruta.read_text().splitlines() == ["t,fase,texto"]
    ev.escribir({"t": 1.5, "fase": "DE_PIE", "texto": "tecla 'c'"})
    assert ruta.read_text().splitlines()[1] == "1.5,DE_PIE,tecla 'c'"   # sin cerrar
    ev.cerrar()
