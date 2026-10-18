"""Lectura del mando a partir de rt/lowstate.wireless_remote[40].

Formato (unitree_sdk2, common/unitree_joystick.hpp): 2 bytes de cabecera, 2 bytes de botones
(uint16, little endian, bit 0 = R1 ... bit 15 = left) y cinco float32: lx, rx, ry, L2, ly.
"""

import struct

from .mensajes import Mando

BOTONES = ("R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
           "A", "B", "X", "Y", "up", "right", "down", "left")


def decodificar(wireless_remote) -> Mando:
    datos = bytes(bytearray(int(b) & 0xFF for b in wireless_remote))
    botones, = struct.unpack_from("<H", datos, 2)
    lx, rx, ry, _l2, ly = struct.unpack_from("<5f", datos, 4)
    return Mando(botones=botones, lx=lx, ly=ly, rx=rx, ry=ry)


def mascara(wireless_remote) -> int:
    """Solo los botones (uint16), sin decodificar los ejes: para el hilo de control a 500 Hz."""
    return (int(wireless_remote[2]) & 0xFF) | ((int(wireless_remote[3]) & 0xFF) << 8)


L2B = (1 << BOTONES.index("L2")) | (1 << BOTONES.index("B"))


def codificar(m: Mando) -> bytes:
    """Inverso de decodificar(), para las pruebas."""
    datos = bytearray(40)
    struct.pack_into("<H", datos, 2, m.botones)
    struct.pack_into("<5f", datos, 4, m.lx, m.rx, m.ry, 0.0, m.ly)
    return bytes(datos)


def pulsados(m: Mando):
    return [b for i, b in enumerate(BOTONES) if m.botones & (1 << i)]
