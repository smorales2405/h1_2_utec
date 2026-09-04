"""CRC de Unitree para mensajes `unitree_hg/LowCmd`.

El robot descarta todo `rt/lowcmd` cuyo CRC no cuadre. No es el CRC32 estándar
de `zlib`: Unitree usa uno propio (polinomio 0x04C11DB7) sobre el mensaje
serializado con el *layout* exacto del struct C++, relleno incluido.

El canal `rt/arm_sdk` no lo comprueba —el ejemplo oficial
`h1_2_arm_sdk_dds_example.cpp` ni siquiera lo calcula—, pero calcularlo bien no
estorba y así el mismo código sirve para los dos canales.

RENDIMIENTO. La versión ingenua recorre bit a bit: 250 palabras × 32 bits =
8000 vueltas de bucle Python por mensaje. A 250 Hz eso son 2 millones de
iteraciones por segundo y el lazo de control no llega: medido, se quedaba en
185 Hz en vez de 250. Aquí se tabula.

La clave es que el paso del registro es **lineal sobre GF(2)**:

    S(c) = (c << 1) ^ (P si el bit 31 de c está a 1)

cumple S(a ^ b) = S(a) ^ S(b). Y procesar una palabra de 32 bits equivale a

    c' = S³²(c) ^ D(w)          con D(w) = Σ bit_j(w) · S^j(P)

Las dos partes son lineales, así que se tabulan por bytes: cuatro tablas de
256 entradas para S³² y otras cuatro para D. Cada palabra sale con ocho
consultas en vez de 32 vueltas de bucle. Se conserva `crc32_reference()` y hay
una prueba que compara las dos implementaciones sobre datos aleatorios.
"""
from __future__ import annotations

import struct

_MASK = 0xFFFFFFFF
_POLY = 0x04C11DB7

# Cada MotorCmd ocupa 28 bytes en el struct C++:
#   [0]     uint8   mode
#   [1..3]  relleno
#   [4]     float32 q
#   [8]     float32 dq
#   [12]    float32 tau     <- ¡tau va ANTES de kp/kd!
#   [16]    float32 kp
#   [20]    float32 kd
#   [24]    uint32  reserve
#
# Y el LowCmd completo, 1004 bytes:
#   [0]         uint8 mode_pr
#   [1]         uint8 mode_machine
#   [2..3]      relleno
#   [4..983]    MotorCmd[35]        (35 x 28 = 980)
#   [984..999]  uint32[4] reserve
#   [1000..3]   uint32 crc          <- fuera del cálculo
NUM_MOTOR_SLOTS = 35
CRC_BYTES = 1000
CRC_WORDS = CRC_BYTES // 4

# Un único formato para todo el mensaje: una sola llamada a struct.pack en vez
# de 35. `x` es un byte de relleno.
_FMT = "<BB2x" + "B3xfffffI" * NUM_MOTOR_SLOTS + "4I"
_PACK = struct.Struct(_FMT).pack
_UNPACK_WORDS = struct.Struct(f"<{CRC_WORDS}I").unpack_from


def _S(c: int) -> int:
    """Un paso del registro, sin bit de datos."""
    return (((c << 1) ^ _POLY) & _MASK) if (c & 0x80000000) else ((c << 1) & _MASK)


def _build_tables():
    s32, dat = [], []
    for k in range(4):                       # k=0 -> byte más significativo
        shift = 8 * (3 - k)
        t_s = [0] * 256
        t_d = [0] * 256
        for b in range(256):
            x = (b << shift) & _MASK
            # S^32 aplicado a este byte colocado en su posición
            c = x
            for _ in range(32):
                c = _S(c)
            t_s[b] = c
            # Aportación de los bits de datos. El bit de posición j entra en
            # el registro en la vuelta 31-j y todavía le quedan j pasos por
            # delante, así que contribuye S^j(P). Ojo con el sentido: la
            # primera versión de esto usaba S^(31-j) y daba otro CRC.
            d = 0
            p = _POLY
            for j in range(32):
                if x & (1 << j):
                    d ^= p
                p = _S(p)
            t_d[b] = d
        s32.append(t_s)
        dat.append(t_d)
    return s32, dat


_S32, _DAT = _build_tables()
_S32_0, _S32_1, _S32_2, _S32_3 = _S32
_DAT_0, _DAT_1, _DAT_2, _DAT_3 = _DAT


def crc32_words(words) -> int:
    """CRC de Unitree sobre una secuencia de palabras uint32. Versión tabulada."""
    c = 0xFFFFFFFF
    for w in words:
        c = (_S32_0[(c >> 24) & 0xFF] ^ _S32_1[(c >> 16) & 0xFF]
             ^ _S32_2[(c >> 8) & 0xFF] ^ _S32_3[c & 0xFF]
             ^ _DAT_0[(w >> 24) & 0xFF] ^ _DAT_1[(w >> 16) & 0xFF]
             ^ _DAT_2[(w >> 8) & 0xFF] ^ _DAT_3[w & 0xFF])
    return c


def crc32_reference(words) -> int:
    """La versión literal del algoritmo, bit a bit. Lenta, pero es la verdad
    contra la que se contrasta la tabulada."""
    reg = 0xFFFFFFFF
    for word in words:
        xbit = 1 << 31
        data = word & _MASK
        for _ in range(32):
            if reg & 0x80000000:
                reg = ((reg << 1) ^ _POLY) & _MASK
            else:
                reg = (reg << 1) & _MASK
            if data & xbit:
                reg ^= _POLY
            xbit >>= 1
    return reg


def serialize(msg) -> bytes:
    """Serializa un `unitree_hg/LowCmd` como lo hace el struct C++."""
    vals = [int(msg.mode_pr), int(msg.mode_machine)]
    n = len(msg.motor_cmd)
    for i in range(NUM_MOTOR_SLOTS):
        if i < n:
            c = msg.motor_cmd[i]
            vals += [int(c.mode), float(c.q), float(c.dq),
                     float(c.tau), float(c.kp), float(c.kd), 0]
        else:
            vals += [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0]
    vals += [0, 0, 0, 0]
    buf = _PACK(*vals)
    if len(buf) != CRC_BYTES:
        raise AssertionError(f"serialización de {len(buf)} B, esperaba {CRC_BYTES}")
    return buf


def set_crc(msg) -> int:
    """Calcula y asigna `msg.crc`. Llamar SIEMPRE justo antes de publicar."""
    msg.crc = crc32_words(_UNPACK_WORDS(serialize(msg)))
    return msg.crc
