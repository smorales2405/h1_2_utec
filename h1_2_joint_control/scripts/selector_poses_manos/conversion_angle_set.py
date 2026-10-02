#!/usr/bin/env python3
"""Convierte los ángulos de la mano Inspire RH56DFTP de radianes (URDF) a ANGLE_SET (0-1000), y al revés.

Para qué sirve. El URDF de las manos (`h1_2_inspire_description`) describe cada
DOF con su junta actuada en radianes, con q = 0 = mano ABIERTA y q_max = mano
CERRADA. La mano real se comanda con el registro ANGLE_SET, de 0 (cerrado) a
1000 (abierto). Este módulo traduce entre los dos.

Orden de los 6 DOF, el del registro ANGLE_SET y de `hand_modbus.DOF_NAMES`:

    DOF  nombre            junta actuada del URDF   clave        q_max (rad)
    0    meñique           *_little_1_joint         little_1     1.620
    1    anular            *_ring_1_joint           ring_1       1.620
    2    medio             *_middle_1_joint         middle_1     1.620
    3    índice            *_index_1_joint          index_1      1.620
    4    pulgar, flexión   *_thumb_1_joint          thumb_1      0.660
    5    pulgar, rotación  *_thumb_swing_joint      thumb_swing  1.190

Las juntas mimic (*_index_2, *_thumb_2, *_thumb_3...) no entran: las arrastra
la actuada. Si llegan en un diccionario (p. ej. un joint_states completo) se
ignoran.

**El mapeo NO es lineal.** Se midió sobre fotos de la mano real a ANGLE_SET =
0, 500 y 1000 (inspire_hand_interface/Caracterizacion/imagenes/angulos limite),
ajustando a cada foto el modelo renderizado en MuJoCo con una cámara virtual.
Con s = 1 - ANGLE_SET/1000:

    s                     0     0.5    1
    dedos (DOF 0-3)       0     1.05   1.620
    pulgar flexión (4)    0     0.38   0.660
    pulgar rotación (5)   0     0.61   1.190

A mitad de comando la proximal de los dedos ya lleva casi dos tercios (64-65 %)
de su recorrido,
así que la fórmula lineal ANGLE_SET = 1000·(1 - q/q_max) se equivoca unos 14°
de proximal hacia la mitad. La flexión del pulgar (58 %) y su rotación (51 %)
son casi lineales.

Entre esos puntos se interpola linealmente a tramos. Son tres puntos por DOF:
la forma de la curva ENTRE ellos no está medida. El punto medio de los dedos es
la media de índice, medio y anular (0.94 a 1.12 por separado); el meñique a 500
no se pudo medir.

Fuera de [0, q_max] los ángulos se recortan al extremo: q < 0 da 1000 (abierto)
y q > q_max da 0 (cerrado). Con `recortar=False` se lanza ValueError.

Uso como módulo:

    from conversion_angle_set import rad_a_angle_set, angle_set_a_rad
    rad_a_angle_set([0, 0, 0, 0, 0, 0])            # [1000]*6, mano abierta
    rad_a_angle_set([1.62]*4 + [0.66, 1.19])       # [0]*6, mano cerrada
    rad_a_angle_set({'index_1': 1.2})              # el resto queda abierto
    rad_a_angle_set({'left_index_1_joint': 1.2, 'left_index_2_joint': 1.02})
    angle_set_a_rad([1000, 1000, 500, 500, 0, 0])  # 6 ángulos en rad

Uso desde la terminal:

    python3 conversion_angle_set.py 1.62 1.62 1.62 1.62 0.66 1.19
    python3 conversion_angle_set.py --grados 90 90 90 90 30 60
    python3 conversion_angle_set.py --urdf right_index_1_joint=1.2 right_thumb_swing_joint=0.6
    python3 conversion_angle_set.py --inversa 1000 1000 500 500 0 0
    python3 conversion_angle_set.py --tabla
    python3 conversion_angle_set.py --test
"""
from __future__ import annotations

import argparse
import math
import random
import re
import sys
from pathlib import Path
from typing import Mapping, Sequence, Union

NDOF = 6
NOMBRES = ["Meñique", "Anular", "Medio", "Índice", "Pulgar (flex.)", "Pulgar (rot.)"]
CLAVES = ["little_1", "ring_1", "middle_1", "index_1", "thumb_1", "thumb_swing"]
MIMIC = {"little_2", "ring_2", "middle_2", "index_2", "thumb_2", "thumb_3"}

# Calibración medida: q (rad) en s = 1 - ANGLE_SET/1000 = 0, 0.5 y 1.
S = (0.0, 0.5, 1.0)
Q = (
    (0.0, 1.05, 1.620),   # 0 meñique   (punto medio: media de los otros tres dedos)
    (0.0, 1.05, 1.620),   # 1 anular
    (0.0, 1.05, 1.620),   # 2 medio
    (0.0, 1.05, 1.620),   # 3 índice
    (0.0, 0.38, 0.660),   # 4 pulgar, flexión
    (0.0, 0.61, 1.190),   # 5 pulgar, rotación
)
Q_MAX = tuple(q[-1] for q in Q)

Angulos = Union[Sequence[float], Mapping[str, float]]


def _interp(x: float, xp: Sequence[float], fp: Sequence[float]) -> float:
    """Interpolación lineal a tramos, con xp creciente; fuera del rango, el extremo."""
    if x <= xp[0]:
        return fp[0]
    if x >= xp[-1]:
        return fp[-1]
    for i in range(len(xp) - 1):
        if x <= xp[i + 1]:
            t = (x - xp[i]) / (xp[i + 1] - xp[i])
            return fp[i] + t * (fp[i + 1] - fp[i])
    return fp[-1]


def _clave(nombre: str) -> str | None:
    """'left_index_1_joint' / 'index_1' -> 'index_1'; mimic -> None; desconocida -> KeyError."""
    k = re.sub(r"^(left|right)_", "", nombre)
    k = re.sub(r"_joint$", "", k)
    if k in CLAVES:
        return k
    if k in MIMIC:
        return None
    raise KeyError(f"junta desconocida: {nombre!r} (actuadas válidas: {', '.join(CLAVES)})")


def _vector(q: Angulos) -> list[float]:
    """Lista de 6 rad en orden DOF a partir de una secuencia o de un diccionario."""
    if isinstance(q, Mapping):
        v = [0.0] * NDOF
        for nombre, valor in q.items():
            k = _clave(nombre)
            if k is not None:
                v[CLAVES.index(k)] = float(valor)
        return v
    v = [float(x) for x in q]
    if len(v) != NDOF:
        raise ValueError(f"se esperaban {NDOF} ángulos (orden DOF 0-5), llegaron {len(v)}")
    return v


def rad_a_angle_set(q: Angulos, recortar: bool = True) -> list[int]:
    """Ángulos del URDF (rad) -> ANGLE_SET (0-1000), en el orden del registro (DOF 0-5).

    `q` es una secuencia de 6 valores en orden DOF, o un diccionario con claves
    'index_1' o nombres de junta del URDF ('left_index_1_joint'); las juntas que
    falten quedan abiertas y las mimic se ignoran.
    """
    salida = []
    for d, qd in enumerate(_vector(q)):
        if not math.isfinite(qd):
            raise ValueError(f"DOF {d} ({NOMBRES[d]}): ángulo no finito {qd!r}")
        if not 0.0 <= qd <= Q_MAX[d]:
            if not recortar:
                raise ValueError(f"DOF {d} ({NOMBRES[d]}): {qd:.4f} rad fuera de [0, {Q_MAX[d]}]")
            qd = min(max(qd, 0.0), Q_MAX[d])
        s = _interp(qd, Q[d], S)
        salida.append(int(round(1000.0 * (1.0 - s))))
    return salida


def angle_set_a_rad(angle_set: Sequence[float]) -> list[float]:
    """ANGLE_SET (0-1000) en orden DOF -> 6 ángulos del URDF en rad. Sirve igual para ANGLE_ACT."""
    a = [float(x) for x in angle_set]
    if len(a) != NDOF:
        raise ValueError(f"se esperaban {NDOF} valores de ANGLE_SET, llegaron {len(a)}")
    return [_interp(1.0 - min(max(ad, 0.0), 1000.0) / 1000.0, S, Q[d]) for d, ad in enumerate(a)]


def recortados(q: Angulos) -> list[int]:
    """DOF cuyo ángulo cae fuera de [0, q_max] y se recortaría."""
    return [d for d, qd in enumerate(_vector(q)) if not 0.0 <= qd <= Q_MAX[d]]


# ----------------------------------------------------------------------------- terminal

def _tabla() -> None:
    print("Calibración (s = 1 - ANGLE_SET/1000), q en rad:\n")
    print(f"  {'DOF':<4}{'nombre':<17}{'s=0':>7}{'s=0.5':>8}{'s=1':>8}   lineal a 500   medido a 500")
    for d in range(NDOF):
        lineal = 0.5 * Q_MAX[d]
        print(f"  {d:<4}{NOMBRES[d]:<17}{Q[d][0]:>7.3f}{Q[d][1]:>8.3f}{Q[d][2]:>8.3f}"
              f"   {lineal:>9.3f}      {Q[d][1]:>9.3f}  ({100 * Q[d][1] / Q_MAX[d]:.0f} % del recorrido)")


def _imprimir(q: list[float], a: list[int], fuera: list[int]) -> None:
    print(f"  {'DOF':<4}{'nombre':<17}{'rad':>8}{'grados':>9}{'ANGLE_SET':>11}")
    for d in range(NDOF):
        marca = "  <- recortado" if d in fuera else ""
        print(f"  {d:<4}{NOMBRES[d]:<17}{q[d]:>8.4f}{math.degrees(q[d]):>9.2f}{a[d]:>11d}{marca}")
    print(f"\nANGLE_SET = {a}")


def _leer_q_max_urdf() -> dict[str, float] | None:
    """q_max de las juntas actuadas según el xacro de la mano izquierda, si está a mano."""
    xacro = (Path(__file__).resolve().parents[3] / "ros_h1_2_ws" / "src" /
             "h1_2_inspire_description" / "urdf" / "inspire_hand_left.urdf.xacro")
    if not xacro.is_file():
        return None
    s = xacro.read_text(encoding="utf-8")
    out = {}
    for k in CLAVES:
        m = re.search(r'<joint name="left_%s_joint".*?upper="([^"]+)"' % k, s, re.S)
        if m:
            out[k] = float(m.group(1))
    return out


def _test() -> int:
    fallos = []

    def check(cond: bool, msg: str) -> None:
        print(("  ok    " if cond else "  FALLO ") + msg)
        if not cond:
            fallos.append(msg)

    check(rad_a_angle_set([0.0] * 6) == [1000] * 6, "q = 0 en los 6 DOF da 1000 (abierta)")
    check(rad_a_angle_set(list(Q_MAX)) == [0] * 6, "q = q_max en los 6 DOF da 0 (cerrada)")
    check(rad_a_angle_set([q[1] for q in Q]) == [500] * 6, "los puntos medidos a mitad dan 500")
    check(rad_a_angle_set([-1.0] * 6) == [1000] * 6 and rad_a_angle_set([9.0] * 6) == [0] * 6,
          "fuera de rango se recorta al extremo")
    try:
        rad_a_angle_set([9.0] * 6, recortar=False)
        check(False, "recortar=False lanza ValueError fuera de rango")
    except ValueError:
        check(True, "recortar=False lanza ValueError fuera de rango")

    mono = True
    for d in range(NDOF):
        prev = 1001
        for i in range(201):
            v = rad_a_angle_set([Q_MAX[d] * i / 200 if k == d else 0.0 for k in range(NDOF)])[d]
            mono &= v <= prev
            prev = v
    check(mono, "ANGLE_SET no crece al cerrar, en los 6 DOF")

    rng = random.Random(0)
    peor = 0.0
    for _ in range(2000):
        q = [rng.uniform(0.0, Q_MAX[d]) for d in range(NDOF)]
        peor = max(peor, max(abs(x - y) for x, y in zip(q, angle_set_a_rad(rad_a_angle_set(q)))))
    check(peor <= 0.0011, f"ida y vuelta rad -> ANGLE_SET -> rad: error máx {peor * 1000:.2f} mrad "
                          f"(cuantización de 1 unidad de ANGLE_SET)")
    check(all(abs(x - y) < 1e-12 for x, y in zip(angle_set_a_rad(rad_a_angle_set(list(Q_MAX))), Q_MAX)),
          "ANGLE_SET 0 vuelve exactamente a q_max")

    a = rad_a_angle_set({"left_index_1_joint": 1.05, "left_index_2_joint": 0.9, "right_thumb_swing_joint": 0.61})
    check(a == [1000, 1000, 1000, 500, 1000, 500], "diccionario con nombres del URDF: mimic ignoradas, faltantes abiertas")
    try:
        rad_a_angle_set({"left_indice_1_joint": 1.0})
        check(False, "una junta mal escrita lanza KeyError")
    except KeyError:
        check(True, "una junta mal escrita lanza KeyError")

    urdf = _leer_q_max_urdf()
    if urdf is None:
        print("  --    no encontré el xacro de la mano: no compruebo q_max contra el URDF")
    else:
        dif = {k: (urdf.get(k), Q_MAX[CLAVES.index(k)]) for k in CLAVES
               if urdf.get(k) is None or abs(urdf[k] - Q_MAX[CLAVES.index(k)]) > 1e-6}
        check(not dif, "q_max de la tabla coincide con los límites del URDF" +
              ("" if not dif else f": difieren {dif}"))

    print(f"\n{'TODO OK' if not fallos else f'{len(fallos)} FALLO(S)'}")
    return 0 if not fallos else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("valores", nargs="*", type=float,
                    help="6 valores en orden DOF 0-5 (rad, o grados con --grados, o ANGLE_SET con --inversa)")
    ap.add_argument("--grados", action="store_true", help="los valores vienen en grados")
    ap.add_argument("--inversa", action="store_true", help="convertir ANGLE_SET -> rad")
    ap.add_argument("--urdf", nargs="+", metavar="JUNTA=RAD",
                    help="ángulos por nombre de junta del URDF; las que falten quedan abiertas")
    ap.add_argument("--tabla", action="store_true", help="muestra la calibración y termina")
    ap.add_argument("--test", action="store_true", help="autoprueba, sin hardware")
    a = ap.parse_args()

    if a.test:
        return _test()
    if a.tabla:
        _tabla()
        return 0

    if a.inversa:
        if len(a.valores) != NDOF:
            ap.error(f"--inversa necesita {NDOF} valores de ANGLE_SET")
        q = angle_set_a_rad(a.valores)
        _imprimir(q, [int(round(x)) for x in a.valores], [])
        print(f"q (rad)   = [{', '.join(f'{x:.4f}' for x in q)}]")
        return 0

    if a.urdf:
        try:
            datos = {k: float(v) for k, v in (item.split("=", 1) for item in a.urdf)}
        except ValueError:
            ap.error("--urdf espera pares JUNTA=VALOR")
        if a.grados:
            datos = {k: math.radians(v) for k, v in datos.items()}
        q = _vector(datos)
    else:
        if len(a.valores) != NDOF:
            ap.error(f"hacen falta {NDOF} valores en orden DOF 0-5 (o usa --urdf / --tabla / --test)")
        q = [math.radians(x) for x in a.valores] if a.grados else list(a.valores)

    fuera = recortados(q)
    _imprimir(q, rad_a_angle_set(q), fuera)
    if fuera:
        sys.stdout.flush()
        print(f"\nAviso: {len(fuera)} DOF fuera de [0, q_max]; se recortaron al extremo.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
