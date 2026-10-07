#!/usr/bin/env python3
"""Gestos predefinidos de las manos Inspire RH56DFTP, y el orden en que se ejecutan.

Lo comparten el editor de poses de MuJoCo (h1_2_mujoco/) y el selector del robot
real (h1_2_robot_selector_manos.py), para que lo que se ve en la simulación sea lo
que hace la mano: mismos gestos, mismas fases y mismos tiempos.

Una mano se describe con sus 6 juntas ACTUADAS del URDF, en radianes, con las
claves de conversion_angle_set.CLAVES:

    little_1  ring_1  middle_1  index_1  thumb_1 (flexión)  thumb_swing (rotación)

q = 0 es la mano ABIERTA y q_max la CERRADA en las seis (ver conversion_angle_set).
La mano real se comanda en ANGLE_SET con conversion_angle_set.rad_a_angle_set.

Orden de ejecución. Solo el gesto 'cerrada' tiene un orden fijo, el de puno() en
code_cap/caja_cuadrado.py (si el pulgar gira con los dedos cerrados, choca con
ellos):

    1. abrir la flexión del pulgar (thumb_1 -> 0)
    2. llevar la rotación del pulgar a su sitio (thumb_swing -> la del gesto)
    3. cerrar los cuatro dedos
    4. flexionar el pulgar (thumb_1 -> la del gesto)

Ese orden solo hace falta si hay que girar el pulgar o cerrar los dedos. Si los dedos
ya están cerrados y la rotación no cambia (de 'cerrada' a 'cerrada', por ejemplo, en
pasos seguidos de una rutina), se va directo y el pulgar no se abre.

Cualquier otra postura va de una vez, en línea recta en q: su orden lo decide
quien diseña la rutina, y las colisiones se ven en el editor.
"""
from __future__ import annotations

from typing import Mapping

from conversion_angle_set import CLAVES, Q_MAX

LADOS = {"izq": "left", "der": "right"}
DEDOS = ("little_1", "ring_1", "middle_1", "index_1")
NOMBRE_DOF = {"little_1": "meñique", "ring_1": "anular", "middle_1": "medio", "index_1": "índice",
              "thumb_1": "pulgar flex.", "thumb_swing": "pulgar rot."}

GESTOS = {
    "abierta":       {"little_1": 0.0, "ring_1": 0.0, "middle_1": 0.0, "index_1": 0.0,
                      "thumb_1": 0.0, "thumb_swing": 0.0},
    "cerrada":       {"little_1": 1.62, "ring_1": 1.62, "middle_1": 1.62, "index_1": 1.62,
                      "thumb_1": 0.492, "thumb_swing": 0.0},
    "pulgar_arriba": {"little_1": 1.62, "ring_1": 1.62, "middle_1": 1.62, "index_1": 1.62,
                      "thumb_1": 0.0, "thumb_swing": 0.0},
    "senalar":       {"little_1": 1.62, "ring_1": 1.62, "middle_1": 1.62, "index_1": 0.0,
                      "thumb_1": 0.66, "thumb_swing": 0.61},
}
ALIAS = {"señalar": "senalar"}

TOL_GESTO = 0.01        # rad: una mano "es" un gesto si sus 6 DOF están a menos de esto
T_FASE_MIN = 0.6        # s: ninguna fase de la secuencia de 'cerrada' dura menos


def normalizar(q: Mapping[str, float], base: Mapping[str, float] | None = None) -> dict[str, float]:
    """Mano completa (6 claves) dentro de [0, q_max]. Las claves que falten salen de `base`
    (o 0, mano abierta); una clave desconocida es un error."""
    raros = set(q) - set(CLAVES)
    if raros:
        raise KeyError(f"DOF de mano desconocidos: {sorted(raros)} (válidos: {', '.join(CLAVES)})")
    base = base or GESTOS["abierta"]
    return {k: min(max(float(q.get(k, base[k])), 0.0), Q_MAX[i]) for i, k in enumerate(CLAVES)}


def gesto(nombre: str) -> dict[str, float]:
    nombre = ALIAS.get(nombre, nombre)
    if nombre not in GESTOS:
        raise KeyError(f"gesto desconocido '{nombre}' (gestos: {', '.join(GESTOS)})")
    return dict(GESTOS[nombre])


def nombre_gesto(q: Mapping[str, float]) -> str | None:
    """El gesto predefinido al que corresponde la mano, o None si es una postura propia."""
    for nombre, g in GESTOS.items():
        if all(abs(float(q[k]) - g[k]) <= TOL_GESTO for k in CLAVES):
            return nombre
    return None


def espejo(q: Mapping[str, float]) -> dict[str, float]:
    """La mano del otro lado con la misma postura. Las dos manos cierran con q positivo
    en sus seis DOF (el URDF las define así), de modo que se copia sin cambiar signos."""
    return {k: float(q[k]) for k in CLAVES}


def fases(q_ini: Mapping[str, float], q_fin: Mapping[str, float]) -> list[dict[str, float]]:
    """Objetivos sucesivos para ir de q_ini a q_fin. Una sola fase salvo para 'cerrada',
    que sigue el orden de puno(). Las fases sin cambio se omiten; [] si no hay nada que mover."""
    actual = {k: float(q_ini[k]) for k in CLAVES}
    fin = {k: float(q_fin[k]) for k in CLAVES}
    gira_pulgar = abs(actual["thumb_swing"] - fin["thumb_swing"]) > 1e-4
    cierran_dedos = any(abs(actual[k] - fin[k]) > 1e-4 for k in DEDOS)
    if nombre_gesto(fin) == "cerrada" and (gira_pulgar or cierran_dedos):
        objetivos = [
            {**actual, "thumb_1": 0.0},
            {**actual, "thumb_1": 0.0, "thumb_swing": fin["thumb_swing"]},
            {**actual, "thumb_1": 0.0, "thumb_swing": fin["thumb_swing"], **{k: fin[k] for k in DEDOS}},
            fin,
        ]
    else:
        objetivos = [fin]
    salida = []
    for obj in objetivos:
        previo = salida[-1] if salida else actual
        if any(abs(obj[k] - previo[k]) > 1e-4 for k in CLAVES):
            salida.append(obj)
    return salida


def duraciones(duracion: float, n_fases: int) -> list[float]:
    """Cuánto dura cada fase de un paso de `duracion` s. Una fase sola dura el paso entero
    (llega a la vez que el brazo); en la secuencia de 'cerrada' se reparte, con un mínimo."""
    if n_fases <= 1:
        return [max(float(duracion), 0.0)] * n_fases
    return [max(float(duracion) / n_fases, T_FASE_MIN)] * n_fases


def interpolar(q0: Mapping[str, float], q1: Mapping[str, float], s: float) -> dict[str, float]:
    s = min(max(s, 0.0), 1.0)
    return {k: float(q0[k]) + (float(q1[k]) - float(q0[k])) * s for k in CLAVES}
