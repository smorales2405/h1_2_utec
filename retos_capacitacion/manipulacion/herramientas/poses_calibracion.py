#!/usr/bin/env python3
"""Genera las poses de calibracion (6.1.3) con la IK y el gemelo, y las escribe en config/poses_calibracion.yaml.

Cada pose lleva el marcador del dorso por encima de la mesa (12-30 cm) mirando a la camara, con la normal
inclinada distinto en cada una (si todas miran igual, la rotacion de la camara queda mal determinada). Se
exige: IK valida dentro de los limites, nada a menos de la holgura en el gemelo (mesa, asas, bandejas y el
propio robot, con el otro brazo en reposo) y el marcador entero dentro de la imagen de color. Despues se
ordenan para que el brazo se mueva poco de una a la siguiente, empezando y acabando cerca del reposo.

    python3 herramientas/poses_calibracion.py --n 16          # en la PC
Lo recorre herramientas/grabar_calibracion.py en el robot (hito 2), con la camara fuera del lazo.
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import calibracion as cb  # noqa: E402
from manip import config  # noqa: E402
from manip.cinematica import Banda, Cinematica, TareaIK  # noqa: E402
from manip.geometria import aplicar, inv  # noqa: E402
from manip.mensajes import LADOS  # noqa: E402
from sim.colisiones import Colisiones  # noqa: E402
from sim.escena import INTR_COLOR, Estado, construir  # noqa: E402
from manip.geometria import Intrinsecos  # noqa: E402

RUTA = os.path.join(config.CONFIG, "poses_calibracion.yaml")


def candidatas(lado, cin, cal, plano_d, rng, n_intentos):
    """Objetivos al azar: centro del marcador sobre la mesa y normal hacia la camara con una inclinacion."""
    cam = cal.T_torso_color[:3, 3]
    s = 1 if lado == "izq" else -1
    for _ in range(n_intentos):
        p = np.array([rng.uniform(0.36, 0.56), s * rng.uniform(-0.06, 0.30), -plano_d + rng.uniform(0.12, 0.30)])
        v = cam - p
        v /= np.linalg.norm(v)
        # inclinar la normal hasta 30 grados en una direccion al azar
        a = rng.normal(size=3)
        a -= (a @ v) * v
        a /= np.linalg.norm(a)
        th = math.radians(rng.uniform(5, 30))
        nrm = math.cos(th) * v + math.sin(th) * a
        yield p, nrm


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n", type=int, default=16, help="poses por mano")
    ap.add_argument("--intentos", type=int, default=600)
    ap.add_argument("--semilla", type=int, default=1)
    ap.add_argument("--max-desde-reposo", type=float, default=1.75, help="rad por junta desde el reposo")
    a = ap.parse_args()
    cin = Cinematica.desde_config(cfg)
    try:
        cal = cb.cargar()
    except FileNotFoundError:
        cal = cb.desde_urdf(cin, np.array([[1, 0, 0, 0.015], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]]))
    mon = config.cargar_montaje()
    m, info = construir(montaje=mon, cal=cal, con_camaras=False)
    est = Estado(m, info)
    col = Colisiones(m, est, cfg["planificacion"]["holgura_m"])
    ic = Intrinsecos.de_meta(INTR_COLOR)
    lado_m = cfg["calibracion"]["marcador_lado_m"]
    K = cb.esquinas_marcador(lado_m * 1.4)          # con el margen blanco, todo dentro de la imagen
    rng = np.random.default_rng(a.semilla)
    reposo = {l: np.array(info["postura"][13:20] if l == "izq" else info["postura"][20:27]) for l in LADOS}
    salida = {}
    for lado in LADOS:
        Td = cal.T_dorso[lado]
        buenas = []
        for p, nrm in candidatas(lado, cin, cal, mon["mesa"]["plano"]["d"], rng, a.intentos):
            t = TareaIK(posicion=p, punto=Td[:3, 3], bandas=[Banda(Td[:3, 2], nrm, math.cos(math.radians(8)), 1.0)],
                        tol_pos=0.005)
            r = cin.ik(lado, t, q0=reposo[lado])
            if not r.ok or np.abs(r.q - reposo[lado]).max() > a.max_desde_reposo:
                continue                # otra rama (p. ej. la muneca girada media vuelta): saltos enormes
            M = cin.fk(lado, r.q, "mano") @ Td
            uv = ic.proyectar(aplicar(inv(cal.T_torso_color) @ M, K))
            if np.isnan(uv).any() or uv[:, 0].min() < 15 or uv[:, 0].max() > ic.ancho - 15 \
                    or uv[:, 1].min() < 15 or uv[:, 1].max() > ic.alto - 15:
                continue
            est.poner_brazo(lado, r.q)
            libre = col.libre()
            est.poner_brazo(lado, reposo[lado])
            if not libre:
                continue
            buenas.append((r.q, M[:3, 3], M[:3, 2]))
        if len(buenas) < a.n:
            sys.exit(f"{lado}: solo {len(buenas)} poses validas de {a.intentos} intentos")
        # las mas distintas entre si (posicion y normal), por muestreo del mas lejano
        X = np.array([np.r_[b[1] / 0.1, b[2] / 0.3] for b in buenas])
        elegidas = [int(np.argmin(np.linalg.norm(np.array([b[0] for b in buenas]) - reposo[lado], axis=1)))]
        while len(elegidas) < a.n:
            dmin = np.min(np.linalg.norm(X[:, None] - X[elegidas][None], axis=2), axis=1)
            elegidas.append(int(np.argmax(dmin)))
        # orden: el vecino mas cercano en articulares, desde el reposo
        resto = list(elegidas)
        orden, q_ant = [], reposo[lado]
        while resto:
            k = min(resto, key=lambda i: np.abs(buenas[i][0] - q_ant).max())
            orden.append(k)
            q_ant = buenas[k][0]
            resto.remove(k)
        salida[lado] = [[round(float(v), 4) for v in buenas[k][0]] for k in orden]
        saltos = [np.abs(np.array(salida[lado][i]) - np.array(salida[lado][i - 1])).max()
                  for i in range(1, len(orden))]
        alt = [buenas[k][1][2] + mon["mesa"]["plano"]["d"] for k in orden]
        print(f"{lado}: {len(buenas)} validas de {a.intentos}; elegidas {a.n}; salto maximo entre poses "
              f"{math.degrees(max(saltos)):.0f} grados; marcador a {min(alt) * 100:.0f}-{max(alt) * 100:.0f} cm de la mesa")
    config.guardar_yaml(RUTA, {"generadas": "herramientas/poses_calibracion.py", "semilla": a.semilla,
                               "calibracion_usada": f"{cal.origen} {cal.fecha}", "poses": salida},
                        "Poses de calibracion (q de 7 juntas por mano, en el orden de arm_sdk). Las recorre\n"
                        "herramientas/grabar_calibracion.py. Generadas con el gemelo: sin colisiones y con el\n"
                        "marcador del dorso dentro de la imagen de color.")
    print(f"-> {RUTA}")


if __name__ == "__main__":
    main()
