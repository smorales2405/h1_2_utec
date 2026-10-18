#!/usr/bin/env python3
"""Genera las poses de calibracion (6.1.3) para el modo arnes con la IK y el gemelo, y las escribe en
config/poses_calibracion.yaml.

Cada pose lleva el marcador del dorso por encima de la mesa (12-30 cm) mirando a la camara, con la normal
inclinada distinto en cada una (si todas miran igual, la rotacion de la camara queda mal determinada). El robot,
de pie en el arnes (postura_pie_deg de manipulacion.yaml) delante del montaje medido; cada brazo sale de 0 y
vuelve a 0 con el otro quieto en 0. Se exige: IK valida dentro de los limites; nada a menos de la holgura en el
gemelo (mesa, asas, bandejas y el propio robot) en la pose y en el camino desde la anterior (recta en
articulares: la minima sacudida de manip/cuerpo.py no se sale de ella), tambien al salir de 0 y al volver; y que
el detector lea el marcador en la imagen de color del gemelo (con los brazos a 0 la otra mano puede taparlo).
Se ordenan para que el brazo se mueva poco de una a la siguiente.

    python3 herramientas/poses_calibracion.py --n 16          # en la PC
Las recorre herramientas/arnes.py con [k] (de pie, despues de [n]) mientras herramientas/grabar_calibracion.py
guarda una muestra en cada una. Dependen de donde esta la mesa: arnes.py no las recorre si el montaje se ha
movido mas de 1 cm o 1 grado desde que se generaron.
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
from sim.camara_virtual import CamaraVirtual  # noqa: E402
from sim.colisiones import Colisiones  # noqa: E402
from sim.escena import INTR_COLOR, Estado, construir, postura_arnes  # noqa: E402
from manip.geometria import Intrinsecos  # noqa: E402

RUTA = os.path.join(config.CONFIG, "poses_calibracion.yaml")
PASO_CAMINO = math.radians(3)       # muestreo del camino entre poses al mirar las colisiones


def pose_mesa(montaje):
    """(x0, y, yaw en grados) de la mesa medida, para saber si se ha movido."""
    T = np.array(montaje["mesa"]["T_torso_mesa"], float)
    return float(T[0, 3]), float(T[1, 3]), math.degrees(math.atan2(T[1, 0], T[0, 0]))


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


def camino_libre(est, col, lado, qa, qb, q_vuelta):
    """Sin acercamientos en la recta articular de qa a qb (sin contar qa); deja el brazo en q_vuelta."""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    n = max(2, int(math.ceil(np.abs(qb - qa).max() / PASO_CAMINO)))
    libre = True
    for s in np.linspace(0.0, 1.0, n + 1)[1:]:
        est.poner_brazo(lado, qa + s * (qb - qa))
        if not col.libre():
            libre = False
            break
    est.poner_brazo(lado, q_vuelta)
    return libre


def ordenar(qs, inicio, conecta, n):
    """Hasta n poses de qs en un recorrido inicio -> ... -> inicio: cada vez la mas cercana (en articulares) a
    la que se llega sin choques, y la ultima con vuelta libre a inicio. Devuelve indices."""
    orden, q_ant, resto = [], inicio, list(range(len(qs)))
    while resto and len(orden) < n:
        for k in sorted(resto, key=lambda i: np.abs(qs[i] - q_ant).max()):
            if conecta(q_ant, qs[k]):
                orden.append(k)
                q_ant = qs[k]
                resto.remove(k)
                break
        else:
            break
    while orden and not conecta(qs[orden[-1]], inicio):
        orden.pop()
    return orden


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n", type=int, default=16, help="poses por mano")
    ap.add_argument("--intentos", type=int, default=900)
    ap.add_argument("--semilla", type=int, default=1)
    ap.add_argument("--max-desde-cero", type=float, default=1.75, help="rad por junta desde los brazos a 0")
    a = ap.parse_args()
    cin = Cinematica.desde_config(cfg)
    try:
        cal = cb.cargar()
    except FileNotFoundError:
        cal = cb.desde_urdf(cin, np.array([[1, 0, 0, 0.015], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]]))
    mon = config.cargar_montaje()
    postura = postura_arnes(cfg)
    m, info = construir(montaje=mon, cal=cal, postura=list(postura))
    est = Estado(m, info)
    est.adelante()
    col = Colisiones(m, est, cfg["planificacion"]["holgura_m"])
    cam = CamaraVirtual(m, info, cal)
    det = cb.detector(cfg["calibracion"]["diccionario"])
    ic = Intrinsecos.de_meta(INTR_COLOR)
    lado_m = cfg["calibracion"]["marcador_lado_m"]
    K = cb.esquinas_marcador(lado_m * 1.4)          # con el margen blanco, todo dentro de la imagen
    rng = np.random.default_rng(a.semilla)
    inicio = {"izq": postura[13:20].copy(), "der": postura[20:27].copy()}
    salida = {}
    for lado in LADOS:
        Td = cal.T_dorso[lado]
        id_m = cfg["calibracion"]["ids"][lado]
        buenas, n_ik, n_col, n_vista = [], 0, 0, 0
        for p, nrm in candidatas(lado, cin, cal, mon["mesa"]["plano"]["d"], rng, a.intentos):
            t = TareaIK(posicion=p, punto=Td[:3, 3], bandas=[Banda(Td[:3, 2], nrm, math.cos(math.radians(8)), 1.0)],
                        tol_pos=0.005)
            r = cin.ik(lado, t, q0=inicio[lado])
            if not r.ok or np.abs(r.q - inicio[lado]).max() > a.max_desde_cero:
                continue                # otra rama (p. ej. la muneca girada media vuelta): saltos enormes
            n_ik += 1
            M = cin.fk(lado, r.q, "mano") @ Td
            uv = ic.proyectar(aplicar(inv(cal.T_torso_color) @ M, K))
            if np.isnan(uv).any() or uv[:, 0].min() < 15 or uv[:, 0].max() > ic.ancho - 15 \
                    or uv[:, 1].min() < 15 or uv[:, 1].max() > ic.alto - 15:
                continue
            est.poner_brazo(lado, r.q)
            libre = col.libre()
            if libre:
                n_col += 1
                est.adelante()
                visto = id_m in cb.detectar(cam.fotograma(est).color, det, {id_m})
                n_vista += visto
            est.poner_brazo(lado, inicio[lado])
            if libre and visto:
                buenas.append((r.q, M[:3, 3], M[:3, 2]))
        print(f"{lado}: {n_ik} con IK, {n_col} sin choques, {n_vista} con el marcador leido en el gemelo", flush=True)
        if len(buenas) < a.n:
            sys.exit(f"{lado}: solo {len(buenas)} poses validas de {a.intentos} intentos")
        # las mas distintas entre si (posicion y normal), por muestreo del mas lejano; de sobra por si alguna
        # no se puede encadenar sin choques
        X = np.array([np.r_[b[1] / 0.1, b[2] / 0.3] for b in buenas])
        qs = np.array([b[0] for b in buenas])
        elegidas = [int(np.argmin(np.abs(qs - inicio[lado]).max(axis=1)))]
        while len(elegidas) < min(len(buenas), a.n + 8):
            dmin = np.min(np.linalg.norm(X[:, None] - X[elegidas][None], axis=2), axis=1)
            elegidas.append(int(np.argmax(dmin)))
        sub = qs[elegidas]
        orden = ordenar(sub, inicio[lado], lambda qa, qb: camino_libre(est, col, lado, qa, qb, inicio[lado]), a.n)
        if len(orden) < a.n:
            sys.exit(f"{lado}: solo {len(orden)} poses encadenadas sin choques desde y hasta 0")
        recorrido = [sub[k] for k in orden]
        salida[lado] = [[round(float(v), 4) for v in q] for q in recorrido]
        saltos = [np.abs(b - a_).max() for a_, b in zip([inicio[lado]] + recorrido, recorrido + [inicio[lado]])]
        alt = [buenas[elegidas[k]][1][2] + mon["mesa"]["plano"]["d"] for k in orden]
        print(f"{lado}: {a.n} poses; salto maximo {math.degrees(max(saltos)):.0f} grados (desde y hasta 0 incluidos); "
              f"marcador a {min(alt) * 100:.0f}-{max(alt) * 100:.0f} cm de la mesa; recorrido ~"
              f"{sum(max(8.0, 1.875 * s / 0.5) for s in saltos) / 60:.1f} min sin las pausas")
    x0, y, yaw = pose_mesa(mon)
    config.guardar_yaml(RUTA, {"generadas": "herramientas/poses_calibracion.py", "modo": "arnes", "semilla": a.semilla,
                               "calibracion_usada": f"{cal.origen} {cal.fecha}",
                               "montaje": {"fecha": str(mon.get("fecha")), "x0_m": round(x0, 4), "y_m": round(y, 4),
                                           "yaw_deg": round(yaw, 2)},
                               "inicio": {l: [round(float(v), 4) for v in inicio[l]] for l in LADOS},
                               "poses": salida},
                        "Poses de calibracion en el arnes (q de 7 juntas por mano, orden DDS 13-19 y 20-26). Las\n"
                        "recorre herramientas/arnes.py con [k], cada brazo desde `inicio` (0) y de vuelta, con el otro\n"
                        "en 0, mientras herramientas/grabar_calibracion.py guarda las muestras. Generadas con el gemelo\n"
                        "delante de `montaje`: sin choques en las poses ni en los caminos, y el marcador leido.")
    print(f"-> {RUTA}")


if __name__ == "__main__":
    main()
