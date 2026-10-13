#!/usr/bin/env python3
"""Elige la pose de observacion de cada brazo con el gemelo y la escribe en config/poses_brazos.yaml.

Por que hace falta: en reposo (la postura de la marcha, con el peso de arm_sdk en 0) la mano cuelga con la
yema a ~17 cm por debajo del tablero y a ~12 cm de su borde. Subir el antebrazo desde ahi barre el borde
de la mesa: la yema describe un arco de ~54 cm de radio alrededor del codo (comprobado en el gemelo,
2026-10-12). Asi que cada tirada pasa del reposo a una pose de observacion con las manos fuera de la mesa
y de la vista de la camara, trabaja desde ella y vuelve por ella al reposo antes de bajar el peso.

Criterios (todos en el gemelo, con la holgura del plan): reposo -> observacion limpio; observacion -> la
zona de recogida (esquinas y centro, a la altura de transporte con la palma abajo) limpio; el brazo sin tapar
la zona en la imagen de color; los dos brazos en espejo (el derecho es el espejo del izquierdo) para que
el centro de masas no se vaya a un lado; y, entre las validas, la de menor movimiento desde el reposo.

    python3 herramientas/poses_brazos.py --intentos 3000         # en la PC
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import calibracion as cb  # noqa: E402
from manip import config  # noqa: E402
from manip.cinematica import Cinematica, TareaIK, espejo, palma_abajo  # noqa: E402
from manip.geometria import Intrinsecos, aplicar, inv  # noqa: E402
from manip.planificacion import Planificador  # noqa: E402
from sim.colisiones import Colisiones  # noqa: E402
from sim.escena import INTR_COLOR, Estado, construir  # noqa: E402

RUTA = os.path.join(config.CONFIG, "poses_brazos.yaml")
# Criterio de la pose (2026-10-12): primero se exigia la mano fuera de la imagen y salia con los brazos en alto;
# ahora basta con que el brazo no tape la zona de recogida en la imagen.


def tramo_limpio(pl, cin, est, col, lado, qa, qb, n=16, holgura=0.015, h_min=0.05):
    for s in np.linspace(0, 1, n + 1):
        q = qa + (qb - qa) * s
        for marco in ("yema", "mano"):
            p = cin.fk(lado, q, marco)[:3, 3]
            if pl.sobre_mesa(p, 0.02) and pl.plano.altura(p) < h_min:
                return False
        est.poner_brazo(lado, q)
        h = col.revisar()
        if h and h[0][2] < holgura:
            return False
    return True


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--intentos", type=int, default=3000)
    ap.add_argument("--semilla", type=int, default=0)
    a = ap.parse_args()
    cin = Cinematica.desde_config(cfg)
    try:
        cal = cb.cargar()
    except FileNotFoundError:
        cal = cb.desde_urdf(cin, np.array([[1, 0, 0, 0.015], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]]))
    mon, obj = config.cargar_montaje(), config.cargar_objetos()
    m, info = construir(montaje=mon, cal=cal, con_camaras=False)
    est = Estado(m, info)
    col = Colisiones(m, est, cfg["planificacion"]["holgura_m"])
    reposo = {"izq": np.array(info["postura"][13:20]), "der": np.array(info["postura"][20:27])}
    pl = Planificador(cfg, mon, obj, cin, (est, col), reposo)
    ic = Intrinsecos.de_meta(INTR_COLOR)
    z = mon["zona_recogida"]
    objetivos = [(x, y) for x in z["x"] for y in z["y"]] + [(np.mean(z["x"]), np.mean(z["y"]))]
    zona_px = np.round(ic.proyectar(aplicar(inv(cal.T_torso_color), np.array(
        [pl.punto((x, y), 0.0) for x, y in ((z["x"][0], z["y"][0]), (z["x"][1], z["y"][0]),
                                             (z["x"][1], z["y"][1]), (z["x"][0], z["y"][1]))])))).astype(np.float32)
    ht = cfg["planificacion"]["altura_transporte_m"]
    rng = np.random.default_rng(a.semilla)
    lo, hi = cin.lo["izq"], cin.hi["izq"]
    # el espejo del derecho tambien tiene que caber en sus limites
    validas = []
    for k in range(a.intentos):
        q = lo + (hi - lo) * rng.random(7)
        if not cin.dentro_de_limites("der", espejo(q)):
            continue
        ok = True
        for lado, qv in (("izq", q), ("der", espejo(q))):
            y = cin.fk(lado, qv, "yema")[:3, 3]
            mano = cin.fk(lado, qv, "mano")[:3, 3]
            if min(pl.plano.altura(y), pl.plano.altura(mano)) < 0.12:
                ok = False
                break
            # el brazo no tapa la zona de recogida en la imagen (la percepcion ya quita el propio brazo; lo que
            # no puede es ocultar los objetos): ni el antebrazo ni la mano caen en la zona ampliada
            esq = cin.esqueleto(qv if lado == "izq" else reposo["izq"], qv if lado == "der" else reposo["der"])[lado]
            puntos = np.vstack([esq[1] + (esq[2] - esq[1]) * s for s in np.linspace(0, 1, 6)] +
                               [esq[2] + (esq[4] - esq[2]) * s for s in np.linspace(0, 1, 6)])
            uv = ic.proyectar(aplicar(inv(cal.T_torso_color), puntos))
            if np.isfinite(uv).all() and any(cv2.pointPolygonTest(zona_px, (float(u), float(v)), True) > -25
                                             for u, v in uv):
                ok = False
                break
            est.poner_brazo("izq", reposo["izq"])
            est.poner_brazo("der", reposo["der"])
            if not tramo_limpio(pl, cin, est, col, lado, reposo[lado], qv):
                ok = False
                break
            for xy in objetivos:
                xy_l = xy if lado == "izq" else (xy[0], xy[1])
                r = cin.ik(lado, TareaIK(posicion=pl.punto(xy_l, ht), punto=np.array([0.215, -0.07 if lado == "izq" else 0.07, 0.01]),
                                         bandas=[palma_abajo(lado, 35)], tol_pos=0.01), q0=qv)
                if not r.ok or not tramo_limpio(pl, cin, est, col, lado, qv, r.q):
                    ok = False
                    break
            if not ok:
                break
        est.poner_brazo("izq", reposo["izq"])
        est.poner_brazo("der", reposo["der"])
        if ok:
            # lo que menos mueve el centro de masas: el desplazamiento del codo y la muneca desde el reposo
            d = sum(float(np.linalg.norm(cin.fk("izq", q, f)[:3, 3] - cin.fk("izq", reposo["izq"], f)[:3, 3]))
                    for f in ("codo", "mano"))
            validas.append((d, q))
    if not validas:
        sys.exit(f"ninguna pose de observacion valida en {a.intentos} intentos")
    validas.sort(key=lambda v: v[0])
    desp, q = validas[0]
    yi = cin.fk("izq", q, "yema")[:3, 3]
    print(f"{len(validas)} validas de {a.intentos}; elegida: codo y muneca se mueven {desp * 100:.0f} cm en total desde "
          f"el reposo, {np.degrees(np.abs(q - reposo['izq']).max()):.0f} grados de salto maximo; yema izquierda en "
          f"{np.round(yi, 2).tolist()} ({pl.plano.altura(yi) * 100:.0f} cm sobre la mesa)")
    config.guardar_yaml(RUTA, {"generadas": "herramientas/poses_brazos.py", "semilla": a.semilla,
                               "reposo_medido": {"izq": [round(float(v), 4) for v in reposo["izq"]],
                                                 "der": [round(float(v), 4) for v in reposo["der"]]},
                               "observacion": {"izq": [round(float(v), 4) for v in q],
                                               "der": [round(float(v), 4) for v in espejo(q)]},
                               "alternativas_izq": [[round(float(v), 4) for v in qq] for _, qq in validas[1:6]]},
                        "Pose de observacion de cada brazo (q de 7 juntas, orden de arm_sdk): las manos fuera de la\n"
                        "mesa y de la vista. Cada tirada va del reposo a ella con los dos brazos en espejo, trabaja\n"
                        "desde ella y vuelve por ella al reposo antes de bajar el peso. La genera\n"
                        "herramientas/poses_brazos.py con el gemelo; se prueba en el robot en el hito 3.")
    print("alternativas (desplazamiento cm, yema):", [(round(d * 100), np.round(cin.fk('izq', qq, 'yema')[:3, 3], 2).tolist()) for d, qq in validas[1:6]])
    print(f"-> {RUTA}")


if __name__ == "__main__":
    main()
