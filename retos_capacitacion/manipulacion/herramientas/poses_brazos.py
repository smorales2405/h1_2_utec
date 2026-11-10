#!/usr/bin/env python3
"""Elige la pose de observacion de cada brazo con el gemelo y la escribe en config/poses_brazos.yaml.

Por que hace falta: en reposo (la postura de la marcha, con el peso de arm_sdk en 0) la mano cuelga con la
yema a ~17 cm por debajo del tablero y a ~12 cm de su borde. Subir el antebrazo desde ahi barre el borde
de la mesa: la yema describe un arco de ~54 cm de radio alrededor del codo (comprobado en el gemelo,
2026-10-12). Asi que cada tirada pasa del reposo a una pose de observacion con las manos fuera de la mesa
y de la vista de la camara, trabaja desde ella y vuelve por ella al reposo antes de bajar el peso.

Desde el 2026-11-11 (PLAN, Hito 0) se parte de los brazos a 0, lo que deja herramientas/arnes.py con el robot de
pie: la pose del 2026-10-12 se habia elegido desde el reposo de la marcha y, desde 0, el menique izquierdo pasaba
a 13-15 mm de la mesa (7-8 mm con 2 grados de error en el hombro, 4 con 3 en el codo; en el robot hubo hasta
3.7 grados de error de seguimiento). Ahora el camino desde 0 se exige limpio con la holgura del plan y, ademas,
con +-ERROR_DEG en el hombro (pitch) y el codo de cada brazo, a mas de HOLGURA_ERROR_M. --desde reposo_fsm,
lo de antes.

Criterios (todos en el gemelo, con la holgura del plan): inicio -> observacion limpio; observacion -> la
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
from sim.escena import INTR_COLOR, Estado, construir, postura_arnes  # noqa: E402

RUTA = os.path.join(config.CONFIG, "poses_brazos.yaml")
ERROR_DEG = 3.0              # error de seguimiento con el que el camino desde el inicio tiene que seguir limpio
HOLGURA_ERROR_M = 0.01       # ... a esta distancia
HOMBRO_PITCH, CODO = 0, 3
HOLGURA_ZONA_M = 0.015       # observacion -> zona en recta articular (el planificador comprueba despues sus planes
                             # con la holgura del plan): con 2 cm no queda ninguna en 4000 intentos
# Criterio de la pose (2026-10-12): primero se exigia la mano fuera de la imagen y salia con los brazos en alto;
# ahora basta con que el brazo no tape la zona de recogida en la imagen.


def tramo_limpio(pl, cin, est, col, lado, qa, qb, n=16, holgura=0.02, h_min=0.05, error=None):
    """Sin nada a menos de `holgura` en la recta articular de qa a qb (n tramos). error: q7 que se suma a todo el
    camino salvo al punto de partida (un brazo que sigue la consigna con ese error)."""
    for s in np.linspace(0, 1, n + 1):
        q = qa + (qb - qa) * s + (error if error is not None and s > 0 else 0.0)
        for marco in ("yema", "mano"):
            p = cin.fk(lado, q, marco)[:3, 3]
            if pl.sobre_mesa(p, 0.02) and pl.plano.altura(p) < h_min:
                return False
        est.poner_brazo(lado, q)
        h = col.revisar()
        if h and h[0][2] < holgura:
            return False
    return True


def errores(lado):
    """Los errores de seguimiento con los que se prueba el camino desde el inicio: +-ERROR_DEG en el pitch del
    hombro y en el codo (los que suben o bajan la mano hacia la mesa)."""
    sal = []
    for j in (HOMBRO_PITCH, CODO):
        for signo in (1.0, -1.0):
            e = np.zeros(7)
            e[j] = signo * np.radians(ERROR_DEG)
            sal.append(e)
    return sal


def inicio_limpio(pl, cin, est, col, lado, qa, qb, holgura):
    """El camino desde el inicio, con la holgura del plan y con los errores de `errores` a HOLGURA_ERROR_M."""
    if not tramo_limpio(pl, cin, est, col, lado, qa, qb, n=40, holgura=holgura):
        return False
    return all(tramo_limpio(pl, cin, est, col, lado, qa, qb, n=40, holgura=HOLGURA_ERROR_M, error=e)
               for e in errores(lado))


def dist_tramo(est, col, qa, qb, error=None, paso=0.02):
    """La menor distancia (hasta el margen de `col`) con los dos brazos en espejo en la recta de qa a qb (q7 del
    izquierdo), con `error` sumado a todo el camino salvo al punto de partida."""
    n = max(2, int(np.ceil(np.abs(qb - qa).max() / paso)))
    d = col.margen
    for s in np.linspace(0, 1, n + 1)[1:]:
        q = qa + (qb - qa) * s + (0.0 if error is None else error)
        est.poner_brazo("izq", q)
        est.poner_brazo("der", espejo(q))
        h = col.revisar()
        if h:
            d = min(d, h[0][2])
    return d


def buscar_paso(est, col, inicio, obs, holgura):
    """El punto de paso entre el inicio y la observacion (q7 del izquierdo; el derecho, en espejo): unas juntas
    van primero a su valor de la observacion y luego el resto. De las combinaciones con los dos tramos a mas de
    `holgura` y, con los errores de `errores`, a mas de HOLGURA_ERROR_M, la de menor recorrido (y menos juntas).
    Devuelve (paso, juntas, distancia nominal, distancia con error) o None."""
    import itertools
    mejores = []
    for r in range(1, 7):
        for S in itertools.combinations(range(7), r):
            paso = inicio.copy()
            paso[list(S)] = obs[list(S)]
            nom = min(dist_tramo(est, col, inicio, paso), dist_tramo(est, col, paso, obs))
            if nom < holgura:
                continue
            rob = min(min(dist_tramo(est, col, inicio, paso, e), dist_tramo(est, col, paso, obs, e)) for e in errores("izq"))
            if rob < HOLGURA_ERROR_M:
                continue
            recorrido = np.abs(paso - inicio).max() + np.abs(obs - paso).max()
            mejores.append((round(recorrido, 3), r, S, paso, nom, rob))
    est.poner_brazo("izq", inicio)
    est.poner_brazo("der", espejo(inicio))
    if not mejores:
        return None
    _, _, S, paso, nom, rob = min(mejores, key=lambda m: (m[0], m[1]))
    return paso, S, nom, rob


def solo_paso(cfg, cin, cal, salida):
    """--solo-paso: la pose de observacion de config/poses_brazos.yaml se queda; se busca su paso_inicio desde los
    brazos a 0 y se escribe en el mismo fichero."""
    mon = config.cargar_montaje()
    poses = config._leer(RUTA)
    m, info = construir(montaje=mon, cal=cal, cfg=cfg, postura=[float(v) for v in postura_arnes(cfg)], con_camaras=False)
    est = Estado(m, info)
    holgura = float(cfg["planificacion"]["holgura_m"])
    col = Colisiones(m, est, 0.05)
    inicio, obs = np.zeros(7), np.asarray(poses["observacion"]["izq"], float)
    recta = (dist_tramo(est, col, inicio, obs), min(dist_tramo(est, col, inicio, obs, e) for e in errores("izq")))
    print(f"recta de 0 a la observacion: {recta[0] * 1000:.1f} mm, con +-{ERROR_DEG:g} grados {recta[1] * 1000:.1f} mm")
    r = buscar_paso(est, col, inicio, obs, holgura)
    if r is None:
        sys.exit("ningun punto de paso deja los dos tramos limpios")
    paso, S, nom, rob = r
    nombres = ["hombro pitch", "hombro roll", "hombro yaw", "codo", "muneca roll", "muneca pitch", "muneca yaw"]
    print(f"punto de paso: primero {', '.join(nombres[j] for j in S)}; luego el resto. Los dos tramos a >= "
          f"{nom * 1000:.0f} mm (tope {col.margen * 1000:.0f}), con +-{ERROR_DEG:g} grados a >= {rob * 1000:.0f} mm")
    poses["paso_inicio"] = {"izq": [round(float(v), 4) for v in paso], "der": [round(float(v), 4) for v in espejo(paso)]}
    poses["paso_inicio_juntas"] = [nombres[j] for j in S]
    poses["paso_inicio_montaje"] = mon.get("fecha")
    config.guardar_yaml(salida, poses,
                        "Pose de observacion de cada brazo (q de 7 juntas, orden de arm_sdk): las manos fuera de la\n"
                        "mesa y de la vista. La genera herramientas/poses_brazos.py con el gemelo. paso_inicio: el punto\n"
                        "de paso desde los brazos a 0 (arnes.py): en recta, el menique izquierdo pasaba a 13 mm de la\n"
                        "mesa y con 3 grados de error la tocaba (2026-11-11); primero van paso_inicio_juntas y luego el\n"
                        "resto (poses_brazos.py --solo-paso). manipulacion.py --arnes y curva_pasos.py pasan por el.")
    print(f"-> {salida}")


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--intentos", type=int, default=3000)
    ap.add_argument("--semilla", type=int, default=0)
    ap.add_argument("--salida", default=RUTA, help="donde se escribe (por defecto, config/poses_brazos.yaml)")
    ap.add_argument("--desde", choices=["cero", "reposo_fsm"], default="cero",
                    help="cero: los brazos a 0 de arnes.py (de pie en el arnes); reposo_fsm: el reposo de la marcha")
    ap.add_argument("--solo-paso", action="store_true",
                    help="no busca pose: deja la de observacion y le busca el punto de paso desde los brazos a 0")
    a = ap.parse_args()
    cin = Cinematica.desde_config(cfg)
    try:
        cal = cb.cargar()
    except FileNotFoundError:
        cal = cb.desde_urdf(cin, np.array([[1, 0, 0, 0.015], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]]))
    if a.solo_paso:
        return solo_paso(cfg, cin, cal, a.salida)
    mon, obj = config.cargar_montaje(), config.cargar_objetos()
    holgura = float(cfg["planificacion"]["holgura_m"])
    postura = [float(v) for v in postura_arnes(cfg)] if a.desde == "cero" else None
    m, info = construir(montaje=mon, cal=cal, cfg=cfg, postura=postura, con_camaras=False)
    est = Estado(m, info)
    col = Colisiones(m, est, holgura)
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
            if not inicio_limpio(pl, cin, est, col, lado, reposo[lado], qv, holgura):
                ok = False
                break
            # trabajando, el otro brazo espera en su pose de observacion (no en el inicio: con los brazos a 0 su mano
            # esta delante, sobre la mesa, y estorbaria en la zona)
            otro = "der" if lado == "izq" else "izq"
            est.poner_brazo(otro, espejo(qv))
            for xy in objetivos:
                xy_l = xy if lado == "izq" else (xy[0], xy[1])
                r = cin.ik(lado, TareaIK(posicion=pl.punto(xy_l, ht), punto=np.array([0.211, -0.07 if lado == "izq" else 0.07, 0.01]),
                                         bandas=[palma_abajo(lado, 35)], tol_pos=0.01), q0=qv)
                if not r.ok or not tramo_limpio(pl, cin, est, col, lado, qv, r.q, holgura=HOLGURA_ZONA_M):
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
    config.guardar_yaml(a.salida, {"generadas": "herramientas/poses_brazos.py", "semilla": a.semilla, "desde": a.desde,
                               "holgura_m": holgura, "error_deg": ERROR_DEG, "holgura_error_m": HOLGURA_ERROR_M,
                               "montaje": mon.get("fecha"),
                               "reposo_medido": {"izq": [round(float(v), 4) for v in reposo["izq"]],
                                                 "der": [round(float(v), 4) for v in reposo["der"]]},
                               "observacion": {"izq": [round(float(v), 4) for v in q],
                                               "der": [round(float(v), 4) for v in espejo(q)]},
                               "alternativas_izq": [[round(float(v), 4) for v in qq] for _, qq in validas[1:6]]},
                        "Pose de observacion de cada brazo (q de 7 juntas, orden de arm_sdk): las manos fuera de la\n"
                        "mesa y de la vista. Cada tirada va del inicio (desde: cero = los brazos a 0 de arnes.py) a\n"
                        "ella con los dos brazos en espejo, trabaja desde ella y vuelve por ella al inicio. La genera\n"
                        "herramientas/poses_brazos.py con el gemelo (camino desde el inicio limpio tambien con\n"
                        "error_deg en hombro y codo a holgura_error_m).")
    print("alternativas (desplazamiento cm, yema):", [(round(d * 100), np.round(cin.fk('izq', qq, 'yema')[:3, 3], 2).tolist()) for d, qq in validas[1:6]])
    print(f"-> {a.salida}")


if __name__ == "__main__":
    main()
