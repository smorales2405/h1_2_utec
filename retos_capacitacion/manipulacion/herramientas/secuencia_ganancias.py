#!/usr/bin/env python3
"""Genera config/secuencia_ganancias.yaml: las poses de los brazos de una tirada del nivel 2 en la simulacion de
la PC (los cuatro objetos de COLOCACION_SIM a sus bandejas), para comparar ganancias en el robot SIN la mesa
delante con herramientas/comparar_ganancias.py.

    python3 herramientas/secuencia_ganancias.py          # en la PC (necesita OpenGL, como --sim)

Cada tramo es un brazo.ir_a de la tirada: la consigna de las 14 juntas de los brazos al final y la duracion
pedida, sin la escala de tiempo (brazo.py la aplica en el robot, como en la tirada). Ademas comprueba la parada
de comparar_ganancias.py: desde cualquier punto de un tramo de trabajo, los dos brazos en linea recta a la
observacion sin chocar con el propio cuerpo. La mesa, las bandejas y los objetos no cuentan (la prueba es sin
mesa): sus choques se anotan aparte, para que se vea que con la mesa delante esa parada no vale.
"""

import os
import sys
import tempfile
from types import SimpleNamespace

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
import manipulacion  # noqa: E402
from manip import config  # noqa: E402
from manip.brazo import duracion_movimiento  # noqa: E402
from manip.mensajes import JUNTAS_BRAZO, LADOS  # noqa: E402

RUTA = os.path.join(config.CONFIG, "secuencia_ganancias.yaml")
FRACCIONES = (0.25, 0.5, 0.75, 1.0)     # puntos de cada tramo desde los que se prueba la parada


def grabar(cfg, carpeta):
    """La tirada simulada del nivel 2, con cada brazo.ir_a anotado. Devuelve (tramos, motivo, resultados)."""
    a = SimpleNamespace(nivel=2, simulacro=False, sim=True, escala_tiempo=1.0, dianas=None,
                        asignacion=dict(manipulacion.ASIGNACION_SIM),
                        colocacion=dict(manipulacion.COLOCACION_SIM), datos=carpeta)
    t = manipulacion.armar(a, cfg, log=lambda *x: None)
    sup, brazo = t.sup, t.brazo
    tramos, ctx = [], {"nombre": "", "lado": "ambos"}
    mover, ambos, ir_a = sup._mover, sup._ambos, brazo.ir_a

    def _mover(lado, punto, extra=None):
        ctx.update(nombre=punto.nombre, lado=lado)
        return mover(lado, punto, extra)

    def _ambos(planes):
        ctx.update(nombre=planes[0].puntos_de_paso[-1].nombre, lado="ambos")
        return ambos(planes)

    def _ir_a(pose, segundos=None, comprobar=None):
        T = ir_a(pose, segundos, comprobar)
        c = brazo.consigna()
        plan = sup._plan_activo
        tramos.append({"estado": sup.estado, "nombre": ctx["nombre"], "lado": ctx["lado"],
                       "objeto": plan.objeto.clase if plan is not None and plan.objeto is not None else None,
                       "base_s": round(float(segundos or 0.0), 2),
                       **{l: [round(c[j], 4) for j in JUNTAS_BRAZO[l]] for l in LADOS}})
        return T

    sup._mover, sup._ambos, brazo.ir_a = _mover, _ambos, _ir_a
    motivo = manipulacion.correr(t, confirmar=False)
    return tramos, motivo, t.sup.resultados


def comprobar_paradas(cfg, tramos, observacion):
    """Para cada tramo de trabajo: (distancia minima al propio cuerpo, choques con el entorno) de la parada, que
    lleva los dos brazos en linea recta a la observacion desde FRACCIONES del tramo."""
    from sim.colisiones import Colisiones, es_entorno, grupo
    from sim.escena import POSTURA_PIE, Estado, construir
    import mujoco
    m, info = construir(cfg=cfg, postura=list(POSTURA_PIE), con_camaras=False)
    est = Estado(m, info)
    col = Colisiones(m, est, cfg["planificacion"]["holgura_m"])
    nombres = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "" for b in range(m.nbody)]
    entorno = {n for n in nombres if es_entorno(n) or n.startswith("aruco_")}
    obs = np.concatenate([observacion[l] for l in LADOS])
    sal = []
    previo = obs
    for tr in tramos:
        q_fin = np.concatenate([tr[l] for l in LADOS])
        if tr["fase"] != "trabajo":
            previo = q_fin
            continue
        d_min, peor, entorno_min = col.margen, "", 0.0
        for f in FRACCIONES:
            q0 = previo + (q_fin - previo) * f
            for u in np.linspace(0.0, 1.0, 9)[1:]:
                q = q0 + (obs - q0) * u
                est.poner_brazo("izq", q[:7])
                est.poner_brazo("der", q[7:])
                for a, b, d in col.revisar():
                    if a in entorno or b in entorno:
                        entorno_min = min(entorno_min, d)
                    elif d < d_min:
                        d_min, peor = d, f"{grupo(a)} / {grupo(b)}"
        sal.append((tr["n"], round(float(d_min), 4), peor, round(float(entorno_min), 4)))
        previo = q_fin
    return sal


def main():
    cfg = config.cargar()
    cfg["mano"]["t_base_s"] = 0.0
    poses = config._leer(os.path.join(config.CONFIG, "poses_brazos.yaml"))
    observacion = {l: np.array(poses["observacion"][l]) for l in LADOS}
    with tempfile.TemporaryDirectory() as tmp:
        tramos, motivo, resultados = grabar(cfg, tmp)
    colocados = [r["objeto"] for r in resultados if r["resultado"] == "colocado"]
    if motivo != "completado" or len(colocados) != len(manipulacion.ASIGNACION_SIM):
        sys.exit(f"La tirada simulada no salio bien ({motivo}, {resultados}): no se escribe la secuencia.")
    for k, tr in enumerate(tramos):
        tr["n"] = k + 1
        tr["fase"] = "ida" if k == 0 else ("vuelta" if k == len(tramos) - 1 else "trabajo")
    tramos[-1]["reposo"] = True
    if tramos[0]["estado"] != "A_OBSERVACION" or tramos[-1]["estado"] != "SALIDA":
        sys.exit(f"La secuencia no empieza en A_OBSERVACION y acaba en SALIDA: {tramos[0]['estado']}, {tramos[-1]['estado']}")

    paradas = comprobar_paradas(cfg, tramos, observacion)
    malas = [p for p in paradas if p[1] < 0.0]
    b = cfg["brazo"]
    previo = np.concatenate([tramos[-1][l] for l in LADOS])     # el reposo simulado
    dur = {1: 0.0, 2: 0.0}
    for tr in tramos:
        q = np.concatenate([tr[l] for l in LADOS])
        for e in dur:
            dur[e] += duracion_movimiento(float(np.abs(q - previo).max()), b["t_min_mov_s"], b["vel_max_rad_s"], e, tr["base_s"])
        previo = q
    cab = f"""Secuencia de poses de los brazos para herramientas/comparar_ganancias.py. NO EDITAR A MANO: la escribe
herramientas/secuencia_ganancias.py a partir de la tirada simulada del nivel 2 en la PC (los objetos de
COLOCACION_SIM a las bandejas de ASIGNACION_SIM, agrupados por brazo).
Cada tramo: la consigna de los dos brazos al final (rad, orden de arm_sdk) y la duracion pedida sin la escala
de tiempo. El ultimo es el reposo: en el robot se sustituye por la postura medida al empezar.
fase: ida (reposo -> observacion), trabajo, vuelta (observacion -> reposo).
{len(tramos)} tramos; solo movimiento, sin pausas ni rampas de peso: {dur[1] / 60:.1f} min con la escala 1,
{dur[2] / 60:.1f} min con la 2.
Parada (los dos brazos a la observacion en linea recta, desde 1/4, 1/2, 3/4 y el final de cada tramo de
trabajo): {'ningun choque con el propio cuerpo' if not malas else f'CHOCA en los tramos {[p[0] for p in malas]}'};
la distancia minima al cuerpo y el peor choque con la mesa (que en la prueba no esta) van en 'paradas'."""
    datos = {"origen": "simulacion del nivel 2 en la PC", "colocacion": {k: [list(xy), yaw] for k, (xy, yaw) in manipulacion.COLOCACION_SIM.items()},
             "asignacion": dict(manipulacion.ASIGNACION_SIM),
             "observacion": {l: observacion[l].round(4).tolist() for l in LADOS},
             "duracion_movimiento_min": {"escala_1": round(dur[1] / 60, 1), "escala_2": round(dur[2] / 60, 1)},
             "tramos": [{k: tr[k] for k in ("n", "fase", "estado", "nombre", "lado", "objeto", "base_s", "izq", "der")
                         if k in tr} | ({"reposo": True} if tr.get("reposo") else {}) for tr in tramos],
             "paradas": [{"tramo": n, "dist_cuerpo_m": d, "pareja": p, "choque_mesa_m": e} for n, d, p, e in paradas]}
    config.guardar_yaml(RUTA, datos, cab)
    print(f"{RUTA}: {len(tramos)} tramos, {dur[1] / 60:.1f} min de movimiento con la escala 1 y {dur[2] / 60:.1f} con la 2")
    for tr in tramos:
        print(f"  {tr['n']:2d} {tr['fase']:8s} {tr['estado']:13s} {tr['nombre']:20s} {tr['lado']:5s} {tr['objeto'] or '':9s} {tr['base_s']:5.1f} s")
    peor = min(paradas, key=lambda p: p[1])
    print(f"parada desde los {len(paradas)} tramos de trabajo: distancia minima al cuerpo {peor[1] * 1000:.0f} mm "
          f"(tramo {peor[0]}, {peor[2] or '-'}); con la mesa delante chocaria hasta {-min(p[3] for p in paradas) * 1000:.0f} mm")
    if malas:
        sys.exit(f"La parada choca con el cuerpo en los tramos {[p[0] for p in malas]}")


if __name__ == "__main__":
    main()
