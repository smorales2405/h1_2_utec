"""El toque de la yema del indice contra la mesa, para el nivel 1 sin tactil (PDF 6.4.4; RESULTADOS §36: el tactil no
responde por Modbus TCP).

Una tirada del nivel 1 con una sola diana fija, la de --diana (manipulacion.py --nivel 1 --arnes --dianas X,Y: la
camara no decide nada; la mesa es la del [g] de config/montaje.yaml), con limites de prueba:
  - aproxima a --aproximar cm sobre la mesa (el nivel 1, 6) y baja a --vel mm/s (de verdad, ya con la escala)
  - para al notar la mesa con la fuerza del indice (--umbral gf sobre su base) o, como mucho, a --bajada cm por
    debajo de la mesa del [g] (1; el nivel 1, 2)
  - congela, sube y vuelve por la observacion a 0, como en el nivel 1; la salida segura y el vigilante, los de
    siempre (error de seguimiento del brazo, corte de fuerza de la mano, IMU, ESPACIO, arnes.py)
Graba cada lectura de la mano (manos.csv) y el brazo (lowstate.csv, 50 Hz); una copia del montaje. Despues compara,
frente a la altura de la yema sobre la mesa (cinematica con las q medidas), cuando sube la fuerza del indice, cuando
crece el error de seguimiento del brazo y cuando cambia el par de la muneca: que aparece antes y con que umbral.

La primera vez, una espuma o un trapo doblado sobre la mesa (--espesor cm: la altura del contacto esperado).
    terminal 1: ./ejecutar.sh herramientas/arnes.py                      (de pie, [n], nivelado)
    terminal 2: ./ejecutar.sh herramientas/probar_toque.py [--diana 0.48,0.10] [--umbral 40] [--espesor 2]
    en la PC:   python3 herramientas/probar_toque.py --analizar datos/tirada_<fecha>_nivel1
La mano es la del lado de la diana (y >= 0: izquierda).
"""

import argparse
import csv
import json
import math
import os
import shutil
import sys

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from manip import config  # noqa: E402
from manip.mensajes import JUNTAS_BRAZO  # noqa: E402

UMBRALES_F = (5, 10, 30)                   # gf, |fuerza del indice - su base| (al tocar baja: RESULTADOS §43)
UMBRALES_ANG = (10, 25, 50)                # cuentas que la mesa abre el indice (su ANGLE_ACT sube)
UMBRALES_ERR = (0.5, 1.0, 2.0)             # grados, la junta del brazo que mas se aparta de su consigna
UMBRALES_TAU = (0.3, 0.6, 1.0)             # N m, cambio del par de la muneca (pitch) desde antes de bajar


def argumentos_tirada(diana, escala, sim=False):
    """Lo que daria manipulacion.leer_args para --nivel 1 --arnes --dianas X,Y (y --sim)."""
    return argparse.Namespace(nivel=1, simulacro=False, sim=sim, escala_tiempo=escala, dianas=[tuple(diana)],
                              asignacion={}, colocacion={}, visor=False, velocidad=1.0, arnes=True, recuperar=False,
                              parada_en=None, sin_comprobar_montaje=False, salida_directa=False)


def _csv(ruta):
    with open(ruta) as fh:
        return list(csv.DictReader(fh))


def _primero(t, x, umbral):
    """El primer t en que x pasa de umbral (o None)."""
    k = np.flatnonzero(np.asarray(x) > umbral)
    return float(t[k[0]]) if len(k) else None


def analizar(carpeta, espesor_m=0.0, log=print):
    """La bajada de una tirada de probar_toque.py: que senal nota antes la mesa. Devuelve un dict."""
    from manip.cinematica import Cinematica
    from manip.geometria import Plano
    from manip.registro import NOMBRES
    cfg = config.cargar()
    res = json.load(open(os.path.join(carpeta, "resumen.json")))
    ruta_mon = os.path.join(carpeta, "montaje.yaml")
    mon = config.cargar_montaje(ruta_mon) if os.path.exists(ruta_mon) else config.cargar_montaje()
    # la mesa con la correccion de altura que uso la tirada (resumen), como en manipulacion.armar
    corr = float((res.get("prueba_toque") or {}).get("correccion_altura_m", 0.0))
    if corr:
        from manip.montaje import corregir_altura
        mon = corregir_altura(mon, corr)
    plano = Plano(mon["mesa"]["plano"]["n"], mon["mesa"]["plano"]["d"])
    ev = _csv(os.path.join(carpeta, "eventos.csv"))
    t_desc = [float(e["t"]) for e in ev if e["estado"] == "DESCENSO"]
    t_tocar = [float(e["t"]) for e in ev if e["estado"] == "TOCAR"]
    if not t_desc:
        return {"error": "la tirada no llego a bajar (sin DESCENSO en eventos.csv)"}
    t0, t1 = t_desc[0], (t_tocar[0] if t_tocar else t_desc[0] + 120.0)
    toque = next((e for e in ev if e["estado"] == "TOCAR"), None)
    detalle = json.loads(toque["detalle"]) if toque and toque["detalle"] else {}
    plan = next((json.loads(e["detalle"]) for e in ev if e["estado"] == "PLANIFICAR" and e["detalle"]), {})
    diana = plan.get("diana") or (res.get("args", {}).get("dianas") or [[0.0, 0.0]])[0]
    lado = "izq" if diana[1] >= 0 else "der"
    # brazo: altura de la yema sobre la mesa, error de seguimiento y par de la muneca
    ls = _csv(os.path.join(carpeta, "lowstate.csv"))
    tl = np.array([float(r["t"]) for r in ls])
    sel = (tl >= t0 - 2.0) & (tl <= t1 + 3.0)
    ls = [r for r, s in zip(ls, sel) if s]
    tl = tl[sel]
    # la yema con los dedos de la preforma del toque (el indice flexionado la mueve)
    from manip.planificacion import Planificador
    pref = next((json.loads(e["detalle"]).get("preforma") for e in ev if e["estado"] == "PLANIFICAR" and e["detalle"]),
                None) or (cfg["planificacion"].get("toque") or {}).get("preforma", "senalar")
    cin = Planificador(cfg, mon, config.cargar_objetos(), Cinematica.desde_config(cfg)).cin_preforma(pref)
    js = JUNTAS_BRAZO[lado]
    q = np.array([[float(r[f"{NOMBRES[j]}_q"]) for j in js] for r in ls])
    obj = np.array([[float(r[f"{NOMBRES[j]}_obj"]) if r[f"{NOMBRES[j]}_obj"] else np.nan for j in js] for r in ls])
    h = np.array([float(plano.altura(cin.fk(lado, qq, "yema")[:3, 3])) for qq in q])
    err = np.degrees(np.nanmax(np.abs(q - obj), axis=1))
    j_mp = js[5]                                       # pitch de la muneca
    tau = np.array([float(r[f"{NOMBRES[j_mp]}_tau"]) for r in ls])
    antes = (tl >= t0 - 2.0) & (tl < t0)
    dtau = np.abs(tau - (np.median(tau[antes]) if antes.any() else tau[0]))
    # el error y el par, solo en la bajada (antes de bajar, el brazo llega de otro movimiento)
    en_bajada = (tl >= t0) & (tl <= t1 + 0.5)
    # mano: fuerza del indice sobre su base
    mn = [r for r in _csv(os.path.join(carpeta, "manos.csv")) if r["lado"] == lado]
    tm = np.array([float(r["t"]) for r in mn])
    fi = np.array([float(r["f3"]) - float(r["base3"]) for r in mn])
    msel = (tm >= t0) & (tm <= t1 + 0.5)
    ai = np.array([float(r["ang3"]) for r in mn])
    a0 = float(np.median(ai[(tm >= t0 - 2.0) & (tm < t0 + 0.5)])) if ((tm >= t0 - 2.0) & (tm < t0 + 0.5)).any() else ai[0]
    abre = ai - a0                                     # la mesa abre el indice: su angulo sube
    t_geo = _primero(tl[en_bajada], espesor_m - h[en_bajada], 0.0)     # la yema llega a la mesa (o a la espuma)

    def h_en(t):
        return None if t is None else float(np.interp(t, tl, h))
    senales = {}
    for u in UMBRALES_F:
        t = _primero(tm[msel], np.abs(fi[msel]), u)
        senales[f"|fuerza del indice| > {u} gf"] = (t, h_en(t))
    for u in UMBRALES_ANG:
        t = _primero(tm[msel], abre[msel], u)
        senales[f"indice abierto > {u} cuentas"] = (t, h_en(t))
    for u in UMBRALES_ERR:
        t = _primero(tl[en_bajada], err[en_bajada], u)
        senales[f"error del brazo > {u} grados"] = (t, h_en(t))
    for u in UMBRALES_TAU:
        t = _primero(tl[en_bajada], dtau[en_bajada], u)
        senales[f"par de la muneca +{u} N m"] = (t, h_en(t))
    r = {"lado": lado, "diana": diana, "resultado": toque["motivo"] if toque else "sin TOCAR",
         "fuerza_al_parar_gf": detalle.get("fuerza_gf"), "abierto_al_parar": detalle.get("abre"),
         "por": detalle.get("por"), "preforma": pref, "espesor_m": espesor_m,
         "indice_abierto_max": float(abre[msel].max()) if msel.any() else None,
         "h_min_mm": round(1000 * float(h[en_bajada].min()), 1) if en_bajada.any() else None,
         "t_llega_s": None if t_geo is None else round(t_geo - t0, 2),
         "vel_bajada_mm_s": None, "senales": {}, "fuerza_max_gf": float(fi[msel].max()) if msel.any() else None,
         "error_max_deg": float(err[en_bajada].max()) if en_bajada.any() else None,
         "lecturas_mano_hz": round(msel.sum() / max(1e-3, float(np.ptp(tm[msel]))), 1) if msel.sum() > 1 else None}
    b = en_bajada & (h > espesor_m + 0.005)
    if b.sum() > 5:
        r["vel_bajada_mm_s"] = round(-1000 * float(np.polyfit(tl[b], h[b], 1)[0]), 1)
    # la altura de la mesa: donde estaba la yema medida cuando la fuerza noto la superficie (si fue dura)
    t10 = senales.get("|fuerza del indice| > 10 gf", (None, None))[1]
    r["yema_al_notar_mm"] = None if t10 is None else round(1000 * t10, 1)
    r["mesa_real_sobre_la_corregida_mm"] = None if t10 is None else round(1000 * (t10 - espesor_m), 1)
    for k, (t, hh) in senales.items():
        r["senales"][k] = {"t_s": None if t is None else round(t - t0, 2),
                           "altura_yema_mm": None if hh is None else round(1000 * hh, 1),
                           "tras_llegar_s": None if t is None or t_geo is None else round(t - t_geo, 2)}
    log(f"\n  Toque con la mano {lado} en {diana} (preforma {pref}): {r['resultado']}" +
        (f" por {r['por']}" if r["por"] else "") + f" (fuerza al parar {r['fuerza_al_parar_gf']} gf, indice abierto "
        f"{r['abierto_al_parar']} cuentas)")
    log(f"  La yema baja a {r['vel_bajada_mm_s']} mm/s y llega a {1000 * espesor_m:.0f} mm sobre la mesa del [g] a los "
        f"{r['t_llega_s']} s; como mucho, a {r['h_min_mm']} mm. Mano leida a {r['lecturas_mano_hz']} Hz.")
    if r["mesa_real_sobre_la_corregida_mm"] is not None:
        log(f"  La fuerza lo noto con la yema medida a {r['yema_al_notar_mm']} mm: la superficie real esta "
            f"{r['mesa_real_sobre_la_corregida_mm']:+.1f} mm de donde la pone la mesa (con la correccion de "
            f"{1000 * corr:.0f} mm). Con superficie dura, eso es lo que hay que sumar a montaje.correccion_altura_m.")
    log("  Cada senal, cuando aparece (s desde que empieza a bajar), a que altura esta la yema y cuanto despues de "
        "llegar a la mesa:")
    for k, d in r["senales"].items():
        log(f"    {k:28s} " + ("no aparece" if d["t_s"] is None else
                                f"{d['t_s']:6.2f} s, yema a {d['altura_yema_mm']:+6.1f} mm" +
                                ("" if d["tras_llegar_s"] is None else f", {d['tras_llegar_s']:+.2f} s")))
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--diana", default="0.48,0.10", help="x,y en el torso (m); y >= 0, mano izquierda")
    ap.add_argument("--umbral", type=float, default=None,
                    help="gf sobre la base del indice que paran la bajada (por defecto, contacto.umbral_gf)")
    ap.add_argument("--bajada", type=float, default=1.0, help="cm por debajo de la mesa del [g], como mucho (<= 2)")
    ap.add_argument("--vel", type=float, default=3.0, help="mm/s de la bajada")
    ap.add_argument("--aproximar", type=float, default=4.0, help="cm sobre la mesa donde empieza la bajada")
    ap.add_argument("--espesor", type=float, default=0.0,
                    help="cm de lo que se toca sobre la mesa (tabla, espuma): la superficie donde para la yema medida")
    ap.add_argument("--escala-tiempo", type=float, default=2.0)
    ap.add_argument("--analizar", metavar="CARPETA", help="solo el analisis de una tirada ya hecha")
    ap.add_argument("--sim", action="store_true", help="en la PC, en el gemelo (manipulacion.py --sim --arnes)")
    a = ap.parse_args()
    if a.analizar:
        r = analizar(a.analizar, a.espesor / 100.0)
        with open(os.path.join(a.analizar, "toque.json"), "w") as fh:
            json.dump(r, fh, indent=1)
        return
    if not 0.0 < a.bajada <= 2.0:
        sys.exit("--bajada entre 0 y 2 cm (el nivel 1 baja como mucho 2 cm bajo la mesa del [g])")
    if not 1.0 <= a.vel <= 10.0:
        sys.exit("--vel entre 1 y 10 mm/s")
    diana = [float(v) for v in a.diana.split(",")]
    cfg = config.cargar()
    if a.umbral is None:
        a.umbral = float(cfg["contacto"]["umbral_gf"])
    cfg["contacto"].update(umbral_gf=float(a.umbral), bajada_max_m=a.bajada / 100.0, vel_descenso_m_s=a.vel / 1000.0,
                           superficie_m=a.espesor / 100.0)
    cfg["planificacion"]["altura_aproximacion_m"] = a.aproximar / 100.0
    import manipulacion as mp
    t = mp.armar(argumentos_tirada(diana, a.escala_tiempo, a.sim), cfg=cfg)
    shutil.copy(config.RUTA_MONTAJE, os.path.join(t.reg.carpeta, "montaje.yaml"))
    t.reg.resumen["prueba_toque"] = {"diana": diana, "umbral_gf": a.umbral, "bajada_cm": a.bajada, "vel_mm_s": a.vel,
                                     "aproximar_cm": a.aproximar, "espesor_cm": a.espesor,
                                     "correccion_altura_m": 0.0 if a.sim else
                                     float((cfg.get("montaje") or {}).get("correccion_altura_m", 0.0))}
    # cada lectura de la mano va a manos.csv (manipulacion.armar)
    c = cfg["contacto"]
    print(f"\n  PRUEBA DE TOQUE: diana {diana}, aproxima a {a.aproximar:g} cm sobre la superficie" +
          (f" ({a.espesor:g} cm sobre la mesa)" if a.espesor else "") + f", baja a {a.vel:g} mm/s (mandada hasta "
          f"{a.bajada + 100 * c.get('margen_brazo_m', 0.0):g} cm bajo la superficie: el brazo va por encima de su consigna) y "
          f"para al apartarse {a.umbral:g} gf la fuerza del indice o, de limite, con la yema medida a "
          f"{1000 * c.get('altura_toque_m', 0.0):+.0f} mm de la superficie")
    mp.correr(t, confirmar=not a.sim)
    try:
        r = analizar(t.reg.carpeta, a.espesor / 100.0)
        with open(os.path.join(t.reg.carpeta, "toque.json"), "w") as fh:
            json.dump(r, fh, indent=1)
    except Exception as ex:                          # el analisis no estorba a la tirada: se repite en la PC
        print(f"  (analisis: {ex!r}; en la PC: probar_toque.py --analizar {t.reg.carpeta})")


if __name__ == "__main__":
    main()
