"""Las manos Inspire reales abriendo y cerrando en el aire (PLAN: Fase 0 y Hito 3), con manip/mano.py, la clase que
usa manipulacion.py.

Por cada mano (--manos), una detras de otra:
  1. conectar y leer: angulos, fuerza, error y estado del firmware, y el tactil de la yema del indice
  2. linea base (mano abierta, sin carga), ruido del tactil con la mano quieta y a que ritmo se leen por Modbus TCP
     el estado (angulo, fuerza, error, estado) y el tactil de la yema: la latencia del contacto del nivel 1
  3. por cada preforma de config/objetos.yaml: ir a ella; si algun objeto la usa, cerrar con su FORCE_SET (el
     primero de la tabla) y comprobar que mano.agarre() dice "vacio" en todos los dedos (en el aire llegan a 0:
     la pista del PDF; un dedo que se queda lejos de 0 con fuerza seria un falso "lleno"); y abrir, comprobando
     que abre (la apertura tras cerrar por fuerza es la trampa conocida del firmware)
  4. con --tactil S: S segundos para apretar con un dedo la yema del indice y ver cuanto sube (el contacto con
     la mesa del nivel 1), con el umbral de inspire_hand_interface: max(5 sigma, 41 cuentas)
  4b. con --presionar A [A ...]: el indice en cada angulo A (los demas recogidos, el pulgar abierto, como la preforma
     del toque) y --segundos S para apretar la yema del indice con un dedo: cuanto suben su fuerza y su angulo. Con
     el indice abierto del todo (1000) la fuerza no cambia al apretar la punta (el usuario, 2026-11-11)
  5. con --mapa-tactil S: las 17 zonas del tactil (el mapa de inspire_hand_interface): 2 s sin tocar (base y
     ruido de cada zona) y S s para apretar en varios sitios (punta y yema de cada dedo, palma): cuanto sube cada
     zona. El 2026-11-11 la yema del indice no subia al apretarla y la izquierda variaba 254 cuentas sin tocarla
Con --speed se cierra mas despacio (el cierre en vacio con el pulgar enfrente acaba con la punta del indice contra
la del pulgar: despacio, el golpe es menor); de cada dedo se apunta donde se paro, a que velocidad cerro y si
llego a su consigna.
Mide los tiempos de cada paso, la fuerza en el aire de cada dedo cerrado (su propia flexion: 70-230 gf en la
caracterizacion), los angulos a los que llega y los errores del firmware.

No mueve los brazos: el robot puede estar colgado o de pie en el arnes con los brazos a 0 (las manos quedan
delante, a ~20 cm sobre la mesa): nada a menos de 10 cm de las manos y nadie con los dedos dentro. Pide la palabra.
    ./ejecutar.sh herramientas/probar_manos.py [--manos izq der] [--tactil 10] [--solo-leer]
    python3 herramientas/probar_manos.py --sim                       (con la mano simulada, en la PC)
Registro en datos/manos_<fecha>/: manos.csv (cada lectura), eventos.csv y resumen.json.
"""

import argparse
import os
import sys
import time

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from manip import config  # noqa: E402
from manip.mano import DEDOS, PULGAR, TACTIL_INDICE, ZONAS_TACTIL, Mano, ManoSinRespuesta  # noqa: E402
from manip.mensajes import Parada  # noqa: E402
from manip.registro import Registro, fila_mano  # noqa: E402

NOMBRES_DOF = ["menique", "anular", "medio", "indice", "pulgar", "rotacion"]


def calibracion_de_registro(carpeta, mover_min=200):
    """{lado: {tope_vacio, residuo_gf, notas}} de un registro de probar_manos.py (cierres en el aire): donde se paro
    cada dedo que se movio (>= mover_min cuentas) y la fuerza neta que marcaba; si un dedo no se movio (FORCE_SET por
    debajo de su propia flexion), su residuo sale de manos.csv con el dedo cerrado del todo por la preforma (800)."""
    import csv
    import json
    with open(os.path.join(carpeta, "resumen.json")) as fh:
        res = json.load(fh)
    with open(os.path.join(carpeta, "manos.csv")) as fh:
        filas = list(csv.DictReader(fh))
    sal = {}
    for lado, r in res.get("manos", {}).items():
        tope, residuo, notas = [None] * 6, [0.0] * 6, []
        for p in r.get("pasos", []):
            for nombre, d in (p.get("dedos") or {}).items():
                k = NOMBRES_DOF.index(nombre)
                if p["en_preforma"][k] - d["se_paro_en"] >= mover_min:
                    tope[k] = max(tope[k] or 0, int(d["se_paro_en"]))
                    residuo[k] = max(residuo[k], float(d["fuerza_neta"]))
                elif tope[k] is None:
                    notas.append(f"{nombre}: no se movio al cerrar (se paro en {d['se_paro_en']}, {d['fuerza_neta']} gf)")
        R = [x for x in filas if x["lado"] == lado]
        for k in range(5):
            if tope[k] is None:
                netas = [float(x[f"f{k}"]) - float(x[f"base{k}"]) for x in R
                         if float(x[f"ang{k}"]) < 30 and float(x[f"fset{k}"]) >= 800]
                if netas:
                    residuo[k] = float(np.median(netas))
                    notas.append(f"{NOMBRES_DOF[k]}: residuo cerrado del todo por la preforma, {residuo[k]:.0f} gf")
        sal[lado] = {"tope_vacio": tope, "residuo_gf": [int(round(v)) for v in residuo], "notas": notas}
    return sal


def guardar_calibracion(cal, carpeta, ruta=None):
    from manip.mano import RUTA_CALIBRACION
    ruta = ruta or RUTA_CALIBRACION
    datos = dict(cal)
    datos["origen"] = os.path.basename(os.path.normpath(carpeta))
    datos["fecha"] = time.strftime("%Y-%m-%d %H:%M")
    config.guardar_yaml(ruta, datos,
                        "Las manos del robot cerrando en vacio (herramientas/probar_manos.py --guardar-cierre). Por mano y\n"
                        "DOF (menique, anular, medio, indice, pulgar, rotacion): tope_vacio, donde se para cerrando en vacio\n"
                        "(null: no se movio); residuo_gf, la fuerza neta que marca cerrado (su propia flexion). manip/mano.py:\n"
                        "el indice cierra hasta su tope + margen_tope (no choca con el pulgar) y FORCE_SET suma el residuo.")
    return ruta


def force_set_de_preforma(objetos):
    """{preforma: (objeto, force_set)} con el primer objeto de la tabla que la usa."""
    sal = {}
    for k, o in objetos["objetos"].items():
        sal.setdefault(o["preforma"], (k, list(o["force_set"])))
    return sal


class Prueba:
    def __init__(self, mano, reg, log=print, reloj=time.monotonic):
        self.m, self.reg, self.log, self.reloj = mano, reg, log, reloj
        self.lado = mano.lado
        self.res = {"lado": self.lado, "pasos": []}
        leer = mano.leer

        def leer_y_apuntar():                       # cada lectura de la mano, al registro
            e = leer()
            reg.mano(e)
            return e
        mano.leer = leer_y_apuntar

    def evento(self, estado, texto, detalle=None):
        self.reg.evento(f"{estado}_{self.lado}", texto, detalle, imprimir=False)
        self.log(f"  [{self.lado}] {texto}")

    def tactil(self):
        """El tactil crudo de la punta y la yema del indice."""
        return np.concatenate([self.m.regs(a, n, signo=False) for a, n in TACTIL_INDICE])

    def leer_estado(self):
        e = self.m.leer()
        d = {"angulo": e.angulo.astype(int).tolist(), "fuerza": e.fuerza.astype(int).tolist(),
             "error": e.error.astype(int).tolist(), "estado": e.estado.astype(int).tolist(),
             "force_set": e.force_set.astype(int).tolist()}
        self.res["inicial"] = d
        self.evento("LEER", f"angulos {d['angulo']}, fuerza {d['fuerza']} gf, error {d['error']}, estado {d['estado']}")
        return e

    def base_y_ruido(self, segundos=3.0):
        f = self.m.linea_base()
        muestras, t_tac = [], []
        t_fin = self.reloj() + segundos
        while self.reloj() < t_fin:
            t0 = time.monotonic()
            muestras.append(self.tactil())
            t_tac.append(time.monotonic() - t0)
        t_est = []
        for _ in range(30):
            t0 = time.monotonic()
            self.m.leer()
            t_est.append(time.monotonic() - t0)
        T = np.array(muestras)
        sube = (T - self.m.base_tactil).max(axis=1)
        self.res["base_gf"] = f.astype(int).tolist()
        self.res["tactil_ruido"] = {"max": float(sube.max()), "std": float(T.std(axis=0).max()), "n": len(T)}
        self.res["lectura_ms"] = {"estado": round(1000 * float(np.median(t_est)), 1),
                                  "estado_max": round(1000 * float(np.max(t_est)), 1),
                                  "tactil": round(1000 * float(np.median(t_tac)), 1),
                                  "tactil_max": round(1000 * float(np.max(t_tac)), 1)}
        r = self.res["lectura_ms"]
        self.evento("BASE", f"linea base {f.astype(int).tolist()} gf; tactil quieto: sube como mucho {sube.max():.0f} "
                            f"(desviacion {T.std(axis=0).max():.1f}) en {len(T)} lecturas; una lectura del estado "
                            f"{r['estado']:.0f} ms (max {r['estado_max']:.0f}), del tactil {r['tactil']:.0f} ms "
                            f"(max {r['tactil_max']:.0f})")

    def preforma_cerrar_abrir(self, nombre, angulos, cierre):
        """cierre: (objeto, force_set) o None (solo preforma y abrir)."""
        paso = {"preforma": nombre, "angulos": list(angulos)}
        t0 = self.reloj()
        e = self.m.preforma(angulos)
        paso["t_preforma_s"] = round(self.reloj() - t0, 2)
        paso["en_preforma"] = e.angulo.astype(int).tolist()
        texto = f"{nombre}: en {paso['t_preforma_s']} s, angulos {paso['en_preforma']}"
        if cierre is not None:
            obj, fs = cierre
            antes = e.angulo.copy()
            t0 = self.reloj()
            e = self.m.cerrar(fs)
            dt = self.reloj() - t0
            paso.update(objeto=obj, force_set=fs, force_set_escrito=self.m.force_set.astype(int).tolist(),
                        t_cierre_s=round(dt, 2), cerrada=e.angulo.astype(int).tolist(),
                        fuerza_neta=(e.fuerza - e.base).astype(int).tolist())
            # cada dedo que cierra: donde se paro, si llego a su consigna y la velocidad media
            dedos = {}
            for k in range(5):
                if fs[k] is None or fs[k] <= 0:
                    continue
                consigna = self.m.cierre[k]
                dedos[NOMBRES_DOF[k]] = {"se_paro_en": int(e.angulo[k]), "consigna": consigna,
                                         "llego": bool(e.angulo[k] <= consigna + 20),
                                         "fuerza_neta": int(e.fuerza[k] - e.base[k]),
                                         "cuentas_s": round(float(antes[k] - e.angulo[k]) / max(dt, 1e-3), 0)}
            paso["dedos"] = dedos
            texto += " | " + ", ".join(f"{n} {d['se_paro_en']}" + ("" if d["llego"] else "*") for n, d in dedos.items())
            if not all(d["llego"] for d in dedos.values()):
                texto += " (* = se paro antes de su consigna)"
            ok, detalle = self.m.agarre(e, fs)
            paso["agarre"] = {NOMBRES_DOF[k]: v for k, v in detalle.items()}
            paso["falso_lleno"] = ok or any(v != "vacio" for v in detalle.values())
            texto += (f"; cerrada (FORCE_SET de {obj}) en {paso['t_cierre_s']} s a {paso['cerrada']}, fuerza en el aire "
                      f"{paso['fuerza_neta']} gf, agarre {paso['agarre']}" +
                      (" <- OJO: en el aire deberia ser todo 'vacio'" if paso["falso_lleno"] else ""))
        t0 = self.reloj()
        try:
            e = self.m.abrir()
            paso.update(abre=True, t_abrir_s=round(self.reloj() - t0, 2), abierta=e.angulo.astype(int).tolist())
            texto += f"; abierta en {paso['t_abrir_s']} s"
        except ManoSinRespuesta as ex:
            paso.update(abre=False, t_abrir_s=round(self.reloj() - t0, 2), fallo=str(ex))
            texto += f"; NO ABRE: {ex}"
        e = self.m.leer()
        paso["error_al_final"] = e.error.astype(int).tolist()
        self.res["pasos"].append(paso)
        self.evento("PREFORMA", texto, paso)
        return paso

    def apretar_yema(self, segundos):
        self.log(f"\n  [{self.lado}] Aprieta con un dedo la yema del indice (la cara de la palma, en la punta) durante "
                 f"los proximos {segundos:g} s; suelta y vuelve a apretar un par de veces.")
        sube = []
        t_fin = self.reloj() + segundos
        while self.reloj() < t_fin:
            sube.append(float((self.tactil() - self.m.base_tactil).max()))
            time.sleep(0.03)
        sube = np.array(sube)
        r = self.res["tactil_ruido"]
        umbral = max(5.0 * r["std"], 41.0)              # el de inspire_hand_interface (fase A1 del tactil)
        self.res["tactil_apretado"] = {"max": float(sube.max()), "lecturas": len(sube),
                                       "por_encima_del_umbral": float(np.mean(sube > umbral)), "umbral": umbral}
        self.evento("TACTIL", f"yema apretada: sube hasta {sube.max():.0f} (ruido quieto {r['max']:.0f}); con un umbral de "
                              f"{umbral:.0f}, por encima el {100 * np.mean(sube > umbral):.0f} % del tiempo")

    def presionar(self, angulos, segundos):
        """El indice quieto en cada angulo; la persona aprieta la yema: lo que sube la fuerza y lo que se abre."""
        res = {}
        for a in angulos:
            self.m.preforma([0, 0, 0, int(a), 1000, 1000])
            time.sleep(1.0)
            base = [self.m.leer() for _ in range(10)]
            f0, a0 = np.median([e.fuerza[3] for e in base]), np.median([e.angulo[3] for e in base])
            ruido = float(np.ptp([e.fuerza[3] for e in base]))
            self.log(f"\n  [{self.lado}] indice en {a}: aprieta la yema del indice (la cara de la palma, en la punta) "
                     f"durante {segundos:g} s, como lo haria la mesa")
            fs, ang, t_fin = [], [], self.reloj() + segundos
            while self.reloj() < t_fin:
                e = self.m.leer()
                fs.append(float(e.fuerza[3] - f0))
                ang.append(float(e.angulo[3] - a0))
            fs, ang = np.array(fs), np.array(ang)
            res[int(a)] = {"angulo_medido": int(a0), "fuerza_sube_max": round(float(fs.max()), 0),
                           "fuerza_baja_max": round(float(fs.min()), 0), "ruido_quieto": round(ruido, 0),
                           "se_abre_max": round(float(ang.max()), 0), "lecturas": len(fs)}
            r = res[int(a)]
            self.evento("PRESIONAR", f"indice en {a} (medido {r['angulo_medido']}): la fuerza sube hasta {r['fuerza_sube_max']:+.0f} "
                                     f"gf (baja hasta {r['fuerza_baja_max']:+.0f}; quieto varia {r['ruido_quieto']:.0f}) y se abre "
                                     f"{r['se_abre_max']:+.0f} cuentas en {r['lecturas']} lecturas")
        self.m.abrir()
        self.res["presionar"] = res

    def mapa_tactil(self, segundos, base_s=2.0):
        """Todas las zonas: base y ruido sin tocar, y cuanto sube cada una mientras se aprieta."""
        def frame():
            return [self.m.regs(a, n, signo=False) for _, a, n in ZONAS_TACTIL]
        self.log(f"\n  [{self.lado}] Mapa del tactil: {base_s:g} s SIN TOCAR la mano...")
        base, t_fin = [], self.reloj() + base_s
        while self.reloj() < t_fin:
            base.append(frame())
        self.log(f"  [{self.lado}] Ahora aprieta durante {segundos:g} s, uno detras de otro: la punta y la yema de cada dedo "
                 "(la cara de la palma) y la palma.")
        sube, t_fin, n = [np.zeros(n) for _, _, n in ZONAS_TACTIL], self.reloj() + segundos, 0
        b_med = [np.mean([f[z] for f in base], axis=0) for z in range(len(ZONAS_TACTIL))]
        quieto = [np.max([np.abs(f[z] - b_med[z]) for f in base], axis=0) for z in range(len(ZONAS_TACTIL))]
        crudos = []
        while self.reloj() < t_fin:
            fr = frame()
            crudos.append(np.concatenate(fr))
            for z in range(len(ZONAS_TACTIL)):
                sube[z] = np.maximum(sube[z], fr[z] - b_med[z])
            n += 1
        np.savez_compressed(os.path.join(self.reg.carpeta, f"tactil_{self.lado}.npz"), base=np.array(
            [np.concatenate(f) for f in base]), apretando=np.array(crudos))
        zonas = {}
        for z, (nombre, a, k) in enumerate(ZONAS_TACTIL):
            zonas[nombre] = {"registro": a, "base_media": round(float(b_med[z].mean()), 1),
                             "base_max": int(b_med[z].max()), "ruido_quieto_max": int(quieto[z].max()),
                             "sube_max": int(sube[z].max()), "taxel_que_mas_sube": int(np.argmax(sube[z]))}
        self.res["mapa_tactil"] = {"lecturas_base": len(base), "lecturas_apretando": n,
                                   "hz": round(n / segundos, 1), "zonas": zonas}
        lineas = [f"{nombre:15s} base {d['base_media']:7.1f} (max {d['base_max']:5d}), sin tocar varia {d['ruido_quieto_max']:4d}, "
                  f"apretando sube {d['sube_max']:5d}" for nombre, d in zonas.items()]
        self.evento("MAPA_TACTIL", f"{n} lecturas de las 17 zonas a {n / segundos:.1f} Hz:\n      " + "\n      ".join(lineas))

    def resumen(self):
        pasos = self.res["pasos"]
        self.res["ok"] = all(p["abre"] and not p.get("falso_lleno", False) for p in pasos)
        return self.res


def main():
    cfg = config.cargar()
    objetos = config.cargar_objetos()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manos", nargs="+", default=["izq", "der"], choices=["izq", "der"])
    ap.add_argument("--preformas", nargs="+", default=list(objetos["preformas"]), choices=list(objetos["preformas"]))
    ap.add_argument("--tactil", type=float, default=0.0, metavar="S", help="S segundos para apretar la yema del indice")
    ap.add_argument("--mapa-tactil", type=float, default=0.0, metavar="S",
                    help="las 17 zonas del tactil: 2 s sin tocar y S s apretando en varios sitios")
    ap.add_argument("--speed", type=int, default=None, help="SPEED_SET al cerrar (por defecto, mano.speed_cierre)")
    ap.add_argument("--presionar", type=int, nargs="+", default=None, metavar="A",
                    help="el indice en cada angulo A (p. ej. 1000 900 800 700) y apretar su yema --segundos")
    ap.add_argument("--segundos", type=float, default=8.0, help="--presionar: s para apretar en cada angulo")
    ap.add_argument("--sin-preformas", action="store_true", help="no cerrar ni abrir (p. ej. solo el mapa del tactil)")
    ap.add_argument("--solo-leer", action="store_true", help="conectar y leer, sin mover los dedos")
    ap.add_argument("--guardar-cierre", metavar="CARPETA",
                    help="no mueve nada: de un registro de cierres en el aire, escribe config/manos.yaml")
    ap.add_argument("--sim", action="store_true", help="con la mano simulada (sim/mano_simulada.py), en la PC")
    a = ap.parse_args()
    if a.guardar_cierre:
        cal = calibracion_de_registro(a.guardar_cierre)
        for lado, c in cal.items():
            print(f"  {lado}: tope en vacio {c['tope_vacio']}, residuo {c['residuo_gf']} gf" +
                  "".join(f"\n    - {n}" for n in c["notas"]))
        print(f"  -> {guardar_calibracion(cal, a.guardar_cierre)}")
        return
    if a.sim:
        cfg["mano"]["t_base_s"] = 0.5
    reg = Registro(getattr(a, "datos", None) or config.ruta_datos(cfg), "manos", "sim" if a.sim else None)
    reg.resumen["args"] = vars(a)
    cierres = force_set_de_preforma(objetos)
    print(f"\n  Manos {', '.join(a.manos)}: " + ("solo leer." if a.solo_leer else
          ("" if a.sin_preformas else f"preformas {', '.join(a.preformas)}, cerrar en el aire" +
           (f" a SPEED {a.speed}" if a.speed else "") + " y abrir") +
          (f"; tactil {a.tactil:g} s" if a.tactil else "") + (f"; mapa del tactil {a.mapa_tactil:g} s" if a.mapa_tactil else "")))
    manos = {}
    try:
        for lado in a.manos:
            cliente = None
            if a.sim:
                from sim.mano_simulada import ManoSimulada
                cliente = ManoSimulada()
            try:
                manos[lado] = Mano(lado, cfg, cliente=cliente)
                if a.speed:
                    manos[lado].speed_cierre = min(int(a.speed), 100)
                    # ~1.5 cuentas/s por unidad de SPEED en el robot: la espera del cierre crece al ir mas despacio
                    manos[lado].t_cerrar = max(manos[lado].t_cerrar, 1000 / (1.5 * manos[lado].speed_cierre) + 3)
            except Parada as ex:
                print(f"  [{lado}] {ex}")
        if not manos:
            sys.exit("  Ninguna mano contesta (red 192.168.124.x desde el PC2).")
        pruebas = {lado: Prueba(m, reg) for lado, m in manos.items()}
        for p in pruebas.values():
            p.leer_estado()
        if a.solo_leer:
            return
        if not a.sim:
            print("\n  Las manos se abren y se cierran (los brazos no se mueven): nada a menos de 10 cm de ellas y nadie "
                  "con los dedos dentro. Ctrl+C las abre.")
            if input(f"  Escribe {cfg['supervisor']['palabra']} para empezar: ").strip() != cfg["supervisor"]["palabra"]:
                print("Cancelado.")
                return
        for lado, p in pruebas.items():
            print(f"\n  --- mano {lado} ---")
            p.base_y_ruido()
            for nombre in ([] if a.sin_preformas else a.preformas):
                p.preforma_cerrar_abrir(nombre, objetos["preformas"][nombre], cierres.get(nombre))
            if a.tactil:
                p.apretar_yema(a.tactil)
            if a.mapa_tactil:
                p.mapa_tactil(a.mapa_tactil)
            if a.presionar:
                p.presionar(a.presionar, a.segundos)
            reg.resumen.setdefault("manos", {})[lado] = p.resumen()
    except KeyboardInterrupt:
        print("\n  Ctrl+C: abriendo las manos")
        reg.resumen["fin"] = "Ctrl+C"
    finally:
        for lado, m in manos.items():
            try:
                m.abrir()
            except Exception as ex:                         # una mano que no abre: que se vea
                print(f"  [{lado}] no se pudo abrir al terminar: {ex}")
            m.cerrar_conexion()
        r = reg.resumen.get("manos", {})
        reg.cerrar(ok={k: v["ok"] for k, v in r.items()})
    for lado, v in r.items():
        print(f"\n  mano {lado}: " + ("bien" if v["ok"] else "REVISAR") + " - " +
              "; ".join(f"{p['preforma']}: " + ("abre" if p["abre"] else "NO ABRE") +
                        (", falso lleno" if p.get("falso_lleno") else "") for p in v["pasos"]))
    print(f"\n  Registro en {reg.carpeta}")


if __name__ == "__main__":
    main()
