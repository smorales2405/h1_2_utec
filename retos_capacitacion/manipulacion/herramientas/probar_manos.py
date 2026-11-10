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
from manip.mano import DEDOS, PULGAR, TACTIL_INDICE, Mano, ManoSinRespuesta  # noqa: E402
from manip.mensajes import Parada  # noqa: E402
from manip.registro import Registro, fila_mano  # noqa: E402

NOMBRES_DOF = ["menique", "anular", "medio", "indice", "pulgar", "rotacion"]


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
            t0 = self.reloj()
            e = self.m.cerrar(fs)
            paso.update(objeto=obj, force_set=fs, t_cierre_s=round(self.reloj() - t0, 2),
                        cerrada=e.angulo.astype(int).tolist(), fuerza_neta=(e.fuerza - e.base).astype(int).tolist())
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

    def resumen(self):
        pasos = self.res["pasos"]
        self.res["ok"] = bool(pasos) and all(p["abre"] and not p.get("falso_lleno", False) for p in pasos)
        return self.res


def main():
    cfg = config.cargar()
    objetos = config.cargar_objetos()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manos", nargs="+", default=["izq", "der"], choices=["izq", "der"])
    ap.add_argument("--preformas", nargs="+", default=list(objetos["preformas"]), choices=list(objetos["preformas"]))
    ap.add_argument("--tactil", type=float, default=0.0, metavar="S", help="S segundos para apretar la yema del indice")
    ap.add_argument("--solo-leer", action="store_true", help="conectar y leer, sin mover los dedos")
    ap.add_argument("--sim", action="store_true", help="con la mano simulada (sim/mano_simulada.py), en la PC")
    a = ap.parse_args()
    if a.sim:
        cfg["mano"]["t_base_s"] = 0.5
    reg = Registro(getattr(a, "datos", None) or config.ruta_datos(cfg), "manos", "sim" if a.sim else None)
    reg.resumen["args"] = vars(a)
    cierres = force_set_de_preforma(objetos)
    print(f"\n  Manos {', '.join(a.manos)}: " + ("solo leer." if a.solo_leer else
          f"preformas {', '.join(a.preformas)}, cerrar en el aire y abrir" + (f"; tactil {a.tactil:g} s." if a.tactil else ".")))
    manos = {}
    try:
        for lado in a.manos:
            cliente = None
            if a.sim:
                from sim.mano_simulada import ManoSimulada
                cliente = ManoSimulada()
            try:
                manos[lado] = Mano(lado, cfg, cliente=cliente)
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
            for nombre in a.preformas:
                p.preforma_cerrar_abrir(nombre, objetos["preformas"][nombre], cierres.get(nombre))
            if a.tactil:
                p.apretar_yema(a.tactil)
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
