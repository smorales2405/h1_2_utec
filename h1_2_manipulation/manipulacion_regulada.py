#!/usr/bin/env python3
"""Manipulacion con el regulador de fuerza (README). En el robot se lanza con banco.sh o manipulacion_regulada.sh.

    ./banco.sh --banco cubo --ensayos 20 --arnes            # el banco: regulador y firmware intercalados
    ./manipulacion_regulada.sh --nivel 2 --arnes            # el reto con la mano regulada
    ./manipulacion_regulada.sh --nivel 2 --arnes --brazo firmware   # ... o con la del reto (como manipulacion.sh)
    python3 manipulacion_regulada.py --banco cubo --ensayos 4 --sim  # en la PC, en el mundo simulado
    python3 manipulacion_regulada.py --nivel 2 --sim

Opciones propias:
    --banco OBJETO     el banco con ese objeto (cubo, esfera, cilindro, prisma; el prisma de pie o tumbado)
    --ensayos N        ensayos del banco (por defecto 20: 10 por brazo)
    --brazos A,B       los brazos que se intercalan (regulador,firmware)
    --semilla S        del orden de los brazos (por defecto, la hora; queda en el registro)
    --sostener-s S     sobre la bandeja, antes de devolverlo (por defecto 3)
    --sin-recolocar    no pedir ponerlo sobre la cruz antes de cada ensayo (solo si se cae o no se agarra)
    --tol-sitio-mm M   con el objeto a mas de M mm de su sitio, se pide recolocarlo (por defecto 5)
    --tol-sitio-deg G  ... o girado mas de G grados (por defecto 5)
    --brazo B          sin --banco: regulador (por defecto) o firmware
    --regulador-config RUTA   otra config/regulador.yaml
El resto (--arnes, --escala-tiempo, --sin-comprobar-montaje, --sim, --asignacion, --verificar-planes...) pasa tal
cual al manipulacion.py del reto, que es quien arma y ejecuta la tirada: de el se usan el robot, los brazos, la
camara, la percepcion, el planificador, el vigilante y el supervisor sin cambiar nada. Lo unico distinto es la mano
(agarre/mano_regulada.py) y, con --banco, el bucle de ensayos (agarre/banco.py).
Registro en datos/banco_<fecha>_<objeto>/ o datos/regulada_<fecha>_nivel<N>_<brazo>/ de esta carpeta.
"""

import argparse
import functools
import math
import os
import sys
import threading
import time

import numpy as np

AQUI =os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import agarre  # noqa: E402  (pone el reto y el regulador en sys.path)
import manipulacion as reto  # noqa: E402  (el manipulacion.py del reto)
from manip import config  # noqa: E402
from manip.registro import Registro  # noqa: E402

from agarre.banco import BRAZOS, SupervisorBanco  # noqa: E402
from agarre.mano_regulada import ManoRegulada, cargar_config  # noqa: E402

DATOS = os.path.join(AQUI, "datos")


def leer_mios(argv):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--banco", metavar="OBJETO", default=None)
    ap.add_argument("--ensayos", type=int, default=20)
    ap.add_argument("--brazos", default=",".join(BRAZOS))
    ap.add_argument("--semilla", type=int, default=None)
    ap.add_argument("--sostener-s", type=float, default=3.0)
    ap.add_argument("--racha-max", type=int, default=2)
    ap.add_argument("--sin-recolocar", action="store_true")
    ap.add_argument("--tol-sitio-mm", type=float, default=5.0)
    ap.add_argument("--tol-sitio-deg", type=float, default=5.0)
    ap.add_argument("--brazo", choices=list(BRAZOS), default="regulador")
    ap.add_argument("--regulador-config", default=None)
    ap.add_argument("-h", "--help", action="store_true")
    mios, resto = ap.parse_known_args(argv)
    if mios.help:
        print(__doc__)
        sys.exit(0)
    return mios, resto


def preparar(argv, cfg, objetos):
    """(args del reto, los mios): los del reto con lo que pide el banco (nivel 2, solo su objeto)."""
    mios, resto = leer_mios(argv)
    if mios.banco:
        o = objetos["objetos"].get(mios.banco)
        if o is None:
            sys.exit(f"--banco: '{mios.banco}' no esta en objetos.yaml ({', '.join(objetos['objetos'])})")
        mios.banco = o.get("variante_de", mios.banco)
        if "--nivel" not in resto:
            resto += ["--nivel", "2"]
        if "--sim" in resto and "--colocacion" not in resto:
            (xy, yaw) = reto.COLOCACION_SIM[mios.banco]
            resto += ["--colocacion", f"{mios.banco}={xy[0]},{xy[1]},{yaw}"]
    sys.argv = [sys.argv[0]] + resto
    a = reto.leer_args(cfg, objetos)
    if mios.banco:
        grupo = lambda c: (objetos["objetos"].get(c) or {}).get("variante_de", c)   # noqa: E731
        a.asignacion = {c: b for c, b in a.asignacion.items() if grupo(c) == mios.banco}
        if not a.asignacion:
            sys.exit(f"--banco {mios.banco}: sin bandeja (objetos.yaml o --asignacion {mios.banco}=...)")
        a.verificar_planes = False
    return a, mios


def armar(a, mios, cfg, objetos, log=print, datos=None):
    """La tirada del reto (manipulacion.armar) con la mano regulada y, con --banco, el supervisor del banco. Se cambian
    los nombres que usa armar (la clase de la mano, del supervisor, del registro y del mundo simulado) mientras arma;
    el codigo del reto no se toca."""
    rc = cargar_config(mios.regulador_config) if mios.regulador_config else cargar_config()
    sim = bool(getattr(a, "sim", False))
    if mios.banco:
        tipo, nombre = "banco", mios.banco
    else:
        tipo, nombre = "regulada", f"nivel{a.nivel}_{mios.brazo}"
    nombre += "_sim" if sim else ("_simulacro" if getattr(a, "simulacro", False) else "")
    a.datos = datos or getattr(a, "datos", None) or DATOS
    viejos = {k: getattr(reto, k) for k in ("Mano", "Supervisor", "Registro")}
    reto.Mano = functools.partial(ManoRegulada, regulador=rc, log=log)
    reto.Registro = lambda base, _tipo, _nombre: Registro(base, tipo, nombre)
    if mios.banco:
        reto.Supervisor = SupervisorBanco
    mundo = None
    if sim:
        import sim.simulacion as simulacion
        from agarre.sim_mano import mundo_fisico
        mundo = simulacion.MundoSimulado
        simulacion.MundoSimulado = mundo_fisico()
    try:
        t = reto.armar(a, cfg, objetos=objetos, log=log)
    finally:
        for k, v in viejos.items():
            setattr(reto, k, v)
        if mundo is not None:
            simulacion.MundoSimulado = mundo
    for m in t.manos.values():
        m.sup, m.carpeta, m.log = t.sup, t.reg.carpeta, log
        m.regular = mios.brazo == "regulador"
    t.reg.resumen["regulador"] = {"config": rc, "brazo": None if mios.banco else mios.brazo,
                                  "origen": "regulador/ORIGEN.md (inspire_hand_interface fb858ca)"}
    if mios.banco:
        t.sup.banco.update({"objeto": mios.banco, "ensayos": mios.ensayos,
                            "brazos": [x.strip() for x in mios.brazos.split(",") if x.strip()],
                            "semilla": mios.semilla if mios.semilla is not None else int(time.time()) % 100000,
                            "racha_max": mios.racha_max, "sostener_s": mios.sostener_s,
                            "recolocar": not mios.sin_recolocar, "tol_sitio_mm": mios.tol_sitio_mm,
                            "tol_sitio_deg": mios.tol_sitio_deg})
    cerrar = t.reg.cerrar

    def cerrar_con_agarres(**kw):            # el resumen de cada agarre (y del banco) en resumen.json
        kw["agarres"] = {l: m.agarres for l, m in t.manos.items()}
        if mios.banco:
            kw["banco"] = t.sup.banco
            kw["ensayos"] = [{k: v for k, v in r.items() if not k.startswith("_")} for r in t.sup.ensayos]
        return cerrar(**kw)
    t.reg.cerrar = cerrar_con_agarres
    return t


def operador_simulado(t):
    """En el mundo simulado, quien recoloca el objeto del banco: lo pone donde estaba al empezar (la "cruz" de la
    simulacion), con el mismo giro."""
    sup, mundo = t.sup, t.mundo
    clase = next((c for c in mundo.poses if sup._grupo(c) == sup.banco["objeto"]), sup.banco["objeto"])
    T0 = np.array(mundo.donde(clase), float)
    xy, yaw = T0[:2, 3].copy(), math.atan2(T0[1, 0], T0[0, 0])

    def esperar(texto):
        with mundo.cerrojo:
            mundo.poner(clase, xy, yaw)
        return True
    return esperar


def correr(t, confirmar=True):
    """manipulacion.correr del reto; con --banco, ademas, Enter para seguir tras recolocar el objeto (q: acabar). En
    la simulacion lo recoloca operador_simulado (salvo que ya haya otro, el de una prueba)."""
    if not isinstance(t.sup, SupervisorBanco):
        return reto.correr(t, confirmar=confirmar)
    if t.mundo is not None and t.sup.esperar_usuario is None:
        t.sup.esperar_usuario = operador_simulado(t)
    tecla, enter, acabar = None, threading.Event(), threading.Event()

    def al_teclear(ch):
        if ch in "\r\n":
            enter.set()
        elif ch in "qQ":
            acabar.set()

    def esperar(texto):
        print(f"\n  >>> {texto}. Enter: seguir; q: acabar el banco; ESPACIO: salida segura.", flush=True)
        enter.clear()
        acabar.clear()
        while not enter.is_set():
            t.sup.comprobar()                        # el vigilante y ESPACIO siguen mandando
            if acabar.is_set():
                return False
            time.sleep(0.05)
        return True
    from manip import teclado
    original = teclado.Teclado

    class TecladoBanco(original):
        def __init__(self, al_pulsar, **kw):
            kw.setdefault("al_teclear", al_teclear)
            super().__init__(al_pulsar, **kw)
            if self.activo and t.mundo is None:
                t.sup.esperar_usuario = esperar          # sin terminal (la PC, pytest) no se puede esperar a nadie
    teclado.Teclado = TecladoBanco
    try:
        return reto.correr(t, confirmar=confirmar)
    finally:
        teclado.Teclado = original


def informe(t):
    """Al acabar el banco: cada brazo, cuantos sostuvo y como."""
    if not isinstance(t.sup, SupervisorBanco) or not t.sup.ensayos:
        return
    from analisis.comparar import tabla
    print("\n" + tabla(t.sup.ensayos))


def main():
    cfg = config.cargar()
    objetos = config.cargar_objetos()
    a, mios = preparar(sys.argv[1:], cfg, objetos)
    t = armar(a, mios, cfg, objetos)
    correr(t)
    informe(t)


if __name__ == "__main__":
    main()
