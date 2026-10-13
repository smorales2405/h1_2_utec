#!/usr/bin/env python3
"""Programa principal de la manipulacion (reto R40-RT-H1_2-0002). Se lanza con manipulacion.sh.

    ./manipulacion.sh --nivel 1 --simulacro            # hito 3: todo el lazo sin publicar en rt/arm_sdk
    ./manipulacion.sh --nivel 1                        # primeras tiradas: --escala-tiempo 2 por defecto
    ./manipulacion.sh --nivel 2 --asignacion tenis=A cubo=C cilindro=B prisma=D
    python3 manipulacion.py --nivel 2 --sim            # en la PC: robot, manos y camara simulados (gemelo)
    python3 manipulacion.py --nivel 2 --sim --visor    # lo mismo en una ventana de MuJoCo, a velocidad real

Pide escribir MOVER antes de mover nada (seccion 10). ESPACIO o Ctrl+C: salida segura. L2+B en el mando: la
parada de emergencia del operador. Registro en datos/tirada_<fecha>_nivel<N>[_simulacro]/.
"""

import argparse
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from manip import calibracion as cb  # noqa: E402
from manip import config  # noqa: E402
from manip import dianas as md  # noqa: E402
from manip.brazo import Brazo  # noqa: E402
from manip.cinematica import Cinematica  # noqa: E402
from manip.mano import Mano  # noqa: E402
from manip.mensajes import JUNTAS_BRAZO, LADOS  # noqa: E402
from manip.percepcion import Percepcion  # noqa: E402
from manip.planificacion import Planificador  # noqa: E402
from manip.registro import Registro  # noqa: E402
from manip.supervisor import Supervisor  # noqa: E402
from manip.vigilante import Vigilante  # noqa: E402


# --sim sin --colocacion/--asignacion: los cuatro objetos en la zona de recogida, cada uno a su bandeja
# (probada: los cuatro a su bandeja al primer intento; el cilindro, el mas alto, el primero)
COLOCACION_SIM = {"cilindro": ((0.42, 0.01), 0.0), "tenis": ((0.45, 0.10), 0.0),
                  "cubo": ((0.43, -0.07), 0.3), "prisma": ((0.50, -0.04), 1.57)}
ASIGNACION_SIM = {"tenis": "A", "cilindro": "B", "cubo": "C", "prisma": "D"}


class FsmFija:
    """En la simulacion de la PC no hay lee_fsm: la FSM es la del robot simulado."""

    def __init__(self, robot):
        self.robot = robot

    @property
    def fsm(self):
        return self.robot.leer_fsm()[0]

    def iniciar(self):
        return self.fsm

    def parar(self):
        pass


def leer_args(cfg, objetos):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--nivel", type=int, choices=[1, 2, 3, 4], required=True)
    ap.add_argument("--simulacro", action="store_true", help="no publica en rt/arm_sdk; la mano abre y cierra en el aire")
    ap.add_argument("--sim", action="store_true", help="en la PC: robot, manos y camara simulados")
    ap.add_argument("--escala-tiempo", type=float, default=2.0, help="duraciones x esto (2 en las primeras tiradas)")
    ap.add_argument("--dianas", nargs="*", default=None, help="nivel 1: x,y en el torso; si no, se detectan")
    ap.add_argument("--asignacion", nargs="*", default=None, help="clase=bandeja (por defecto, la de objetos.yaml)")
    ap.add_argument("--colocacion", nargs="*", default=None, help="--sim: clase=x,y,yaw de cada objeto en la mesa")
    ap.add_argument("--visor", action="store_true", help="--sim: ventana de MuJoCo, a velocidad real (o --velocidad)")
    ap.add_argument("--velocidad", type=float, default=1.0, help="--visor: x esto la velocidad real")
    ap.add_argument("--parada-en", default=None, metavar="ESTADO[+s]",
                    help="--sim: pide la salida segura s segundos despues de entrar en ESTADO (p. ej. TRANSPORTE+4)")
    ap.add_argument("--salida-directa", action="store_true",
                    help="salida segura del PDF tal cual: sin pasar por la observacion y el reposo (para comparar)")
    a = ap.parse_args()
    if (a.visor or a.parada_en) and not a.sim:
        sys.exit("--visor y --parada-en son de la simulacion de la PC: con --sim.")
    if a.sim and a.nivel >= 2 and not a.colocacion:
        a.colocacion = [f"{k}={xy[0]},{xy[1]},{yaw}" for k, (xy, yaw) in COLOCACION_SIM.items()]
    a.dianas = [tuple(float(v) for v in d.split(",")) for d in a.dianas] if a.dianas else None
    asign = {k: v["bandeja"] for k, v in objetos["objetos"].items() if v.get("bandeja")}
    if a.sim and not asign and not a.asignacion:
        asign = {k: b for k, b in ASIGNACION_SIM.items() if any(x.startswith(k + "=") for x in a.colocacion or [])}
    for x in a.asignacion or []:
        k, b = x.split("=")
        asign[k] = b
    a.asignacion = asign
    coloc = {}
    for x in a.colocacion or []:
        k, v = x.split("=")
        xs = [float(c) for c in v.split(",")]
        coloc[k] = ((xs[0], xs[1]), xs[2] if len(xs) > 2 else 0.0)
    a.colocacion = coloc
    if a.nivel >= 2 and not a.asignacion:
        sys.exit("niveles 2-4: falta la asignacion de bandejas (--asignacion clase=bandeja ...)")
    return a


class Tirada:
    """Las piezas de una tirada, conectadas: lo que arma `armar` y ejecuta `correr`."""
    pass


def armar(a, cfg=None, montaje=None, objetos=None, log=print):
    """Crea y conecta robot, brazo, manos, camara, percepcion, planificador, vigilante, registro y supervisor.
    Con a.sim, todo simulado en la PC (sim/simulacion.py); si no, el robot real (con a.simulacro, sin publicar)."""
    cfg = cfg or config.cargar()
    montaje = montaje or config.cargar_montaje()
    objetos = objetos or config.cargar_objetos()
    if getattr(a, "salida_directa", False):
        cfg["supervisor"]["salida_por_observacion"] = False
    t = Tirada()
    t.a, t.cfg, t.mundo, t.visor = a, cfg, None, None
    poses = config._leer(os.path.join(config.CONFIG, "poses_brazos.yaml"))
    observacion = {l: np.array(poses["observacion"][l]) for l in LADOS}
    cin = Cinematica.desde_config(cfg)
    if a.sim:
        os.environ.setdefault("MUJOCO_GL", "glfw")
        from sim.simulacion import FuenteMundo, MundoSimulado
        cal = cb.cargar_o_urdf(cin, {})
        t.mundo = MundoSimulado(cfg, montaje, objetos, cal, cin, a.colocacion or {})
        robot = t.mundo.robot
        fuente = FuenteMundo(t.mundo)
        # sin visor, tiempo acelerado (el brazo no espera); con visor, el reloj real x --velocidad. La mano va
        # siempre a tiempo real (decide que esta quieta por lo que cambia entre lecturas cada 50 ms): solo se
        # acorta la espera de la tara
        vel = max(0.1, float(getattr(a, "velocidad", 1.0))) if getattr(a, "visor", False) else None
        if vel:
            cfg["mano"]["t_base_s"] = cfg["mano"]["t_base_s"] / vel
        manos = {l: Mano(l, cfg, cliente=t.mundo.manos[l]) for l in LADOS}
        lector_fsm = FsmFija(robot)
        brazo = Brazo(robot, cfg, simulacro=True, destino=robot.recibir_armsdk, escala_tiempo=a.escala_tiempo,
                      dormir=(lambda s: time.sleep(s / vel)) if vel else (lambda s: None), sincrono=True)
        edad = fuente.edad
    else:
        from manip.fuentes import FuenteZmq, LectorContinuo
        from manip.robot import FSM_VALIDA, LectorFsm, Robot
        cal = cb.cargar_o_urdf(cin)
        if cal.origen != "calibrada" and not a.simulacro:
            sys.exit("Sin calibracion camara-torso (config/calibracion_*.yaml): solo --simulacro.")
        robot = Robot(cfg["red"]["iface"])
        if not robot.esperar_lowstate(3.0):
            sys.exit("No llega rt/lowstate.")
        e = robot.estado()
        if e.motores_en_fallo:
            sys.exit(f"Motores en fallo: {e.motores_en_fallo}")
        lector_fsm = LectorFsm(robot, cfg["supervisor"]["fsm_cada_s"])
        fsm = lector_fsm.iniciar()
        log(f"lowstate OK, mode_machine {e.mode_machine}, FSM {fsm}")
        if fsm != FSM_VALIDA:
            sys.exit(f"La FSM es {fsm}: tiene que ser {FSM_VALIDA} (L2+UP y R2+X en el mando).")
        fuente = LectorContinuo(FuenteZmq(cfg["red"]["zmq_fotogramas"]))
        if fuente.fresco(timeout_s=5.0) is None:
            sys.exit("No llegan fotogramas: ./camara_servidor.sh")
        manos = {l: Mano(l, cfg) for l in LADOS}
        brazo = Brazo(robot, cfg, simulacro=a.simulacro, escala_tiempo=a.escala_tiempo)
        edad = fuente.edad
    q = robot.q()
    reposo = {l: q[JUNTAS_BRAZO[l]] for l in LADOS}
    # gemelo para comprobar los planes, con la postura medida
    from sim.colisiones import Colisiones
    from sim.escena import Estado, construir
    m, info = construir(montaje=montaje, objetos=objetos, cal=cal, cfg=cfg, postura=list(q), con_camaras=False)
    est = Estado(m, info)
    pl = Planificador(cfg, montaje, objetos, cin, (est, Colisiones(m, est, cfg["planificacion"]["holgura_m"])),
                      reposo, observacion, a.escala_tiempo)
    per = Percepcion(cfg, montaje, objetos, cal, cin)
    vig = Vigilante(robot, cfg, brazo, manos, lector_fsm, edad_fotograma=edad)
    if a.simulacro or a.sim:
        vig.err_max = float("inf")          # el brazo no se mueve (simulacro) o va siempre a su consigna (sim)
    nombre = f"nivel{a.nivel}" + ("_simulacro" if a.simulacro else "") + ("_sim" if a.sim else "")
    reg = Registro(getattr(a, "datos", None) or config.ruta_datos(cfg), "tirada", nombre)
    sup = Supervisor(cfg, robot, brazo, manos, per, pl, fuente, vig, reg, nivel=a.nivel, simulacro=a.simulacro or a.sim,
                     dianas=a.dianas, asignacion=a.asignacion,
                     detector_dianas=lambda f: md.detectar(f, cal, montaje), log=log)
    reg.enganchar_lowstate(robot, sup.contexto_registro, cada=10)
    reg.resumen.update({"args": {k: v for k, v in vars(a).items()}, "calibracion": f"{cal.origen} {cal.fecha}",
                        "escala_tiempo": a.escala_tiempo, "reposo": {l: v.round(4).tolist() for l, v in reposo.items()}})
    t.robot, t.brazo, t.manos, t.fuente, t.lector_fsm, t.vig, t.reg, t.sup, t.cal = \
        robot, brazo, manos, fuente, lector_fsm, vig, reg, sup, cal
    if getattr(a, "visor", False):
        from sim.visor import Visor
        t.visor = Visor(t.mundo, texto=lambda: _texto_visor(t), al_espacio=lambda: vig.pedir_parada("tecla ESPACIO (visor)"))
    return t


def _texto_visor(t):
    sup = t.sup
    salida = "observacion -> reposo -> peso 0" if t.cfg["supervisor"].get("salida_por_observacion", True) else \
        "directa (peso 0 tras subir)"
    return ("estado\n\npeso arm_sdk\nsalida segura\nvelocidad",
            f"{sup.estado}\n{sup.motivo_estado[:70]}\n{t.brazo.peso:.2f}\n{salida}\nx{t.a.velocidad:g}")


def _parada_en(t, orden):
    """--parada-en ESTADO[+s]: un hilo que pide la salida segura s segundos (de simulacion) despues de que el
    supervisor entre en ESTADO por primera vez."""
    estado, _, s = orden.partition("+")
    estado, s = estado.strip().upper(), float(s or 0.0)
    vel = t.a.velocidad if getattr(t.a, "visor", False) else None

    def vigilar():
        while t.sup.estado != estado:
            if t.sup.estado == "FIN":
                return
            time.sleep(0.005)
        if vel:
            time.sleep(s / vel)
        print(f"\n  [prueba] salida segura pedida {s:g} s despues de entrar en {estado}")
        t.vig.pedir_parada(f"prueba: --parada-en {orden}")
    threading.Thread(target=vigilar, daemon=True).start()


def correr(t, confirmar=True):
    """Pide la palabra (salvo en la PC), ejecuta la tirada y cierra todo. Devuelve el motivo de fin."""
    a, cfg = t.a, t.cfg
    print(f"\n  Nivel {a.nivel}{' en SIMULACRO (no se publica en rt/arm_sdk)' if a.simulacro else ''}"
          f"{' SIMULADO en la PC' if a.sim else ''}; duraciones x{a.escala_tiempo}.")
    if confirmar and not a.sim:
        print("  Robot de pie en FSM 201, nadie al alcance de los brazos, operador con L2+B. ESPACIO: salida segura.")
        if input(f"  Escribe {cfg['supervisor']['palabra']} para empezar: ").strip() != cfg["supervisor"]["palabra"]:
            print("Cancelado.")
            t.brazo.parar()
            t.reg.cerrar(motivo="cancelado")
            return "cancelado"
    if t.visor is not None:
        input("  Visor abierto (raton: girar y acercar; ESPACIO en la ventana o aqui: salida segura). "
              "Enter para empezar: ")
    if getattr(a, "parada_en", None):
        _parada_en(t, a.parada_en)
    from manip.teclado import Teclado
    teclado = Teclado(lambda: t.vig.pedir_parada("tecla ESPACIO"))
    teclado.abrir()
    t0 = time.monotonic()
    try:
        motivo = t.sup.ejecutar()
    finally:
        teclado.cerrar()
        try:
            t.brazo.parar()
        except RuntimeError as e:
            print(f"ATENCION: {e}")
        for mano in t.manos.values():
            mano.cerrar_conexion()
        if hasattr(t.fuente, "cerrar"):
            t.fuente.cerrar()
        if t.lector_fsm is not None:
            t.lector_fsm.parar()
    final = {k: t.mundo.bandeja_de(k) for k in t.mundo.poses} if t.mundo is not None else None
    t.reg.cerrar(motivo=motivo, resultados=t.sup.resultados, duracion_tirada_s=round(time.monotonic() - t0, 1),
                 objetos_al_final=final)
    print(f"\n{motivo}. Resultados: {t.sup.resultados}\nRegistro: {t.reg.carpeta}")
    if t.mundo is not None:
        print("Objetos al final:", final)
        if t.visor is not None:
            from sim.visor import informe_choques
            print("Visor:", informe_choques(t.visor.choques))
            if t.visor.abierto():
                print("Cierra la ventana del visor para terminar.")
                t.visor.esperar_cierre()
            t.visor.cerrar()
        t.mundo.parar()
    return motivo


def main():
    cfg = config.cargar()
    objetos = config.cargar_objetos()
    a = leer_args(cfg, objetos)
    correr(armar(a, cfg, objetos=objetos))


if __name__ == "__main__":
    main()
