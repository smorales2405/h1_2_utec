#!/usr/bin/env python3
"""Compara juegos de ganancias del brazo con el robot de pie (FSM 201, rt/arm_sdk): la misma secuencia de poses
(config/secuencia_ganancias.yaml: reposo -> observacion -> las poses de alcanzar, coger y soltar de la tirada
simulada del nivel 2 -> observacion -> reposo) con cada juego, registrando consigna, medida e IMU.

    ./ejecutar.sh herramientas/comparar_ganancias.py --simulacro --objetos cilindro   # sin publicar: nada se mueve
    ./ejecutar.sh herramientas/comparar_ganancias.py                            # actual y luego tuned
    ./ejecutar.sh herramientas/comparar_ganancias.py --ganancias tuned actual   # en otro orden
    ./ejecutar.sh herramientas/comparar_ganancias.py --objetos cilindro --escala-tiempo 1
    python3 herramientas/comparar_ganancias.py --sim --visor --velocidad 4       # en la PC, sin robot

Juegos: "actual" (lo que usa manipulacion.py: brazo.kp/kd de config/manipulacion.yaml) o uno de
h1_2_joint_control/config/gains.yaml (tuned, official_arm_sdk, xr_teleoperate...; los *_gff no: piden la
compensacion de gravedad y aqui tau = 0). La cintura se sujeta en la postura medida con la ganancia del juego.
Por decision del usuario (2026-10-14) aqui NO se aplica el tope kp <= 150 de la seccion 10 del PDF; solo en
esta herramienta: manipulacion.py lo sigue aplicando.

Cada juego es una pasada completa: peso 0 -> 1 (12 s), los tramos (cada uno con la duracion de la tirada por
la escala de tiempo, y --pausa s quieto al final), vuelta al reposo medido y peso 1 -> 0 (5 s). Entre juegos
el robot queda con el control de Unitree y se pide Enter.

Parada en cualquier momento: ESPACIO o Ctrl+C en el terminal (o el --boton del mando). Para sola si da un
paso, gira mas de 5 grados, se inclina mas de 10, el error del brazo pasa de err_seguimiento_max_rad, un motor
falla, se pierde rt/lowstate o la FSM deja de ser 201 (manip/vigilante.py). La parada deja la consigna donde
esta, lleva los dos brazos en linea recta a la observacion (comprobado en el gemelo: sin choques con el cuerpo;
con la mesa delante NO vale), luego al reposo medido, y baja el peso a 0: la postura de Motion Mode. L2+B sigue
siendo la parada de emergencia.

Registro: datos/ganancias_<fecha>_<juego>/ con lowstate.csv (rt/lowstate a --hz-registro: IMU, mando, las 27
juntas con q, dq, par, temperatura y estado, la consigna 12-26, el peso y el tramo en la columna estado),
eventos.csv y resumen.json (ganancias, tramos y metricas). Se analiza en la PC con analizar_ganancias.py.
"""

import argparse
import math
import os
import signal
import sys
import threading
import time

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from manip import config  # noqa: E402
from manip.brazo import BRAZOS, JUNTAS, Brazo  # noqa: E402
from manip.mando import BOTONES  # noqa: E402
from manip.mensajes import CINTURA, JUNTAS_BRAZO, LADOS, PIERNAS, Parada  # noqa: E402
from manip.registro import NOMBRES, Registro  # noqa: E402
from manip.vigilante import Vigilante  # noqa: E402

RUTA_SECUENCIA = os.path.join(config.CONFIG, "secuencia_ganancias.yaml")
RUTA_GAINS = "h1_2_joint_control/config/gains.yaml"       # dentro del repo (en el PC2, en modelo/: desplegar.sh)
JUNTAS_GAINS = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]


def leer_args():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ganancias", nargs="+", default=["actual", "tuned"], help="juegos, en este orden")
    ap.add_argument("--escala-tiempo", type=float, default=2.0, help="duraciones x esto (como manipulacion.py)")
    ap.add_argument("--pausa", type=float, default=2.0, help="s quieto al final de cada tramo (error estable)")
    ap.add_argument("--objetos", nargs="*", default=None, help="solo los tramos de estos objetos (y la ida y la vuelta)")
    ap.add_argument("--hz-registro", type=float, default=100.0, help="filas por segundo de lowstate.csv (<= 500)")
    ap.add_argument("--boton", default=None, choices=BOTONES, help="boton del mando que tambien para (uno sin funcion en FSM 201)")
    ap.add_argument("--secuencia", default=RUTA_SECUENCIA)
    ap.add_argument("--simulacro", action="store_true", help="en el robot, sin publicar en rt/arm_sdk: nada se mueve")
    ap.add_argument("--sim", action="store_true", help="en la PC: robot simulado (no depende de las ganancias)")
    ap.add_argument("--visor", action="store_true", help="--sim: ventana de MuJoCo")
    ap.add_argument("--velocidad", type=float, default=1.0, help="--sim: x esto la velocidad real")
    ap.add_argument("--parada-en", default=None, metavar="N[+s]", help="--sim: pide la parada s s despues de empezar el tramo N")
    a = ap.parse_args()
    if (a.visor or a.parada_en or a.velocidad != 1.0) and not a.sim:
        sys.exit("--visor, --velocidad y --parada-en son de la simulacion de la PC: con --sim.")
    if a.sim and a.simulacro:
        sys.exit("--sim (en la PC) o --simulacro (en el robot), no los dos.")
    return a


def leer_juego(nombre):
    """{junta: (kp, kd)} de las 15 juntas de arm_sdk (12-26) de un juego de gains.yaml."""
    ruta = os.path.join(config.ruta_repo(), RUTA_GAINS)
    if not os.path.exists(ruta):
        sys.exit(f"No esta {ruta} (en el PC2 lo copia ./desplegar.sh)")
    juegos = config._leer(ruta)["sets"]
    if nombre not in juegos:
        sys.exit(f"'{nombre}' no esta en {RUTA_GAINS}: actual, {', '.join(juegos)}")
    if nombre.endswith("_gff"):
        sys.exit(f"'{nombre}' necesita la compensacion de gravedad (tau): rt/arm_sdk va aqui con tau = 0")
    J = juegos[nombre]["joints"]
    sal = {CINTURA: (float(J["waist_yaw"]["kp"]), float(J["waist_yaw"]["kd"]))}
    for lado, pre in (("izq", "L_"), ("der", "R_")):
        for j, n in zip(JUNTAS_BRAZO[lado], JUNTAS_GAINS):
            sal[j] = (float(J[pre + n]["kp"]), float(J[pre + n]["kd"]))
    return sal


def leer_secuencia(ruta, objetos=None):
    sec = config._leer(ruta)
    tramos = sec["tramos"]
    if objetos:
        desconocidos = set(objetos) - {t["objeto"] for t in tramos if t["objeto"]}
        if desconocidos:
            sys.exit(f"--objetos: {sorted(desconocidos)} no estan en la secuencia")
        # cada objeto empieza y acaba en la observacion: quitar los de otros objetos no deja saltos
        tramos = [t for t in tramos if t["fase"] != "trabajo" or t["objeto"] in objetos]
    if tramos[0]["fase"] != "ida" or tramos[-1]["fase"] != "vuelta" or not tramos[-1].get("reposo"):
        sys.exit(f"{ruta}: tiene que empezar con la ida a la observacion y acabar con la vuelta al reposo")
    return sec, tramos


def grados(x):
    return math.degrees(float(x))


class Metricas:
    """Lo que se mide en cada tramo, con las muestras del ciclo del brazo (50 Hz): error de seguimiento (medida
    menos consigna) en movimiento, al llegar y tras la pausa; IMU y piernas respecto al inicio de la pasada; par."""

    def __init__(self, robot, brazo, e0):
        self.robot, self.brazo, self.e0 = robot, brazo, e0
        self.nuevo(0, "", 0.0)

    def nuevo(self, n, nombre, T):
        self.n, self.nombre, self.T = n, nombre, T
        self.err_mov = np.zeros(len(BRAZOS))
        self.err_llegar = None
        self.ultimo = np.zeros(len(BRAZOS))
        self.roll = self.pitch = self.gyro = self.pierna = 0.0
        self.tau = np.zeros(len(BRAZOS))
        self.yaw = 0.0

    def muestra(self, moviendo):
        e = self.robot.estado()
        obj = self.brazo.consigna()
        err = np.array([e.q[j] - obj[j] for j in BRAZOS])
        if moviendo:
            self.err_mov = np.maximum(self.err_mov, np.abs(err))
        self.ultimo = err
        self.roll = max(self.roll, abs(e.imu.roll - self.e0.imu.roll))
        self.pitch = max(self.pitch, abs(e.imu.pitch - self.e0.imu.pitch))
        self.yaw = (e.imu.yaw - self.e0.imu.yaw + math.pi) % (2 * math.pi) - math.pi
        self.gyro = max(self.gyro, math.hypot(e.imu.gx, e.imu.gy, e.imu.gz))
        self.pierna = max(self.pierna, float(np.abs(e.q[PIERNAS] - self.e0.q[PIERNAS]).max()))
        self.tau = np.maximum(self.tau, np.abs(e.tau[BRAZOS]))

    def llegar(self):
        self.err_llegar = self.ultimo.copy()

    def cerrar(self):
        def peor(v):
            k = int(np.argmax(np.abs(v)))
            return round(grados(abs(v[k])), 2), NOMBRES[BRAZOS[k]]
        llegar = self.err_llegar if self.err_llegar is not None else self.ultimo
        return {"n": self.n, "nombre": self.nombre, "T_s": round(self.T, 2),
                "err_mov_max_deg": peor(self.err_mov), "err_al_llegar_deg": peor(llegar),
                "err_estable_deg": peor(self.ultimo),
                "err_estable_por_junta_deg": {NOMBRES[j]: round(grados(v), 2) for j, v in zip(BRAZOS, self.ultimo)},
                "roll_max_deg": round(grados(self.roll), 2), "pitch_max_deg": round(grados(self.pitch), 2),
                "yaw_deg": round(grados(self.yaw), 2), "gyro_max_deg_s": round(grados(self.gyro), 1),
                "pierna_max_deg": round(grados(self.pierna), 2),
                "tau_max_nm": [round(float(self.tau.max()), 1), NOMBRES[BRAZOS[int(np.argmax(self.tau))]]]}


def linea(m):
    return (f"{m['T_s']:5.1f} s | err mov {m['err_mov_max_deg'][0]:4.1f} ({m['err_mov_max_deg'][1]}) | llegar "
            f"{m['err_al_llegar_deg'][0]:4.1f} | estable {m['err_estable_deg'][0]:4.1f} ({m['err_estable_deg'][1]}) | "
            f"roll {m['roll_max_deg']:3.1f} pitch {m['pitch_max_deg']:3.1f} yaw {m['yaw_deg']:+4.1f} | "
            f"pierna {m['pierna_max_deg']:3.1f} | tau {m['tau_max_nm'][0]:4.1f} Nm")


class Prueba:
    def __init__(self, a, cfg):
        self.a, self.cfg = a, cfg
        self.mundo = self.visor = None
        self.vel = max(0.1, a.velocidad) if a.sim else 1.0
        if a.sim:
            from sim.robot_simulado import RobotSimulado
            self.robot = RobotSimulado(tau_s=0.25 / self.vel)
            if a.visor:
                os.environ.setdefault("MUJOCO_GL", "glfw")
                from manip import calibracion as cb
                from manip.cinematica import Cinematica
                from sim.simulacion import MundoSimulado
                cin = Cinematica.desde_config(cfg)
                self.mundo = MundoSimulado(cfg, config.cargar_montaje(), config.cargar_objetos(), cb.cargar_o_urdf(cin, {}), cin, {})
                self.mundo.robot.parar()
                self.mundo.robot = self.robot          # el visor dibuja este, que va con retraso y se queda corto
            self.lector_fsm = None
            self.brazo = Brazo(self.robot, cfg, simulacro=True, destino=self.robot.recibir_armsdk,
                               escala_tiempo=a.escala_tiempo, dormir=lambda s: time.sleep(s / self.vel), sincrono=True)
        else:
            from manip.robot import FSM_VALIDA, LectorFsm, Robot
            self.robot = Robot(cfg["red"]["iface"])
            if not self.robot.esperar_lowstate(3.0):
                sys.exit("No llega rt/lowstate.")
            e = self.robot.estado()
            if e.motores_en_fallo:
                sys.exit(f"Motores en fallo: {e.motores_en_fallo}")
            self.lector_fsm = LectorFsm(self.robot, cfg["supervisor"]["fsm_cada_s"])
            fsm = self.lector_fsm.iniciar()
            print(f"lowstate OK, mode_machine {e.mode_machine}, FSM {fsm}")
            if fsm != FSM_VALIDA:
                sys.exit(f"La FSM es {fsm}: tiene que ser {FSM_VALIDA} (L2+UP y R2+X en el mando).")
            self.brazo = Brazo(self.robot, cfg, simulacro=a.simulacro, escala_tiempo=a.escala_tiempo)
        self.dormir = self.brazo._dormir
        # "actual": lo que pone Brazo con config/manipulacion.yaml (con su tope de 150)
        self.juegos = {"actual": {j: (self.brazo.kp[j], self.brazo.kd[j]) for j in JUNTAS}}
        for n in a.ganancias:
            if n != "actual":
                self.juegos[n] = leer_juego(n)
        self.vig = Vigilante(self.robot, cfg, self.brazo, manos={}, lector_fsm=self.lector_fsm, edad_fotograma=None)
        self.vig.boton = a.boton
        if a.simulacro:
            self.vig.err_max = float("inf")     # el brazo no sigue la consigna: no se publica
        self.sec, self.tramos = leer_secuencia(a.secuencia, a.objetos)
        self.observacion = {l: np.array(self.sec["observacion"][l]) for l in LADOS}
        self.tramo = "ESPERA"
        self.fase = None
        if self.mundo is not None:
            from sim.visor import Visor
            self.visor = Visor(self.mundo, texto=lambda: ("tramo\npeso arm_sdk\nvelocidad",
                                                          f"{self.tramo}\n{self.brazo.peso:.2f}\nx{self.vel:g}"),
                               al_espacio=lambda: self.vig.pedir_parada("tecla ESPACIO (visor)"))

    # --- utilidades -------------------------------------------------------------------------------------
    def pose(self, q_izq, q_der):
        p = self.brazo.pose_brazo("izq", q_izq)
        p.update(self.brazo.pose_brazo("der", q_der))
        return p

    def duracion_estimada(self):
        b = self.brazo
        prev = {j: float(q) for j, q in zip(BRAZOS, self.robot.q()[BRAZOS])}
        total = b.t_entrada + b.t_salida
        for tr in self.tramos:
            fin = prev if tr.get("reposo") else self.pose(tr["izq"], tr["der"])
            total += b.duracion(prev, fin, tr["base_s"]) + self.a.pausa
            prev = fin
        return total

    def quieto(self, segundos, chequeo):
        for _ in range(max(1, int(segundos * self.brazo.hz))):
            chequeo()
            self.dormir(1.0 / self.brazo.hz)

    def grave(self):
        if self.robot.edad_lowstate() > self.cfg["supervisor"]["lowstate_max_s"]:
            raise Parada("rt/lowstate perdido en la parada")
        if self.robot.estado().motores_en_fallo:
            raise Parada("motor en fallo en la parada")

    def bajar_peso(self):
        """Peso a 0 sin que un Ctrl+C lo corte a medias."""
        viejo = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            self.brazo.rampa_peso(0.0)
        finally:
            signal.signal(signal.SIGINT, viejo)

    # --- una pasada -------------------------------------------------------------------------------------
    def pasada(self, nombre):
        a, brazo, robot = self.a, self.brazo, self.robot
        ganancias = self.juegos[nombre]
        with brazo.cerrojo:                  # con el peso en 0: no cambia nada de lo que hace el robot
            brazo.kp = {j: ganancias[j][0] for j in JUNTAS}
            brazo.kd = {j: ganancias[j][1] for j in JUNTAS}
        reg = Registro(config.ruta_datos(self.cfg), "ganancias", nombre + ("_sim" if a.sim else "") +
                       ("_simulacro" if a.simulacro else ""))
        reg.enganchar_lowstate(robot, lambda: {"estado": self.tramo, "peso": brazo.peso, "obj": brazo.consigna()},
                               cada=max(1, round(500.0 / a.hz_registro)))
        msg0, _ = robot.ultimo()
        e0 = robot.estado()
        reposo = {l: e0.q[JUNTAS_BRAZO[l]].copy() for l in LADOS}
        reg.resumen.update({"juego": nombre, "ganancias": {NOMBRES[j]: list(g) for j, g in sorted(ganancias.items())},
                            "args": vars(a), "secuencia": os.path.relpath(a.secuencia, RAIZ), "n_tramos": len(self.tramos),
                            "reposo_medido": {l: v.round(4).tolist() for l, v in reposo.items()},
                            "imu_inicial_deg": [round(grados(x), 2) for x in (e0.imu.roll, e0.imu.pitch, e0.imu.yaw)],
                            "temperatura_inicial": {NOMBRES[j]: int(msg0.motor_state[j].temperature[0]) for j in JUNTAS}})
        met = Metricas(robot, brazo, e0)
        metricas = []
        print(f"\n=== Pasada con '{nombre}': {len(self.tramos)} tramos, ~{self.duracion_estimada() / 60 / self.vel:.1f} min."
              f" ESPACIO o Ctrl+C: parada. Registro: {reg.carpeta}")

        def chequeo_mov():
            self.vig.comprobar()
            met.muestra(True)

        def chequeo_pausa():
            self.vig.comprobar()
            met.muestra(False)

        motivo = "completado"
        try:
            self.vig.iniciar()
            self.tramo, self.fase = "ENTRADA", "entrada"
            reg.evento("ENTRADA", f"peso 0 -> 1 en {brazo.t_entrada:.0f} s siguiendo la postura medida")
            brazo.congelar()
            brazo.rampa_peso(1.0, comprobar=self.vig.comprobar)
            for tr in self.tramos:
                n = tr["n"]
                pose = self.pose(reposo["izq"], reposo["der"]) if tr.get("reposo") else self.pose(tr["izq"], tr["der"])
                T = brazo.duracion(brazo.consigna(), pose, tr["base_s"])
                self.tramo, self.fase = f"T{n:02d}", tr["fase"]
                desc = f"{n}/{self.tramos[-1]['n']} {tr['nombre']} ({tr['lado']}{', ' + tr['objeto'] if tr['objeto'] else ''})"
                reg.evento("TRAMO", f"{desc} en {T:.1f} s", {"n": n, "nombre": tr["nombre"], "fase": tr["fase"],
                                                           "lado": tr["lado"], "objeto": tr["objeto"], "T_s": T,
                                                           "obj": {NOMBRES[j]: round(q, 4) for j, q in pose.items()}})
                self._parada_en(n)
                met.nuevo(n, tr["nombre"], T)
                brazo.ir_a(pose, tr["base_s"], chequeo_mov)
                met.llegar()
                self.tramo = f"P{n:02d}"
                self.quieto(a.pausa, chequeo_pausa)
                m = met.cerrar()
                metricas.append(m)
                reg.evento("FIN_TRAMO", linea(m), m, imprimir=False)
                print(f"  T{n:02d} {tr['nombre'][:18]:18s} {linea(m)}", flush=True)
            self.tramo, self.fase = "SALIDA", "salida"
            reg.evento("SALIDA", f"peso 1 -> 0 en {brazo.t_salida:.0f} s")
            self.bajar_peso()
        except (Parada, KeyboardInterrupt) as e:
            motivo = f"parada: {e if isinstance(e, Parada) else 'Ctrl+C'}"
            self.parada(reg, motivo, reposo)
        finally:
            if brazo.peso > 0.01:            # cualquier otra excepcion: no dejar el peso arriba
                self.bajar_peso()
        msg1, _ = robot.ultimo()
        self.tramo = "FIN"
        reg.evento("FIN", motivo)
        reg.cerrar(motivo=motivo, metricas=metricas,
                   temperatura_final={NOMBRES[j]: int(msg1.motor_state[j].temperature[0]) for j in JUNTAS})
        return motivo, reg.carpeta

    def parada(self, reg, motivo, reposo):
        """La consigna se queda donde esta (congelada en la medida si el motivo es el error de seguimiento), los
        dos brazos a la observacion (solo desde un tramo de trabajo), al reposo medido y peso a 0."""
        brazo = self.brazo
        fase = self.fase
        self.tramo = "PARADA"
        reg.evento("PARADA", motivo)
        if "error de seguimiento" in motivo:
            brazo.congelar()
        try:
            if brazo.peso > 0.01 and fase not in ("entrada", "salida"):
                q = brazo.consigna()
                obs = self.pose(self.observacion["izq"], self.observacion["der"])
                if fase == "trabajo" and max(abs(q[j] - obs[j]) for j in obs) > 0.05:
                    reg.evento("PARADA", "los dos brazos a la observacion")
                    brazo.ir_a(obs, None, self.grave)
                rep = self.pose(reposo["izq"], reposo["der"])
                if max(abs(brazo.consigna()[j] - rep[j]) for j in rep) > 0.02:
                    reg.evento("PARADA", "al reposo medido")
                    brazo.ir_a(rep, None, self.grave)
        except (Parada, KeyboardInterrupt) as e:
            reg.evento("PARADA", f"{e if isinstance(e, Parada) else 'Ctrl+C'}: directo a bajar el peso")
        reg.evento("PARADA", f"peso -> 0 en {brazo.t_salida:.0f} s")
        self.bajar_peso()

    def _parada_en(self, n):
        if not self.a.parada_en:
            return
        tramo, _, s = self.a.parada_en.partition("+")
        if int(tramo) == n:
            t = threading.Timer(float(s or 0.0) / self.vel, lambda: self.vig.pedir_parada(f"prueba: --parada-en {self.a.parada_en}"))
            t.daemon = True
            t.start()

    def cerrar(self):
        try:
            self.brazo.parar()
        except RuntimeError as e:
            print(f"ATENCION: {e}")
        if self.lector_fsm is not None:
            self.lector_fsm.parar()
        if self.visor is not None:
            from sim.visor import informe_choques
            print("Visor:", informe_choques(self.visor.choques))
            if self.visor.abierto():
                print("Cierra la ventana del visor para terminar.")
                self.visor.esperar_cierre()
            self.visor.cerrar()
        if self.a.sim:
            self.robot.parar()


def main():
    a = leer_args()
    cfg = config.cargar()
    p = Prueba(a, cfg)
    print(f"\nComparacion de ganancias: {' -> '.join(a.ganancias)}; {len(p.tramos)} tramos de {os.path.relpath(a.secuencia, RAIZ)}"
          f"{' (' + ', '.join(a.objetos) + ')' if a.objetos else ''}; duraciones x{a.escala_tiempo:g}, pausa {a.pausa:g} s;"
          f" ~{p.duracion_estimada() / 60:.1f} min por pasada.")
    for n in a.ganancias:
        g = p.juegos[n]
        txt = " ".join(f"{JUNTAS_GAINS[k].replace('shoulder_', 's_').replace('wrist_', 'w_')} {g[13 + k][0]:g}/{g[13 + k][1]:g}"
                       for k in range(7))
        dist = "" if all(g[13 + k] == g[20 + k] for k in range(7)) else " (der distinto, ver resumen.json)"
        print(f"  {n:8s} kp/kd izq: {txt}; cintura {g[CINTURA][0]:g}/{g[CINTURA][1]:g}{dist}")
    if a.simulacro:
        print("\n  SIMULACRO: no se publica en rt/arm_sdk, los brazos no se mueven. ESPACIO o Ctrl+C prueban la parada.")
        input("  Enter para empezar: ")
    elif not a.sim:
        print("\n  Robot de pie en FSM 201, SIN la mesa delante, nadie al alcance de los brazos, operador con L2+B.")
        print("  Durante la pasada: ESPACIO o Ctrl+C para parar (los brazos vuelven al reposo y el peso baja a 0).")
        if input(f"  Escribe {cfg['supervisor']['palabra']} para empezar: ").strip() != cfg["supervisor"]["palabra"]:
            print("Cancelado.")
            p.cerrar()
            return
    elif a.visor:
        input("  Visor abierto. Enter para empezar: ")
    from manip.teclado import Teclado
    hechas = []
    try:
        for i, nombre in enumerate(a.ganancias):
            if i > 0 and not a.sim:
                r = input(f"\n  Enter para la pasada con '{nombre}' (q y Enter para terminar): ").strip().lower()
                if r == "q":
                    break
            p.vig.parada_pedida = None
            teclado = Teclado(lambda: p.vig.pedir_parada("tecla ESPACIO"))
            teclado.abrir()
            try:
                motivo, carpeta = p.pasada(nombre)
            finally:
                teclado.cerrar()
            hechas.append((nombre, motivo, carpeta))
            print(f"--- '{nombre}': {motivo}")
            if motivo != "completado":
                break                       # una parada termina la prueba: no se empieza el juego siguiente
    finally:
        p.cerrar()
    print("\nPasadas:")
    for nombre, motivo, carpeta in hechas:
        print(f"  {nombre:8s} {motivo:40s} {os.path.relpath(carpeta, RAIZ)}")
    if hechas:
        print("En la PC: ./desplegar.sh --traer && python3 herramientas/analizar_ganancias.py "
              + " ".join(os.path.relpath(c, RAIZ) for _, _, c in hechas))


if __name__ == "__main__":
    main()
