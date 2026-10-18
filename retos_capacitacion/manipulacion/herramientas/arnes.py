#!/usr/bin/env python3
"""Hito 0: el robot de pie por rt/lowcmd en el arnes, en tres pasos (PLAN.md).

    ./ejecutar.sh herramientas/arnes.py [--solo-colgado]      # en el robot, desde la PC
    python3 herramientas/arnes.py --sim [--visor]              # en la PC, con fisica

Antes (en el robot): colgado del portico con los pies ~5 cm sobre el suelo, en amortiguacion (L2+B), una
persona en la manivela del polipasto y otra con el mando y el terminal. El programa:
  1. COLGADO   Debug con ReleaseMode() (tras escribir SOLTAR), toma (kp de 0 a 1 en 3 s sin mover nada),
               piernas a la postura de pie (10 s), brazos a 0 (saliendo con el hombro) y tara de la carga
               de los pies (colgado marcan ~-40 N cada uno).
  2. APOYO     se baja el polipasto hasta que los pies llevan 40-90 % del peso (la cuerda aun sostiene) y [e]
               enciende el estabilizador (tobillos con la IMU) y DESPUES pasa el par de sosten a "de pie".
               En ese orden: el par de pie empuja con las piernas y afloja la cuerda solo; sin el estabilizador
               ya encendido el robot se balancea y cae en la cuerda (simulado el 2026-10-15).
  3. DE PIE    con el par de pie las piernas descargan la cuerda solas y [n] nivela: la referencia del
               estabilizador va a la fija (pitch +0.5, la de FSM 201) y la integral quita lo que falte.
               Nivelado, se centra el portico (la cuerda vertical: al nivelar el torso se echa atras y
               mueve la cinta ~5 cm) y se sube el polipasto hasta que los pies lleven 70-85 % del peso: es
               el procedimiento de trabajo desde el 2026-10-16 (la cuerda lleva el resto).
               [m] mueve un brazo adelante y atras para ver la inclinacion. [k] recorre las poses de
               calibracion (config/poses_calibracion.yaml, de herramientas/poses_calibracion.py), cada brazo
               desde 0 y de vuelta, quieto 3 s en cada una, mientras herramientas/grabar_calibracion.py
               guarda las muestras en otra terminal (~6 min). [c] para colgar: se sube el
               polipasto; el estabilizador se apaga solo y, sin carga, el par vuelve a "colgado"; [f] termina:
               brazos a la postura colgada y kp a 0.
Teclas (da igual mayuscula o minuscula): ESPACIO (o el boton del mando de config, L1) = parada: los brazos se
quedan donde estan y la postura se sostiene; luego colgar y [f]. Ctrl+C igual; un segundo Ctrl+C suelta (kp a 0
en 0.5 s). L2+B en el mando = amortiguacion ya (kp a 0 en 0.5 s, queda kd), como fuera de Debug: de pie, el
robot cae en la cuerda. En Debug el robot no hace nada con L2+B; lo hace el hilo de control (manip/cuerpo.py).
Si se corta el terminal (SSH) el programa sigue: parada, y cuando el polipasto lo cuelga termina solo.
En --sim: [u]/[j] suben/bajan el polipasto 1 cm ([U]/[J] 5 cm) y [l] pulsa el boton de parada del mando.
Registro en datos/arnes_<fecha>/: ciclo.csv (100 Hz), eventos.csv y resumen.json.
"""

import argparse
import csv
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
from manip.cuerpo import BRAZOS, Cuerpo  # noqa: E402
from manip.registro import NOMBRES, CsvSimple, carpeta_tirada, guardar_json  # noqa: E402

CARGA_APOYAR = (0.40, 0.90)  # [e] solo con los pies llevando esto: cargados, pero con la cuerda aun sosteniendo
CARGA_COLGADO = 0.15         # por debajo, colgando: el par vuelve a "colgado" y se puede terminar
CARGA_TRABAJO = (0.70, 0.85)  # de pie con la cuerda llevando el resto: el procedimiento de trabajo
CARGA_MOVER = 0.6            # [m] y [n] solo de pie (la carga estimada se mueve +-10 % con el par de los tobillos)
# movimiento de prueba: el brazo adelante como en un agarre (la tirada simulada del nivel 2) y vuelta a 0
ALCANCE_IZQ = [-0.32, 0.18, -1.07, 0.48, 0.28, 0.30, 0.70]
T_QUIETO_CAL = 3.0          # s quieto en cada pose de calibracion ([k]): grabar_calibracion.py pide 1 s sin moverse


def argumentos():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="en la PC: robot con fisica y polipasto con teclas")
    ap.add_argument("--visor", action="store_true", help="--sim: ventana de MuJoCo")
    ap.add_argument("--operador-auto", action="store_true",
                    help="--sim: un operador simulado contesta, maneja el polipasto y pulsa las teclas (ensayo completo)")
    ap.add_argument("--con-calibracion", action="store_true",
                    help="--operador-auto: ademas pulsa [k] (las poses de calibracion) despues de [m]")
    ap.add_argument("--solo-colgado", action="store_true", help="solo el paso 1 (no deja apoyar)")
    a = ap.parse_args()
    if (a.visor or a.operador_auto or a.con_calibracion) and not a.sim:
        sys.exit("--visor y --operador-auto son de la simulacion: con --sim")
    return a


class Arnes:
    def __init__(self, a, cfg):
        self.a, self.cfg = a, cfg
        self.fase = "INICIO"
        self.parada = None
        self.salir = False
        self.teclas = []
        self.cerrojo = threading.Lock()
        self.carpeta = carpeta_tirada(config.ruta_datos(cfg), "arnes", "sim" if a.sim else None)
        self.eventos = CsvSimple(os.path.join(self.carpeta, "eventos.csv"), ["t", "fase", "texto"])
        self.resumen = {"inicio": time.strftime("%Y-%m-%d %H:%M:%S"), "args": vars(a), "eventos": []}
        self.t0 = time.monotonic()
        self.sim = None
        self._avisos_vistos = 0
        self.boton = cfg["cuerpo"].get("boton_parada", "L1")
        self.sin_terminal = False
        self._t_linea = 0.0
        self.nivelado_visto = False

    # --- registro ------------------------------------------------------------------------------------
    def evento(self, texto):
        t = time.monotonic() - self.t0
        self.eventos.escribir({"t": round(t, 3), "fase": self.fase, "texto": texto})
        self.resumen["eventos"].append([round(t, 2), self.fase, texto])
        decir(f"[{t:7.1f} s] {self.fase}: {texto}")

    def terminal_cortado(self, *_):
        """SIGHUP (se corto el SSH): sin el, Python terminaria aqui, sin publicar, y en 0.2 s los motores quedan
        sin par. La salida pasa a un fichero y se para como con ESPACIO; colgado, se termina solo."""
        try:
            f = open(os.path.join(self.carpeta, "salida_sin_terminal.txt"), "a", buffering=1)
            sys.stdout = sys.stderr = f
        except OSError:
            pass
        self.sin_terminal = True
        self.pedir_parada("se corto el terminal (SSH)")

    def _abrir_ciclo(self):
        cab = ["t", "fase", "f_kp", "apoyo", "f_estab", "roll", "pitch", "ref_roll", "ref_pitch", "gx", "gy",
               "carga", "fz_izq", "fz_der", "fzc_izq", "fzc_der", "int_roll", "int_pitch", "edad", "temp"]
        cab += [f"{n}_{k}" for n in NOMBRES for k in ("q", "dq", "tau", "qd", "tauc")]
        if self.sim is not None:
            cab += ["sim_cuerda_n", "sim_carga"]
        self._f = open(os.path.join(self.carpeta, "ciclo.csv"), "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(cab)
        self._n = 0

    def al_ciclo(self, cu, msg):
        self._n += 1
        if self._n % max(1, int(cu.hz / 100)):
            return
        e, c = cu.estado, cu.ultimo_cmd
        imu = msg.imu_state
        fila = [f"{e['t']:.4f}", self.fase, f"{e['f_kp']:.3f}", f"{e['apoyo']:.3f}", f"{e['f_estab']:.3f}",
                f"{e['roll']:.5f}", f"{e['pitch']:.5f}", f"{cu.ref[0]:.5f}", f"{cu.ref[1]:.5f}",
                f"{imu.gyroscope[0]:.4f}", f"{imu.gyroscope[1]:.4f}", f"{e['carga']:.3f}",
                f"{e['fz']['izq']:.1f}", f"{e['fz']['der']:.1f}", f"{e['fz_crudo']['izq']:.1f}",
                f"{e['fz_crudo']['der']:.1f}", f"{e['integral'][0]:.2f}", f"{e['integral'][1]:.2f}",
                f"{e['edad']:.4f}", e["temp"]]
        ms = msg.motor_state
        for i in range(27):
            fila += [f"{ms[i].q:.5f}", f"{ms[i].dq:.4f}", f"{ms[i].tau_est:.3f}", f"{c['q'][i]:.5f}", f"{c['tau'][i]:.3f}"]
        if self.sim is not None:
            fila += [f"{self.sim.tension:.1f}", f"{self.sim.carga_real():.3f}"]
        self._w.writerow(fila)

    # --- teclas ----------------------------------------------------------------------------------------
    def al_teclear(self, ch):
        with self.cerrojo:
            self.teclas.append(ch)

    def pedir_parada(self, motivo):
        if self.parada is None:
            self.parada = motivo

    def leer_teclas(self):
        """Las teclas pulsadas, en minuscula (con Bloq Mayus la [f] llega como 'F': 2026-10-16). Cada una se
        apunta en eventos.csv. En --sim, [u]/[j] (polipasto) y [l] (boton del mando) se quedan aqui."""
        with self.cerrojo:
            t, self.teclas = self.teclas, []
        sal = []
        for ch in t:
            if self.sim is not None and ch in "uUjJ":
                self.sim.polipasto({"u": 0.01, "U": 0.05, "j": -0.01, "J": -0.05}[ch])
                decir(f"  (polipasto {'+' if ch in 'uU' else '-'}{1 if ch in 'uj' else 5} cm)")
                continue
            if not ch.isprintable() or ch == " ":
                continue
            self.evento(f"tecla '{ch}'" + (" (Bloq Mayus?)" if ch.isupper() else ""))
            if self.sim is not None and ch.lower() == "l":
                self._pulsar_boton_sim()
                continue
            sal.append(ch.lower())
        return sal

    def _pulsar_boton_sim(self):
        from manip.mando import BOTONES
        from manip.mensajes import Mando
        self.sim.mando = Mando(1 << BOTONES.index(self.boton), 0.0, 0.0, 0.0, 0.0)
        threading.Timer(0.3, lambda: setattr(self.sim, "mando", Mando(0, 0.0, 0.0, 0.0, 0.0))).start()

    # --- vigilancia en el hilo principal -------------------------------------------------------------
    def vigilar(self, cu):
        """Avisos del cuerpo, boton del mando y temperatura. Devuelve True si hay que parar."""
        for t, texto in cu.avisos[self._avisos_vistos:]:
            self.evento(f"vigilancia: {texto}")
        self._avisos_vistos = len(cu.avisos)
        e = cu.estado
        msg, _ = cu.robot.ultimo()
        from manip import mando as mando_mod
        if mando_mod.decodificar(msg.wireless_remote).pulsado(self.boton):
            self.pedir_parada(f"boton {self.boton} del mando")
        if e and e["temp"] >= cu.temp_max:
            self.pedir_parada(f"temperatura {e['temp']} C >= {cu.temp_max:.0f}")
        return self.parada is not None or cu.amortiguado

    def siguiente(self, cu):
        """Lo que toca hacer ahora, al final de la linea de estado (el aviso de "nivelado" se perdia entre las
        lineas, 2026-10-17)."""
        lo, hi = CARGA_TRABAJO
        if self.fase == "APOYO":
            return f"baja el polipasto hasta {CARGA_APOYAR[0]:.0%}-{CARGA_APOYAR[1]:.0%} en los pies y [e]"
        if self.fase == "DE_PIE":
            if not cu.nivelado:
                return "[n] nivelar, sin tocar el polipasto"
            if not self.nivelado_visto:
                return "nivelando, espera"
            if cu.carga > hi:
                return f"NIVELADO: centra el portico (cuerda vertical) y sube el polipasto hasta {lo:.0%}-{hi:.0%}"
            return "listo: [m] brazos, [k] calibracion, [c] colgar"
        if self.fase == "COLGAR":
            return "sube el polipasto hasta colgar; luego [f]"
        return ""

    def linea(self, cu, cada=1.0):
        e = cu.estado
        if not e or time.monotonic() - self._t_linea < cada:
            return
        self._t_linea = time.monotonic()
        r, p = math.degrees(e["roll"] - cu.ref[0]), math.degrees(e["pitch"] - cu.ref[1])
        imu = f"IMU {math.degrees(e['roll']):+.1f}/{math.degrees(e['pitch']):+.1f}"
        sim = f" | sim: cuerda {self.sim.tension:4.0f} N, pies {self.sim.carga_real():.0%}" if self.sim else ""
        if cu.nivelado and cu.integral_max > 0 and abs(e["integral"][1]) >= cu.integral_max - 0.1:
            # integral positiva al tope: falta par de puntillas, la cuerda tira del torso hacia delante (el gancho
            # por delante de la cinta): en el robot real se arreglo retrasando el portico (2026-10-17)
            sim += (f"  << INTEGRAL AL TOPE: la cuerda tira del torso hacia "
                    f"{'delante: retrasa' if e['integral'][1] > 0 else 'atras: adelanta'} el portico hasta que quede vertical >>")
        sig = self.siguiente(cu)
        sim += f"  >> {sig}" if sig else ""
        decir(f"  {self.fase:8s} pies {e['carga']:4.0%} ({e['fz']['izq']:4.0f}/{e['fz']['der']:4.0f} N)  "
              f"roll {r:+5.1f} pitch {p:+5.1f} ({imu}, ref {math.degrees(cu.ref[0]):+.1f}/{math.degrees(cu.ref[1]):+.1f}"
              f"{' nivelando' if cu.nivelado else ''})  "
              f"apoyo {e['apoyo']:.0%} estab {e['f_estab']:.0%} (int {e['integral'][1]:+4.1f} N m) kp {e['f_kp']:.0%}  "
              f"rodillas {e['tau'][3]:+5.0f}/{e['tau'][9]:+5.0f} N m  T {e['temp']} C  dt max {cu.dt_max * 1000:.1f} ms{sim}")


def decir(texto):
    """print que no falla con el terminal cortado (EIO antes de que llegue SIGHUP), con una linea en blanco
    detras: los mensajes se separan a la vista."""
    try:
        print(texto + "\n", flush=True)
    except (OSError, ValueError):
        pass


def nivelado(cu, margen_deg=0.5):
    """[n] terminado: la referencia en la fija y el torso a menos de margen_deg de ella."""
    if not cu.nivelado or not cu.estado:
        return False
    e = cu.estado
    llegada = all(o is None or abs(o - r) < 1e-4 for o, r in zip(cu.ref_objetivo, cu.ref))
    return llegada and abs(e["pitch"] - cu.ref[1]) < math.radians(margen_deg)


def esperar(arnes, cu, segundos=None, hasta=None, cada_linea=1.0, guardar_teclas=False):
    """Espera vigilando (y mostrando la linea de estado) hasta `segundos`, `hasta()` o una parada.
    Devuelve las teclas pulsadas (lista) o None si hay parada. guardar_teclas: no las lee (ni corta la espera):
    quedan para despues (una [n] pulsada mientras se enciende el estabilizador se perdia y adelantaba el par de
    pie, 2026-10-17)."""
    t0 = time.monotonic()
    while True:
        if arnes.vigilar(cu):
            return None
        teclas = [] if guardar_teclas else arnes.leer_teclas()
        if teclas:
            return teclas
        if hasta is not None and hasta():
            return []
        if segundos is not None and time.monotonic() - t0 >= segundos:
            return []
        arnes.linea(cu, cada_linea)
        time.sleep(0.02)


def mover(arnes, cu, fin, juntas, segundos=None, texto=""):
    T = cu.mover(fin, segundos, juntas=juntas, esperar=False)
    arnes.evento(f"{texto} ({T:.1f} s)")
    r = esperar(arnes, cu, hasta=lambda: not cu.moviendo(), cada_linea=2.0)
    while r:                                   # teclas durante el movimiento: se ignoran salvo la parada
        r = esperar(arnes, cu, hasta=lambda: not cu.moviendo(), cada_linea=2.0)
    if r is None:
        cu.congelar()
        return False
    return True


def brazos_a(arnes, cu, destino14, texto):
    """Brazos a `destino14` sacando antes el hombro (roll) y dejandolo para el final: la mano no roza la pierna."""
    r = math.radians(cu.c["roll_salida_deg"])
    q = cu.consigna()
    paso1 = q.copy()
    paso1[14], paso1[21] = max(q[14], r), min(q[21], -r)
    paso2 = paso1.copy()
    for j in BRAZOS:
        if j not in (14, 21):
            paso2[j] = destino14[j - 13]
    paso3 = paso2.copy()
    paso3[14], paso3[21] = destino14[1], destino14[8]
    return (mover(arnes, cu, paso1, BRAZOS, texto=f"{texto}: hombros fuera") and
            mover(arnes, cu, paso2, BRAZOS, texto=f"{texto}: codos y munecas") and
            mover(arnes, cu, paso3, BRAZOS, texto=f"{texto}: hombros a su sitio"))


def poses_calibracion():
    """config/poses_calibracion.yaml si es del arnes y de la mesa de ahora (config/montaje.yaml, que escribe el
    [g] de marcar_montaje.py); si no, ValueError: las poses se comprobaron sin choques delante de esa mesa."""
    pc = config._leer(os.path.join(config.CONFIG, "poses_calibracion.yaml"))
    if pc.get("modo") != "arnes":
        raise ValueError("config/poses_calibracion.yaml no es del arnes (herramientas/poses_calibracion.py)")
    T = np.array(config.cargar_montaje()["mesa"]["T_torso_mesa"], float)
    x0, y, yaw = T[0, 3], T[1, 3], math.degrees(math.atan2(T[1, 0], T[0, 0]))
    g = pc["montaje"]
    if abs(x0 - g["x0_m"]) > 0.01 or abs(y - g["y_m"]) > 0.01 or abs(yaw - g["yaw_deg"]) > 1.0:
        raise ValueError(f"la mesa no es la de las poses (borde/lateral/giro {g['x0_m']:.3f}/{g['y_m']:+.3f}/"
                         f"{g['yaw_deg']:+.1f} y ahora {x0:.3f}/{y:+.3f}/{yaw:+.1f}): regenerarlas en la PC con "
                         "herramientas/poses_calibracion.py y desplegar")
    return pc


def recorrer_calibracion(arnes, cu, pc):
    """[k]: cada brazo por sus poses desde 0 y de vuelta a 0 (el otro quieto en 0), T_QUIETO_CAL s en cada una.
    False si hay parada."""
    for lado in ("izq", "der"):
        js = list(range(13, 20)) if lado == "izq" else list(range(20, 27))
        poses = pc["poses"][lado]
        for k, q7 in enumerate(list(poses) + [pc["inicio"][lado]]):
            q = cu.consigna()
            q[js] = q7
            texto = f"calibracion {lado} {k + 1}/{len(poses)}" if k < len(poses) else f"brazo {lado} a 0"
            if not mover(arnes, cu, q, js, texto=texto):
                return False
            # las teclas no acortan la pausa (se quedan para despues); ESPACIO y el mando si paran
            if esperar(arnes, cu, segundos=T_QUIETO_CAL if k < len(poses) else 1.0, guardar_teclas=True) is None:
                return False
    arnes.evento("calibracion recorrida: Ctrl+C en grabar_calibracion.py")
    return True


def colgar_y_terminar(arnes, cu):
    """Fase COLGAR: se sube el polipasto; sin carga el par vuelve a colgado; [f] lleva los brazos a la postura
    colgada y suelta. Sin terminal (SSH cortado) termina solo tras 3 s colgado; L2+B amortigua en cualquier
    momento."""
    arnes.fase = "COLGAR"
    cu.nivelar(False)                                # la cuerda vuelve a tirar: la referencia y la integral quietas
    arnes.evento("[f] para terminar" if cu.carga < CARGA_COLGADO and cu.apoyo.meta == 0.0 else
                 "sube el polipasto hasta que los pies no toquen el suelo; luego [f] para terminar")
    t_colgado, visto = None, None
    while not cu.amortiguado:
        t = esperar(arnes, cu, segundos=0.5)
        if t is None:                                # una parada ya en COLGAR: se sigue colgando
            if arnes.parada != visto:
                arnes.evento(f"parada ({arnes.parada}): ya se esta colgando, se sigue")
                visto = arnes.parada
            arnes.parada = None
            continue
        if cu.carga < CARGA_COLGADO:
            if cu.apoyo.meta > 0.0:
                cu.apoyar(False)
                arnes.evento(f"pies con el {cu.carga:.0%} del peso: par de sosten a colgado")
            t_colgado = t_colgado or time.monotonic()
        else:
            t_colgado = None
        if "f" in t:
            if t_colgado is None:
                arnes.evento(f"todavia apoyado ({cu.carga:.0%} del peso en los pies): sube mas el polipasto")
                continue
            break
        if arnes.sin_terminal and t_colgado is not None and time.monotonic() - t_colgado > 3.0:
            arnes.evento("sin terminal y colgado: se termina")
            break
    if cu.amortiguado:
        return
    arnes.fase = "FIN"
    if cu.f_estab.valor > 0.0:
        cu.estabilizador(False, 1.0)
    brazos_a(arnes, cu, cu.brazos_colgado, "brazos a la postura colgada")
    if not cu.amortiguado:
        cu.soltar(motivo="fin")
        arnes.evento(f"kp a 0 en {cu.c['t_soltar_s']} s (queda kd: amortiguacion)")


def operador_auto(arnes, cu, sim):
    """El operador de la simulacion: lo que haria la persona del polipasto y la del terminal."""
    def hasta(cond, tmax=600):
        t0 = time.monotonic()
        while not cond() and time.monotonic() - t0 < tmax and not arnes.salir:
            time.sleep(0.05)
        return cond()
    def bajar_mientras(cond, paso=0.002, cada=0.2, tmax=300):
        t0 = time.monotonic()
        while cond() and time.monotonic() - t0 < tmax and not arnes.salir:
            sim.polipasto(-paso)
            time.sleep(cada)
    hasta(lambda: arnes.fase == "APOYO")
    time.sleep(2.0)
    bajar_mientras(lambda: cu.carga < 0.6)                     # 1 cm/s hasta el 60 % en los pies
    time.sleep(2.0)
    arnes.al_teclear("e")
    time.sleep(1.0)
    arnes.al_teclear("n")                                       # pronto, como en el robot: se guarda hasta DE_PIE
    hasta(lambda: arnes.fase == "DE_PIE")
    hasta(lambda: arnes.nivelado_visto, 90)
    time.sleep(3.0)
    bajar_mientras(lambda: cu.carga > 0.78, paso=-0.001)         # subir hasta que la cuerda lleve ~25 %
    time.sleep(10.0)
    arnes.al_teclear("m")
    time.sleep(1.0)
    hasta(lambda: not cu.moviendo() and "brazo der a 0" in str(arnes.resumen["eventos"][-1:]), 200)
    time.sleep(4.0)
    if arnes.a.con_calibracion:
        arnes.al_teclear("k")
        hasta(lambda: "calibracion recorrida" in str(arnes.resumen["eventos"][-1:]) or arnes.parada is not None, 900)
        time.sleep(4.0)
    arnes.al_teclear("c")
    hasta(lambda: arnes.fase == "COLGAR")
    bajar_mientras(lambda: cu.carga > 0.05, paso=-0.002)       # subir
    sim.polipasto(0.03)
    time.sleep(3.0)
    arnes.al_teclear("f")


def esperar_colgado(arnes, cu):
    """Al salir por un error con los pies cargados: no soltar (el robot caeria en la cuerda) hasta que este
    colgado. Ctrl+C suelta ya."""
    if cu.carga < CARGA_COLGADO and cu.apoyo.valor == 0.0:
        return
    decir("*** El robot sigue sostenido. Sube el polipasto hasta colgarlo: se soltara cuando los pies no lleven "
          "peso (Ctrl+C o L2+B suelta ya). ***")
    t_ok = None
    try:
        while True:
            if cu.amortiguado:
                return
            if cu.carga < CARGA_COLGADO:
                if cu.apoyo.meta > 0.0:
                    cu.apoyar(False)
                t_ok = t_ok or time.monotonic()
                if time.monotonic() - t_ok > 3.5:
                    return
            else:
                t_ok = None
            e = cu.estado
            decir(f"  pies {e.get('carga', 0):4.0%}  apoyo {e.get('apoyo', 0):.0%}  estab {e.get('f_estab', 0):.0%}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        decir("Ctrl+C: se suelta ya")


def main():
    a = argumentos()
    cfg = config.cargar()
    arnes = Arnes(a, cfg)
    from manip.dinamica import G, Dinamica
    din = Dinamica()
    if not din.masas_identificadas:
        sys.exit("Falta config/gravedad_brazos.yaml (herramientas/gravedad_brazos.py): sin el, tuned_gff no vale")
    from manip import modo as modo_mod
    if a.sim:
        from sim.cuerpo_simulado import ConmutadorSimulado, CuerpoSimulado
        robot = arnes.sim = CuerpoSimulado(hilo=True, visor=a.visor)
        conmutador, contar, reloj = ConmutadorSimulado(), (lambda s: 0), robot.reloj
    else:
        from manip.robot import Robot
        robot = Robot(cfg["red"]["iface"])
        if not robot.esperar_lowstate(3.0):
            sys.exit("No llega rt/lowstate: ¿robot encendido y la red eth0?")
        conmutador, contar, reloj = modo_mod.Conmutador(), modo_mod.contar_lowcmd, time.monotonic
    e = robot.estado()
    decir(f"Robot: mode_machine {e.mode_machine}, motores en fallo {e.motores_en_fallo or 'ninguno'}, "
          f"rodillas {math.degrees(e.q[3]):.0f}/{math.degrees(e.q[9]):.0f} grados, "
          f"roll {math.degrees(e.imu.roll):+.1f} pitch {math.degrees(e.imu.pitch):+.1f}")
    if e.motores_en_fallo:
        sys.exit("Hay motores en fallo: no se empieza")
    decir("ANTES: robot COLGADO del portico con los pies sin tocar el suelo, en amortiguacion (L2+B), una persona\n"
          "en la manivela del polipasto y otra con el mando. Nadie al alcance de brazos y piernas.")
    preguntar = (lambda txt: print(txt + "(operador simulado)\n") or ("COLGADO" if "COLGADO" in txt else "SOLTAR")) \
        if a.operador_auto else input
    if preguntar("Escribe COLGADO para seguir: ").strip() != "COLGADO":
        sys.exit("cancelado")
    try:
        arnes.fase = "DEBUG"
        previo = modo_mod.entrar_debug(conmutador, contar=contar, preguntar=preguntar, log=arnes.evento)
        arnes.resumen["modo_previo"] = previo
    except RuntimeError as ex:
        arnes.evento(f"no se entra en Debug: {ex}")
        guardar_json(os.path.join(arnes.carpeta, "resumen.json"), arnes.resumen)
        sys.exit(1)
    cu = Cuerpo(robot, cfg, din, publicar=not a.sim, destino=(robot.recibir_lowcmd if a.sim else None), reloj=reloj)
    arnes._abrir_ciclo()
    cu.al_ciclo = arnes.al_ciclo
    from manip.teclado import Teclado
    teclado = Teclado(lambda: arnes.pedir_parada("ESPACIO"), al_teclear=arnes.al_teclear)
    signal.signal(signal.SIGHUP, arnes.terminal_cortado)
    ctrl_c = [0]
    motivo_fin = "completado"
    try:
        cu.iniciar()
        teclado.abrir()
        if a.operador_auto:
            threading.Thread(target=operador_auto, args=(arnes, cu, robot), daemon=True).start()
        # --- 1. colgado ---------------------------------------------------------------------------------
        arnes.fase = "TOMA"
        arnes.evento(f"kp de 0 a 1 en {cu.tomar():.0f} s sin mover nada")
        if esperar(arnes, cu, segundos=float(cu.c["t_toma_s"]) + 0.5) is None:
            raise KeyboardInterrupt
        arnes.fase = "POSTURA"
        q = cu.consigna()
        q[:13] = cu.postura_pie
        ok = mover(arnes, cu, q, range(13), float(cu.c["t_postura_s"]), "piernas a la postura de pie")
        ok = ok and brazos_a(arnes, cu, cu.brazos_cero, "brazos a 0")
        if not ok:
            raise KeyboardInterrupt
        arnes.fase = "COLGADO"
        if esperar(arnes, cu, segundos=2.5) is None:            # la tara con ~2 s de postura quieta
            raise KeyboardInterrupt
        try:
            tara = cu.tarar()
        except RuntimeError as ex:
            arnes.evento(f"sin tara: {ex}")
            raise KeyboardInterrupt
        arnes.resumen["tara_n"] = tara
        arnes.evento(f"tara de los pies colgado: izq {tara['izq']:+.0f} N, der {tara['der']:+.0f} N (sin ella "
                     f"marcarian {(tara['izq'] + tara['der']) * cu.escala_carga / (din.masa * G):+.0%})")
        if a.solo_colgado:
            colgar_y_terminar(arnes, cu)
            return
        # --- 2. apoyo -----------------------------------------------------------------------------------
        arnes.fase = "APOYO"
        arnes.evento("baja el polipasto DESPACIO hasta que los pies lleven 50-80 % del peso y pulsa [e] "
                     "(estabilizador y par de pie); [c] colgar")
        while True:
            t = esperar(arnes, cu, hasta=lambda: not arnes.nivelado_visto and nivelado(cu))
            if t is None:
                raise KeyboardInterrupt
            if not arnes.nivelado_visto and nivelado(cu):
                arnes.nivelado_visto = True
                lo, hi = CARGA_TRABAJO
                decir("=" * 100)
                arnes.evento(f"NIVELADO (IMU pitch {math.degrees(cu.estado['pitch']):+.1f}): centra el portico hasta que "
                             f"la cuerda quede vertical y sube el polipasto hasta que los pies lleven {lo:.0%}-{hi:.0%}; "
                             "luego [m] brazo adelante y atras, [k] calibracion, [c] colgar")
                decir("=" * 100)
            if "c" in t:
                break
            if "e" in t and arnes.fase == "APOYO":
                lo, hi = CARGA_APOYAR
                if not lo <= cu.carga <= hi:
                    arnes.evento(f"los pies llevan el {cu.carga:.0%}: hace falta {lo:.0%}-{hi:.0%} "
                                 f"({'baja' if cu.carga < lo else 'sube'} un poco el polipasto)")
                    continue
                try:
                    T = cu.estabilizador(True)
                except RuntimeError as ex:
                    arnes.evento(f"no se enciende: {ex}")
                    continue
                arnes.evento(f"estabilizador encendiendose en {T:.0f} s (referencia roll {math.degrees(cu.ref[0]):+.1f}, "
                             f"pitch {math.degrees(cu.ref[1]):+.1f})")
                if esperar(arnes, cu, hasta=lambda: cu.f_estab.valor >= 1.0 or cu.f_estab.meta == 0.0,
                           guardar_teclas=True) is None or cu.f_estab.meta == 0.0:
                    raise KeyboardInterrupt
                arnes.evento(f"par de sosten a de pie en {cu.apoyar(True):.0f} s")
                if esperar(arnes, cu, segundos=3.5, guardar_teclas=True) is None:
                    raise KeyboardInterrupt
                arnes.fase = "DE_PIE"
                arnes.evento("de pie con el estabilizador: pulsa [n] (nivelar) ya, sin tocar el polipasto; [c] colgar")
            if "n" in t and arnes.fase == "DE_PIE":
                if cu.carga < CARGA_MOVER:
                    arnes.evento(f"[n] de pie: los pies llevan el {cu.carga:.0%} (< {CARGA_MOVER:.0%}): baja un poco el polipasto")
                    continue
                T = cu.nivelar(True)
                obj = [("-" if o is None else f"{math.degrees(o):+.1f}") for o in cu.ref_objetivo]
                arnes.evento(f"nivelando: referencia de {math.degrees(cu.ref[0]):+.1f}/{math.degrees(cu.ref[1]):+.1f} "
                             f"a {obj[0]}/{obj[1]} grados (roll/pitch) en {T:.0f} s, y la integral")
            if "k" in t and arnes.fase == "DE_PIE":
                if cu.carga < CARGA_MOVER or cu.f_estab.valor < 1.0 or not cu.nivelado:
                    arnes.evento(f"[k] solo de pie ({cu.carga:.0%}, hace falta {CARGA_MOVER:.0%}), con el "
                                 "estabilizador y despues de [n]")
                    continue
                try:
                    pc = poses_calibracion()
                except (ValueError, OSError) as ex:
                    arnes.evento(f"[k] no: {ex}")
                    continue
                arnes.evento(f"calibracion: {len(pc['poses']['izq'])} + {len(pc['poses']['der'])} poses "
                             "(grabar_calibracion.py en otra terminal); ESPACIO para")
                if not recorrer_calibracion(arnes, cu, pc):
                    raise KeyboardInterrupt
            if "m" in t and arnes.fase == "DE_PIE":
                if cu.carga < CARGA_MOVER or cu.f_estab.valor < 1.0 or not cu.nivelado:
                    arnes.evento(f"[m] solo de pie ({cu.carga:.0%}, hace falta {CARGA_MOVER:.0%}), con el "
                                 "estabilizador y despues de [n]")
                    continue
                if cu.carga > CARGA_TRABAJO[1] + 0.05:
                    arnes.evento(f"ojo: los pies llevan el {cu.carga:.0%}; el procedimiento pide {CARGA_TRABAJO[0]:.0%}-"
                                 f"{CARGA_TRABAJO[1]:.0%} (sube el polipasto despues); se mueve igual")
                for lado, q7 in (("izq", ALCANCE_IZQ), ("der", list(np.array(ALCANCE_IZQ) * [1, -1, -1, 1, -1, 1, -1]))):
                    js = list(range(13, 20)) if lado == "izq" else list(range(20, 27))
                    q = cu.consigna()
                    q[js] = q7
                    if not mover(arnes, cu, q, js, texto=f"brazo {lado} adelante"):
                        raise KeyboardInterrupt
                    esperar(arnes, cu, segundos=2.0)
                    q[js] = 0.0
                    if not mover(arnes, cu, q, js, texto=f"brazo {lado} a 0"):
                        raise KeyboardInterrupt
                    esperar(arnes, cu, segundos=2.0)
        colgar_y_terminar(arnes, cu)
    except (KeyboardInterrupt, Exception) as ex:
        ctrl_c[0] += 1
        if not isinstance(ex, KeyboardInterrupt):
            import traceback
            traceback.print_exc()
            arnes.pedir_parada(f"error del programa: {ex!r}")
        motivo_fin = ("amortiguado: " + cu.motivo) if cu.amortiguado else (arnes.parada or "Ctrl+C")
        try:
            if cu.amortiguado:
                arnes.evento(f"AMORTIGUACION ({cu.motivo}): kp a 0, queda kd; si estaba de pie, cae en la cuerda")
            else:
                arnes.evento(f"PARADA ({motivo_fin}): brazos quietos, postura sostenida")
                cu.congelar()
                arnes.parada = None
                colgar_y_terminar(arnes, cu)
        except KeyboardInterrupt:
            ctrl_c[0] += 1
            arnes.evento("segundo Ctrl+C: kp a 0 en 0.5 s")
            cu.soltar(0.5, "segundo Ctrl+C")
    finally:
        if cu.f_kp.valor > 0.0 and not cu.amortiguado:
            esperar_colgado(arnes, cu)
            cu.soltar(float(cu.c["t_soltar_s"]), "salida")
        t_fin = time.monotonic() + float(cu.c["t_soltar_s"]) + 3.0
        while time.monotonic() < t_fin:              # se publica hasta acabar la rampa y 1 s mas
            try:
                if cu.f_kp.valor == 0.0:
                    t_fin = min(t_fin, time.monotonic() + 1.0)
                time.sleep(0.05)
            except KeyboardInterrupt:
                cu.soltar(0.5, "Ctrl+C al salir")
        teclado.cerrar()
        try:
            cu.parar()
        except RuntimeError as ex:
            decir(str(ex))
        arnes.resumen.update(fin=motivo_fin, dt_max_ms=round(cu.dt_max * 1000, 2), ciclos=cu.n,
                             carpeta=arnes.carpeta)
        guardar_json(os.path.join(arnes.carpeta, "resumen.json"), arnes.resumen)
        arnes._f.close()
        arnes.eventos.cerrar()
        arnes.salir = True
        if arnes.sim is not None:
            arnes.sim.parar()
        decir(f"Registro en {arnes.carpeta}")


if __name__ == "__main__":
    main()
