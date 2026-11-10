"""Curva de inclinacion y giro del cuerpo frente a la duracion de un mismo movimiento de los brazos (PDF 6.3.1-2).

Con el robot de pie en el arnes (herramientas/arnes.py nivelado, pies al 70-85 %) y como cliente de arnes.py,
igual que manipulacion.py --arnes (manip/brazo_arnes.py): cada movimiento va de los brazos a 0 a su pose y vuelve,
a cada duracion, de la mas lenta a la mas rapida (30, 20, 10 y 8 s; nunca menos de 8 ni mas de 0.5 rad/s), con
--quieto s quietos antes, en la pose y al volver. Graba a --hz lo que mide arnes.py en su ciclo de 500 Hz (IMU con
el yaw, carga y fuerza de los pies, integral del estabilizador, q y par de las 27) y su consigna.

Movimientos (los dos primeros, por defecto):
  observacion   los dos brazos a la pose de observacion (config/poses_brazos.yaml) por su punto de paso
                (paso_inicio), como manipulacion.py --arnes: centro de masas 4 cm atras
  alcance_izq   el brazo izquierdo adelante como en un agarre (el [m] de arnes.py): centro de masas casi quieto
  alcance_der   el derecho, en espejo
Velocidades articulares parecidas con desplazamientos del centro de masas muy distintos: separa lo que hace la
velocidad o la aceleracion de lo que hace el centro de masas (herramientas/analizar_pasos.py).

Antes de mover: cada camino (rectas articulares de 0 a cada punto, las que hace arnes.py) sin choques en el
gemelo con config/montaje.yaml, con la holgura del plan y con +-3 grados en hombro y codo a 1 cm, y la palabra.
Para con ESPACIO, Ctrl+C, una parada de arnes.py o la IMU fuera de lo permitido (--max-incl grados de roll o
pitch, --max-yaw de yaw, desde lo medido al empezar): congela, vuelve a 0 deshaciendo el camino y suelta. L2+B,
como siempre, en arnes.py.

    terminal 1: ./ejecutar.sh herramientas/arnes.py          (de pie, [n], portico y polipasto)
    terminal 2: ./ejecutar.sh herramientas/curva_pasos.py [--duraciones 30 20 10 8] [--movimientos ...]
    en la PC:   python3 herramientas/analizar_pasos.py datos/pasos_<fecha>
En la PC, con fisica: herramientas/arnes.py --sim --operador-auto --esperar-cliente y, en otra terminal,
herramientas/curva_pasos.py --sim (sin palabra ni gemelo de colisiones; --rapido acorta las esperas).
Registro en datos/pasos_<fecha>/: muestras.csv, eventos.csv y resumen.json.
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RAIZ)
from manip import config  # noqa: E402
from manip.cuerpo import BRAZOS  # noqa: E402
from manip.mensajes import JUNTAS_BRAZO, Parada  # noqa: E402
from manip.registro import CsvSimple, carpeta_tirada, guardar_json  # noqa: E402

DURACIONES = [30.0, 20.0, 10.0, 8.0]
ALCANCE_IZQ = [-0.32, 0.18, -1.07, 0.48, 0.28, 0.30, 0.70]          # el [m] de arnes.py
ESPEJO = np.array([1, -1, -1, 1, -1, 1, -1])
TRAMOS = ("antes", "ida", "en_pose", "vuelta", "despues")
CABECERA = (["t", "mov", "T", "rep", "tramo", "carga", "fz_izq", "fz_der", "int_roll", "int_pitch", "roll", "pitch",
             "yaw", "gx", "gy", "gz"] + [f"c{j}" for j in BRAZOS] + [f"q{j}" for j in range(27)] +
            [f"tau{j}" for j in range(27)])


def movimientos_definidos():
    """{nombre: [pose, ...]}: los puntos por los que pasa la ida desde 0 (la vuelta, al reves); cada pose,
    {lado: q7}, y lo que no aparece se queda a 0."""
    poses = config._leer(os.path.join(config.CONFIG, "poses_brazos.yaml"))
    obs = {l: list(poses["observacion"][l]) for l in ("izq", "der")}
    paso = [{l: list(poses["paso_inicio"][l]) for l in ("izq", "der")}] if "paso_inicio" in poses else []
    return {"observacion": paso + [obs],
            "alcance_izq": [{"izq": ALCANCE_IZQ}],
            "alcance_der": [{"der": list(np.array(ALCANCE_IZQ) * ESPEJO)}]}


def pose14(mov):
    """Los 14 de los brazos (13-26) de un movimiento."""
    q = np.zeros(len(BRAZOS))
    for lado, q7 in mov.items():
        for j, v in zip(JUNTAS_BRAZO[lado], q7):
            q[BRAZOS.index(j)] = v
    return q


ERROR_DEG, HOLGURA_ERROR_M = 3.0, 0.01        # como herramientas/poses_brazos.py


def _errores():
    sal = []
    for j in (0, 3):                                    # pitch del hombro y codo: suben o bajan la mano
        for signo in (1.0, -1.0):
            e = np.zeros(7)
            e[j] = signo * math.radians(ERROR_DEG)
            sal.append(e)
    return sal


def caminos_con_choques(cfg, montaje, movs, paso=0.02):
    """{nombre: [lineas del informe]} de los movimientos cuyas rectas articulares (desde 0 por sus puntos) chocan o
    se acercan a menos de la holgura del plan en el gemelo (mesa, asas, bandejas, el cuerpo), o a menos de
    HOLGURA_ERROR_M con ERROR_DEG de error en hombro o codo. Vacio si todos estan limpios."""
    from manip import calibracion as cb
    from manip.cinematica import Cinematica
    from sim.colisiones import Colisiones, informe
    from sim.escena import Estado, construir, postura_arnes
    cin = Cinematica.desde_config(cfg)
    cal = cb.cargar_o_urdf(cin)
    m, info = construir(montaje=montaje, cal=cal, cfg=cfg, postura=[float(v) for v in postura_arnes(cfg)],
                        con_camaras=False)
    est = Estado(m, info)
    col = Colisiones(m, est, cfg["planificacion"]["holgura_m"])
    malos = {}
    for nombre, mov in movs.items():
        puntos = [{l: np.zeros(7) for l in ("izq", "der")}]
        for pose in mov:
            puntos.append({l: np.asarray(pose.get(l, np.zeros(7)), float) for l in ("izq", "der")})
        for k in range(1, len(puntos)):
            a, b = puntos[k - 1], puntos[k]
            n = max(2, int(math.ceil(max(np.abs(b[l] - a[l]).max() for l in b) / paso)))
            for err, holgura in [(None, col.margen)] + [(e, HOLGURA_ERROR_M) for e in _errores()]:
                for s in np.linspace(0.0, 1.0, n + 1)[1:]:
                    for l in b:
                        e = 0.0 if err is None or not np.any(b[l] - a[l]) else (err if l == "izq" else err * ESPEJO)
                        est.poner_brazo(l, a[l] + s * (b[l] - a[l]) + e)
                    h = [x for x in col.revisar() if x[2] < holgura]
                    if h:
                        txt = "" if err is None else f" con {np.degrees(err).round(0).tolist()} grados de error"
                        malos[nombre] = [f"tramo {k} al {s:.0%}{txt}:"] + informe(h)
                        break
                if nombre in malos:
                    break
            if nombre in malos:
                break
        for l in ("izq", "der"):
            est.poner_brazo(l, np.zeros(7))
    return malos


class CurvaPasos:
    """El recorrido y su registro. `b`: el cliente de arnes.py (BrazoArnes)."""

    def __init__(self, b, movs, duraciones, carpeta, quieto_s=8.0, hz=50.0, repeticiones=1, max_incl_deg=3.0,
                 max_yaw_deg=3.0, reloj=time.monotonic, dormir=time.sleep, log=print):
        self.b, self.movs, self.duraciones = b, movs, sorted((float(d) for d in duraciones), reverse=True)
        self.carpeta, self.quieto, self.cada, self.reps = carpeta, float(quieto_s), 1.0 / float(hz), int(repeticiones)
        self.max_incl, self.max_yaw = math.radians(max_incl_deg), math.radians(max_yaw_deg)
        self.reloj, self.dormir, self.log = reloj, dormir, log
        self.muestras = CsvSimple(os.path.join(carpeta, "muestras.csv"), CABECERA)
        self.eventos = CsvSimple(os.path.join(carpeta, "eventos.csv"), ["t", "mov", "T", "rep", "tramo", "texto"],
                                 al_momento=True)
        self.etiqueta = {"mov": "", "T": "", "rep": "", "tramo": "inicio"}
        self.parada = None
        self.rpy0 = None
        self.ultimo = None
        self.hechos = []
        self.pila = [np.zeros(len(BRAZOS))]          # los puntos por los que se llego a donde estan los brazos
        self._vivo = False
        self._hilo = None

    # --- registro --------------------------------------------------------------------------------------
    def evento(self, texto):
        e = self.etiqueta
        self.eventos.escribir({"t": round(self.reloj(), 3), "mov": e["mov"], "T": e["T"], "rep": e["rep"],
                               "tramo": e["tramo"], "texto": texto})
        self.log(f"  {e['mov']} {e['T']} {e['tramo']}: {texto}" if e["mov"] else f"  {texto}")

    def _muestrear(self):
        while self._vivo:
            t0 = self.reloj()
            r = self.b.medir()
            c = (r or {}).get("cuerpo")
            if c:
                self.ultimo = c
                self._fila(r, c)
                self._vigilar(c)
            self.dormir(max(0.0, self.cada - (self.reloj() - t0)))

    def _fila(self, r, c):
        fila = dict(self.etiqueta)
        fila.update(t=c["t"], carga=c["carga"], fz_izq=c["fz"]["izq"], fz_der=c["fz"]["der"],
                    int_roll=c["integral"][0], int_pitch=c["integral"][1], roll=c["rpy"][0], pitch=c["rpy"][1],
                    yaw=c["rpy"][2], gx=c["gyro"][0], gy=c["gyro"][1], gz=c["gyro"][2])
        for j in BRAZOS:
            fila[f"c{j}"] = r["consigna"][j]
        for j in range(27):
            fila[f"q{j}"], fila[f"tau{j}"] = c["q"][j], c["tau"][j]
        self.muestras.escribir(fila)

    def _vigilar(self, c):
        rpy = np.asarray(c["rpy"], float)
        if self.rpy0 is None:
            self.rpy0 = rpy
            return
        d = rpy - self.rpy0
        d[2] = (d[2] + math.pi) % (2 * math.pi) - math.pi
        if self.parada is None and (abs(d[0]) > self.max_incl or abs(d[1]) > self.max_incl):
            self.parada = (f"inclinacion {math.degrees(d[0]):+.1f}/{math.degrees(d[1]):+.1f} grados (roll/pitch) "
                           f"desde el inicio, mas de {math.degrees(self.max_incl):.1f}")
        if self.parada is None and abs(d[2]) > self.max_yaw:
            self.parada = f"giro {math.degrees(d[2]):+.1f} grados desde el inicio, mas de {math.degrees(self.max_yaw):.1f}"

    def pedir_parada(self, motivo):
        if self.parada is None:
            self.parada = motivo

    def comprobar(self):
        if self.parada:
            raise Parada(self.parada)
        m = self.b.motivo_parada()
        if m:
            raise Parada(m)

    # --- recorrido -------------------------------------------------------------------------------------
    def quieto_durante(self, s):
        t_fin = self.reloj() + s
        while self.reloj() < t_fin:
            self.comprobar()
            self.dormir(0.05)

    def poner(self, mov="", T="", rep="", tramo=""):
        self.etiqueta = {"mov": mov, "T": T, "rep": rep, "tramo": tramo}

    def correr(self):
        """Toma los brazos, hace el recorrido y los devuelve a 0. Devuelve el motivo de fin."""
        cero = {j: 0.0 for j in BRAZOS}
        self._vivo = True
        self._hilo = threading.Thread(target=self._muestrear, daemon=True)
        self._hilo.start()
        motivo = "completado"
        try:
            self.b.rampa_peso(1.0)
            self.evento("brazos tomados")
            self.quieto_durante(2.0)
            for nombre, mov in self.movs.items():
                ida = [pose14(p) for p in mov]
                vuelta = ida[-2::-1] + [np.zeros(len(BRAZOS))]
                for T in self.duraciones:
                    for rep in range(1, self.reps + 1):
                        self.poner(nombre, T, rep, "antes")
                        self.quieto_durante(self.quieto)
                        self.poner(nombre, T, rep, "ida")
                        Tr = sum(self._ir(q, T) for q in ida)
                        self.evento(f"ida en {Tr:.1f} s ({len(ida)} tramo/s)")
                        self.poner(nombre, T, rep, "en_pose")
                        self.quieto_durante(self.quieto)
                        self.poner(nombre, T, rep, "vuelta")
                        Tr = sum(self._ir(q, T) for q in vuelta)
                        self.evento(f"vuelta en {Tr:.1f} s ({len(vuelta)} tramo/s)")
                        self.poner(nombre, T, rep, "despues")
                        self.quieto_durante(self.quieto)
                        self.hechos.append((nombre, T, rep))
            self.poner(tramo="fin")
        except (Parada, KeyboardInterrupt) as ex:
            motivo = f"parada: {ex}" if isinstance(ex, Parada) else "Ctrl+C"
            self.salir(motivo)
        except Exception as ex:
            motivo = f"error: {ex!r}"
            self.salir(motivo)
            raise
        finally:
            try:
                self.b.rampa_peso(0.0)
            finally:
                self._vivo = False
                if self._hilo is not None:
                    self._hilo.join(timeout=2.0)
                self.muestras.cerrar()
                self.evento(f"fin: {motivo}")
                self.eventos.cerrar()
        return motivo

    def _ir(self, q14, T):
        """Un tramo en recta articular hasta q14; la pila guarda los puntos por los que se ha llegado (para
        deshacer el camino al parar)."""
        Tr = self.b.ir_a({j: float(v) for j, v in zip(BRAZOS, q14)}, T, comprobar=self.comprobar)
        if len(self.pila) >= 2 and np.allclose(q14, self.pila[-2], atol=1e-6):
            self.pila.pop()
        else:
            self.pila.append(np.asarray(q14, float))
        return Tr

    def salir(self, motivo):
        """Congelados (lo hizo ir_a al saltar): de vuelta a 0 deshaciendo el camino (al principio del tramo
        cortado, por la misma recta, y de ahi por los puntos de antes), como salida (arnes.py la acepta tras su
        parada). Con arnes.py amortiguado o sin el control no se puede: los brazos se quedan donde estan."""
        self.poner(self.etiqueta["mov"], self.etiqueta["T"], self.etiqueta["rep"], "salida")
        self.evento(f"PARADA ({motivo}): brazos a 0")
        self.parada = None
        self.b.en_salida_segura()
        try:
            for q in reversed(self.pila):
                self.b.ir_a({j: float(v) for j, v in zip(BRAZOS, q)}, 10.0)
            self.evento("brazos a 0")
        except (Parada, KeyboardInterrupt) as ex:
            self.evento(f"no se pudieron llevar a 0 ({ex}): arnes.py los sostiene donde estan; si estan sobre la "
                        "mesa, manipulacion.py --arnes --recuperar")


class _Medido:
    """El "robot" de BrazoArnes: lo medido que llega de arnes.py (no hace falta DDS)."""

    def __init__(self):
        self.curva = None

    def q(self):
        u = self.curva.ultimo if self.curva is not None else None
        return np.asarray(u["q"], float) if u else np.zeros(27)


def main():
    cfg = config.cargar()
    movs_todos = movimientos_definidos()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duraciones", type=float, nargs="+", default=DURACIONES, help="s; nunca por debajo de 8")
    ap.add_argument("--movimientos", nargs="+", default=["observacion", "alcance_izq"], choices=sorted(movs_todos))
    ap.add_argument("--quieto", type=float, default=8.0, help="s quietos antes, en la pose y al volver")
    ap.add_argument("--repeticiones", type=int, default=1)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--max-incl", type=float, default=3.0, help="grados de roll o pitch desde el inicio: para")
    ap.add_argument("--max-yaw", type=float, default=3.0, help="grados de yaw desde el inicio: para")
    ap.add_argument("--sim", action="store_true", help="contra arnes.py --sim en la PC: sin palabra ni gemelo")
    ap.add_argument("--rapido", action="store_true", help="--sim: 3 s quietos")
    a = ap.parse_args()
    if min(a.duraciones) < 8.0:
        sys.exit("Ningun movimiento por debajo de 8 s (seccion 10 del PDF).")
    if a.rapido and not a.sim:
        sys.exit("--rapido es de la simulacion (--sim).")
    movs = {k: movs_todos[k] for k in a.movimientos}
    print(f"\n  Curva de pasos: {', '.join(movs)} a {', '.join(f'{d:g}' for d in sorted(a.duraciones, reverse=True))} s,"
          f" {a.repeticiones} vez/veces, {a.quieto if not a.rapido else 3.0:g} s quietos.")
    if not a.sim:
        print("  Comprobando los caminos en el gemelo con config/montaje.yaml ...")
        malos = caminos_con_choques(cfg, config.cargar_montaje(), movs)
        if malos:
            for k, lineas in malos.items():
                print(f"    {k}: " + "\n      ".join(lineas))
            sys.exit("  Hay caminos con choques o demasiado cerca: no se mueve nada.")
        print("  Caminos limpios.")
    from manip.brazo_arnes import BrazoArnes
    medido = _Medido()
    try:
        b = BrazoArnes(medido, cfg, escala_tiempo=1.0, nombre="curva_pasos.py")
    except RuntimeError as ex:
        sys.exit(str(ex))
    e = b.medir() or {}
    if not e.get("listo"):
        b.parar()
        sys.exit(f"  arnes.py no da los brazos: {e.get('porque')}")
    carpeta = carpeta_tirada(config.ruta_datos(cfg), "pasos", "sim" if a.sim else None)
    curva = CurvaPasos(b, movs, a.duraciones, carpeta, quieto_s=3.0 if a.rapido else a.quieto, hz=a.hz,
                       repeticiones=a.repeticiones, max_incl_deg=a.max_incl, max_yaw_deg=a.max_yaw)
    medido.curva = curva
    resumen = {"inicio": time.strftime("%Y-%m-%d %H:%M:%S"), "args": vars(a), "movimientos": movs,
               "pose14": {k: [pose14(p).round(4).tolist() for p in m] for k, m in movs.items()}}
    try:
        from manip.dinamica import Dinamica
        from sim.escena import postura_arnes
        din, q0 = Dinamica(), postura_arnes(cfg)
        c0 = din.centro_de_masas(q0)
        resumen["dcom_m"] = {}
        for k, m in movs.items():
            q = q0.copy()
            q[BRAZOS] = pose14(m[-1])
            resumen["dcom_m"][k] = (din.centro_de_masas(q) - c0).round(4).tolist()
        print("  Centro de masas en la pose (cm, x adelante / y izquierda / z arriba): " +
              ", ".join(f"{k} {100 * np.array(v)[0]:+.1f}/{100 * np.array(v)[1]:+.1f}/{100 * np.array(v)[2]:+.1f}"
                        for k, v in resumen["dcom_m"].items()))
    except Exception as ex:                                     # el modelo es para el informe, no para moverse
        print(f"  (sin el centro de masas: {ex!r})")
    if not a.sim:
        print("  Robot de pie en el arnes (arnes.py nivelado), nadie al alcance de los brazos, operador con L2+B. "
              "ESPACIO: parar y brazos a 0.")
        if input(f"  Escribe {cfg['supervisor']['palabra']} para empezar: ").strip() != cfg["supervisor"]["palabra"]:
            b.parar()
            print("Cancelado.")
            return
    from manip.teclado import Teclado
    teclado = Teclado(lambda: curva.pedir_parada("tecla ESPACIO"))
    teclado.abrir()
    t0 = time.monotonic()
    try:
        motivo = curva.correr()
    finally:
        teclado.cerrar()
        try:
            b.parar()
        except RuntimeError as ex:
            print(f"ATENCION: {ex}")
    resumen.update(fin=motivo, duracion_s=round(time.monotonic() - t0, 1), hechos=curva.hechos, carpeta=carpeta)
    guardar_json(os.path.join(carpeta, "resumen.json"), resumen)
    print(f"\n  {motivo}. Registro en {carpeta}\n  Analisis: python3 herramientas/analizar_pasos.py {carpeta}")


if __name__ == "__main__":
    main()
