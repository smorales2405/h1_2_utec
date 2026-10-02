#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Editor de poses del H1-2 CON LAS MANOS Inspire RH56DFTP en MuJoCo, con los
# sliders del visor.
#
# Parte de code_cap/pc/simulacion_mujoco_h1_2/.../editor_poses_mujoco_h1_2.py, que
# no se toca. Cambia el modelo: el URDF del H1-2 con las manos de este repositorio
# (ros_h1_2_ws/src/h1_2_inspire_description), cargado por modelo_h1_2_manos.py.
#
# Igual que el original: robot LEVANTADO (pelvis fija a 1.4 m), SIN GRAVEDAD, sin
# contactos ni actuadores en el visor; con la simulación en PAUSA (barra
# espaciadora) los sliders del panel "Joint" mueven las juntas. Torso y brazos
# (12-26) quedan limitados a los límites del selector del robot real, leídos de su
# código; los dedos, a los del URDF (0 = abierta, q_max = cerrada).
#
# Lo nuevo:
#   - Las manos empiezan abiertas (q = 0). Se mueven con los sliders de sus juntas
#     actuadas (*_little_1 ... *_thumb_swing); los de las falanges acopladas
#     (*_2, *_3) no hacen nada: el editor las arrastra con su junta actuada.
#   - mano <gesto> izq|der|ambas: abierta, cerrada, pulgar_arriba, senalar
#     (definidos en h1_2_joint_control/scripts/selector_poses_manos/gestos_mano.py).
#   - c captura también los dedos; cero también los pone a 0; espejo ... manos
#     también copia la mano.
#   - Colisiones: una segunda copia del modelo, con contactos, comprueba la postura
#     del visor y avisa en la terminal de los choques (brazo con brazo, brazo con
#     cuerpo, mano con cualquier parte) y de lo que queda a menos del margen. Lo
#     que choca se ve en rojo.
#
# Las rutinas se guardan para el selector del robot real con manos:
#   h1_2_joint_control/scripts/selector_poses_manos/h1_2_robot_selector_manos.py (en poses/)
#
#   python3 editor_poses_mujoco_h1_2_manos.py
#   python3 editor_poses_mujoco_h1_2_manos.py --poses /otra/carpeta --margen 0.03
# -----------------------------------------------------------------------------

import argparse
import ast
import json
import math
import os
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import mujoco
import mujoco.viewer

import modelo_h1_2_manos as mod

AQUI = Path(__file__).resolve().parent
SELECTOR_DIR = mod.REPO / "h1_2_joint_control" / "scripts" / "selector_poses_manos"
SELECTOR = SELECTOR_DIR / "h1_2_robot_selector_manos.py"
POSES_DEFECTO = SELECTOR_DIR / "poses"
BORRADORES = AQUI / "borradores"

sys.path.insert(0, str(SELECTOR_DIR))
import gestos_mano as gm                          # noqa: E402
from conversion_angle_set import CLAVES, Q_MAX    # noqa: E402

DUR_PASO_DEFECTO = 2.0      # s
DUR_GESTO_DEFECTO = 1.0     # s, para 'mano'
DT_ANIMACION = 0.02         # s
PERIODO_VIGILANCIA = 0.05   # s
JUNTAS = list(range(12, 27))
NOMBRES = {
    12: "torso",
    13: "izq hombro pitch", 14: "izq hombro roll", 15: "izq hombro yaw", 16: "izq codo",
    17: "izq muneca roll", 18: "izq muneca pitch", 19: "izq muneca yaw",
    20: "der hombro pitch", 21: "der hombro roll", 22: "der hombro yaw", 23: "der codo",
    24: "der muneca roll", 25: "der muneca pitch", 26: "der muneca yaw",
}
# DDS: junta i del mensaje = junta del modelo con este nombre (mismo orden, sin huecos)
NOMBRE_MJCF = {
    12: "torso_joint",
    13: "left_shoulder_pitch_joint", 14: "left_shoulder_roll_joint", 15: "left_shoulder_yaw_joint",
    16: "left_elbow_joint", 17: "left_wrist_roll_joint", 18: "left_wrist_pitch_joint",
    19: "left_wrist_yaw_joint",
    20: "right_shoulder_pitch_joint", 21: "right_shoulder_roll_joint", 22: "right_shoulder_yaw_joint",
    23: "right_elbow_joint", 24: "right_wrist_roll_joint", 25: "right_wrist_pitch_joint",
    26: "right_wrist_yaw_joint",
}
# Espejo izquierda <-> derecha: roll, yaw y wrist_roll cambian de signo (como en caja_cuadrado.py)
SIGNO_ESPEJO = {0: 1, 1: -1, 2: -1, 3: 1, 4: -1, 5: 1, 6: -1}

AYUDA = f"""
En el visor: BARRA ESPACIADORA para pausar; con la pausa, los sliders del panel "Joint"
(derecha) mueven las juntas. Se guardan torso y brazos (12-26) y los 6 DOF de cada mano
(sliders *_little_1, *_ring_1, *_middle_1, *_index_1, *_thumb_1, *_thumb_swing).
Comandos aquí:
  c [nombre] [t=2]          capturar la postura del visor (brazos y manos) como paso
  l                         listar las 15 juntas y las dos manos (grados) y sus límites
  v                         ver los pasos capturados
  b                         borrar el último paso
  ir <n>                    poner el paso n en el visor (para retocarlo)
  reemplazar <n>            sustituir el paso n por la postura actual
  p                         previsualizar la rutina animada, comprobando colisiones
  mano <gesto> izq|der|ambas [t=1]   gestos: {', '.join(gm.GESTOS)}
  espejo izq|der [manos]    copiar el brazo izquierdo al derecho (o al revés); con 'manos', la mano también
  cero                      torso, brazos y manos a 0 (manos abiertas)
  col                       colisiones de la postura actual
  cargar <n|fichero>        abrir una rutina de la carpeta de poses (sustituye a los pasos actuales)
  s <nombre>                guardar la rutina en la carpeta de poses (la ve el selector)
  h                         esta ayuda
  x                         salir (o cerrar la ventana: lo no guardado va a borradores/)
"""


def limites_del_selector():
    """LIMITES y MARGEN del selector real, leidos de su codigo sin importarlo (no hace falta el SDK)."""
    arbol = ast.parse(SELECTOR.read_text(encoding="utf-8"))
    lim, margen = None, None
    for nodo in arbol.body:
        if isinstance(nodo, ast.Assign) and isinstance(nodo.targets[0], ast.Name):
            if nodo.targets[0].id == "LIMITES":
                lim = ast.literal_eval(nodo.value)
            elif nodo.targets[0].id == "MARGEN":
                margen = ast.literal_eval(nodo.value)
    if lim is None or margen is None:
        raise RuntimeError(f"no encuentro LIMITES/MARGEN en {SELECTOR}")
    return {j: (lo + margen, hi - margen) for j, (lo, hi) in lim.items()}


def texto_mano(q):
    nombre = gm.nombre_gesto(q)
    if nombre:
        return nombre
    return "propia (" + " ".join(f"{math.degrees(q[k]):.0f}" for k in CLAVES) + ")"


class Editor:
    def __init__(self, poses_dir: Path, margen: float):
        self.m, m_col = mod.modelos()
        self.d = mujoco.MjData(self.m)
        self.poses_dir = poses_dir
        self.lim = limites_del_selector()
        self.qadr = {}
        for j, nombre in NOMBRE_MJCF.items():
            jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, nombre)
            if jid < 0:
                raise RuntimeError(f"el modelo no tiene la junta {nombre}")
            self.qadr[j] = self.m.jnt_qposadr[jid]
            # sliders limitados a lo que el selector deja hacer en el robot
            lo = max(self.lim[j][0], self.m.jnt_range[jid][0])
            hi = min(self.lim[j][1], self.m.jnt_range[jid][1])
            self.m.jnt_range[jid] = [lo, hi]
            self.m.jnt_limited[jid] = 1
        # manos: las 6 juntas actuadas de cada lado, con el rango del URDF (= el de la conversión)
        self.qadr_mano = {}
        for lado, prefijo in gm.LADOS.items():
            self.qadr_mano[lado] = {}
            for k, qmax in zip(CLAVES, Q_MAX):
                jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, f"{prefijo}_{k}_joint")
                if jid < 0:
                    raise RuntimeError(f"el modelo no tiene la junta {prefijo}_{k}_joint")
                if abs(self.m.jnt_range[jid][1] - qmax) > 1e-6:
                    raise RuntimeError(f"{prefijo}_{k}_joint: el URDF cierra en {self.m.jnt_range[jid][1]} "
                                       f"y conversion_angle_set en {qmax}")
                self.qadr_mano[lado][k] = self.m.jnt_qposadr[jid]
        self.mimic = mod.Mimic(self.m)
        self.colisiones = mod.Colisiones(m_col, margen)
        self.rgba_original = self.m.geom_rgba.copy()
        self.visuales_de = {}
        for g in range(self.m.ngeom):
            if not mod.es_de_colision(self.m, g):
                self.visuales_de.setdefault(mod.nombre_cuerpo(self.m, self.m.geom_bodyid[g]), []).append(g)
        mujoco.mj_forward(self.m, self.d)
        self.pasos = []
        self.guardado = True
        self.terminar = False
        self.animando = False           # la vigilancia no informa mientras 'p' o 'mano' animan
        self.ultimo_informe = None

    # --------------------------------------------------------- postura
    def leer(self):
        return {j: float(self.d.qpos[self.qadr[j]]) for j in JUNTAS}

    def leer_manos(self):
        return {lado: {k: float(self.d.qpos[a]) for k, a in adr.items()} for lado, adr in self.qadr_mano.items()}

    def poner(self, pose: dict, manos: dict | None = None):
        for j, q in pose.items():
            j = int(j)
            if j in self.qadr:
                lo, hi = self.lim[j]
                self.d.qpos[self.qadr[j]] = min(max(float(q), lo), hi)
        for lado, q in (manos or {}).items():
            q = gm.normalizar(q, self.leer_manos()[lado])
            for k, a in self.qadr_mano[lado].items():
                self.d.qpos[a] = q[k]
        self.mimic.aplicar(self.d.qpos)
        self.d.qvel[:] = 0.0

    # --------------------------------------------------------- colisiones
    def revisar(self):
        return self.colisiones.revisar(self.d.qpos.copy())

    def resaltar(self, hallazgos):
        """Pinta de rojo los cuerpos que chocan; el resto, con su color."""
        rojos = {c for a, b, dist in hallazgos if dist < 0 for c in (a, b)}
        self.m.geom_rgba[:] = self.rgba_original
        for cuerpo in rojos:
            for g in self.visuales_de.get(cuerpo, []):
                self.m.geom_rgba[g] = mod.ROJO

    def vigilar(self):
        """Hilo: arrastra las falanges acopladas tras los sliders y avisa de las colisiones."""
        while not self.terminar:
            time.sleep(PERIODO_VIGILANCIA)
            if self.animando:
                continue
            self.mimic.aplicar(self.d.qpos)
            hallazgos = self.revisar()
            self.resaltar(hallazgos)
            lineas = mod.informe(hallazgos)
            if lineas != self.ultimo_informe:
                if lineas:
                    print("\n" + "\n".join(lineas))
                elif self.ultimo_informe:
                    print("\n[OK] Sin colisiones.")
                self.ultimo_informe = lineas

    def mostrar_colisiones(self, titulo="Postura actual"):
        lineas = mod.informe(self.revisar())
        print(f"[{titulo}] " + ("sin colisiones." if not lineas else "\n  " + "\n  ".join(lineas)))

    # --------------------------------------------------------- animación (p y mano)
    def animar(self, ini_brazos, fin_brazos, ini_manos, fin_manos, duracion):
        """Lleva el visor de una postura a otra como lo hará el robot: brazos en línea recta
        durante `duracion`; cada mano por sus fases (gestos_mano), en paralelo. Devuelve
        {(cuerpo_a, cuerpo_b): peor distancia} de lo encontrado por el camino."""
        plan = {}
        for lado in fin_manos:
            fases = gm.fases(ini_manos[lado], fin_manos[lado])
            plan[lado] = list(zip(fases, gm.duraciones(duracion, len(fases))))
        total = max([duracion] + [sum(t for _, t in p) for p in plan.values()])
        peor = {}
        t0 = time.time()
        while True:
            t = time.time() - t0
            s = min(1.0, t / max(duracion, 0.05))
            brazos = {j: ini_brazos[j] + (fin_brazos.get(j, ini_brazos[j]) - ini_brazos[j]) * s for j in JUNTAS}
            manos = {}
            for lado, fases in plan.items():
                q, resto = dict(ini_manos[lado]), t
                for objetivo, tf in fases:
                    if resto >= tf:
                        q, resto = objetivo, resto - tf
                    else:
                        q = gm.interpolar(q, objetivo, resto / max(tf, 1e-3))
                        break
                manos[lado] = q
            self.poner(brazos, manos)
            hallazgos = self.revisar()
            self.resaltar(hallazgos)
            for a, b, dist in hallazgos:
                peor[(a, b)] = min(peor.get((a, b), dist), dist)
            if t >= total or self.terminar:
                break
            time.sleep(DT_ANIMACION)
        return peor

    @staticmethod
    def resumen(peor):
        lineas = mod.informe([(a, b, dist) for (a, b), dist in peor.items()])
        return "sin colisiones" if not lineas else "\n       " + "\n       ".join(lineas)

    # --------------------------------------------------------- comandos
    def listar(self):
        pose = self.leer()
        print("\n  junta  nombre              grados   límites del selector")
        for j in JUNTAS:
            lo, hi = self.lim[j]
            print(f"  {j:5d}  {NOMBRES[j]:18s} {math.degrees(pose[j]):+7.1f}   "
                  f"[{math.degrees(lo):+.0f}, {math.degrees(hi):+.0f}]")
        print(f"\n  mano  {'':4s}" + "".join(f"{gm.NOMBRE_DOF[k]:>14s}" for k in CLAVES) + "   gesto")
        for lado, q in self.leer_manos().items():
            print(f"  {lado:4s}  {'':4s}" + "".join(f"{math.degrees(q[k]):>14.1f}" for k in CLAVES)
                  + f"   {gm.nombre_gesto(q) or '-'}")
        print(f"  máx.  {'':4s}" + "".join(f"{math.degrees(x):>14.1f}" for x in Q_MAX) + "   (0 = abierta)\n")

    def paso_desde_visor(self, nombre, dur):
        return {"nombre": nombre, "duracion": dur,
                "posiciones": {str(j): round(q, 4) for j, q in self.leer().items()},
                "manos": {lado: {k: round(x, 4) for k, x in q.items()} for lado, q in self.leer_manos().items()}}

    def capturar(self, nombre, dur):
        self.pasos.append(self.paso_desde_visor(nombre or f"Paso {len(self.pasos) + 1}", dur))
        self.guardado = False
        manos = self.pasos[-1]["manos"]
        print(f"[OK] Paso {len(self.pasos)} '{self.pasos[-1]['nombre']}' ({dur:.1f} s), "
              f"manos: izq {texto_mano(manos['izq'])}, der {texto_mano(manos['der'])}.")
        lineas = mod.informe(self.revisar())
        if lineas:
            print("[AVISO] El paso capturado tiene colisiones:\n  " + "\n  ".join(lineas))

    def ver(self):
        if not self.pasos:
            print("[INFO] Sin pasos.")
            return
        for n, p in enumerate(self.pasos, 1):
            r = ", ".join(f"{k}={math.degrees(v):+.0f}" for k, v in p["posiciones"].items() if abs(v) > math.radians(2))
            manos = p.get("manos")
            m = (f" | manos: izq {texto_mano(manos['izq'])}, der {texto_mano(manos['der'])}"
                 if manos else " | manos: sin cambio")
            print(f"  {n:02d}. {p['nombre']:<20s} {p['duracion']:.1f} s   {r or 'todo ~0'}{m}")

    def ir(self, n):
        paso = self.pasos[n - 1]
        self.poner(paso["posiciones"], paso.get("manos"))

    def previsualizar(self):
        if not self.pasos:
            print("[INFO] Sin pasos.")
            return
        self.animando = True
        try:
            brazos, manos = self.leer(), self.leer_manos()
            for n, p in enumerate(self.pasos, 1):
                fin_b = {int(k): v for k, v in p["posiciones"].items()}
                fin_m = {lado: gm.normalizar(q, manos[lado]) for lado, q in (p.get("manos") or {}).items()}
                print(f"  -> {n:02d}. {p['nombre']} ({p['duracion']:.1f} s): ", end="", flush=True)
                peor = self.animar(brazos, fin_b, manos, fin_m, p["duracion"])
                print(self.resumen(peor))
                brazos = {j: fin_b.get(j, brazos[j]) for j in JUNTAS}
                manos = {**manos, **fin_m}
                if self.terminar:
                    break
        finally:
            self.ultimo_informe = None
            self.animando = False
        print("[INFO] Fin de la previsualización (el visor queda en el último paso).")

    def mano(self, nombre, lado, dur):
        objetivo = gm.gesto(nombre)
        lados = list(gm.LADOS) if lado == "ambas" else [lado]
        self.animando = True
        try:
            brazos, manos = self.leer(), self.leer_manos()
            peor = self.animar(brazos, brazos, manos, {l: objetivo for l in lados}, dur)
        finally:
            self.ultimo_informe = None
            self.animando = False
        print(f"[OK] Mano {lado}: {gm.ALIAS.get(nombre, nombre)}. Camino: {self.resumen(peor)}")

    def espejo(self, lado, con_manos):
        pose = self.leer()
        origen, destino = (13, 20) if lado == "izq" else (20, 13)
        otro = "der" if lado == "izq" else "izq"
        manos = {otro: gm.espejo(self.leer_manos()[lado])} if con_manos else None
        self.poner({destino + k: SIGNO_ESPEJO[k] * pose[origen + k] for k in range(7)}, manos)
        print(f"[OK] Brazo {'derecho' if lado == 'izq' else 'izquierdo'}{' y su mano' if con_manos else ''} "
              f"= espejo del {lado}.")

    def cargar(self, arg):
        ruta = Path(arg).expanduser()
        if not ruta.is_file():
            candidatas = sorted(self.poses_dir.glob(f"{arg}_*.json")) or sorted(self.poses_dir.glob(f"{arg}*.json"))
            if not candidatas:
                print(f"[ERROR] No encuentro '{arg}' en {self.poses_dir}")
                return
            ruta = candidatas[0]
        rutina = json.loads(ruta.read_text(encoding="utf-8"))
        if rutina.get("robot") not in (None, "unitree_h1_2"):
            print(f"[ERROR] {ruta.name} es para '{rutina.get('robot')}', no para el H1-2.")
            return
        pasos = []
        for n, p in enumerate(rutina.get("pasos", []), 1):
            paso = {"nombre": p.get("nombre", f"Paso {n}"), "duracion": float(p.get("duracion", 1.0)),
                    "posiciones": dict(p.get("posiciones", {}))}
            if p.get("manos"):
                paso["manos"] = {lado: gm.normalizar(q) for lado, q in p["manos"].items() if lado in gm.LADOS}
            pasos.append(paso)
        self.pasos = pasos
        self.guardado = True
        sin_manos = sum(1 for p in self.pasos if "manos" not in p)
        print(f"[OK] Cargada {ruta.name}: {len(self.pasos)} pasos"
              f"{f' ({sin_manos} sin manos: las dejan como estén)' if sin_manos else ''}. "
              f"'p' para verla, 'ir <n>' para retocar.")
        if self.pasos:
            self.ir(1)

    def siguiente_numero(self):
        nums = []
        for f in self.poses_dir.glob("*.json"):
            mo = re.match(r"^\s*(\d+)", f.stem)
            if mo:
                nums.append(int(mo.group(1)))
        return max(nums, default=-1) + 1

    def guardar(self, nombre):
        if not self.pasos:
            print("[AVISO] No hay pasos: usa 'c' primero.")
            return
        base = re.sub(r"[^a-zA-Z0-9_-]+", "_", nombre.strip()).strip("_") or "rutina"
        self.poses_dir.mkdir(parents=True, exist_ok=True)
        ruta = self.poses_dir / f"{self.siguiente_numero()}_{base}.json"
        rutina = {
            "nombre_rutina": base,
            "fecha_creacion": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "robot": "unitree_h1_2",
            "modelo": "h1_2_27dof_manos_rh56dftp",
            "requiere_robot_quieto": True,
            "descripcion": ("Creada con editor_poses_mujoco_h1_2_manos.py (sliders de MuJoCo). posiciones: "
                            "juntas 12-26 en radianes; manos: las 6 juntas actuadas del URDF en radianes "
                            "(0 = abierta), se pasan a ANGLE_SET con conversion_angle_set.py."),
            "numero_pasos": len(self.pasos),
            "pasos": self.pasos,
        }
        ruta.write_text(json.dumps(rutina, indent=2, ensure_ascii=False), encoding="utf-8")
        self.guardado = True
        print(f"[OK] Guardada: {ruta}\n     En el selector sale con el número {ruta.name.split('_')[0]}.")

    # --------------------------------------------------------- bucle
    def ejecutar(self, linea):
        partes = linea.split()
        cmd, t, resto = partes[0].lower(), None, []
        for p in partes[1:]:
            if p.lower().startswith("t="):
                t = float(p[2:])
            else:
                resto.append(p)
        if cmd == "h":
            print(AYUDA)
        elif cmd == "l":
            self.listar()
        elif cmd == "c":
            self.capturar(" ".join(resto), t or DUR_PASO_DEFECTO)
        elif cmd == "v":
            self.ver()
        elif cmd == "b":
            if self.pasos:
                print(f"[OK] Borrado '{self.pasos.pop()['nombre']}'.")
                self.guardado = False
        elif cmd == "ir":
            n = int(resto[0])
            self.ir(n)
            print(f"[OK] En el visor: paso {n} '{self.pasos[n - 1]['nombre']}'.")
        elif cmd == "reemplazar":
            n = int(resto[0])
            viejo = self.pasos[n - 1]
            self.pasos[n - 1] = self.paso_desde_visor(viejo["nombre"], t or viejo["duracion"])
            self.guardado = False
            print(f"[OK] Paso {n} '{viejo['nombre']}' reemplazado por la postura actual.")
        elif cmd == "p":
            self.previsualizar()
        elif cmd == "mano":
            if len(resto) != 2 or resto[1] not in ("izq", "der", "ambas"):
                raise ValueError(f"formato: mano <{'|'.join(gm.GESTOS)}> izq|der|ambas [t=1]")
            try:
                self.mano(resto[0].lower(), resto[1], t or DUR_GESTO_DEFECTO)
            except KeyError as e:
                raise ValueError(e.args[0]) from None
        elif cmd == "espejo":
            if not resto or resto[0] not in ("izq", "der") or resto[1:] not in ([], ["manos"]):
                raise ValueError("formato: espejo izq|der [manos]")
            self.espejo(resto[0], resto[1:] == ["manos"])
        elif cmd == "cero":
            self.poner({j: 0.0 for j in JUNTAS}, {lado: gm.gesto("abierta") for lado in gm.LADOS})
            print("[OK] Torso y brazos a 0, manos abiertas.")
        elif cmd == "col":
            self.mostrar_colisiones()
        elif cmd == "cargar":
            self.cargar(" ".join(resto))
        elif cmd == "s":
            self.guardar(" ".join(resto) or input("Nombre de la rutina: "))
        elif cmd == "x":
            if self.pasos and not self.guardado:
                if input("Hay pasos sin guardar. ¿Salir igualmente? (SI/no): ").strip().upper() != "SI":
                    return True
            return False
        else:
            print("[?] Comando no reconocido. 'h' para la ayuda.")
        return True

    def menu(self):
        """Hilo de la terminal; el visor ocupa el hilo principal."""
        time.sleep(1.0)
        print(AYUDA)
        while not self.terminar:
            try:
                linea = input("editor> ").strip()
            except EOFError:
                break
            if not linea:
                continue
            try:
                if not self.ejecutar(linea):
                    break
            except (ValueError, IndexError, KeyError, OSError) as e:
                print(f"[ERROR] {e}")
        self.terminar = True
        print("[INFO] Saliendo.")
        os._exit(0)                       # cierra tambien el visor

    def borrador(self):
        if self.pasos and not self.guardado:
            BORRADORES.mkdir(parents=True, exist_ok=True)
            ruta = BORRADORES / f"borrador_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            ruta.write_text(json.dumps({"robot": "unitree_h1_2", "pasos": self.pasos}, indent=2,
                                       ensure_ascii=False), encoding="utf-8")
            print(f"[INFO] {len(self.pasos)} pasos sin guardar -> {ruta}  (cargar {ruta} para seguir)")

    def correr(self):
        print(f"[INFO] Poses: {self.poses_dir}")
        print(f"[INFO] Colisiones: margen {self.colisiones.margen * 1000:.0f} mm. Pares excluidos (ya se tocan "
              f"en la pose cero): {', '.join('/'.join(p) for p in sorted(self.colisiones.excluidos))}")
        threading.Thread(target=self.menu, daemon=True).start()
        threading.Thread(target=self.vigilar, daemon=True).start()
        mujoco.viewer.launch(self.m, self.d)      # bloquea hasta cerrar la ventana
        self.terminar = True
        self.borrador()
        print("\n[INFO] Visor cerrado.")


def main():
    ap = argparse.ArgumentParser(description="Editor de poses del H1-2 con manos, con los sliders de MuJoCo.")
    ap.add_argument("--poses", default=str(POSES_DEFECTO), help="carpeta donde se guardan las rutinas")
    ap.add_argument("--margen", type=float, default=mod.MARGEN_DEFECTO,
                    help="m: avisar también de lo que quede a menos de esto sin tocarse (0 = solo choques)")
    a = ap.parse_args()
    Editor(Path(a.poses).expanduser().resolve(), a.margen).correr()


if __name__ == "__main__":
    main()
