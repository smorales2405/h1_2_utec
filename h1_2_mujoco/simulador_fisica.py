#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Modo física del editor de poses del H1-2 con manos:  ./editor_mujoco.sh --fisica
#
# El mismo robot que el editor (modelo_h1_2_manos.modelo_fisica), pero con
# gravedad, contactos y motores, colgado de un pórtico por dos cuerdas a la placa
# superior del torso. Aquí no se diseña: se EJECUTAN rutinas y gestos como lo haría
# el selector real (selector_poses_manos/h1_2_robot_selector_manos.py), y se ve
# qué hace el cuerpo:
#   - torso y brazos con un PD por junta, con las ganancias del selector (Kp, Kd,
#     leídas de su código) y sin compensar la gravedad: el brazo cede como en el
#     robot; las piernas, sujetas en su postura; límites, interpolación lineal y
#     primer movimiento de al menos 3 s, como en el selector;
#   - cada mano recibe postura a HZ_MANO y su "firmware" la sigue con el retardo y
#     la velocidad medidos en la caracterización; 'cerrada' por fases, esperando a
#     que cada una termine; si un dedo empuja más de FMAX_MANO gf, se abre y la
#     mano se para (como en el selector);
#   - los contactos (brazo con cuerpo, mano con cualquier parte, con el suelo) se
#     informan por paso, con su fuerza, y el error entre lo mandado y lo alcanzado.
#
# Empieza como el selector real tras MOVER: colgado y sujetando la POSE SEGURA
# (poses/0_pose_segura.json, manos abiertas), la postura por defecto. 'p' ejecuta
# la rutina desde ahí y termina en la pose segura (la añade si la rutina no acaba
# en ella), y se queda sujetándola: no pasa a amortiguación.
#
# Teclas en el visor (como unitree_mujoco):  7 sube el pórtico 5 cm, 8 lo baja,
# 9 suelta las cuerdas o vuelve a engancharlas. La altura de la pelvis sale en la
# terminal cuando el robot se asienta.
# -----------------------------------------------------------------------------

import math
import os
import threading
import time
from collections import deque

import mujoco
import mujoco.viewer
import numpy as np

import modelo_h1_2_manos as mod
import editor_poses_mujoco_h1_2_manos as ed

gm = ed.gm
CLAVES = ed.CLAVES
from conversion_angle_set import rad_a_angle_set   # noqa: E402  (en sys.path desde el editor)

SEL = ed.constantes_del_selector("Kp", "Kd", "DURACION_MINIMA_PRIMER_MOVIMIENTO", "HZ_MANO", "FMAX_MANO",
                                 "TOL_LLEGADA", "T_LLEGADA_MAX")
KP, KD = np.array(SEL["Kp"], float), np.array(SEL["Kd"], float)
DUR_MIN = SEL["DURACION_MINIMA_PRIMER_MOVIMIENTO"]

PASOS_POR_FOTOGRAMA = 8         # 8 x 2 ms = 16 ms de simulación por fotograma: tiempo real
VEL_PORTICO = 0.25              # m/s al subir o bajar con 7/8
PASO_PORTICO = 0.05             # m por pulsación
T_ASENTAR = 1.5                 # s tras la última tecla para informar de la altura
DUR_CERO = 3.0                  # s para 'cero'
TECLA_SUBIR, TECLA_BAJAR, TECLA_CUERDAS = ord("7"), ord("8"), ord("9")
G = 9.80665

AYUDA = f"""
MODO FÍSICA: el robot cuelga del pórtico y se mueve como lo mandaría el selector real.
Empieza sujetando la pose segura (la postura por defecto); cada rutina termina en ella.
Teclas en el visor: 7 sube el pórtico 5 cm, 8 lo baja, 9 suelta / engancha las cuerdas.
Comandos aquí:
  cargar <n|fichero>        abrir una rutina de la carpeta de poses
  v                         ver los pasos de la rutina
  p                         ejecutar la rutina entera; termina en la pose segura (como el selector)
  ir <n>                    ejecutar solo el paso n
  mano <gesto> izq|der|ambas [t=1]   gestos: {', '.join(gm.GESTOS)}
  cero                      torso y brazos a 0 y manos abiertas ({DUR_CERO:.0f} s)
  l                         juntas: mandado, medido y error (grados)
  col                       contactos ahora mismo
  segura                    ir a la pose segura
  reset                     volver al principio: colgado a la altura inicial, en la pose segura
  h                         esta ayuda
  x                         salir (o cerrar la ventana)
"""


def recortar(j, q, lim):
    lo, hi = lim[j]
    return min(max(float(q), lo), hi)


class ManoSim:
    """Una mano Inspire vista desde el selector: recibe posturas a HZ_MANO y su firmware las sigue con
    RETARDO_MANO de retraso y a velocidad limitada (servo de posición sobre esa consigna)."""

    def __init__(self, m, lado):
        self.lado = lado
        prefijo = gm.LADOS[lado]
        self.qadr = np.array([m.jnt_qposadr[m.joint(f"{prefijo}_{k}_joint").id] for k in CLAVES])
        self.act = np.array([m.actuator(f"{prefijo}_{k}").id for k in CLAVES])
        self.vel = np.array([mod.VEL_MANO[k] for k in CLAVES])
        # cuerpos de cada DOF, para la fuerza de contacto
        piezas = {0: ("little_1", "little_2"), 1: ("ring_1", "ring_2"), 2: ("middle_1", "middle_2"),
                  3: ("index_1", "index_2"), 4: ("thumb_1", "thumb_2", "thumb_3"), 5: ("thumb_swing",)}
        self.dof_de_cuerpo = {m.body(f"{prefijo}_{p}").id: dof for dof, ps in piezas.items() for p in ps}
        self.reiniciar()

    def reiniciar(self):
        self.q_cmd = np.zeros(6)                # lo último que mandó el selector
        self.consigna = np.zeros(6)             # lo que persigue el firmware
        self.historial = deque([(-1.0, np.zeros(6))])
        self.fases, self.duraciones = [], []
        self.detenida = ""
        # se empieza ya en la pose segura: el primer movimiento del selector (>= 3 s, el que va a ella) está hecho
        self.primer_movimiento = False
        self.mensajes = []

    def poner(self, q, d):
        """Coloca la mano en q (rad, orden CLAVES) sin moverla: postura, orden y consigna."""
        self.q_cmd, self.consigna = q.copy(), q.copy()
        self.historial = deque([(-1.0, q.copy())])
        d.qpos[self.qadr] = q

    @property
    def ocupada(self):
        return bool(self.fases)

    def tramo(self, q_fin, duracion, t):
        """Como Mano.tramo del selector: desde lo último mandado; 'cerrada', por fases."""
        if self.detenida:
            return
        q_fin = gm.normalizar(q_fin, dict(zip(CLAVES, self.q_cmd)))
        if self.primer_movimiento:
            duracion = max(duracion, DUR_MIN)
            self.primer_movimiento = False
        objetivos = gm.fases(dict(zip(CLAVES, self.q_cmd)), q_fin)
        self.fases = [np.array([o[k] for k in CLAVES]) for o in objetivos]
        self.duraciones = gm.duraciones(duracion, len(objetivos))
        self.secuencia = gm.nombre_gesto(q_fin) == "cerrada"
        self.k, self.t_fase, self.q0, self.esperando = 0, t, self.q_cmd.copy(), None

    def _siguiente_fase(self, t):
        self.k += 1
        self.t_fase, self.q0, self.esperando = t, self.q_cmd.copy(), None
        if self.k >= len(self.fases):
            self.fases = []

    def _mandar(self, q, t):
        self.q_cmd = q
        self.historial.append((t, q.copy()))

    def tick(self, t, d, fuerzas_gf):
        """A HZ_MANO, como el hilo de la mano del selector: protección, interpolación y espera de llegada."""
        malos = [i for i in range(6) if fuerzas_gf[i] > SEL["FMAX_MANO"]]
        if malos and not self.detenida:
            q = self.q_cmd.copy()
            q[malos] = 0.0                          # ANGLE_SET 1000: esos dedos se abren
            self._mandar(q, t)
            self.fases = []
            self.detenida = (f"{[gm.NOMBRE_DOF[CLAVES[i]] for i in malos]} con "
                             f"{[round(fuerzas_gf[i]) for i in malos]} gf > {SEL['FMAX_MANO']}: se abren")
            self.mensajes.append(f"[MANOS] {self.lado} PARADA: {self.detenida}")
        if self.detenida or not self.fases:
            return
        objetivo, tf = self.fases[self.k], self.duraciones[self.k]
        if self.esperando is not None:              # fase de 'cerrada' mandada: que la mano llegue
            act = rad_a_angle_set(list(d.qpos[self.qadr]))
            meta = rad_a_angle_set(list(objetivo))
            if max(abs(a - b) for a, b in zip(act, meta)) <= SEL["TOL_LLEGADA"]:
                self._siguiente_fase(t)
            elif t - self.esperando > SEL["T_LLEGADA_MAX"]:
                self.detenida = f"una fase de 'cerrada' no terminó en {SEL['T_LLEGADA_MAX']:.0f} s"
                self.mensajes.append(f"[MANOS] {self.lado} PARADA: {self.detenida}")
                self.fases = []
            return
        s = 1.0 if tf <= 0 else min(1.0, (t - self.t_fase) / tf)
        self._mandar(self.q0 + (objetivo - self.q0) * s, t)
        if s >= 1.0:
            if self.secuencia:
                self.esperando = t
            else:
                self._siguiente_fase(t)

    def servo(self, t, dt, d):
        """Cada paso de física: la orden de hace RETARDO_MANO, seguida a velocidad limitada."""
        while len(self.historial) > 1 and self.historial[1][0] <= t - mod.RETARDO_MANO:
            self.historial.popleft()
        destino = self.historial[0][1]
        self.consigna += np.clip(destino - self.consigna, -self.vel * dt, self.vel * dt)
        d.ctrl[self.act] = self.consigna


class Simulador:
    def __init__(self, poses_dir):
        self.poses_dir = poses_dir
        self.m, self.excluidos = mod.modelo_fisica()
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.lim = ed.limites_del_selector()
        self.qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in mod.JUNTAS_CUERPO])
        self.dadr = np.array([m.jnt_dofadr[m.joint(n).id] for n in mod.JUNTAS_CUERPO])
        self.manos = {lado: ManoSim(m, lado) for lado in gm.LADOS}
        self.portico = m.body("portico").mocapid[0]
        self.z_portico0 = float(m.body("portico").pos[2])
        self.cuerdas = [m.tendon(f"cuerda_{l}").id for l in ("izq", "der")]
        self.largo_cuerda0 = m.tendon_range[self.cuerdas, 1].copy()
        self.suelo = m.geom("floor").id
        self.pies = {m.body(f"{l}_ankle_roll_link").id for l in ("left", "right")} | \
                    {m.body(f"{l}_ankle_pitch_link").id for l in ("left", "right")}
        self.cuerpo_parte = {b: mod.grupo(mod.nombre_cuerpo(m, b)) for b in range(m.nbody)}
        self.pasos = []
        self.segura = ed.pose_segura(poses_dir)
        self.mimic = mod.Mimic(m)
        self.lock = threading.RLock()
        self.terminar = False
        self.teclas = deque()
        self.registro = None
        self.reiniciar()

    # --------------------------------------------------------- estado
    def reiniciar(self):
        with self.lock:
            mujoco.mj_resetData(self.m, self.d)
            self.m.tendon_limited[self.cuerdas] = 1
            self.m.tendon_range[self.cuerdas, 1] = self.largo_cuerda0
            self.z_portico = self.z_portico0
            self.colgado = True
            self.q_ini = self.d.qpos[self.qadr].copy()
            self.q_fin = self.q_ini.copy()
            self.t0, self.T = 0.0, 0.0
            self.primer_movimiento = False      # ya en la pose segura: ver ManoSim.reiniciar
            for mano in self.manos.values():
                mano.reiniciar()
            if self.segura:                     # colgado y sujetando la pose segura
                for j, q in self.segura["posiciones"].items():
                    self.d.qpos[self.qadr[int(j)]] = q
                for lado, mano in self.manos.items():
                    mano.poner(np.array([self.segura["manos"][lado][k] for k in CLAVES]), self.d)
                self.mimic.aplicar(self.d.qpos)
                self.q_ini = self.d.qpos[self.qadr].copy()
                self.q_fin = self.q_ini.copy()
            self.t_informe = None
            self.t_tick = 0.0
            mujoco.mj_forward(self.m, self.d)

    def q_mandada(self, t):
        s = 1.0 if self.T <= 0 else min(max((t - self.t0) / self.T, 0.0), 1.0)
        return self.q_ini + (self.q_fin - self.q_ini) * s

    # --------------------------------------------------------- un paso de física
    def contactos(self):
        """{(cuerpo_a, cuerpo_b): fuerza normal N} de los contactos que interesan: todo salvo pierna con
        pierna o pelvis (no se mueven) y los pies con el suelo."""
        m, d = self.m, self.d
        salida, f = {}, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            p1, p2 = self.cuerpo_parte[b1], self.cuerpo_parte[b2]
            if {p1, p2} <= {"pierna izq", "pierna der", "pelvis"}:
                continue
            if "suelo" in (p1, p2) and ({b1, b2} & self.pies):
                continue
            mujoco.mj_contactForce(m, d, i, f)
            par = tuple(sorted((mod.nombre_cuerpo(m, b1), mod.nombre_cuerpo(m, b2))))
            salida[par] = salida.get(par, 0.0) + abs(f[0])
        return salida

    def fuerzas_dedos(self, mano):
        """Fuerza de contacto por DOF de una mano, en gf (como FORCE_ACT)."""
        m, d = self.m, self.d
        fuerzas, f = np.zeros(6), np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            for b in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]):
                dof = mano.dof_de_cuerpo.get(b)
                if dof is not None:
                    mujoco.mj_contactForce(m, d, i, f)
                    fuerzas[dof] += abs(f[0]) * 1000.0 / G
        return fuerzas

    def paso(self):
        m, d = self.m, self.d
        t = d.time
        # pórtico: hacia su altura objetivo a velocidad limitada
        z = d.mocap_pos[self.portico, 2]
        d.mocap_pos[self.portico, 2] = z + float(np.clip(self.z_portico - z, -VEL_PORTICO * m.opt.timestep,
                                                         VEL_PORTICO * m.opt.timestep))
        # torso, brazos y piernas: PD como el LowCmd del selector (dq = 0, tau = 0)
        q, dq = d.qpos[self.qadr], d.qvel[self.dadr]
        d.ctrl[:len(self.qadr)] = KP * (self.q_mandada(t) - q) - KD * dq
        # manos
        if t >= self.t_tick:
            self.t_tick = t + 1.0 / SEL["HZ_MANO"]
            for mano in self.manos.values():
                mano.tick(t, d, self.fuerzas_dedos(mano))
        for mano in self.manos.values():
            mano.servo(t, m.opt.timestep, d)
        mujoco.mj_step(m, d)
        if self.registro is not None:
            for par, fuerza in self.contactos().items():
                self.registro[par] = max(self.registro.get(par, 0.0), fuerza)
        if self.t_informe is not None and d.time >= self.t_informe:
            self.t_informe = None
            self.informar_altura()

    # --------------------------------------------------------- arnés y teclas
    def tecla(self, codigo):
        """Hilo del visor: solo se apunta; se atiende en el bucle de física."""
        if codigo in (TECLA_SUBIR, TECLA_BAJAR, TECLA_CUERDAS):
            self.teclas.append(codigo)

    def atender_teclas(self):
        while self.teclas:
            codigo = self.teclas.popleft()
            if codigo == TECLA_CUERDAS:
                if self.colgado:
                    self.m.tendon_limited[self.cuerdas] = 0
                    print("\n[ARNÉS] 9: cuerdas sueltas, el robot cae.")
                else:
                    # se enganchan con el largo que tienen ahora: sin tirón; luego 7 para subir
                    self.m.tendon_range[self.cuerdas, 1] = self.d.ten_length[self.cuerdas] + 1e-3
                    self.m.tendon_limited[self.cuerdas] = 1
                    print("\n[ARNÉS] 9: cuerdas enganchadas (7 para subir).")
                self.colgado = not self.colgado
            else:
                self.z_portico += PASO_PORTICO if codigo == TECLA_SUBIR else -PASO_PORTICO
            # informar cuando el pórtico haya llegado y el robot se haya asentado
            recorrido = abs(self.z_portico - self.d.mocap_pos[self.portico, 2])
            self.t_informe = self.d.time + recorrido / VEL_PORTICO + T_ASENTAR

    def informar_altura(self):
        d = self.d
        pies = any(self.suelo in (c.geom1, c.geom2) and
                   ({self.m.geom_bodyid[c.geom1], self.m.geom_bodyid[c.geom2]} & self.pies)
                   for c in d.contact[:d.ncon])
        estado = "colgado" if self.colgado else "SUELTO"
        print(f"\n[ARNÉS] pelvis a {d.qpos[2]:.2f} m ({estado}, pórtico a {self.z_portico:.2f} m)"
              f"{', pies en el suelo' if pies else ''}.")

    # --------------------------------------------------------- movimientos (hilo de la terminal)
    def mover(self, brazos, manos, duracion):
        """Como un paso del selector: brazos desde lo último mandado, manos en paralelo. Bloquea hasta que
        termina y devuelve (contactos con su fuerza máxima, error máximo de brazos, juntas recortadas)."""
        with self.lock:
            t = self.d.time
            self.q_ini = self.q_mandada(t)
            self.q_fin = self.q_ini.copy()
            recortados = []
            for k, v in (brazos or {}).items():
                j = int(k)
                if 12 <= j <= 26:
                    self.q_fin[j] = recortar(j, v, self.lim)
                    if abs(self.q_fin[j] - float(v)) > 1e-6:
                        recortados.append((j, round(float(v), 3), round(self.q_fin[j], 3)))
            dur = max(float(duracion), 0.001)
            if self.primer_movimiento:
                dur = max(dur, DUR_MIN)
                self.primer_movimiento = False
            self.t0, self.T = t, dur
            for lado, q in (manos or {}).items():
                self.manos[lado].tramo(q, float(duracion), t)
            self.registro = {}
        while not self.terminar:
            time.sleep(0.02)
            with self.lock:
                if self.d.time >= self.t0 + self.T and not any(m.ocupada for m in self.manos.values()):
                    break
        with self.lock:
            registro, self.registro = self.registro, None
            err = np.abs(self.q_fin - self.d.qpos[self.qadr])[12:27]
            mensajes = [msg for mano in self.manos.values() for msg in mano.mensajes]
            for mano in self.manos.values():
                mano.mensajes = []
        j = int(np.argmax(err)) + 12
        return registro, (j, float(err[j - 12])), recortados, mensajes

    @staticmethod
    def texto_contactos(registro):
        peor = {}
        for (a, b), f in registro.items():
            clave = tuple(sorted((mod.grupo(a), mod.grupo(b))))
            if clave not in peor or f > peor[clave][2]:
                peor[clave] = (a, b, f)
        return [f"[CONTACTO] {ga if ga == gb else f'{ga} <-> {gb}'}: {a} / {b}  hasta {f:.0f} N"
                for (ga, gb), (a, b, f) in sorted(peor.items(), key=lambda kv: -kv[1][2])]

    def informe(self, titulo, resultado):
        """Una línea con el resumen y, debajo, los contactos y avisos."""
        registro, (j, err), recortados, mensajes = resultado
        lineas = self.texto_contactos(registro) + mensajes
        if recortados:
            lineas.append(f"[LÍMITE] recortados (junta, pedido, usado): {recortados}")
        print(f"{titulo}: {'sin contactos' if not registro else 'CON CONTACTOS'}; "
              f"error al terminar {math.degrees(err):.1f}° ({ed.NOMBRES[j]})")
        for linea in lineas:
            print("       " + linea)

    def ir_a_pose_segura(self, titulo):
        """Un paso a la pose segura, como el que el selector añade al final de cada rutina."""
        if not self.segura:
            print(f"[AVISO] No hay 0_pose_segura.json en {self.poses_dir}: los brazos se quedan donde están.")
            return
        s = self.segura
        self.informe(f"{titulo} ({s['duracion']:.1f} s)", self.mover(s["posiciones"], s["manos"], s["duracion"]))

    def ejecutar_paso(self, n, p):
        self.informe(f"  -> {n:02d}. {p['nombre']} ({p['duracion']:.1f} s)",
                     self.mover(p["posiciones"], p.get("manos"), p["duracion"]))

    # --------------------------------------------------------- comandos
    def listar(self):
        with self.lock:
            mandada, medida = self.q_mandada(self.d.time), self.d.qpos[self.qadr].copy()
            manos = {lado: (mano.q_cmd.copy(), self.d.qpos[mano.qadr].copy()) for lado, mano in self.manos.items()}
        print("\n  junta  nombre              mandado   medido    error (grados)")
        for j in range(12, 27):
            print(f"  {j:5d}  {ed.NOMBRES[j]:18s} {math.degrees(mandada[j]):+7.1f}  {math.degrees(medida[j]):+7.1f}"
                  f"  {math.degrees(medida[j] - mandada[j]):+7.1f}")
        print(f"\n  mano  {'':9s}" + "".join(f"{gm.NOMBRE_DOF[k]:>14s}" for k in CLAVES))
        for lado, (cmd, med) in manos.items():
            print(f"  {lado:4s}  mandado  " + "".join(f"{math.degrees(x):>14.1f}" for x in cmd))
            print(f"  {'':4s}  medido   " + "".join(f"{math.degrees(x):>14.1f}" for x in med))
        print()

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
        elif cmd == "cargar":
            ruta, self.pasos = ed.leer_rutina(" ".join(resto), self.poses_dir)
            print(f"[OK] Cargada {ruta.name}: {len(self.pasos)} pasos. 'p' para ejecutarla, 'ir <n>' para un paso.")
        elif cmd == "v":
            print("\n".join(ed.texto_pasos(self.pasos)) if self.pasos else "[INFO] Sin rutina: 'cargar <n>'.")
        elif cmd == "p":
            if not self.pasos:
                print("[INFO] Sin rutina: 'cargar <n>'.")
            for n, p in enumerate(self.pasos, 1):
                if self.terminar:
                    break
                self.ejecutar_paso(n, p)
            if self.pasos and not (self.segura and ed.es_pose_segura(self.pasos[-1], self.segura)):
                self.ir_a_pose_segura("  -> pose segura (automática)")
            if self.pasos:
                print("[INFO] Rutina terminada: en la pose segura.")
        elif cmd == "ir":
            n = int(resto[0])
            self.ejecutar_paso(n, self.pasos[n - 1])
        elif cmd == "mano":
            if len(resto) != 2 or resto[1] not in ("izq", "der", "ambas"):
                raise ValueError(f"formato: mano <{'|'.join(gm.GESTOS)}> izq|der|ambas [t=1]")
            objetivo = gm.gesto(resto[0].lower())
            lados = list(gm.LADOS) if resto[1] == "ambas" else [resto[1]]
            self.informe(f"[OK] Mano {resto[1]}: {gm.ALIAS.get(resto[0], resto[0])}",
                         self.mover({}, {lado: objetivo for lado in lados}, t or ed.DUR_GESTO_DEFECTO))
        elif cmd == "cero":
            self.informe("[OK] Torso y brazos a 0, manos abiertas",
                         self.mover({j: 0.0 for j in range(12, 27)},
                                    {lado: gm.gesto("abierta") for lado in gm.LADOS}, t or DUR_CERO))
        elif cmd == "l":
            self.listar()
        elif cmd == "col":
            with self.lock:
                lineas = self.texto_contactos(self.contactos())
            print("[Ahora] " + ("sin contactos." if not lineas else "\n  " + "\n  ".join(lineas)))
        elif cmd == "segura":
            self.ir_a_pose_segura("[OK] Pose segura")
        elif cmd == "reset":
            self.reiniciar()
            print("[OK] Reiniciado: colgado a la altura inicial, en la pose segura."
                  if self.segura else "[OK] Reiniciado: colgado a la altura inicial, todo a 0.")
        elif cmd == "x":
            return False
        elif cmd in ("c", "s", "b", "reemplazar", "espejo"):
            print("[INFO] En modo física no se diseña: usa ./editor_mujoco.sh sin --fisica.")
        else:
            print("[?] Comando no reconocido. 'h' para la ayuda.")
        return True

    def menu(self):
        time.sleep(1.0)
        print(AYUDA)
        while not self.terminar:
            try:
                linea = input("fisica> ").strip()
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

    def correr(self):
        print(f"[INFO] Poses: {self.poses_dir}")
        print(f"[INFO] Física: colgado del pórtico, pelvis a {self.d.qpos[2]:.2f} m. Pares excluidos (ya se tocan "
              f"en la pose cero): {', '.join('/'.join(p) for p in sorted(self.excluidos))}")
        threading.Thread(target=self.menu, daemon=True).start()
        with mujoco.viewer.launch_passive(self.m, self.d, key_callback=self.tecla) as visor:
            periodo = PASOS_POR_FOTOGRAMA * self.m.opt.timestep
            while visor.is_running() and not self.terminar:
                inicio = time.perf_counter()
                with self.lock:
                    self.atender_teclas()
                    for _ in range(PASOS_POR_FOTOGRAMA):
                        self.paso()
                    visor.sync()
                time.sleep(max(0.0, periodo - (time.perf_counter() - inicio)))
        self.terminar = True
        print("\n[INFO] Visor cerrado.", flush=True)
        # Como el editor: salir sin los atexit. Si se sale con 'x', el hilo del visor aún está cerrando la
        # ventana y el glfw.terminate() de la salida normal de Python choca con él (fallo de segmentación).
        os._exit(0)
