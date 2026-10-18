"""Robot con fisica para probar rt/lowcmd en la PC (manip/cuerpo.py, herramientas/arnes.py).

El H1-2 con manos de h1_2_mujoco (modelo_fisica: gravedad, contactos y motores con los pares maximos de
unitree_mujoco), colgado de UN gancho como en el portico del laboratorio (cintas al soporte del cuello y a
los cancamos de la placa superior: se modela en la placa, como manip.dinamica.GANCHO). La cuerda es un
resorte que solo tira desde un punto fijo encima; el polipasto sube y baja ese punto. Obedece LowCmd como
los motores: tau = kp (q - qm) + kd (dq - dqm) + tau, recortado al par maximo. Da el lowstate con la interfaz
de manip.robot.Robot (y tau_est de las piernas con el sesgo medido en el robot real, ~22 % de mas).

Empieza colgado en amortiguacion (kp 0, kd 2), piernas estiradas y brazos colgando, con los pies a
`holgura_m` del suelo. Con hilo=True avanza en tiempo real (x velocidad); con hilo=False lo avanza quien
llame a paso() (pruebas), con su propio reloj.
"""

import math
import threading
import time
from types import SimpleNamespace

import mujoco
import numpy as np

from manip import config
from manip import mando as mando_mod
from manip.dinamica import GANCHO, JUNTAS_URDF
from manip.mensajes import Mando

N = 27
SESGO_TAU_PIERNAS = 1.33        # con la tara colgado, el robot real mide ~33 % mas par en las piernas que el modelo
                                # (config cuerpo.carga.escala 0.75, de pie con la cuerda floja el 2026-10-16)
# El URDF deja la suela plana, con la postura de pie, con el torso a +5.9 grados; el robot real (FSM 201, misma
# postura) a +0.5 (IMU, que coincide con un nivel puesto en el torso): ~5.4 grados entre el modelo y el robot.
# Se simula como un desfase repartido entre cadera y rodilla (el robot dice q = verdadero - desfase): la misma
# consigna deja el torso como en el real y al modelo de pie le faltan ~16 N m de pitch por tobillo (el real
# pidio 13 con la cuerda floja; en la cadera serian 21, en la rodilla 11, en el tobillo 0). El controlador sigue
# usando el URDF sin desfase, como en el robot
DESFASE_PIERNA_DEG = {"cadera": 2.7, "rodilla": 2.7}
BRAZOS_COLGANDO = [0.0, 0.087, 0.0, 1.396, 0.0, 0.0, 0.0, 0.0, -0.087, 0.0, 1.396, 0.0, 0.0, 0.0]


class CuerpoSimulado:
    def __init__(self, hilo=True, velocidad=1.0, holgura_m=0.05, k_cuerda=2.0e4, c_cuerda=300.0, largo_cuerda=0.8,
                 sesgo_tau_piernas=SESGO_TAU_PIERNAS, visor=False, postura=None, cdm_torso_dx=0.0,
                 desfase_pierna_deg=DESFASE_PIERNA_DEG):
        """postura: 27 juntas con las que empieza colgado (las pruebas empiezan ya en la postura de pie).
        cdm_torso_dx: desplaza adelante el centro de masas del torso (m), para que el modelo del controlador no
        coincida con el robot (el real se queda inclinado adelante con el estabilizador solo proporcional)."""
        import sys
        sys.path.insert(0, config.ruta_repo())
        from h1_2_mujoco import modelo_h1_2_manos as mm
        m, _ = mm.modelo_fisica()
        m.tendon_limited[:] = 0                 # las cuerdas del portico de h1_2_mujoco no: va la del gancho
        m.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        m.opt.impratio = 10.0
        self.m, self.d = m, mujoco.MjData(m)
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.qadr = np.array([m.jnt_qposadr[jid(n)] for n in JUNTAS_URDF])
        self.vadr = np.array([m.jnt_dofadr[jid(n)] for n in JUNTAS_URDF])
        self.act = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JUNTAS_URDF])
        self.lim = m.actuator_ctrlrange[self.act].copy()
        self.torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
        m.body_ipos[self.torso, 0] += cdm_torso_dx
        self.pies = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{l}_ankle_roll_link") for l in ("left", "right")]
        self.suelo = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.dt = m.opt.timestep
        self.k, self.c, self.largo = k_cuerda, c_cuerda, largo_cuerda
        self.sesgo = np.ones(N)
        self.sesgo[:12] = sesgo_tau_piernas
        self.desfase = np.zeros(N)
        for n, js in (("cadera", (1, 7)), ("rodilla", (3, 9)), ("tobillo", (4, 10))):
            self.desfase[list(js)] = math.radians((desfase_pierna_deg or {}).get(n, 0.0))
        self.cerrojo = threading.Lock()
        self.cmd = {"q": np.zeros(N), "dq": np.zeros(N), "kp": np.zeros(N), "kd": np.full(N, 2.0), "tau": np.zeros(N)}
        self.n_cmd = 0
        self.mando = Mando(0, 0.0, 0.0, 0.0, 0.0)
        self.fallos = []
        self.t = 0.0
        self.tension = 0.0
        self._carga = 0.0
        self._oyentes = []
        self._msg = None
        # colocar: piernas estiradas, brazos colgando, pelvis nivelada con los pies a holgura_m del suelo
        q0 = np.zeros(N)
        q0[13:] = BRAZOS_COLGANDO
        if postura is not None:
            q0 = np.asarray(postura, float)
        self.q_inicial = q0.copy()
        d = self.d
        d.qpos[self.qadr] = q0 + self.desfase
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[2] = 1.5
        mujoco.mj_forward(m, d)
        zmin = min(self._zmin_pie(b) for b in self.pies)
        d.qpos[2] -= zmin - holgura_m
        mujoco.mj_forward(m, d)
        W = float(m.body_subtreemass[1]) * 9.81
        self.ancla = self._gancho() + np.array([0.0, 0.0, self.largo + W / self.k])
        if postura is not None:
            self.cmd["q"], self.cmd["kp"] = q0.copy(), np.full(N, 200.0)
        for _ in range(int(1.0 / self.dt)):    # que se asiente colgado
            self._avanzar()
        self.t = 0.0
        self._mensaje_nuevo()
        self.visor = None
        if visor:
            from mujoco import viewer
            self.visor = viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False)
        self._vivo = hilo
        self._velocidad = velocidad
        self._t_real0 = time.monotonic()
        if hilo:
            self._hilo = threading.Thread(target=self._bucle, daemon=True)
            self._hilo.start()

    # --- fisica ---------------------------------------------------------------------------------------
    def _zmin_pie(self, b):
        m, d = self.m, self.d
        z = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == b and m.geom_contype[g]:
                mid = m.geom_dataid[g]
                v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
                z.append((v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].min())
        return min(z)

    def _gancho(self):
        R = self.d.xmat[self.torso].reshape(3, 3)
        return self.d.xpos[self.torso] + R @ GANCHO

    def _avanzar(self):
        m, d = self.m, self.d
        with self.cerrojo:
            c = {k: v.copy() for k, v in self.cmd.items()}
            ancla = self.ancla.copy()
        q, dq = d.qpos[self.qadr] - self.desfase, d.qvel[self.vadr]
        tau = c["kp"] * (c["q"] - q) + c["kd"] * (c["dq"] - dq) + c["tau"]
        for i in self.fallos:
            tau[i] = 0.0
        d.ctrl[self.act] = np.clip(tau, self.lim[:, 0], self.lim[:, 1])
        d.ctrl[27:] = 0.0                     # dedos abiertos
        d.qfrc_applied[:] = 0.0
        p = self._gancho()
        u = ancla - p
        L = float(np.linalg.norm(u))
        u /= L
        T = 0.0
        if L > self.largo:
            v = np.zeros(6)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.torso, v, 0)
            w, vo = v[:3], v[3:]
            vp = vo + np.cross(w, p - d.xpos[self.torso])
            T = max(0.0, self.k * (L - self.largo) - self.c * float(np.dot(vp, u)))
            mujoco.mj_applyFT(m, d, T * u, np.zeros(3), p, self.torso, d.qfrc_applied)
        self.tension = T
        mujoco.mj_step(m, d)
        self.t += self.dt
        self._carga = self._carga_contactos()

    def _mensaje_nuevo(self):
        m, d = self.m, self.d
        R = d.xmat[self.torso].reshape(3, 3)
        roll, pitch, yaw = math.atan2(R[2, 1], R[2, 2]), math.asin(-max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0])
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, d.xmat[self.torso])
        v = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.torso, v, 1)
        q, dq = d.qpos[self.qadr] - self.desfase, d.qvel[self.vadr]
        tau = d.actuator_force[self.act] * self.sesgo
        ms = [SimpleNamespace(q=float(q[i]) if i < N else 0.0, dq=float(dq[i]) if i < N else 0.0,
                              tau_est=float(tau[i]) if i < N else 0.0, temperature=[40, 40],
                              motorstate=(1 if i in self.fallos else 0)) for i in range(35)]
        imu = SimpleNamespace(rpy=[roll, pitch, yaw], quaternion=list(quat), gyroscope=list(v[:3]),
                              accelerometer=[0.0, 0.0, 9.81])
        self._msg = SimpleNamespace(tick=int(self.t * 1000), motor_state=ms, imu_state=imu, mode_machine=6,
                                    wireless_remote=list(mando_mod.codificar(self.mando)))
        self._t_msg = self.reloj()

    def paso(self, n=1):
        for _ in range(n):
            self._avanzar()
        self._mensaje_nuevo()
        for f in list(self._oyentes):
            f(self._msg, self._t_msg)

    def _bucle(self):
        t_vis = 0.0
        while self._vivo:
            objetivo = (time.monotonic() - self._t_real0) * self._velocidad
            while self.t < objetivo and self._vivo:
                self.paso()
            if self.visor is not None and time.monotonic() - t_vis > 1 / 30:
                t_vis = time.monotonic()
                if self.visor.is_running():
                    self.visor.sync()
            time.sleep(0.001)

    def reloj(self):
        """El tiempo de la simulacion (con hilo, el real escalado: lo usa tambien el controlador)."""
        return self.t

    # --- lo que manda el controlador --------------------------------------------------------------------
    def recibir_lowcmd(self, cmd):
        with self.cerrojo:
            self.cmd = {k: np.asarray(cmd[k], float).copy() for k in ("q", "dq", "kp", "kd", "tau")}
            self.n_cmd += 1

    # --- el operador ------------------------------------------------------------------------------------
    def polipasto(self, dz):
        """Sube (+) o baja (-) el gancho dz metros (el brazo del portico siempre queda encima del robot)."""
        with self.cerrojo:
            self.ancla[2] = max(self.ancla[2] + dz, self._gancho()[2] + 0.3)

    def carga_real(self):
        """Parte del peso en los pies (de los contactos con el suelo; se calcula en el hilo de la fisica)."""
        return self._carga

    def _carga_contactos(self):
        m, d = self.m, self.d
        f6, F = np.zeros(6), 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if self.suelo in (c.geom1, c.geom2):
                mujoco.mj_contactForce(m, d, i, f6)
                F += f6[0]
        return F / (float(m.body_subtreemass[1]) * 9.81)

    def pies_xy(self):
        return np.array([self.d.xpos[b][:2] for b in self.pies])

    # --- interfaz de manip.robot.Robot ------------------------------------------------------------------
    def al_lowstate(self, funcion):
        self._oyentes.append(funcion)

    def esperar_lowstate(self, timeout_s=3.0):
        return True

    def ultimo(self):
        return self._msg, self._t_msg

    def edad_lowstate(self):
        return self.reloj() - self._t_msg

    def estado(self):
        from manip.robot import estado_de
        return estado_de(self._msg, self._t_msg)

    def q(self):
        return np.array([s.q for s in self._msg.motor_state[:N]])

    def mode_machine(self):
        return 6

    def parar(self):
        self._vivo = False
        if getattr(self, "_hilo", None) is not None:      # sin esperar, al salir de Python puede quedar en mj_step
            self._hilo.join(timeout=1.0)
        if self.visor is not None:
            self.visor.close()


class ConmutadorSimulado:
    """MotionSwitcher de mentira: 'ai' hasta que se llama a soltar()."""

    def __init__(self, nombre="ai", falla=0):
        self.nombre, self.falla, self.llamadas = nombre, falla, 0

    def activo(self):
        return self.nombre

    def soltar(self):
        self.llamadas += 1
        if self.llamadas > self.falla:
            self.nombre = ""
        return True
