"""Visor de MuJoCo para la simulacion de la PC (manipulacion.py --sim --visor): el mundo simulado en una
ventana, con el estado del supervisor, el peso de arm_sdk y los choques del robot contra mesa y bandejas.

Lleva su propio MjData: un hilo copia a ~30 Hz la postura del mundo (robot, dedos y objetos) y lo dibuja, sin
tocar el MjData que renderiza la camara virtual. Los puntos de contacto del robot con el entorno se ven en
rojo (en el gemelo de la PC no hay fisica: un contacto es una interpenetracion, es decir, un choque).
ESPACIO en la ventana pide la salida segura, como en el terminal.
"""

import threading
import time

import mujoco
import mujoco.viewer

from sim.colisiones import es_entorno, grupo

TECLA_ESPACIO = 32


def _nombre(m, b):
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)


class Visor:
    def __init__(self, mundo, texto=None, al_espacio=None, hz=30.0):
        """texto(): (izquierda, derecha) del recuadro de arriba a la izquierda. al_espacio(): la tecla ESPACIO."""
        self.mundo, self.texto, self.al_espacio, self.hz = mundo, texto, al_espacio, hz
        self.m = mundo.m
        self.d = mujoco.MjData(self.m)
        self.choques = {}             # (grupo a, grupo b) -> peor interpenetracion (m) en toda la tirada
        self.v = mujoco.viewer.launch_passive(self.m, self.d, key_callback=self._tecla,
                                              show_left_ui=False, show_right_ui=False)
        with self.v.lock():
            self.v.cam.lookat[:] = [0.30, 0.0, -0.05]
            self.v.cam.distance, self.v.cam.azimuth, self.v.cam.elevation = 2.3, 215.0, -28.0
            self.v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        self.m.vis.scale.contactwidth, self.m.vis.scale.contactheight = 0.03, 0.01
        self._vivo = True
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def _tecla(self, tecla):
        if tecla == TECLA_ESPACIO and self.al_espacio:
            self.al_espacio()

    def _copiar(self):
        with self.mundo.cerrojo:
            self.mundo.actualizar()
            e = self.mundo.est.d
            self.d.qpos[:] = e.qpos
            self.d.mocap_pos[:] = e.mocap_pos
            self.d.mocap_quat[:] = e.mocap_quat
        mujoco.mj_forward(self.m, self.d)

    def _choques_ahora(self):
        """{(grupo, grupo): dist} del brazo o la mano contra mesa, bandejas o suelo (no contra los objetos)."""
        sal = {}
        for c in self.d.contact[:self.d.ncon]:
            if c.dist >= 0.0:
                continue
            a, b = _nombre(self.m, self.m.geom_bodyid[c.geom1]), _nombre(self.m, self.m.geom_bodyid[c.geom2])
            if es_entorno(a) == es_entorno(b) or a.startswith("obj_") or b.startswith("obj_"):
                continue
            robot = b if es_entorno(a) else a
            if not grupo(robot).startswith(("brazo", "mano")) and "wrist" not in robot:
                continue
            par = (grupo(robot), grupo(a if es_entorno(a) else b))
            sal[par] = min(sal.get(par, 0.0), float(c.dist))
        return sal

    def _bucle(self):
        periodo = 1.0 / self.hz
        while self._vivo and self.v.is_running():
            t = time.monotonic()
            try:
                self._copiar()
                ahora = self._choques_ahora()
                for par, dist in ahora.items():
                    self.choques[par] = min(self.choques.get(par, 0.0), dist)
                izq, der = self.texto() if self.texto else ("", "")
                if ahora:
                    izq += "\nCHOQUE"
                    der += "\n" + ", ".join(f"{a}-{b} {d * 1000:.0f} mm" for (a, b), d in ahora.items())
                # set_texts y sync toman el cerrojo del visor por su cuenta: dentro de v.lock() se bloquean
                self.v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, izq, der))
                self.v.sync()
            except Exception as e:          # el visor no debe tumbar la tirada
                print(f"visor: {e}")
                break
            time.sleep(max(0.0, periodo - (time.monotonic() - t)))

    def abierto(self):
        return self.v.is_running()

    def esperar_cierre(self):
        while self.v.is_running():
            time.sleep(0.2)

    def cerrar(self):
        self._vivo = False
        self._hilo.join(timeout=1.0)
        self.v.close()


def informe_choques(choques):
    if not choques:
        return "sin choques del brazo o la mano contra mesa, bandejas o suelo"
    return "; ".join(f"{a} contra {b}: {-d * 1000:.0f} mm" for (a, b), d in sorted(choques.items(), key=lambda x: x[1]))


__all__ = ["Visor", "informe_choques"]
