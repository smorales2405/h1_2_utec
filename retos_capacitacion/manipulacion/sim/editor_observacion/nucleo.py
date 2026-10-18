"""Nucleo del editor de la pose de observacion (sin Qt): escena, espejo, trayectoria, colisiones, criterios y
el archivo de poses. Solo LEE el codigo de la manipulacion (gemelo, colisiones, limites, duraciones): no lo
cambia ni escribe en config/.

- La escena es el gemelo de manipulacion.py (sim/escena.py) con el robot de pie en la postura medida
  (POSTURA_PIE), las manos abiertas y el carrito medido, sin bandejas ni objetos.
- Una pose es la q del brazo IZQUIERDO (7 juntas, orden de arm_sdk); el derecho es su espejo
  (manip.cinematica.espejo), como la usa manipulacion.py.
- El movimiento reposo -> pose es el de manip/brazo.py: los dos brazos a la vez, en linea recta en
  articulares, con minima sacudida y la duracion de la seccion 10 (>= 8 s, pico <= 0.5 rad/s, x escala).
- Las colisiones son las del planificador (sim/colisiones.py): choque (< 0) o cerca (< holgura, 2 cm), del
  robot contra el carrito y contra si mismo.
"""

import json
import os
import sys
import threading
from datetime import datetime

import mujoco
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))          # retos_capacitacion/manipulacion
if RAIZ not in sys.path:
    sys.path.insert(0, RAIZ)

from manip import config  # noqa: E402
from manip.brazo import duracion_movimiento, minima_sacudida  # noqa: E402
from manip.cinematica import SIGNO_ESPEJO, Cinematica, espejo  # noqa: E402
from manip.mensajes import JUNTAS_BRAZO, LADOS  # noqa: E402
from sim.colisiones import Colisiones, es_entorno, grupo  # noqa: E402
from sim.escena import POSTURA_PIE, Estado, construir  # noqa: E402

RUTA_POSES = os.path.join(AQUI, "poses_observacion.json")
RUTA_POSES_BRAZOS = os.path.join(config.CONFIG, "poses_brazos.yaml")
NOMBRES = ("hombro pitch", "hombro roll", "hombro yaw", "codo", "muñeca roll", "muñeca pitch", "muñeca yaw")
ROJO, AMBAR = (0.95, 0.10, 0.10, 1.0), (1.0, 0.65, 0.0, 1.0)


def _a_lista(q, n=4):
    return [round(float(v), n) for v in q]


# ================================================================ archivo de poses
def leer_poses(ruta=RUTA_POSES):
    """[{nombre, fecha, izq, der, izq_grados, notas}] del archivo (vacio si no existe)."""
    if not os.path.exists(ruta):
        return []
    with open(ruta, encoding="utf-8") as f:
        return json.load(f).get("poses", [])


def escribir_poses(poses, ruta=RUTA_POSES):
    datos = {"descripcion": "Poses de observacion candidatas (editor_observacion.py). 'izq' en rad, orden de "
                            "arm_sdk (13-19); 'der' = espejo de 'izq' (20-26). La que valga se copia a mano a "
                            "config/poses_brazos.yaml (observacion).",
             "poses": poses}
    tmp = ruta + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(datos, f, indent=2, ensure_ascii=False)
    os.replace(tmp, ruta)


def pose_nueva(nombre, q_izq, notas=""):
    q = np.asarray(q_izq, float)
    return {"nombre": nombre, "fecha": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "izq": _a_lista(q), "der": _a_lista(espejo(q)), "izq_grados": _a_lista(np.degrees(q), 1), "notas": notas}


def poses_de_config(ruta=RUTA_POSES_BRAZOS):
    """[(nombre, q_izq)] de config/poses_brazos.yaml: la observacion en uso y las alternativas (solo lectura)."""
    if not os.path.exists(ruta):
        return []
    d = config._leer(ruta)
    sal = []
    if d.get("observacion", {}).get("izq") is not None:
        sal.append(("config: observacion en uso", np.array(d["observacion"]["izq"], float)))
    for k, q in enumerate(d.get("alternativas_izq") or []):
        sal.append((f"config: alternativa {k + 1}", np.array(q, float)))
    return sal


def texto_yaml(q_izq):
    """Las lineas para config/poses_brazos.yaml (se copian a mano: el editor no escribe en config/)."""
    q = np.asarray(q_izq, float)
    return f"observacion:\n  izq: {_a_lista(q)}\n  der: {_a_lista(espejo(q))}"


# ================================================================ escena
class Escena:
    """El gemelo con solo el carrito. `cerrojo` protege d frente al render (la animacion va en otro hilo)."""

    def __init__(self, holgura=None):
        self.cfg = config.cargar()
        self.montaje = config.cargar_montaje()
        self.holgura = float(self.cfg["planificacion"]["holgura_m"] if holgura is None else holgura)
        self.cin = Cinematica.desde_config(self.cfg)
        self.m, self.info = construir(montaje=self.montaje, objetos={"objetos": {}}, cal=None, cfg=self.cfg,
                                      postura=list(POSTURA_PIE), con_camaras=False, con_bandejas=False)
        self.est = Estado(self.m, self.info)
        self.d = self.est.d
        for lado in LADOS:
            self.est.poner_mano(lado, np.zeros(6))          # manos abiertas, como manipulacion.py al empezar
        self.est.adelante()
        # con el robot en reposo: los pares del cuerpo que ya se tocan quedan fuera (como en el planificador)
        self.col = Colisiones(self.m, self.est, self.holgura)
        self.reposo = {l: np.array(POSTURA_PIE)[JUNTAS_BRAZO[l]] for l in LADOS}
        # los sliders mueven el izquierdo; su rango tambien deja al derecho (el espejo) dentro de sus limites
        lo_d, hi_d = SIGNO_ESPEJO * self.cin.lo["der"], SIGNO_ESPEJO * self.cin.hi["der"]
        self.lo = np.maximum(self.cin.lo["izq"], np.minimum(lo_d, hi_d))
        self.hi = np.minimum(self.cin.hi["izq"], np.maximum(lo_d, hi_d))
        b = self.cfg["brazo"]
        self.t_min, self.vel_max = float(b["t_min_mov_s"]), float(b["vel_max_rad_s"])
        self.rgba = self.m.geom_rgba.copy()
        self.visibles = {}
        for g in range(self.m.ngeom):
            if self.rgba[g][3] > 0:
                self.visibles.setdefault(self.m.geom_bodyid[g], []).append(g)
        self.cerrojo = threading.RLock()
        self.q_izq = self.reposo["izq"].copy()
        self.poner_brazos(self.reposo["izq"], self.reposo["der"])

    # --- postura ------------------------------------------------------------------------------------
    def poner_brazos(self, q_izq, q_der):
        with self.cerrojo:
            self.est.poner_brazo("izq", q_izq)
            self.est.poner_brazo("der", q_der)
            self.est.adelante()

    def poner_pose(self, q_izq):
        """La pose de los sliders: el izquierdo, recortado a su rango, y el derecho en espejo."""
        self.q_izq = np.clip(np.asarray(q_izq, float), self.lo, self.hi)
        self.poner_brazos(self.q_izq, espejo(self.q_izq))

    def poner_reposo(self):
        self.poner_brazos(self.reposo["izq"], self.reposo["der"])

    def brazos_ahora(self):
        with self.cerrojo:
            q = np.array([self.d.qpos[self.est.adr[n]] for n in self._nombres_juntas()])
        return q[:7], q[7:]

    def _nombres_juntas(self):
        from manip.cinematica import nombre_junta
        return [nombre_junta(l, i) for l in LADOS for i in range(7)]

    # --- colisiones ----------------------------------------------------------------------------------
    def revisar(self):
        """[(cuerpo a, cuerpo b, dist)] choques (< 0) y acercamientos (< holgura) en la postura actual."""
        with self.cerrojo:
            return self.col.revisar()

    def resaltar(self, hallazgos):
        """Las partes del robot: rojo lo que choca, ambar lo que queda a menos de la holgura. El carrito no se
        pinta (es un solo cuerpo: entero en rojo no diria donde)."""
        rojos = {c for a, b, dist in hallazgos if dist < 0 for c in (a, b) if not es_entorno(c)}
        ambar = {c for a, b, dist in hallazgos if dist >= 0 for c in (a, b) if not es_entorno(c)} - rojos
        with self.cerrojo:
            self.m.geom_rgba[:] = self.rgba
            for nombres, color in ((ambar, AMBAR), (rojos, ROJO)):
                for cuerpo in nombres:
                    b = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, cuerpo)
                    for g in self.visibles.get(b, []):
                        self.m.geom_rgba[g] = color

    # --- movimiento -----------------------------------------------------------------------------------
    def duracion(self, desde, hasta, escala):
        """La de manip/brazo.py para mover los dos brazos de `desde` a `hasta` ({lado: q7})."""
        dq = max(float(np.abs(hasta[l] - desde[l]).max()) for l in LADOS)
        return duracion_movimiento(dq, self.t_min, self.vel_max, escala), dq

    @staticmethod
    def en(desde, hasta, s):
        """La postura a la fraccion de tiempo s (0..1) con minima sacudida, en linea recta en articulares."""
        f = minima_sacudida(min(max(s, 0.0), 1.0))
        return {l: desde[l] + (hasta[l] - desde[l]) * f for l in LADOS}

    def recorrer(self, desde, hasta, n=120):
        """Revisa el camino en n+1 muestras, sin animar. {(a, b): peor dist}."""
        peor = {}
        for s in np.linspace(0, 1, n + 1):
            q = self.en(desde, hasta, s)
            self.poner_brazos(q["izq"], q["der"])
            for a, b, dist in self.revisar():
                peor[(a, b)] = min(peor.get((a, b), np.inf), dist)
        return peor

    # --- informacion ------------------------------------------------------------------------------------
    def plano(self):
        from manip.geometria import Plano
        pl = self.montaje["mesa"]["plano"]
        return Plano(pl["n"], pl["d"])

    def datos_pose(self, q_izq):
        """Yema y mano izquierdas (m del torso) y su altura sobre la mesa."""
        pl = self.plano()
        y = self.cin.fk("izq", q_izq, "yema")[:3, 3]
        mano = self.cin.fk("izq", q_izq, "mano")[:3, 3]
        return {"yema": y, "altura_yema": float(pl.altura(y)), "altura_mano": float(pl.altura(mano))}


def es_contra_mesa(a, b):
    return es_entorno(a) or es_entorno(b)


def lineas(hallazgos_o_peor):
    """[(texto, cuerpos, es_choque, contra_mesa)], una por pareja de partes (la mas grave), la mesa primero."""
    if isinstance(hallazgos_o_peor, dict):
        hallazgos = [(a, b, d) for (a, b), d in hallazgos_o_peor.items()]
    else:
        hallazgos = hallazgos_o_peor
    peor = {}
    for a, b, dist in hallazgos:
        clave = tuple(sorted((grupo(a), grupo(b))))
        if clave not in peor or dist < peor[clave][2]:
            peor[clave] = (a, b, dist)
    sal = []
    for (ga, gb), (a, b, dist) in peor.items():
        mesa = es_contra_mesa(a, b)
        partes = ga if ga == gb else f"{ga} <-> {gb}"
        sal.append((f"{'[CHOQUE]' if dist < 0 else '[CERCA]'} {partes}  {dist * 1000:+.0f} mm"
                    f"{'' if mesa else '  (propio cuerpo)'}", f"{a} / {b}", dist < 0, mesa))
    return sorted(sal, key=lambda x: (not x[3], not x[2], x[0]))


def resumen(peor):
    if not peor:
        return "sin choques ni acercamientos (< holgura)"
    return "; ".join(t for t, _, _, _ in lineas(peor))


# ================================================================ criterios de manipulacion.py
class Criterios:
    """Lo que comprueba el programa con una pose de observacion, sobre su propio gemelo (CON las bandejas, como
    en la tirada): el tramo reposo <-> pose de cada brazo (pl.entre_reposo_y_observacion: si no pasa, la tirada
    no empieza) y, como herramientas/poses_brazos.py, que la mano quede alta, que el brazo no tape la zona de
    recogida en la imagen y que desde la pose se llegue limpio a las esquinas y al centro de la zona."""

    def __init__(self, escena):
        from manip import calibracion as cb
        from manip.geometria import Intrinsecos, aplicar, inv
        from manip.planificacion import Planificador
        from sim.escena import INTR_COLOR
        self.e = escena
        cfg, mon = escena.cfg, escena.montaje
        obj = config.cargar_objetos()
        self.cin = escena.cin
        try:
            self.cal = cb.cargar()
        except FileNotFoundError:
            self.cal = cb.desde_urdf(self.cin, np.array([[1, 0, 0, 0.015], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]]))
        m, info = construir(montaje=mon, objetos=obj, cal=self.cal, cfg=cfg, postura=list(POSTURA_PIE), con_camaras=False)
        self.est = Estado(m, info)
        self.col = Colisiones(m, self.est, cfg["planificacion"]["holgura_m"])
        self.pl = Planificador(cfg, mon, obj, self.cin, (self.est, self.col), dict(escena.reposo),
                               {l: escena.reposo[l].copy() for l in LADOS})
        self.ic = Intrinsecos.de_meta(INTR_COLOR)
        z = mon["zona_recogida"]
        self.objetivos = [(x, y) for x in z["x"] for y in z["y"]] + [(float(np.mean(z["x"])), float(np.mean(z["y"])))]
        esquinas = ((z["x"][0], z["y"][0]), (z["x"][1], z["y"][0]), (z["x"][1], z["y"][1]), (z["x"][0], z["y"][1]))
        self.zona_px = np.round(self.ic.proyectar(aplicar(inv(self.cal.T_torso_color), np.array(
            [self.pl.punto(xy, 0.0) for xy in esquinas])))).astype(np.float32)
        self.ht = cfg["planificacion"]["altura_transporte_m"]
        self._aplicar, self._inv = aplicar, inv

    def _tramo_limpio(self, lado, qa, qb, n=16, holgura=0.015, h_min=0.05):
        """El de herramientas/poses_brazos.py: ni yema ni mano por debajo de h_min sobre la mesa, y nada a menos
        de `holgura` en el gemelo."""
        for s in np.linspace(0, 1, n + 1):
            q = qa + (qb - qa) * s
            for marco in ("yema", "mano"):
                p = self.cin.fk(lado, q, marco)[:3, 3]
                if self.pl.sobre_mesa(p, 0.02) and self.pl.plano.altura(p) < h_min:
                    return False, f"{marco} a {self.pl.plano.altura(p) * 100:.0f} cm de la mesa"
            self.est.poner_brazo(lado, q)
            h = self.col.revisar()
            if h and h[0][2] < holgura:
                return False, f"{grupo(h[0][0])} / {grupo(h[0][1])} a {h[0][2] * 1000:+.0f} mm"
        return True, ""

    def comprobar(self, q_izq, cancelado=lambda: False):
        """[(ok, aviso, texto)]: ok False = no vale para manipulacion.py; aviso = criterio de poses_brazos.py."""
        import cv2
        from manip.cinematica import TareaIK, palma_abajo
        q = {"izq": np.asarray(q_izq, float), "der": espejo(q_izq)}
        sal = []
        for lado in LADOS:
            if not self.cin.dentro_de_limites(lado, q[lado], 1e-6):
                sal.append((False, False, f"brazo {lado}: fuera de los limites del programa"))
                return sal
        self.pl.observacion = {l: q[l].copy() for l in LADOS}
        for lado in LADOS:
            for ida in (True, False):
                self.est.poner_brazo("izq", self.e.reposo["izq"])
                self.est.poner_brazo("der", self.e.reposo["der"])
                p = self.pl.entre_reposo_y_observacion(lado, ida)
                tramo = "reposo -> pose" if ida else "pose -> reposo"
                sal.append((p.comprobado, False, f"brazo {lado}, {tramo} (planificador): "
                            + ("pasa" if p.comprobado else f"NO pasa: {p.motivo}")))
        for lado in LADOS:
            y = self.cin.fk(lado, q[lado], "yema")[:3, 3]
            mano = self.cin.fk(lado, q[lado], "mano")[:3, 3]
            h = min(self.pl.plano.altura(y), self.pl.plano.altura(mano))
            sal.append((True, h < 0.12, f"brazo {lado}: mano a {h * 100:.0f} cm sobre la mesa"
                        + (" (poses_brazos.py pide >= 12)" if h < 0.12 else "")))
        for lado in LADOS:
            esq = self.cin.esqueleto(q["izq"] if lado == "izq" else self.e.reposo["izq"],
                                     q["der"] if lado == "der" else self.e.reposo["der"])[lado]
            puntos = np.vstack([esq[1] + (esq[2] - esq[1]) * s for s in np.linspace(0, 1, 6)] +
                               [esq[2] + (esq[4] - esq[2]) * s for s in np.linspace(0, 1, 6)])
            uv = self.ic.proyectar(self._aplicar(self._inv(self.cal.T_torso_color), puntos))
            tapa = bool(np.isfinite(uv).all() and any(cv2.pointPolygonTest(self.zona_px, (float(u), float(v)), True) > -25
                                                      for u, v in uv))
            sal.append((True, tapa, f"brazo {lado}: " + ("TAPA la zona de recogida en la imagen" if tapa
                                                          else "no tapa la zona de recogida en la imagen")))
        for lado in LADOS:
            malos = []
            for xy in self.objetivos:
                if cancelado():
                    return sal
                r = self.cin.ik(lado, TareaIK(posicion=self.pl.punto(xy, self.ht),
                                              punto=np.array([0.215, -0.07 if lado == "izq" else 0.07, 0.01]),
                                              bandas=[palma_abajo(lado, 35)], tol_pos=0.01), q0=q[lado])
                if not r.ok:
                    malos.append(f"({xy[0]:.2f}, {xy[1]:+.2f}) sin IK")
                    continue
                self.est.poner_brazo("izq", q["izq"])
                self.est.poner_brazo("der", q["der"])
                ok, motivo = self._tramo_limpio(lado, q[lado], r.q)
                if not ok:
                    malos.append(f"({xy[0]:.2f}, {xy[1]:+.2f}) {motivo}")
            sal.append((True, bool(malos), f"brazo {lado}: de la pose a la zona (esquinas y centro, a "
                        f"{self.ht * 100:.0f} cm) " + ("limpio" if not malos else "con problemas: " + "; ".join(malos))))
        self.est.poner_brazo("izq", self.e.reposo["izq"])
        self.est.poner_brazo("der", self.e.reposo["der"])
        return sal


__all__ = ["Escena", "Criterios", "leer_poses", "escribir_poses", "pose_nueva", "poses_de_config", "texto_yaml",
           "lineas", "resumen", "NOMBRES"]
