"""Gemelo del montaje en MuJoCo: el H1-2 con manos, la mesa medida, las bandejas, los objetos, los ArUco con
su textura real y la D435i virtual (color e IR/profundidad, con los intrinsecos de la camara).

El mundo es el marco del torso (la pelvis con la cintura a 0): la mesa, la zona y la calibracion ya estan en
el, asi que no hace falta saber como esta la pelvis respecto del suelo. El suelo queda a la altura de la
pelvis medida (mesa a 85.5 cm). Sin gravedad: es un modelo cinematico (se ponen las q, no se integra).

Colisiones: robot (contype 1, conaffinity 3) contra entorno (contype 2, conaffinity 1) y contra si mismo;
el entorno no choca consigo mismo. Lo usa sim/colisiones.py.

Todo sale de config/montaje.yaml, config/objetos.yaml y la calibracion: aqui no hay medidas escritas, salvo
las del carrito que la camara no ve (patas y baldas) y la postura de pie por defecto.
"""

import math
import os
import sys

import cv2
import mujoco
import numpy as np

from manip import config
from manip.cinematica import DOF_MANO, MIMIC, PREFIJO
from manip.geometria import T, aplicar, inv
from manip.mensajes import LADOS

sys.path.insert(0, config.ruta_repo())
from h1_2_mujoco import modelo_h1_2_manos as mm  # noqa: E402

ALTO_MESA = 0.855           # m del suelo a la superficie (usuario)
# Postura de pie en FSM 201 leida de rt/lowstate el 2026-10-13 (orden DDS 0-26)
POSTURA_PIE = [-0.009, -0.413, -0.038, 0.667, -0.371, -0.019, 0.015, -0.423, 0.014, 0.711, -0.379, 0.051,
               0.001, 0.282, 0.24, -0.109, 0.68, -0.0, 0.031, -0.005, 0.289, -0.241, 0.122, 0.707, 0.003, 0.049, 0.01]


def postura_arnes(cfg):
    """Las 27 juntas de pie en el arnes con los brazos a 0 (lo que manda herramientas/arnes.py)."""
    c = cfg["cuerpo"]
    return np.radians(np.r_[c["postura_pie_deg"], c["brazos_cero_deg"]])


COLOR_BANDEJA = {"blanca": (0.92, 0.92, 0.90), "roja": (0.85, 0.08, 0.08), "azul": (0.05, 0.25, 0.85),
                 "naranja": (1.0, 0.45, 0.05)}
COLORES_OBJETO = {"amarillo": (0.85, 0.95, 0.15), "blanco": (0.95, 0.95, 0.93), "azul": (0.05, 0.22, 0.80),
                  "naranja": (1.0, 0.42, 0.05)}
NEGRO_MESA = (0.03, 0.03, 0.035, 1.0)
GRIS_ALFOMBRA = (0.17, 0.17, 0.18, 1.0)
LEJOS = np.array([0.0, 0.0, -5.0])     # donde se guardan los objetos que no estan en la mesa


def _cache():
    d = os.path.join(config.RAIZ, "datos", "cache_texturas")
    os.makedirs(d, exist_ok=True)
    return d


BORDE_ARUCO = 0.2          # margen blanco de los impresos, en lados del cuadrado negro (fotos de montajes/)
BORDE_ARUCO_MESA = 0.12    # los de las esquinas de la mesa: 6 mm de blanco en 5 cm, contra el labio (wincha, 2026-10-18)


def textura_aruco(i, dicc="DICT_4X4_50", px=240, borde=BORDE_ARUCO):
    """PNG del marcador con su margen blanco (`borde` del lado a cada lado). Devuelve la ruta."""
    ruta = os.path.join(_cache(), f"aruco_{dicc}_{i}_{px}" + ("" if borde == BORDE_ARUCO else f"_b{borde:g}") + ".png")
    if not os.path.exists(ruta):
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dicc))
        m = cv2.aruco.generateImageMarker(d, int(i), px)
        b = int(round(borde * px))
        m = cv2.copyMakeBorder(m, b, b, b, b, cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(ruta, cv2.cvtColor(m, cv2.COLOR_GRAY2BGR))
    return ruta


def _quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel())
    return q


def _caja(cuerpo, nombre, centro, medias, rgba, colision=True, material=None):
    g = cuerpo.add_geom()
    g.name, g.type = nombre, mujoco.mjtGeom.mjGEOM_BOX
    g.pos, g.size = list(centro), list(medias)
    g.rgba = list(rgba) if len(rgba) == 4 else list(rgba) + [1.0]
    if material:
        g.material = material
    g.contype, g.conaffinity = (2, 1) if colision else (0, 0)
    return g


def _capsula(cuerpo, nombre, a, b, r, rgba, colision=True):
    g = cuerpo.add_geom()
    g.name, g.type = nombre, mujoco.mjtGeom.mjGEOM_CAPSULE
    g.fromto = list(a) + list(b)
    g.size = [r, 0, 0]
    g.rgba = list(rgba)
    g.contype, g.conaffinity = (2, 1) if colision else (0, 0)
    return g


def _marcador(spec, cuerpo, nombre, i, centro, R, lado_m, dicc, borde=BORDE_ARUCO):
    """Un ArUco de lado `lado_m` (cuadrado negro) como placa fina con su textura, normal = R[:, 2]."""
    tex = spec.add_texture()
    tex.name, tex.type, tex.file = f"tex_{nombre}", mujoco.mjtTexture.mjTEXTURE_2D, textura_aruco(i, dicc, borde=borde)
    mat = spec.add_material()
    mat.name = f"mat_{nombre}"
    mat.textures[int(mujoco.mjtTextureRole.mjTEXROLE_RGB)] = tex.name
    mat.texrepeat = [1, 1]
    mat.texuniform = False
    mat.specular, mat.shininess, mat.reflectance = 0.0, 0.0, 0.0
    total = lado_m * (1 + 2 * borde) / 2             # con el margen blanco
    g = cuerpo.add_geom()
    g.name, g.type = nombre, mujoco.mjtGeom.mjGEOM_BOX
    g.size = [total, total, 0.0005]
    g.pos = list(np.asarray(centro) + 0.0006 * R[:, 2])
    g.quat = list(_quat(R))
    g.material = mat.name
    g.rgba = [1, 1, 1, 1]
    g.contype = g.conaffinity = 0
    return g


def _camara(cuerpo, nombre, T_cuerpo_optico, intr):
    """Camara de MuJoCo (mira por -z, y arriba) a partir del marco optico (z adelante, y abajo)."""
    c = cuerpo.add_camera()
    c.name = nombre
    R = T_cuerpo_optico[:3, :3] @ np.diag([1.0, -1.0, -1.0])
    c.pos = list(T_cuerpo_optico[:3, 3])
    c.quat = list(_quat(R))
    paso = 3e-6                       # tamano de pixel supuesto: solo fija las unidades del sensor
    c.resolution = [intr.ancho, intr.alto]
    c.sensor_size = [intr.ancho * paso, intr.alto * paso]
    c.focal_pixel = [intr.fx, intr.fy]
    # MuJoCo da el punto principal como desplazamiento DESDE el centro con el signo cambiado y en coordenadas
    # continuas (el pixel i ocupa [i, i+1]); OpenCV pone el centro del pixel i en i (comprobado proyectando
    # esferas conocidas: sin esto la imagen salia desplazada (-3.1, +6.8) px)
    c.principal_pixel = [intr.ancho / 2.0 - (intr.cx + 0.5), intr.alto / 2.0 - (intr.cy + 0.5)]
    return c


def _mesa(spec, montaje):
    me = montaje["mesa"]
    T_mesa = np.array(me["T_torso_mesa"], float)
    b = spec.worldbody.add_body()
    b.name = "mesa"
    b.pos = list(T_mesa[:3, 3])
    b.quat = list(_quat(T_mesa[:3, :3]))
    x0, x1 = me["superficie"]["x"]
    y0, y1 = me["superficie"]["y"]
    mat = spec.add_material()
    mat.name, mat.rgba, mat.specular, mat.shininess, mat.reflectance = "mesa_negra", list(NEGRO_MESA), 0.6, 0.8, 0.1
    _caja(b, "tablero", [(x0 + x1) / 2, (y0 + y1) / 2, -0.01], [(x1 - x0) / 2, (y1 - y0) / 2, 0.01], NEGRO_MESA,
          material="mesa_negra")
    h = me.get("labio_alto_m") or 0.023
    g = 0.02
    _caja(b, "labio_cerca", [x0 - g / 2, (y0 + y1) / 2, h / 2], [g / 2, (y1 - y0) / 2 + g, h / 2], NEGRO_MESA)
    _caja(b, "labio_lejos", [x1 + g / 2, (y0 + y1) / 2, h / 2], [g / 2, (y1 - y0) / 2 + g, h / 2], NEGRO_MESA)
    _caja(b, "labio_izq", [(x0 + x1) / 2, y1 + g / 2, h / 2], [(x1 - x0) / 2, g / 2, h / 2], NEGRO_MESA)
    _caja(b, "labio_der", [(x0 + x1) / 2, y0 - g / 2, h / 2], [(x1 - x0) / 2, g / 2, h / 2], NEGRO_MESA)
    # alfombrillas grises: 2 filas x 3 columnas (solo se ven)
    ancho_y, fondo_x = (y1 - y0 - 0.08) / 3, (x1 - x0 - 0.06) / 2
    for fila in range(2):
        for col in range(3):
            cx = x0 + 0.03 + fondo_x * (fila + 0.5)
            cy = y0 + 0.04 + ancho_y * (col + 0.5)
            _caja(b, f"alfombra_{fila}{col}", [cx, cy, 0.0004], [fondo_x / 2 - 0.012, ancho_y / 2 - 0.012, 0.0004],
                  GRIS_ALFOMBRA, colision=False)
    # asas: tubo a lo largo de x a su altura, y bajadas en los extremos
    r = 0.018
    for nombre, a in me.get("asas", {}).items():
        za = a["alto"] - r
        xa, xb = a["x"]
        _capsula(b, f"asa_{nombre}", [xa, a["y"], za], [xb, a["y"], za], r, NEGRO_MESA)
        for k, xe in enumerate((xa, xb)):
            # se apoyan por fuera del labio, no sobre la superficie (taparian los ArUco de las esquinas)
            fuera = y1 + g + r if a["y"] > 0 else y0 - g - r
            _capsula(b, f"asa_{nombre}_bajada{k}", [xe, a["y"], za], [xe, fuera, h], r, NEGRO_MESA)
    # patas y baldas del carrito (no se ven desde la camara: aproximadas)
    alto = ALTO_MESA
    for k, (px, py) in enumerate(((x0 - 0.01, y0 - 0.01), (x0 - 0.01, y1 + 0.01), (x1 + 0.01, y0 - 0.01), (x1 + 0.01, y1 + 0.01))):
        _capsula(b, f"pata{k}", [px, py, -0.02], [px, py, -alto + 0.10], 0.015, NEGRO_MESA)
    for k, zb in enumerate((-0.42, -0.72)):
        _caja(b, f"balda{k}", [(x0 + x1) / 2, (y0 + y1) / 2, zb], [(x1 - x0) / 2, (y1 - y0) / 2, 0.01], NEGRO_MESA)
    return b, T_mesa


def _bandeja(spec, nombre, b, centro_torso, plano, color, yaw=0.0):
    """Bandeja abierta: fondo y cuatro paredes, sobre el plano de la mesa y con los ejes del torso girados `yaw`
    (rad) alrededor de su normal."""
    from manip.montaje import punto_en_mesa
    c = punto_en_mesa({"mesa": {"plano": plano}}, centro_torso)
    n = np.asarray(plano["n"], float)
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    x = x - (x @ n) * n
    x /= np.linalg.norm(x)
    R = np.column_stack([x, np.cross(n, x), n])
    cu = spec.worldbody.add_body()
    cu.name = f"bandeja_{nombre}"
    cu.pos, cu.quat = list(c), list(_quat(R))
    s, e, h = b["hueco_m"] / 2, b["pared_m"], b["alto_m"]
    rgba = list(color) + [1.0]
    _caja(cu, f"bandeja_{nombre}_fondo", [0, 0, e / 2], [s + e, s + e, e / 2], rgba)
    _caja(cu, f"bandeja_{nombre}_cerca", [-(s + e / 2), 0, h / 2], [e / 2, s + e, h / 2], rgba)
    _caja(cu, f"bandeja_{nombre}_lejos", [s + e / 2, 0, h / 2], [e / 2, s + e, h / 2], rgba)
    _caja(cu, f"bandeja_{nombre}_izq", [0, s + e / 2, h / 2], [s, e / 2, h / 2], rgba)
    _caja(cu, f"bandeja_{nombre}_der", [0, -(s + e / 2), h / 2], [s, e / 2, h / 2], rgba)


def _objeto(spec, clave, o):
    """Cuerpo mocap (se mueve poniendo data.mocap_pos/quat) con la forma del objeto; empieza LEJOS."""
    cu = spec.worldbody.add_body()
    cu.name, cu.mocap, cu.pos = f"obj_{clave}", True, list(LEJOS)
    g = cu.add_geom()
    g.name = f"obj_{clave}"
    m = o["medidas_m"]
    if o["forma"] == "esfera":
        g.type, g.size = mujoco.mjtGeom.mjGEOM_SPHERE, [m[0] / 2, 0, 0]
    elif o["forma"] == "cilindro":
        g.type, g.size = mujoco.mjtGeom.mjGEOM_CYLINDER, [m[0] / 2, m[1] / 2, 0]
    else:
        g.type, g.size = mujoco.mjtGeom.mjGEOM_BOX, [m[0] / 2, m[1] / 2, m[2] / 2]
    g.rgba = list(COLORES_OBJETO.get(o["color"]["nombre"], (0.5, 0.5, 0.5))) + [1.0]
    g.contype, g.conaffinity = 2, 1
    return cu


def construir(montaje=None, objetos=None, cal=None, cfg=None, postura=None, con_camaras=True, con_bandejas=True,
              dianas=None):
    """dianas: [(x, y)] en el torso: cruces blancas de 2 cm sobre la mesa (nivel 1)."""
    """(MjModel, info). info: nombres de camaras, intrinsecos usados, T_torso_mesa y objetos."""
    from manip.geometria import Intrinsecos
    cfg = cfg or config.cargar()
    montaje = montaje or config.cargar_montaje()
    objetos = objetos if objetos is not None else config.cargar_objetos()
    spec = mm.construir_spec(config.ruta_urdf())
    pelvis_suelo = ALTO_MESA + float(montaje["mesa"]["plano"]["d"])
    spec.body("pelvis").pos = [0.0, 0.0, 0.0]
    spec.geom("floor").pos = [0.0, 0.0, -pelvis_suelo]
    for g in spec.geoms:                       # el robot: contra el entorno y contra si mismo
        if g.contype or g.conaffinity:
            g.contype, g.conaffinity = 1, 3
    for l in spec.lights:
        l.pos = [0.3, 0.0, 1.5]
    suelo = spec.texture("groundplane")          # el suelo gris claro del laboratorio, no el damero
    suelo.rgb1, suelo.rgb2, suelo.mark = [0.72, 0.72, 0.72], [0.70, 0.70, 0.70], mujoco.mjtMark.mjMARK_NONE
    luz = spec.worldbody.add_light()
    luz.pos, luz.dir, luz.diffuse = [0.6, 0.6, 1.2], [-0.3, -0.3, -1.0], [0.4, 0.4, 0.4]
    _, T_mesa = _mesa(spec, montaje)
    plano = montaje["mesa"]["plano"]
    colores = list(COLOR_BANDEJA)
    for k, (nombre, b) in enumerate(sorted(montaje.get("bandejas", {}).items() if con_bandejas else [])):
        color = COLOR_BANDEJA.get(b.get("color") or colores[k % len(colores)])
        # donde se midio con el [g] de las bandejas (el giro, respecto de la mesa) o su objetivo
        from manip.montaje import bandeja_real
        c, yaw_rel = bandeja_real(b)
        yaw = math.radians(yaw_rel) + math.atan2(T_mesa[1, 0], T_mesa[0, 0]) if b.get("medida") else 0.0
        _bandeja(spec, nombre, b, c, plano, color, yaw)
    lado_m = cfg["calibracion"]["marcador_lado_m"]
    dicc = cfg["calibracion"]["diccionario"]
    # ArUco de la mesa: en sus esquinas con las medidas de wincha (mesa_fisica) o, sin ellas, los medidos y en
    # la esquina que le toca el que falte. Los medidos no: la camara real los ve ~3 % mas separados (RESULTADOS
    # §22) y aqui caerian bajo el labio
    me = montaje["mesa"]
    x0, x1 = me["superficie"]["x"]
    y0, y1 = me["superficie"]["y"]
    from manip.montaje import MARGEN_MARCADOR, fisica
    fis = fisica(montaje)
    mg = fis["margen"] if fis else MARGEN_MARCADOR
    esquinas = {2: (x1 - mg, y1 - mg), 3: (x1 - mg, y0 + mg), 4: (x0 + mg, y1 - mg), 5: (x0 + mg, y0 + mg)}
    for i, (xm, ym) in esquinas.items():
        p = aplicar(T_mesa, [xm, ym, 0.0])
        if fis is None and str(i) in map(str, me.get("aruco_torso", {})):
            p = np.asarray(me["aruco_torso"].get(i, me["aruco_torso"].get(str(i))), float)
        R = T_mesa[:3, :3] @ np.diag([-1.0, -1.0, 1.0])      # el "arriba" del marcador hacia el robot
        _marcador(spec, spec.worldbody, f"aruco_{i}", i, p, R, lado_m, dicc, BORDE_ARUCO_MESA)
    # ArUco del dorso de cada mano, con la calibracion (o su semilla)
    from manip import calibracion as cb
    T_dorso = cal.T_dorso if cal is not None else cb.T_DORSO_DEFECTO
    for lado in LADOS:
        Td = T_dorso[lado]
        _marcador(spec, spec.body(f"{PREFIJO[lado]}_wrist_yaw_link"), f"aruco_mano_{lado}",
                  cfg["calibracion"]["ids"][lado], Td[:3, 3], Td[:3, :3], lado_m, dicc)
    for clave, o in objetos.get("objetos", {}).items():
        _objeto(spec, clave, o)
    if dianas:
        from manip.dianas import cruz_en_mesa
        from manip.montaje import punto_en_mesa
        for k, xy in enumerate(dianas):
            cruz_en_mesa(spec.worldbody, f"diana_{k}", punto_en_mesa(montaje, xy), T_mesa[:3, :3])
    info = {"T_torso_mesa": T_mesa, "objetos": list(objetos.get("objetos", {})), "camaras": {}}
    if con_camaras and cal is not None:
        intr = cfg.get("_intrinsecos") or {}
        ic = Intrinsecos.de_meta(intr.get("color", INTR_COLOR))
        ip = Intrinsecos.de_meta(intr.get("prof", INTR_PROF))
        torso = spec.body("torso_link")
        _camara(torso, "d435_color", cal.T_torso_color, ic)
        _camara(torso, "d435_prof", cal.T_torso_prof, ip)
        info["camaras"] = {"color": ("d435_color", ic), "prof": ("d435_prof", ip)}
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1280, 960
    spec.visual.quality.shadowsize = 2048
    m = spec.compile()
    m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
    info["postura"] = list(postura or POSTURA_PIE)
    return m, info


# Intrinsecos de la D435i de este robot (meta.json del 2026-10-13); los de cada captura mandan
INTR_COLOR = {"fx": 604.1116943359375, "fy": 603.5626831054688, "cx": 321.32781982421875, "cy": 236.3570098876953,
              "ancho": 640, "alto": 480}
INTR_PROF = {"fx": 378.8092041015625, "fy": 378.8092041015625, "cx": 321.52960205078125, "cy": 235.98672485351562,
             "ancho": 640, "alto": 480}


class Estado:
    """Pone posturas del robot, de las manos y de los objetos en un MjData del gemelo."""

    def __init__(self, m, info):
        self.m, self.d = m, mujoco.MjData(m)
        self.mimic = mm.Mimic(m)
        self.adr = {}
        for j in range(m.njnt):
            self.adr[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)] = m.jnt_qposadr[j]
        self.poner_cuerpo(info["postura"])
        self.objetos = info["objetos"]

    def poner_cuerpo(self, q27):
        for i, nombre in enumerate(mm.JUNTAS_CUERPO):
            self.d.qpos[self.adr[nombre]] = q27[i]

    def poner_brazo(self, lado, q7):
        from manip.cinematica import nombre_junta
        for i in range(7):
            self.d.qpos[self.adr[nombre_junta(lado, i)]] = q7[i]

    def poner_mano(self, lado, rad6):
        """rad6: las 6 juntas actuadas en el orden de DOF_MANO (0 = abierta)."""
        for k, v in zip(DOF_MANO, rad6):
            self.d.qpos[self.adr[f"{PREFIJO[lado]}_{k}_joint"]] = v

    def poner_objeto(self, clave, p=None, R=None):
        b = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, f"obj_{clave}")
        i = self.m.body_mocapid[b]
        self.d.mocap_pos[i] = LEJOS if p is None else p
        self.d.mocap_quat[i] = [1, 0, 0, 0] if R is None else _quat(R)

    def adelante(self):
        self.mimic.aplicar(self.d.qpos)
        mujoco.mj_forward(self.m, self.d)


def angle_set_a_rad(a6):
    """ANGLE_SET (0 cerrado, 1000 abierto) -> rad de las 6 juntas actuadas, a tramos como
    selector_poses_manos/conversion_angle_set.py (medido con fotos a 0, 500 y 1000)."""
    s = 1.0 - np.clip(np.asarray(a6, float), 0, 1000) / 1000.0
    medio = np.array([1.05, 1.05, 1.05, 1.05, 0.38, 0.61])
    total = np.array([1.62, 1.62, 1.62, 1.62, 0.66, 1.19])
    return np.where(s <= 0.5, medio * s / 0.5, medio + (total - medio) * (s - 0.5) / 0.5)


__all__ = ["construir", "Estado", "angle_set_a_rad", "MIMIC", "inv", "T"]
