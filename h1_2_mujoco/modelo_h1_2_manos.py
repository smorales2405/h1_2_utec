#!/usr/bin/env python3
"""El H1-2 con las manos Inspire RH56DFTP en MuJoCo, y la comprobación de colisiones.

Modelo: ros_h1_2_ws/src/h1_2_inspire_description/urdf/h1_2_with_RH56DFTP_hands.urdf,
el URDF plano del repositorio (27 juntas del cuerpo + 24 de las manos, 12 de ellas
mimic). No se modifica: se lee y, en memoria,
  - las rutas package:// pasan a absolutas, con strippath="false": las dos palmas se
    llaman base_link.STL en carpetas distintas;
  - la pelvis queda fija a ALTURA_PELVIS (el URDF no tiene articulación libre) y sin
    gravedad, como en el editor de code_cap;
  - *_wrist_yaw_link recibe como geometría de colisión su malla visual (el URDF no le
    da ninguna);
  - cada eslabón usa SU malla: MuJoCo nombra las mallas de un URDF por el nombre del
    archivo, y las dos palmas (hand_left/base_link.STL y hand_right/base_link.STL)
    acababan siendo una sola, la izquierda, con la palma derecha flotando entre las dos
    manos (y su colisión calculada con la forma y el sitio equivocados);
  - se añade la escena del editor de code_cap (luces, neblina y suelo de
    unitree_mujoco/unitree_robots/h1_2/scene.xml); sin ella el robot, gris oscuro, no
    se distingue del fondo.
La cinemática del cuerpo es la de unitree_mujoco/unitree_robots/h1_2: mismos nombres
de junta y rangos, y 0.00 mm de diferencia en las muñecas (comprobado el 2026-10-01).

Juntas mimic. MuJoCo convierte cada <mimic> en una restricción de igualdad, que solo
actúa al integrar la dinámica. El editor no la integra, así que Mimic.aplicar() copia
cada falange acoplada de su junta actuada.

Colisiones. Se comprueban en una SEGUNDA copia del modelo, con contactos, que nadie
integra: la del visor sigue sin contactos ni restricciones y no se mueve sola.
  - Se excluyen los pares que ya se tocan en la pose cero (pelvis/torso, la muñeca
    roll/yaw de cada lado, y la palma con la falange thumb_1 de cada mano).
  - Se avisa también de lo que queda a menos de `margen` sin tocarse entre partes
    distintas (brazo y torso, mano y pierna, un brazo y el otro...), salvo los pares
    que ya están así de cerca en la pose cero (hombro y torso): esos solo avisan si
    chocan. Dentro de una misma mano los dedos quedan siempre cerca al cerrar, y
    también solo avisan si chocan.
  - MuJoCo choca con la envolvente CONVEXA de cada malla: en las zonas cóncavas (el
    hueco de la palma, el torso) puede avisar de más, nunca de menos.
"""
from __future__ import annotations

import re
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

REPO = Path(__file__).resolve().parents[1]
PAQUETE = REPO / "ros_h1_2_ws" / "src" / "h1_2_inspire_description"
URDF_DEFECTO = PAQUETE / "urdf" / "h1_2_with_RH56DFTP_hands.urdf"

ALTURA_PELVIS = 1.40        # m: pies en el aire
MARGEN_DEFECTO = 0.005      # m: aviso de "cerca"
GRUPO_COLISION_OCULTO = 3   # el visor muestra los grupos 0-2: las mallas de colisión no se ven
ROJO = (0.95, 0.10, 0.10, 1.0)

_PIEZAS_MANO = ("hand_base", "thumb", "index", "middle", "ring", "little")

# La escena del editor de code_cap: unitree_mujoco/unitree_robots/h1_2/scene.xml, copiada tal cual,
# salvo que el suelo no colisiona (solo se comprueba el robot contra sí mismo, y los pies quedan en el aire).
ESCENA = """
<mujoco>
  <statistic center="1 0.7 1.5" extent="0.8"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0.9 0.9 0.9"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-140" elevation="-30"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="flat" rgb1="0 0 0" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
      rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>
"""


def _mallas_propias(spec: mujoco.MjSpec, urdf: ET.Element) -> None:
    """Que cada eslabón use la malla de su archivo. El importador de URDF de MuJoCo nombra cada
    malla por el nombre del archivo y reutiliza la primera con ese nombre: las dos palmas
    (base_link.STL en hand_left/ y en hand_right/) quedaban como una sola, la izquierda."""
    mallas = {m.name: m for m in spec.meshes}
    for link in urdf.iter("link"):
        archivos = {e.get("filename") for e in link.iter("mesh")}
        if len(archivos) != 1:
            continue
        archivo = archivos.pop()
        for g in spec.body(link.get("name")).geoms:
            if g.type != mujoco.mjtGeom.mjGEOM_MESH or mallas[g.meshname].file == archivo:
                continue
            nombre = f"{link.get('name')}_malla"
            if nombre not in mallas:
                nueva = spec.add_mesh()
                nueva.name, nueva.file, nueva.scale = nombre, archivo, mallas[g.meshname].scale
                mallas[nombre] = nueva
            g.meshname = nombre


def _con_escena(spec: mujoco.MjSpec) -> mujoco.MjSpec:
    """El robot dentro de ESCENA, uniendo los dos MJCF."""
    raiz = ET.fromstring(spec.to_xml())
    for parte in ET.fromstring(ESCENA):
        destino = raiz.find(parte.tag)
        if destino is None:
            raiz.append(parte)
        elif parte.tag in ("visual", "statistic"):
            destino.attrib.update(parte.attrib)
            for hijo in parte:
                viejo = destino.find(hijo.tag)
                if viejo is None:
                    destino.append(hijo)
                else:
                    viejo.attrib.update(hijo.attrib)
        else:
            destino.extend(list(parte))
    return mujoco.MjSpec.from_string(ET.tostring(raiz, encoding="unicode"))


def construir_spec(urdf: Path = URDF_DEFECTO) -> mujoco.MjSpec:
    texto = Path(urdf).read_text(encoding="utf-8")
    texto = texto.replace("package://h1_2_inspire_description/", str(PAQUETE) + "/")
    texto = re.sub(r"(<robot[^>]*>)",
                   r'\1<mujoco><compiler strippath="false" discardvisual="false" fusestatic="false"/></mujoco>',
                   texto, count=1)
    spec = mujoco.MjSpec.from_string(texto)
    _mallas_propias(spec, ET.fromstring(texto))
    spec.body("pelvis").pos = [0.0, 0.0, ALTURA_PELVIS]
    spec.option.gravity = [0.0, 0.0, 0.0]
    for lado in ("left", "right"):
        cuerpo = spec.body(f"{lado}_wrist_yaw_link")
        visual = next(g for g in cuerpo.geoms if g.type == mujoco.mjtGeom.mjGEOM_MESH)
        col = cuerpo.add_geom()
        col.type = mujoco.mjtGeom.mjGEOM_MESH
        col.meshname = visual.meshname
        col.pos, col.quat = visual.pos, visual.quat
        col.contype, col.conaffinity, col.group = 1, 1, 0
    return _con_escena(spec)


def es_de_colision(m: mujoco.MjModel, g: int) -> bool:
    return bool(m.geom_contype[g] or m.geom_conaffinity[g])


def modelos(urdf: Path = URDF_DEFECTO) -> tuple[mujoco.MjModel, mujoco.MjModel]:
    """(modelo del visor, modelo para comprobar colisiones), compilados por separado."""
    spec = construir_spec(urdf)
    visor, colision = spec.compile(), spec.compile()
    # Visor: nada se mueve salvo con los sliders. Sin contactos (en la pose cero el modelo se
    # toca a sí mismo), sin restricciones (las mimic las impone Mimic) y sin actuadores.
    visor.opt.disableflags |= (int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
                               | int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)
                               | int(mujoco.mjtDisableBit.mjDSBL_ACTUATION))
    for g in range(visor.ngeom):
        if es_de_colision(visor, g):
            visor.geom_group[g] = GRUPO_COLISION_OCULTO
    return visor, colision


def nombre_cuerpo(m: mujoco.MjModel, b: int) -> str:
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)


def grupo(nombre: str) -> str:
    """Parte del robot a la que pertenece un cuerpo, para el informe de colisiones."""
    if nombre == "world":
        return "suelo"
    for prefijo, lado in (("left_", "izq"), ("right_", "der")):
        if nombre.startswith(prefijo):
            if any(p in nombre for p in _PIEZAS_MANO):
                return f"mano {lado}"
            if any(p in nombre for p in ("shoulder", "elbow", "wrist")):
                return f"brazo {lado}"
            return f"pierna {lado}"
    return "torso" if nombre.startswith("torso") else "pelvis"


class Mimic:
    """Falanges acopladas: qpos[destino] = polinomio(qpos[origen]), de las igualdades del modelo."""

    def __init__(self, m: mujoco.MjModel):
        self.pares = []
        for i in range(m.neq):
            if m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT:
                destino = m.jnt_qposadr[m.eq_obj1id[i]]
                origen = m.jnt_qposadr[m.eq_obj2id[i]]
                self.pares.append((destino, origen, [float(c) for c in m.eq_data[i][:5]]))

    def aplicar(self, qpos) -> None:
        for destino, origen, c in self.pares:
            x = float(qpos[origen])
            qpos[destino] = c[0] + x * (c[1] + x * (c[2] + x * (c[3] + x * c[4])))


class Colisiones:
    """Comprueba una postura (qpos del visor) en la copia del modelo con contactos."""

    def __init__(self, m: mujoco.MjModel, margen: float = MARGEN_DEFECTO):
        self.m, self.d = m, mujoco.MjData(m)
        self.margen = max(float(margen), 0.0)
        self.cerrojo = threading.Lock()
        for g in range(m.ngeom):
            if es_de_colision(m, g):
                m.geom_margin[g] = self.margen
        self.mimic = Mimic(m)
        # Pose cero: lo que ya se toca se excluye; lo que ya está cerca solo avisa si choca.
        base = self._pares(self.d.qpos * 0.0)
        self.excluidos = {p for p, dist in base.items() if dist < 0.0}
        self.solo_choque = {p for p, dist in base.items() if dist >= 0.0}

    def _pares(self, qpos) -> dict[tuple[str, str], float]:
        """{(cuerpo_a, cuerpo_b): distancia mínima en m (negativa = penetración)}."""
        with self.cerrojo:
            self.d.qpos[:] = qpos
            self.mimic.aplicar(self.d.qpos)
            mujoco.mj_forward(self.m, self.d)
            pares = {}
            for c in self.d.contact[:self.d.ncon]:
                a = nombre_cuerpo(self.m, self.m.geom_bodyid[c.geom1])
                b = nombre_cuerpo(self.m, self.m.geom_bodyid[c.geom2])
                par = tuple(sorted((a, b)))
                pares[par] = min(pares.get(par, float("inf")), float(c.dist))
            return pares

    def revisar(self, qpos) -> list[tuple[str, str, float]]:
        """[(cuerpo_a, cuerpo_b, distancia_m)] de choques (< 0) y acercamientos (< margen) entre
        partes distintas, sin los pares excluidos, ordenados de peor a mejor."""
        salida = []
        for (a, b), dist in self._pares(qpos).items():
            if (a, b) in self.excluidos:
                continue
            cerca = (a, b) not in self.solo_choque and grupo(a) != grupo(b) and dist < self.margen
            if dist < 0.0 or cerca:
                salida.append((a, b, dist))
        return sorted(salida, key=lambda x: x[2])


def informe(hallazgos: list[tuple[str, str, float]]) -> list[str]:
    """Una línea por pareja de partes (brazo izq / torso...), con su par de cuerpos más grave."""
    peor = {}
    for a, b, dist in hallazgos:
        clave = tuple(sorted((grupo(a), grupo(b))))
        if clave not in peor or dist < peor[clave][2]:
            peor[clave] = (a, b, dist)
    lineas = []
    for (ga, gb), (a, b, dist) in sorted(peor.items(), key=lambda kv: kv[1][2]):
        etiqueta = "[COLISIÓN]" if dist < 0 else "[CERCA]   "
        partes = ga if ga == gb else f"{ga} <-> {gb}"
        lineas.append(f"{etiqueta} {partes}: {a} / {b}  {dist * 1000:+.0f} mm")
    return lineas


# ============================================================================= modo física
# El mismo robot con gravedad, contactos y motores, colgado de un pórtico. Lo usa
# simulador_fisica.py (./editor_mujoco.sh --fisica).

# Juntas del cuerpo en el orden DDS (índice i de motor_cmd = JUNTAS_CUERPO[i]).
JUNTAS_CUERPO = [f"{l}_{j}_joint" for l in ("left", "right")
                 for j in ("hip_yaw", "hip_pitch", "hip_roll", "knee", "ankle_pitch", "ankle_roll")] + ["torso_joint"] + \
                [f"{l}_{j}_joint" for l in ("left", "right")
                 for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")]
# Como en unitree_mujoco/unitree_robots/h1_2/h1_2_handless.xml: <joint damping="1" armature="0.1"
# frictionloss="0.2"/> y el ctrlrange de cada <motor> (N·m).
JUNTA_CUERPO = dict(damping=1.0, armature=0.1, frictionloss=0.2)
PAR_MAX = [200, 200, 200, 300, 60, 40] * 2 + [200] + [40, 40, 18, 18, 19, 19, 19] * 2

# Manos. De la caracterización de la RH56DFTP (inspire_hand_interface/Caracterizacion): con SPEED_SET 1000
# (el que pone el selector) el actuador avanza a ~3040 cuentas/s sin sobreimpulso, con ~65 ms de retardo
# entre la orden y el movimiento. Recorridos libres: dedos 1820 cuentas, flexión del pulgar 860, rotación
# 1505. Cada DOF va a velocidad constante hasta su objetivo: un servo de posición sigue a una consigna
# limitada en velocidad.
DOF_MANO = ("little_1", "ring_1", "middle_1", "index_1", "thumb_1", "thumb_swing")
VEL_MANO = {"little_1": 2.71, "ring_1": 2.71, "middle_1": 2.71, "index_1": 2.71,   # 1.62 rad en 0.60 s
            "thumb_1": 2.33, "thumb_swing": 2.40}                                  # 0.66 en 0.28 s, 1.19 en 0.50 s
RETARDO_MANO = 0.065        # s
# Servo de cada DOF: rigidez y amortiguamiento que siguen la consigna sin oscilar con dt = 2 ms; el par máximo
# equivale a ~15 N en la yema (la hoja de datos da >= 30 N de agarre en las puntas de los dedos).
JUNTA_DEDO = dict(damping=0.01, armature=0.002, frictionloss=0.0)
KP_DEDO, KV_DEDO, PAR_MAX_DEDO = 20.0, 0.4, 1.0

# Pórtico: una barra de 30 cm, 1 m por encima de dos enganches en la placa superior del torso, a cada lado
# del soporte del cuello (y = ±0.10 m, 0.473 m sobre el origen de torso_link). Dos cuerdas rígidas (solo
# tiran, no empujan) bajan de la barra a los enganches sin cruzar la cabeza. No se dibuja nada del pórtico
# (ni barra, ni cuerdas, ni enganches). Las teclas 7/8 suben y bajan la barra; 9 suelta o vuelve a enganchar
# las cuerdas.
GANCHO = (0.0, 0.10, 0.473)     # en torso_link; el otro, con y cambiado de signo
BARRA_MEDIA = 0.15              # m, media longitud de la barra
ALTURA_BARRA = 1.0              # m por encima de los enganches


def modelo_fisica(urdf: Path = URDF_DEFECTO):
    """(modelo con física, pares excluidos). Motores 0-26 = juntas del cuerpo en orden DDS (par, N·m);
    27-38 = servos de los dedos, mano izquierda y luego derecha en el orden de DOF_MANO (posición, rad)."""
    spec = construir_spec(urdf)
    excluidos = Colisiones(spec.compile(), 0.0).excluidos      # los que ya se tocan en la pose cero
    for a, b in sorted(excluidos):
        ex = spec.add_exclude()
        ex.bodyname1, ex.bodyname2 = a, b
    spec.body("pelvis").add_freejoint()
    spec.option.gravity = [0.0, 0.0, -9.81]
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    suelo = spec.geom("floor")
    suelo.contype, suelo.conaffinity = 1, 1
    for j in spec.joints:
        if j.type != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        p = JUNTA_CUERPO if j.name in JUNTAS_CUERPO else JUNTA_DEDO
        j.damping[0], j.armature, j.frictionloss = p["damping"], p["armature"], p["frictionloss"]
    for nombre, par in zip(JUNTAS_CUERPO, PAR_MAX):
        a = spec.add_actuator()
        a.name, a.target, a.trntype = nombre, nombre, mujoco.mjtTrn.mjTRN_JOINT
        a.gainprm[0] = 1.0
        a.ctrllimited, a.ctrlrange = True, [-par, par]
    for lado in ("left", "right"):
        for k in DOF_MANO:
            a = spec.add_actuator()
            a.name, a.target, a.trntype = f"{lado}_{k}", f"{lado}_{k}_joint", mujoco.mjtTrn.mjTRN_JOINT
            a.gainprm[0] = KP_DEDO
            a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            a.biasprm[1], a.biasprm[2] = -KP_DEDO, -KV_DEDO
            a.forcelimited, a.forcerange = True, [-PAR_MAX_DEDO, PAR_MAX_DEDO]
    # pórtico
    torso = spec.body("torso_link")
    z_gancho = ALTURA_PELVIS + GANCHO[2]               # en la pose cero, torso_link está en la pelvis
    barra = spec.worldbody.add_body()
    barra.name, barra.mocap, barra.pos = "portico", True, [GANCHO[0], 0.0, z_gancho + ALTURA_BARRA]
    largo = float(((BARRA_MEDIA - GANCHO[1]) ** 2 + ALTURA_BARRA ** 2) ** 0.5)
    for lado, s in (("izq", 1.0), ("der", -1.0)):
        sb = barra.add_site()
        sb.name, sb.pos, sb.group = f"portico_{lado}", [0.0, s * BARRA_MEDIA, 0.0], GRUPO_COLISION_OCULTO
        st = torso.add_site()
        st.name, st.pos = f"gancho_{lado}", [GANCHO[0], s * GANCHO[1], GANCHO[2]]
        st.size, st.rgba, st.group = [0.012, 0, 0], [0.85, 0.65, 0.10, 1.0], GRUPO_COLISION_OCULTO
        t = spec.add_tendon()
        t.name = f"cuerda_{lado}"
        t.wrap_site(f"portico_{lado}")
        t.wrap_site(f"gancho_{lado}")
        t.limited, t.range = True, [0.0, largo]
        t.width, t.rgba, t.group = 0.006, [0.95, 0.80, 0.20, 1.0], GRUPO_COLISION_OCULTO
    return spec.compile(), excluidos
