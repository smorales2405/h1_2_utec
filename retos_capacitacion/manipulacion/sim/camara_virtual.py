"""D435i virtual del gemelo: devuelve Fotograma como camara_servidor.py (color BGR, profundidad uint16 en mm
en el marco del IR izquierdo, IR en gris y los mismos metadatos), para que percepcion y calibracion corran
sin cambios sobre imagenes sinteticas con verdad conocida.

Necesita OpenGL: en la PC con MUJOCO_GL=glfw (el defecto con escritorio); en el PC2 no se renderiza.
"""

import mujoco
import numpy as np

from manip.mensajes import Fotograma

PROF_MAX_M = 4.0          # mas lejos que esto: sin dato (0), como el fondo de la D435i


class CamaraVirtual:
    def __init__(self, m, info, cal, ruido_mm=0.0, semilla=0):
        if not info.get("camaras"):
            raise ValueError("el gemelo se construyo sin camaras (falta la calibracion)")
        self.m, self.info, self.cal = m, info, cal
        self.nombre_color, self.intr_color = info["camaras"]["color"]
        self.nombre_prof, self.intr_prof = info["camaras"]["prof"]
        self.r = mujoco.Renderer(m, self.intr_color.alto, self.intr_color.ancho)
        # la malla del cuerpo que lleva la camara (torso_link, con la cabeza) no se pinta: con la camara calibrada
        # (~1 cm hacia atras y arriba del URDF, dentro de la incertidumbre a lo largo del eje) quedaba dentro de
        # la visera y su borde cruzaba la imagen (2026-10-18). Desde la cabeza no se ve el propio torso
        cuerpo = m.cam_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, self.nombre_color)]
        self._propios = np.array([g for g in range(m.ngeom) if m.geom_bodyid[g] == cuerpo], int)
        self._opt = mujoco.MjvOption()                  # grupos 0-2 visibles; el 5 no
        self.ruido_mm = ruido_mm
        self.rng = np.random.default_rng(semilla)
        self.n = 0

    def meta(self):
        return {"color": self.intr_color.a_dict(), "prof": self.intr_prof.a_dict(), "ir": self.intr_prof.a_dict(),
                "escala_prof": 0.001, "fps": 30, "emisor": True, "prof_a_color": np.asarray(self.cal.T_color_prof).tolist(),
                "virtual": True}

    def _escena(self, d, camara):
        grupos = self.m.geom_group[self._propios].copy()
        self.m.geom_group[self._propios] = 5
        try:
            self.r.update_scene(d, camara, scene_option=self._opt)
        finally:
            self.m.geom_group[self._propios] = grupos

    def _rgb(self, d, camara):
        self.r.disable_depth_rendering()
        self.r.disable_segmentation_rendering()
        self._escena(d, camara)
        return self.r.render().copy()

    def _profundidad(self, d, camara):
        self.r.enable_depth_rendering()
        self._escena(d, camara)
        z = self.r.render().copy()
        self.r.disable_depth_rendering()
        return z

    def segmentacion(self, d, flujo="color"):
        """(alto, ancho) con el id de geom de cada pixel (-1 = fondo)."""
        self.r.enable_segmentation_rendering()
        self._escena(d, self.nombre_color if flujo == "color" else self.nombre_prof)
        s = self.r.render().copy()
        self.r.disable_segmentation_rendering()
        ids = s[:, :, 0].astype(int)
        ids[s[:, :, 1] != int(mujoco.mjtObj.mjOBJ_GEOM)] = -1
        return ids

    def fotograma(self, estado, t=0.0):
        d = estado.d
        color = self._rgb(d, self.nombre_color)[:, :, ::-1].copy()
        ir_rgb = self._rgb(d, self.nombre_prof)
        ir = (0.299 * ir_rgb[:, :, 0] + 0.587 * ir_rgb[:, :, 1] + 0.114 * ir_rgb[:, :, 2]).astype(np.uint8)
        z = self._profundidad(d, self.nombre_prof)
        if self.ruido_mm > 0:
            z = z + self.rng.normal(0.0, self.ruido_mm / 1000.0, z.shape)
        z[(z <= 0) | (z > PROF_MAX_M)] = 0
        prof = np.round(z * 1000.0).astype(np.uint16)
        self.n += 1
        return Fotograma(n=self.n, t_cam=t, t_rx=t, ir=ir, color=color, prof=prof, emisor=True, meta=self.meta())

    def cerrar(self):
        self.r.close()
