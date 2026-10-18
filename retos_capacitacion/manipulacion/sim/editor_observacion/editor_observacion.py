#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Editor de la pose de observacion del reto de manipulacion (MuJoCo, sin fisica).
#
# Una ventana como h1_2_mujoco/editor_gui.py: la vista 3D del gemelo de
# manipulacion.py (robot de pie en la postura medida, manos abiertas, el carrito
# medido, sin bandejas ni objetos) y los controles:
#   - un solo grupo de sliders, en grados, para los dos brazos en espejo (el
#     izquierdo; el derecho es su espejo, como lo usa manipulacion.py)
#   - poses guardadas en poses_observacion.json (y, solo para ver, las de
#     config/poses_brazos.yaml)
#   - el movimiento reposo <-> pose como lo hara el robot (minima sacudida,
#     duracion de la seccion 10) con las colisiones del camino
#   - «Comprobar como manipulacion.py»: los criterios del programa
# No escribe en config/: la pose elegida se copia a mano (botón «Copiar YAML»).
#
#   ./editor_observacion.sh
# -----------------------------------------------------------------------------

import os
os.environ.setdefault("MUJOCO_GL", "glfw")       # antes de importar mujoco: el render fuera de pantalla

import argparse
import math
import sys
import threading
import time

import mujoco
import numpy as np
from PyQt5.QtCore import Qt, QObject, QSize, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QInputDialog, QLabel,
                             QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QPlainTextEdit, QProgressBar,
                             QPushButton, QScrollArea, QSizePolicy, QSlider, QSplitter, QVBoxLayout, QWidget)

import nucleo as nu

# nucleo importa cv2 (sim/escena.py), y opencv-python apunta Qt a sus propios plugins: con ellos PyQt5 no arranca
for _k in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
    if "cv2" in os.environ.get(_k, ""):
        os.environ.pop(_k)

OFFSCREEN_MAX = (2560, 1600)
COLOR_CHOQUE, COLOR_CERCA, COLOR_OK = QColor(255, 205, 205), QColor(255, 230, 190), QColor(210, 245, 210)
COLOR_CONFIG = QColor(90, 90, 90)
LINEAS_ABAJO = 6
FPS = 30


class Puente(QObject):
    """Senales de los hilos (animacion, comprobacion) hacia la ventana."""
    progreso = pyqtSignal(float)
    fin_animacion = pyqtSignal(str, object, bool)
    fin_comprobacion = pyqtSignal(object, object)


class FilaJunta(QWidget):
    """Una junta del brazo izquierdo: slider y casilla en grados, y al lado el valor del derecho (el espejo)."""

    def __init__(self, nombre, lo, hi, signo, al_cambiar):
        super().__init__()
        self.signo, self.al_cambiar = signo, al_cambiar
        etiqueta = QLabel(nombre)
        etiqueta.setMinimumWidth(92)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(round(math.degrees(lo) * 10), round(math.degrees(hi) * 10))
        self.casilla = QDoubleSpinBox()
        self.casilla.setRange(math.degrees(lo), math.degrees(hi))
        self.casilla.setDecimals(1)
        self.casilla.setSuffix("°")
        self.casilla.setKeyboardTracking(False)
        self.casilla.setFixedWidth(80)
        self.der = QLabel()
        self.der.setFixedWidth(78)
        self.der.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.der.setToolTip("brazo derecho (espejo)")
        self.setToolTip(f"{nombre}: de {math.degrees(lo):+.1f}° a {math.degrees(hi):+.1f}° (límites del programa, "
                        "también para el espejo del derecho)")
        fila = QHBoxLayout(self)
        fila.setContentsMargins(0, 0, 0, 0)
        fila.addWidget(etiqueta)
        fila.addWidget(self.slider, 1)
        fila.addWidget(self.casilla)
        fila.addWidget(self.der)
        self.slider.valueChanged.connect(self._desde_slider)
        self.casilla.valueChanged.connect(self._desde_casilla)

    def _der(self, grados):
        self.der.setText(f"der {self.signo * grados:+.1f}°")

    def _desde_slider(self, v):
        self.casilla.blockSignals(True)
        self.casilla.setValue(v / 10.0)
        self.casilla.blockSignals(False)
        self._der(v / 10.0)
        self.al_cambiar()

    def _desde_casilla(self, v):
        self.slider.blockSignals(True)
        self.slider.setValue(round(v * 10))
        self.slider.blockSignals(False)
        self._der(v)
        self.al_cambiar()

    def grados(self):
        return self.casilla.value()

    def poner(self, grados):
        for w, v in ((self.slider, round(grados * 10)), (self.casilla, grados)):
            w.blockSignals(True)
            w.setValue(v)
            w.blockSignals(False)
        self._der(self.casilla.value())


class Vista(QLabel):
    """La escena. Raton como en el visor de MuJoCo: izquierdo gira, derecho desplaza (con Mayusculas, en el
    otro plano), central o rueda acercan; doble clic vuelve a la vista inicial."""

    def __init__(self, escena):
        super().__init__()
        self.e = escena
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setMinimumSize(480, 360)
        self.setAlignment(Qt.AlignCenter)
        self.cam = mujoco.MjvCamera()
        self.camara_inicial()
        self.renderer, self.tam = None, None
        self.boton, self.ultimo = None, None

    def sizeHint(self):
        return QSize(900, 700)

    def camara_inicial(self):
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.lookat[:] = [0.25, 0.0, 0.0]
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 2.6, 210.0, -22.0

    def renderizar(self):
        w = min(max(self.width(), 16), OFFSCREEN_MAX[0])
        h = min(max(self.height(), 16), OFFSCREEN_MAX[1])
        if self.tam != (w, h):
            if self.renderer is not None:
                self.renderer.close()
            self.renderer, self.tam = mujoco.Renderer(self.e.m, h, w), (w, h)
        with self.e.cerrojo:
            self.renderer.update_scene(self.e.d, self.cam)
        img = self.renderer.render()
        self.setPixmap(QPixmap.fromImage(QImage(img.data, w, h, 3 * w, QImage.Format_RGB888)))

    def cerrar(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def mousePressEvent(self, ev):
        self.boton, self.ultimo = ev.button(), ev.pos()

    def mouseReleaseEvent(self, ev):
        self.boton = None

    def mouseMoveEvent(self, ev):
        if self.boton is None:
            return
        dx, dy = ev.x() - self.ultimo.x(), ev.y() - self.ultimo.y()
        self.ultimo = ev.pos()
        otro_plano = bool(ev.modifiers() & Qt.ShiftModifier)
        m = mujoco.mjtMouse
        if self.boton == Qt.LeftButton:
            accion = m.mjMOUSE_ROTATE_H if otro_plano else m.mjMOUSE_ROTATE_V
        elif self.boton == Qt.RightButton:
            accion = m.mjMOUSE_MOVE_H if otro_plano else m.mjMOUSE_MOVE_V
        else:
            accion = m.mjMOUSE_ZOOM
        h = max(self.height(), 1)
        mujoco.mjv_moveCamera(self.e.m, accion, dx / h, dy / h, self.cam)

    def wheelEvent(self, ev):
        mujoco.mjv_moveCamera(self.e.m, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * ev.angleDelta().y() / 120, self.cam)

    def mouseDoubleClickEvent(self, ev):
        self.camara_inicial()


class Ventana(QMainWindow):
    def __init__(self, ruta_poses):
        super().__init__()
        self.ruta_poses = ruta_poses
        self.e = nu.Escena()
        self.e.m.vis.global_.offwidth, self.e.m.vis.global_.offheight = OFFSCREEN_MAX
        self.criterios = None
        self.poses = nu.leer_poses(ruta_poses)
        self.de_config = nu.poses_de_config()
        self.ocupado = False
        self.detener = False
        self.hilo = None
        self.q_cargada = None           # la pose cargada, exacta, mientras no se toque un slider (van a 0.1 grados)
        self.mostrando = "reposo"
        self.ultimas = None
        self.puente = Puente()
        self.puente.progreso.connect(lambda f: self.barra.setValue(round(1000 * f)))
        self.puente.fin_animacion.connect(self._fin_animacion)
        self.puente.fin_comprobacion.connect(self._fin_comprobacion)
        self._construir()
        self._llenar_lista()
        self._poner_sliders(self.e.reposo["izq"])
        self.e.poner_reposo()
        self._actualizar_info()
        self.t_vista = QTimer(self, timeout=self.vista.renderizar, interval=1000 // FPS)
        self.t_vista.start()
        self.t_estado = QTimer(self, timeout=self._refrescar_colisiones, interval=100)
        self.t_estado.start()
        self.setWindowTitle("Pose de observación — reto de manipulación H1-2")
        donde = os.path.relpath(ruta_poses, nu.RAIZ) if ruta_poses.startswith(nu.RAIZ) else ruta_poses
        self.registrar(f"Robot en reposo (postura medida), manos abiertas, carrito sin bandejas. Poses en "
                       f"{donde}: {len(self.poses)}.")
        self.statusBar().showMessage("Vista: botón izquierdo gira, derecho desplaza, rueda acerca; doble clic, "
                                     "vista inicial.")

    # ================================================================ construccion
    def _construir(self):
        self.vista = Vista(self.e)
        self.lista_colisiones = QListWidget()
        self.lista_colisiones.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.lista_colisiones.addItem("Sin colisiones.")
        self.lista_colisiones.setFixedHeight(self.lista_colisiones.sizeHintForRow(0) * LINEAS_ABAJO
                                             + 2 * self.lista_colisiones.frameWidth())
        self.registro = QPlainTextEdit(readOnly=True)
        self.registro.setMaximumBlockCount(1000)
        self.registro.setFixedHeight(self.registro.fontMetrics().lineSpacing() * LINEAS_ABAJO
                                     + 2 * self.registro.frameWidth()
                                     + 2 * round(self.registro.document().documentMargin()))
        abajo = QSplitter(Qt.Horizontal)
        abajo.addWidget(self._caja("Colisiones ahora (choque < 0, cerca < holgura de "
                                   f"{self.e.holgura * 100:.0f} cm)", self.lista_colisiones))
        abajo.addWidget(self._caja("Registro", self.registro))
        izquierda = QWidget()
        capas = QVBoxLayout(izquierda)
        capas.setContentsMargins(0, 0, 0, 0)
        capas.addWidget(self.vista, 1)
        capas.addWidget(abajo)

        panel = QWidget()
        col = QVBoxLayout(panel)
        col.setContentsMargins(6, 4, 6, 4)
        col.setSpacing(4)
        col.addWidget(self._grupo_poses(), 1)
        col.addWidget(self._grupo_sliders())
        col.addWidget(self._grupo_movimiento())
        col.addWidget(self._grupo_info())
        desplazable = QScrollArea()
        desplazable.setWidget(panel)
        desplazable.setWidgetResizable(True)
        desplazable.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        ancho = panel.minimumSizeHint().width() + desplazable.verticalScrollBar().sizeHint().width() + 8
        desplazable.setMinimumWidth(max(ancho, 560))

        principal = QSplitter(Qt.Horizontal)
        principal.addWidget(izquierda)
        principal.addWidget(desplazable)
        principal.setStretchFactor(0, 1)
        principal.setStretchFactor(1, 0)
        principal.setCollapsible(0, False)
        principal.setCollapsible(1, False)
        self.setCentralWidget(principal)
        self.resize(1600, 1000)
        principal.setSizes([1600 - max(ancho, 560), max(ancho, 560)])

    @staticmethod
    def _caja(titulo, widget):
        caja = QGroupBox(titulo)
        QVBoxLayout(caja).addWidget(widget)
        return caja

    @staticmethod
    def _boton(texto, accion, ayuda=None):
        b = QPushButton(texto)
        b.clicked.connect(accion)
        if ayuda:
            b.setToolTip(ayuda)
        return b

    def _grupo_poses(self):
        g = QGroupBox("Poses de observación")
        v = QVBoxLayout(g)
        self.lista = QListWidget()
        self.lista.setMinimumHeight(150)
        self.lista.itemClicked.connect(self._al_elegir)
        v.addWidget(self.lista)
        fila = QHBoxLayout()
        self.botones_poses = [
            self._boton("Guardar nueva…", self.guardar_nueva, "La pose de los sliders, con un nombre"),
            self._boton("Sobrescribir", self.sobrescribir, "La pose seleccionada = la de los sliders"),
            self._boton("Borrar", self.borrar),
            self._boton("Copiar YAML", self.copiar_yaml,
                        "Las líneas de config/poses_brazos.yaml con la pose de los sliders (al portapapeles)"),
        ]
        for b in self.botones_poses:
            fila.addWidget(b)
        v.addLayout(fila)
        nota = QLabel("En gris, las de config/poses_brazos.yaml (solo para ver). El editor no escribe en config/.")
        nota.setWordWrap(True)
        v.addWidget(nota)
        return g

    def _grupo_sliders(self):
        g = QGroupBox("Brazos en espejo (izquierdo; el derecho es su espejo)")
        v = QVBoxLayout(g)
        v.setSpacing(1)
        self.filas = []
        for k, nombre in enumerate(nu.NOMBRES):
            f = FilaJunta(nombre, self.e.lo[k], self.e.hi[k], nu.SIGNO_ESPEJO[k], self._al_mover_slider)
            v.addWidget(f)
            self.filas.append(f)
        fila = QHBoxLayout()
        self.botones_postura = [
            self._boton("Ver reposo", self.ver_reposo, "El robot en la postura de partida (los sliders no cambian)"),
            self._boton("Ver pose", self.ver_pose, "El robot en la pose de los sliders"),
            self._boton("Sliders = reposo", lambda: self._cargar(self.e.reposo["izq"], "reposo del izquierdo")),
        ]
        for b in self.botones_postura:
            fila.addWidget(b)
        v.addLayout(fila)
        self.caja_sliders = g
        return g

    def _grupo_movimiento(self):
        g = QGroupBox("Movimiento (como manip/brazo.py, sin física)")
        v = QVBoxLayout(g)
        fila = QHBoxLayout()
        fila.addWidget(QLabel("Duración"))
        self.combo_escala = QComboBox()
        self.combo_escala.addItem("×2 (primeras tiradas)", 2.0)
        self.combo_escala.addItem("×1 (mínima)", 1.0)
        self.combo_escala.currentIndexChanged.connect(self._actualizar_info)
        fila.addWidget(self.combo_escala)
        fila.addWidget(QLabel("Reproducir a"))
        self.combo_vel = QComboBox()
        for x in (1, 2, 4, 8):
            self.combo_vel.addItem(f"×{x}", float(x))
        fila.addWidget(self.combo_vel)
        fila.addStretch(1)
        v.addLayout(fila)
        fila = QHBoxLayout()
        self.b_ida = self._boton("▶ Reposo → pose", lambda: self.animar(True))
        self.b_vuelta = self._boton("▶ Pose → reposo", lambda: self.animar(False))
        self.b_detener = self._boton("Detener", self.parar)
        self.b_detener.setEnabled(False)
        self.b_comprobar = self._boton("Comprobar como manipulacion.py", self.comprobar,
                                       "Tramos reposo <-> pose con el planificador (con bandejas, como en la tirada), "
                                       "mano alta, zona sin tapar en la imagen y alcance a la zona desde la pose")
        for b in (self.b_ida, self.b_vuelta, self.b_detener):
            fila.addWidget(b)
        v.addLayout(fila)
        self.barra = QProgressBar()
        self.barra.setRange(0, 1000)
        self.barra.setTextVisible(False)
        self.barra.setFixedHeight(8)
        v.addWidget(self.barra)
        v.addWidget(self.b_comprobar)
        return g

    def _grupo_info(self):
        g = QGroupBox("La pose de los sliders")
        v = QVBoxLayout(g)
        self.info = QLabel()
        self.info.setWordWrap(True)
        self.info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        v.addWidget(self.info)
        return g

    # ================================================================ utilidades
    def registrar(self, texto):
        self.registro.appendPlainText(texto)

    def avisar(self, texto):
        self.registrar(f"[AVISO] {texto}")
        self.statusBar().showMessage(texto, 6000)

    def confirmar(self, texto):
        return QMessageBox.question(self, "Pose de observación", texto) == QMessageBox.Yes

    def q_sliders(self):
        if self.q_cargada is not None:
            return self.q_cargada.copy()
        return np.radians([f.grados() for f in self.filas])

    def _poner_sliders(self, q_izq):
        for f, v in zip(self.filas, np.degrees(np.clip(q_izq, self.e.lo, self.e.hi))):
            f.poner(float(v))

    def _actualizar_info(self):
        q = self.q_sliders()
        hasta = {"izq": q, "der": nu.espejo(q)}
        escala = self.combo_escala.currentData()
        T, dq = self.e.duracion(self.e.reposo, hasta, escala)
        dp = self.e.datos_pose(q)
        y = dp["yema"]
        self.info.setText(
            f"Reposo → pose: <b>{T:.1f} s</b> a ×{escala:g} (salto mayor {math.degrees(dq):.0f}°, pico "
            f"{1.875 * dq / T:.2f} rad/s).<br>"
            f"Yema izquierda en ({y[0]:.2f}, {y[1]:+.2f}, {y[2]:+.2f}) m del torso: <b>{dp['altura_yema'] * 100:.0f} cm"
            f"</b> sobre la mesa; mano a {dp['altura_mano'] * 100:.0f} cm.<br>"
            f"izq (rad): {[round(float(v), 4) for v in q]}")

    # ================================================================ poses
    def _llenar_lista(self, seleccionar=None):
        self.lista.clear()
        for nombre, q in self.de_config:
            it = QListWidgetItem(nombre)
            it.setData(Qt.UserRole, ("config", q))
            it.setForeground(COLOR_CONFIG)
            f = QFont()
            f.setItalic(True)
            it.setFont(f)
            self.lista.addItem(it)
        for k, p in enumerate(self.poses):
            it = QListWidgetItem(f"{p['nombre']}   ({p.get('fecha', '')[:16]})")
            it.setData(Qt.UserRole, ("archivo", k))
            self.lista.addItem(it)
            if seleccionar == k:
                self.lista.setCurrentItem(it)

    def _seleccion(self):
        it = self.lista.currentItem()
        return it.data(Qt.UserRole) if it is not None else None

    def _al_elegir(self, it):
        if self.ocupado:
            return
        tipo, x = it.data(Qt.UserRole)
        if tipo == "config":
            self._cargar(np.asarray(x, float), it.text())
        else:
            p = self.poses[x]
            self._cargar(np.array(p["izq"], float), p["nombre"])
            if p.get("notas"):
                self.registrar(f"  notas: {p['notas']}")

    def _cargar(self, q_izq, nombre):
        q = np.asarray(q_izq, float)
        fuera = np.any(q < self.e.lo - 1e-6) or np.any(q > self.e.hi + 1e-6)
        self._poner_sliders(q)
        self.q_cargada = np.clip(q, self.e.lo, self.e.hi)
        self.ver_pose()
        self.registrar(f"Cargada «{nombre}»" + (" (recortada a los límites del programa)" if fuera else "") + ".")

    def guardar_nueva(self):
        nombre, ok = QInputDialog.getText(self, "Guardar pose", "Nombre de la pose:")
        if not ok or not nombre.strip():
            return
        notas, _ = QInputDialog.getText(self, "Guardar pose", "Notas (opcional):")
        self.poses.append(nu.pose_nueva(nombre.strip(), self.q_sliders(), notas.strip()))
        nu.escribir_poses(self.poses, self.ruta_poses)
        self._llenar_lista(seleccionar=len(self.poses) - 1)
        self.registrar(f"Guardada «{nombre.strip()}» en {os.path.basename(self.ruta_poses)}.")

    def sobrescribir(self):
        sel = self._seleccion()
        if sel is None or sel[0] != "archivo":
            self.avisar("Selecciona en la lista una pose del archivo (las de config/ no se modifican aquí).")
            return
        p = self.poses[sel[1]]
        if not self.confirmar(f"¿Sobrescribir «{p['nombre']}» con la pose de los sliders?"):
            return
        self.poses[sel[1]] = nu.pose_nueva(p["nombre"], self.q_sliders(), p.get("notas", ""))
        nu.escribir_poses(self.poses, self.ruta_poses)
        self._llenar_lista(seleccionar=sel[1])
        self.registrar(f"Sobrescrita «{p['nombre']}».")

    def borrar(self):
        sel = self._seleccion()
        if sel is None or sel[0] != "archivo":
            self.avisar("Selecciona en la lista una pose del archivo.")
            return
        p = self.poses[sel[1]]
        if not self.confirmar(f"¿Borrar «{p['nombre']}» del archivo?"):
            return
        self.poses.pop(sel[1])
        nu.escribir_poses(self.poses, self.ruta_poses)
        self._llenar_lista()
        self.registrar(f"Borrada «{p['nombre']}».")

    def copiar_yaml(self):
        texto = nu.texto_yaml(self.q_sliders())
        QApplication.clipboard().setText(texto)
        self.registrar("Copiado al portapapeles (para config/poses_brazos.yaml, a mano):\n" + texto)

    # ================================================================ postura
    def _al_mover_slider(self):
        if self.ocupado:
            return
        self.q_cargada = None
        self.e.poner_pose(self.q_sliders())
        self.mostrando = "pose"
        self._actualizar_info()

    def ver_reposo(self):
        self.e.poner_reposo()
        self.mostrando = "reposo"

    def ver_pose(self):
        self.e.poner_pose(self.q_sliders())
        self.mostrando = "pose"
        self._actualizar_info()

    # ================================================================ animacion (en un hilo)
    def animar(self, ida):
        if self.ocupado:
            return
        q = self.q_sliders()
        pose = {"izq": q, "der": nu.espejo(q)}
        desde, hasta = (self.e.reposo, pose) if ida else (pose, self.e.reposo)
        escala, vel = self.combo_escala.currentData(), self.combo_vel.currentData()
        T, _ = self.e.duracion(desde, hasta, escala)
        nombre = "reposo → pose" if ida else "pose → reposo"
        self.registrar(f"Animando {nombre}: {T:.1f} s a ×{escala:g} (reproducido a ×{vel:g}).")
        self._ocupar(True)

        def trabajo():
            peor = {}
            t0 = time.monotonic()
            completo = True
            while True:
                if self.detener:
                    completo = False
                    break
                s = min(1.0, (time.monotonic() - t0) * vel / T)
                qq = self.e.en(desde, hasta, s)
                self.e.poner_brazos(qq["izq"], qq["der"])
                h = self.e.revisar()
                self.e.resaltar(h)
                for a, b, dist in h:
                    peor[(a, b)] = min(peor.get((a, b), np.inf), dist)
                self.puente.progreso.emit(s)
                if s >= 1.0:
                    break
                time.sleep(1.0 / 60)
            if completo:
                # muestreo fino del camino entero: que no se escape un roce entre dos fotogramas
                for (a, b), d in self.e.recorrer(desde, hasta).items():
                    peor[(a, b)] = min(peor.get((a, b), np.inf), d)
                self.e.poner_brazos(hasta["izq"], hasta["der"])
            self.puente.fin_animacion.emit(nombre, peor, completo)
        self.hilo = threading.Thread(target=trabajo, daemon=True)
        self.hilo.start()

    def _fin_animacion(self, nombre, peor, completo):
        self._ocupar(False)
        self.mostrando = ("pose" if nombre.endswith("pose") else "reposo") if completo else "parado"
        if not completo:
            self.registrar(f"  {nombre}: detenido.")
            return
        mesa = {k: d for k, d in peor.items() if nu.es_contra_mesa(*k)}
        cuerpo = {k: d for k, d in peor.items() if not nu.es_contra_mesa(*k)}
        choque_mesa = any(d < 0 for d in mesa.values())
        self.registrar(f"  {nombre}, camino contra el carrito: "
                       + ("CHOCA" if choque_mesa else ("limpio" if not mesa else "sin choque, pero cerca"))
                       + (f" — {nu.resumen(mesa)}" if mesa else ""))
        if cuerpo:
            self.registrar(f"  {nombre}, propio cuerpo: {nu.resumen(cuerpo)}")

    def parar(self):
        self.detener = True

    def _ocupar(self, si):
        self.ocupado = si
        self.detener = False
        for w in (self.caja_sliders, self.lista, self.b_ida, self.b_vuelta, self.b_comprobar, self.combo_escala,
                  self.combo_vel, *self.botones_poses):
            w.setEnabled(not si)
        self.b_detener.setEnabled(si)
        if not si:
            self.barra.setValue(0)

    # ================================================================ comprobacion (en un hilo)
    def comprobar(self):
        if self.ocupado:
            return
        q = self.q_sliders()
        self.registrar("Comprobando como manipulacion.py (unos segundos)…")
        self._ocupar(True)

        def trabajo():
            try:
                if self.criterios is None:
                    self.criterios = nu.Criterios(self.e)
                res = self.criterios.comprobar(q, cancelado=lambda: self.detener)
            except Exception as ex:                   # que el hilo no muera en silencio
                res = [(False, False, f"[ERROR] {type(ex).__name__}: {ex}")]
            self.puente.fin_comprobacion.emit(q, res)
        self.hilo = threading.Thread(target=trabajo, daemon=True)
        self.hilo.start()

    def _fin_comprobacion(self, q, res):
        self._ocupar(False)
        for ok, aviso, texto in res:
            self.registrar(f"  {'NO' if not ok else ('aviso' if aviso else 'bien')}  {texto}")
        malos = sum(not ok for ok, _, _ in res)
        avisos = sum(ok and av for ok, av, _ in res)
        self.registrar("  => " + ("NO vale: la tirada no empezaría (reposo -> observación no pasa)." if malos else
                                  ("Vale para manipulacion.py" + (f", con {avisos} aviso(s) de poses_brazos.py."
                                                                  if avisos else " y cumple los criterios de "
                                                                                 "poses_brazos.py."))))

    # ================================================================ estado periodico
    def _refrescar_colisiones(self):
        h = self.e.revisar()
        if not self.ocupado:
            self.e.resaltar(h)
        lin = nu.lineas(h)
        if lin != self.ultimas:
            self.ultimas = lin
            self.lista_colisiones.clear()
            if not lin:
                it = QListWidgetItem("Sin colisiones ni acercamientos.")
                it.setBackground(COLOR_OK)
                self.lista_colisiones.addItem(it)
            for texto, cuerpos, choque, _ in lin:
                it = QListWidgetItem(texto)
                it.setToolTip(cuerpos)
                it.setBackground(COLOR_CHOQUE if choque else COLOR_CERCA)
                self.lista_colisiones.addItem(it)

    # ================================================================ cierre
    def closeEvent(self, ev):
        self.detener = True
        if self.hilo is not None:
            self.hilo.join(timeout=3.0)
        self.t_vista.stop()
        self.t_estado.stop()
        self.vista.cerrar()
        ev.accept()


def main():
    ap = argparse.ArgumentParser(description="Editor de la pose de observación del reto de manipulación.")
    ap.add_argument("--poses", default=nu.RUTA_POSES, help="archivo JSON de las poses")
    ap.add_argument("--captura", help=argparse.SUPPRESS)              # pruebas: guardar la ventana en un PNG
    ap.add_argument("--salir-tras", type=float, help=argparse.SUPPRESS)
    ap.add_argument("--demo", choices=["animar", "comprobar"], help=argparse.SUPPRESS)
    a = ap.parse_args()
    app = QApplication(sys.argv[:1])
    v = Ventana(os.path.abspath(os.path.expanduser(a.poses)))
    v.show()
    if a.demo:
        it = v.lista.item(0)
        if it is not None:
            v.lista.setCurrentItem(it)
            v._al_elegir(it)
        v.combo_vel.setCurrentIndex(3)
        QTimer.singleShot(300, (lambda: v.animar(True)) if a.demo == "animar" else v.comprobar)
    if a.salir_tras:
        def salir():
            if a.captura:
                v.grab().save(a.captura)
            v.close()
        QTimer.singleShot(int(a.salir_tras * 1000), salir)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
