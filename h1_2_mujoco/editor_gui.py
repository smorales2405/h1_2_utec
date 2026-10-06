#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Interfaz gráfica del editor de poses del H1-2 con manos (MuJoCo, sin física).
#
# Una sola ventana: la vista 3D del robot (MuJoCo dibuja fuera de pantalla y Qt
# muestra la imagen; el ratón mueve la cámara como en el visor de MuJoCo) y los
# controles: rutinas, tabla de pasos, postura, gestos, sliders de brazos y manos
# en grados, y la lista de colisiones.
#
# Por debajo es el mismo editor que editor_poses_mujoco_h1_2_manos.py: modelo,
# colisiones, gestos (cerrada en el orden de puno()), pose segura y formato de
# las rutinas. Lo que se ve aquí es lo que hará el selector real.
#
#   ./editor_gui.sh
#   ./editor_gui.sh --poses /otra/carpeta --margen 0.01
# -----------------------------------------------------------------------------

import os
os.environ.setdefault("MUJOCO_GL", "glfw")       # antes de importar mujoco: el render fuera de pantalla

import argparse
import copy
import json
import math
import sys
import threading
from datetime import datetime
from pathlib import Path

import mujoco
from PyQt5.QtCore import Qt, QObject, QSize, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPixmap
from PyQt5.QtWidgets import (QAbstractItemView, QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout,
                             QGroupBox, QHBoxLayout, QHeaderView, QInputDialog, QLabel, QListWidget,
                             QListWidgetItem, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QScrollArea,
                             QSizePolicy, QSlider, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

import editor_poses_mujoco_h1_2_manos as ed
import modelo_h1_2_manos as mod

gm, CLAVES, Q_MAX = ed.gm, ed.CLAVES, ed.Q_MAX
POSE_PROTEGIDA = "0_pose_segura.json"           # la postura por defecto del robot: ni se lista ni se sobrescribe
DUR_PASO_NUEVO = 2.0                            # s
DUR_GESTO = ed.DUR_GESTO_DEFECTO
OFFSCREEN_MAX = (2560, 1600)                    # tamaño máximo de la vista
BRAZO = ("hombro pitch", "hombro roll", "hombro yaw", "codo", "muñeca roll", "muñeca pitch", "muñeca yaw")
GESTOS = (("abierta", "Abierta"), ("cerrada", "Cerrada"), ("pulgar_arriba", "Pulgar arriba"), ("senalar", "Señalar"))
COLOR_CHOQUE, COLOR_CERCA = QColor(255, 205, 205), QColor(255, 230, 190)
LINEAS_ABAJO = 5                                # líneas visibles en «Colisiones ahora» y «Registro»


def texto_colision(peor):
    """Resumen de una celda: el peor par de lo encontrado en un paso."""
    if not peor:
        return "—", None
    (a, b), dist = min(peor.items(), key=lambda kv: kv[1])
    partes = f"{mod.grupo(a)} / {mod.grupo(b)}" if mod.grupo(a) != mod.grupo(b) else mod.grupo(a)
    if dist < 0:
        return f"{dist * 1000:+.0f} mm {partes}", COLOR_CHOQUE
    return f"cerca {dist * 1000:.0f} mm {partes}", COLOR_CERCA


def lineas_colisiones(hallazgos):
    """[(texto corto, cuerpos, es_choque)]: una por pareja de partes, la más grave. Los nombres de los cuerpos
    van aparte (ayuda emergente) para que cada línea quepa entera en la lista."""
    peor = {}
    for a, b, dist in hallazgos:
        clave = tuple(sorted((mod.grupo(a), mod.grupo(b))))
        if clave not in peor or dist < peor[clave][2]:
            peor[clave] = (a, b, dist)
    salida = []
    for (ga, gb), (a, b, dist) in sorted(peor.items(), key=lambda kv: kv[1][2]):
        partes = ga if ga == gb else f"{ga} <-> {gb}"
        salida.append((f"{'[COLISIÓN]' if dist < 0 else '[CERCA]'} {partes}   {dist * 1000:+.0f} mm",
                       f"{a} / {b}", dist < 0))
    return salida


class Puente(QObject):
    """Señales del hilo que anima hacia la ventana (Qt las entrega en el hilo de la interfaz)."""
    paso_inicio = pyqtSignal(int, str)
    paso_fin = pyqtSignal(int, object, str)
    fin_trabajo = pyqtSignal(str)


class FilaJunta(QWidget):
    """Una junta: nombre, slider y casilla, en grados. `al_cambiar(rad)` cuando la mueve el usuario."""

    def __init__(self, nombre, lo, hi, al_cambiar):
        super().__init__()
        self.al_cambiar = al_cambiar
        self.etiqueta = QLabel(nombre)
        self.etiqueta.setMinimumWidth(92)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(round(math.degrees(lo) * 10), round(math.degrees(hi) * 10))
        self.casilla = QDoubleSpinBox()
        self.casilla.setRange(math.degrees(lo), math.degrees(hi))
        self.casilla.setDecimals(1)
        self.casilla.setSuffix("°")
        self.casilla.setKeyboardTracking(False)
        self.casilla.setFixedWidth(78)
        self.setToolTip(f"{nombre}: de {math.degrees(lo):+.1f}° a {math.degrees(hi):+.1f}°")
        fila = QHBoxLayout(self)
        fila.setContentsMargins(0, 0, 0, 0)
        fila.addWidget(self.etiqueta)
        fila.addWidget(self.slider, 1)
        fila.addWidget(self.casilla)
        self.slider.valueChanged.connect(self._desde_slider)
        self.casilla.valueChanged.connect(self._desde_casilla)

    def _desde_slider(self, v):
        self.casilla.blockSignals(True)
        self.casilla.setValue(v / 10.0)
        self.casilla.blockSignals(False)
        self.al_cambiar(math.radians(v / 10.0))

    def _desde_casilla(self, v):
        self.slider.blockSignals(True)
        self.slider.setValue(round(v * 10))
        self.slider.blockSignals(False)
        self.al_cambiar(math.radians(v))

    def mostrar(self, rad):
        """Refleja la postura del modelo sin avisar, salvo si el usuario la está tocando."""
        if self.slider.isSliderDown() or self.casilla.hasFocus():
            return
        grados = math.degrees(rad)
        if abs(grados - self.casilla.value()) < 0.05:
            return
        for w, v in ((self.slider, round(grados * 10)), (self.casilla, grados)):
            w.blockSignals(True)
            w.setValue(v)
            w.blockSignals(False)


class Vista(QLabel):
    """La escena de MuJoCo. Ratón como en su visor: izquierdo gira, derecho desplaza (con Mayúsculas, en el
    otro plano), central o rueda acercan; doble clic vuelve a la vista inicial."""

    def __init__(self, editor):
        super().__init__()
        self.e = editor
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
        self.cam.lookat[:] = [0.10, 0.0, 1.45]
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 2.3, 155.0, -14.0

    def renderizar(self):
        w = min(max(self.width(), 16), OFFSCREEN_MAX[0])
        h = min(max(self.height(), 16), OFFSCREEN_MAX[1])
        if self.tam != (w, h):
            if self.renderer is not None:
                self.renderer.close()
            self.renderer, self.tam = mujoco.Renderer(self.e.m, h, w), (w, h)
        mujoco.mj_forward(self.e.m, self.e.d)
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


class VentanaEditor(QMainWindow):
    def __init__(self, poses_dir: Path, margen: float):
        super().__init__()
        self.poses_dir = poses_dir
        self.e = ed.Editor(poses_dir, margen)
        self.e.m.vis.global_.offwidth, self.e.m.vis.global_.offheight = OFFSCREEN_MAX
        self.ruta, self.original, self.pasos = None, None, []
        self.modificada = False
        self.ocupado = False
        self.hilo = None
        self.ultimas_colisiones = None
        self.puente = Puente()
        self.puente.paso_inicio.connect(self._paso_inicio)
        self.puente.paso_fin.connect(self._paso_fin)
        self.puente.fin_trabajo.connect(self._fin_trabajo)
        self._construir()
        self.llenar_rutinas()
        self.cargar_seleccion()
        self.temporizador_vista = QTimer(self, timeout=self.vista.renderizar, interval=33)
        self.temporizador_vista.start()
        self.temporizador_estado = QTimer(self, timeout=self._refrescar_estado, interval=100)
        self.temporizador_estado.start()
        self.registrar("Empieza en la pose segura." if self.e.segura else
                       f"No hay {POSE_PROTEGIDA} en {poses_dir}: empieza en 0.")
        self.statusBar().showMessage("Vista: botón izquierdo gira, derecho desplaza, rueda acerca; "
                                     "doble clic, vista inicial.")

    # ================================================================ construcción
    def _construir(self):
        self.vista = Vista(self.e)
        self.lista_colisiones = QListWidget()
        self.registro = QPlainTextEdit(readOnly=True)
        self.registro.setMaximumBlockCount(500)
        # alto exacto de LINEAS_ABAJO líneas: al desplazarse, nunca queda una línea cortada a medias
        self.registro.setFixedHeight(self.registro.fontMetrics().lineSpacing() * LINEAS_ABAJO
                                     + 2 * self.registro.frameWidth()
                                     + 2 * round(self.registro.document().documentMargin()))
        self.lista_colisiones.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.lista_colisiones.addItem("Sin colisiones.")
        self.lista_colisiones.setFixedHeight(self.lista_colisiones.sizeHintForRow(0) * LINEAS_ABAJO
                                             + 2 * self.lista_colisiones.frameWidth())
        abajo = QSplitter(Qt.Horizontal)
        abajo.addWidget(self._caja("Colisiones ahora", self.lista_colisiones))
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
        col.addWidget(self._grupo_rutina(), 1)
        col.addWidget(self._grupo_postura())
        col.addWidget(self._grupo_gestos())
        col.addWidget(self._grupo_sliders())
        desplazable = QScrollArea()
        desplazable.setWidget(panel)
        desplazable.setWidgetResizable(True)
        desplazable.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # nunca más estrecho que su contenido: así ningún botón ni casilla queda cortado
        ancho = panel.minimumSizeHint().width() + desplazable.verticalScrollBar().sizeHint().width() + 8
        desplazable.setMinimumWidth(ancho)

        principal = QSplitter(Qt.Horizontal)
        principal.addWidget(izquierda)
        principal.addWidget(desplazable)
        principal.setStretchFactor(0, 1)
        principal.setStretchFactor(1, 0)
        principal.setCollapsible(0, False)
        principal.setCollapsible(1, False)
        self.setCentralWidget(principal)
        self.resize(max(1500, ancho + 1000), 1000)
        principal.setSizes([self.width() - ancho, ancho])

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

    def _grupo_rutina(self):
        g = QGroupBox("Rutina")
        v = QVBoxLayout(g)
        fila = QHBoxLayout()
        self.combo = QComboBox()
        self.combo.currentIndexChanged.connect(self._al_elegir_rutina)
        self.b_ejecutar = self._boton("Ejecutar", self.ejecutar,
                                      "Como en el robot: desde la pose segura y de vuelta a ella al final")
        self.b_detener = self._boton("Detener", self.detener)
        self.b_guardar = self._boton("Guardar", self.guardar)
        fila.addWidget(self.combo, 1)
        for b in (self.b_ejecutar, self.b_detener, self.b_guardar):
            fila.addWidget(b)
        v.addLayout(fila)
        self.tabla = QTableWidget(0, 6)
        self.tabla.setHorizontalHeaderLabels(["#", "Nombre", "Duración (s)", "Mano izq", "Mano der", "Colisión"])
        self.tabla.verticalHeader().setVisible(False)
        self.tabla.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tabla.setSelectionMode(QAbstractItemView.SingleSelection)
        cab = self.tabla.horizontalHeader()
        cab.setSectionResizeMode(QHeaderView.ResizeToContents)
        cab.setStretchLastSection(True)
        self.tabla.setTextElideMode(Qt.ElideNone)
        self.tabla.setWordWrap(False)
        self.tabla.setMinimumHeight(150)
        self.tabla.itemSelectionChanged.connect(self._al_seleccionar_paso)
        self.tabla.itemChanged.connect(self._al_editar_celda)
        v.addWidget(self.tabla)
        fila = QHBoxLayout()
        self.botones_pasos = [
            self._boton("Animar paso", self.animar_paso, "Desde el paso anterior (el 1, desde la pose segura)"),
            self._boton("Sobrescribir paso", self.sobrescribir_paso, "El paso seleccionado = la postura actual"),
            self._boton("Añadir paso", self.anadir_paso, "La postura actual, detrás del paso seleccionado"),
            self._boton("Borrar paso", self.borrar_paso),
            self._boton("Subir", lambda: self.mover_paso(-1)),
            self._boton("Bajar", lambda: self.mover_paso(+1)),
        ]
        for b in self.botones_pasos:
            fila.addWidget(b)
        v.addLayout(fila)
        return g

    def _grupo_postura(self):
        g = QGroupBox("Postura")
        fila = QHBoxLayout(g)
        self.botones_postura = [
            self._boton("Zero", self.postura_cero, "Torso y brazos a 0, manos abiertas"),
            self._boton("Pose segura", self.postura_segura, "La postura por defecto, manos abiertas"),
            self._boton("Reset", self.reset, "Pose segura y descartar los cambios sin guardar de la rutina"),
            self._boton("Espejo izq → der", lambda: self.espejo("izq")),
            self._boton("Espejo der → izq", lambda: self.espejo("der")),
        ]
        self.espejo_manos = QCheckBox("con manos")
        for b in self.botones_postura[:3]:
            fila.addWidget(b)
        fila.addSpacing(16)
        for b in self.botones_postura[3:]:
            fila.addWidget(b)
        fila.addWidget(self.espejo_manos)
        return g

    def _grupo_gestos(self):
        g = QGroupBox("Gestos de mano")
        fila = QHBoxLayout(g)
        self.combo_lado = QComboBox()
        for texto, lado in (("Izquierda", "izq"), ("Derecha", "der"), ("Ambas", "ambas")):
            self.combo_lado.addItem(texto, lado)
        fila.addWidget(self.combo_lado)
        self.botones_gestos = [self._boton(texto, lambda _, n=nombre: self.gesto(n)) for nombre, texto in GESTOS]
        for b in self.botones_gestos:
            fila.addWidget(b)
        return g

    def _grupo_sliders(self):
        caja = QWidget()
        fila = QHBoxLayout(caja)
        fila.setContentsMargins(0, 0, 0, 0)
        self.filas = []                                     # (FilaJunta, ("brazo", j) | ("mano", lado, clave))
        for titulo, lado, base in (("Derecha", "der", 20), ("Izquierda", "izq", 13)):
            g = QGroupBox(titulo)
            v = QVBoxLayout(g)
            v.setSpacing(1)
            v.setContentsMargins(6, 4, 6, 4)
            v.addWidget(self._titulo("Brazo"))
            for k, nombre in enumerate(BRAZO):
                j = base + k
                lo, hi = self.e.m.jnt_range[self.e.m.joint(ed.NOMBRE_MJCF[j]).id]
                f = FilaJunta(nombre, lo, hi, lambda rad, j=j: self._mover_brazo(j, rad))
                v.addWidget(f)
                self.filas.append((f, ("brazo", j)))
            v.addWidget(self._titulo("Mano"))
            for clave, qmax in zip(CLAVES, Q_MAX):
                f = FilaJunta(gm.NOMBRE_DOF[clave], 0.0, qmax,
                              lambda rad, lado=lado, clave=clave: self._mover_mano(lado, clave, rad))
                v.addWidget(f)
                self.filas.append((f, ("mano", lado, clave)))
            fila.addWidget(g)
        self.caja_sliders = caja
        return caja

    @staticmethod
    def _titulo(texto):
        t = QLabel(texto)
        fuente = QFont()
        fuente.setBold(True)
        t.setFont(fuente)
        return t

    # ================================================================ utilidades
    def registrar(self, texto):
        self.registro.appendPlainText(texto)

    def confirmar(self, texto):
        return QMessageBox.question(self, "Editor de poses H1-2", texto) == QMessageBox.Yes

    def pedir_nombre(self):
        nombre, ok = QInputDialog.getText(self, "Guardar rutina nueva", "Nombre de la rutina:")
        return nombre.strip() if ok else ""

    def avisar(self, texto):
        self.registrar(f"[AVISO] {texto}")
        self.statusBar().showMessage(texto, 6000)

    def _marcar(self, modificada=True):
        self.modificada = modificada
        nombre = self.ruta.name if self.ruta else "rutina nueva"
        self.setWindowTitle(f"Editor de poses H1-2 — {nombre}{' *' if modificada else ''}")

    def _fila_actual(self):
        filas = self.tabla.selectionModel().selectedRows()
        return filas[0].row() if filas else None

    # ================================================================ rutinas
    def llenar_rutinas(self, seleccionar=None):
        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItem("— Nueva rutina —", None)
        archivos = [f for f in self.poses_dir.glob("*.json") if f.name != POSE_PROTEGIDA]
        numero = lambda f: (int(f.stem.split("_")[0]) if f.stem.split("_")[0].isdigit() else 9999, f.name)
        for f in sorted(archivos, key=numero):
            self.combo.addItem(f.stem, str(f))
        i = self.combo.findData(str(seleccionar)) if seleccionar else 0
        self.combo.setCurrentIndex(max(i, 0))
        self.indice_combo = self.combo.currentIndex()
        self.combo.blockSignals(False)

    def _al_elegir_rutina(self, i):
        if self.modificada and not self.confirmar("Hay cambios sin guardar en la rutina. ¿Descartarlos?"):
            self.combo.blockSignals(True)
            self.combo.setCurrentIndex(self.indice_combo)
            self.combo.blockSignals(False)
            return
        self.indice_combo = i
        self.cargar_seleccion()

    def cargar_seleccion(self):
        dato = self.combo.currentData()
        if dato is None:
            self.ruta, self.original, self.pasos = None, None, []
            self.registrar("Rutina nueva: coloca el robot y pulsa «Añadir paso».")
        else:
            self.ruta = Path(dato)
            _, self.pasos = ed.leer_rutina(str(self.ruta), self.poses_dir)
            self.original = json.loads(self.ruta.read_text(encoding="utf-8"))
            self.registrar(f"Cargada {self.ruta.name}: {len(self.pasos)} pasos.")
        self._llenar_tabla()
        self._marcar(False)

    def _llenar_tabla(self, seleccionar=None):
        self.tabla.blockSignals(True)
        self.tabla.setRowCount(len(self.pasos))
        for i, p in enumerate(self.pasos):
            self._llenar_fila(i, p)
        self.tabla.blockSignals(False)
        if seleccionar is not None and 0 <= seleccionar < len(self.pasos):
            self.tabla.selectRow(seleccionar)

    def _llenar_fila(self, i, p, colision=None):
        manos = p.get("manos") or {}
        textos = [str(i + 1), p["nombre"], f"{p['duracion']:.2f}",
                  ed.texto_mano(manos["izq"]) if "izq" in manos else "sin cambio",
                  ed.texto_mano(manos["der"]) if "der" in manos else "sin cambio"]
        bloqueado = self.tabla.signalsBlocked()
        self.tabla.blockSignals(True)
        for c, texto in enumerate(textos):
            item = QTableWidgetItem(texto)
            if c not in (1, 2):
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self.tabla.setItem(i, c, item)
        texto, color = colision if colision else ("", None)
        item = QTableWidgetItem(texto)
        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        if color:
            item.setBackground(color)
        self.tabla.setItem(i, 5, item)
        self.tabla.blockSignals(bloqueado)

    def _al_editar_celda(self, item):
        i, c = item.row(), item.column()
        if c == 1:
            self.pasos[i]["nombre"] = item.text().strip() or f"Paso {i + 1}"
        elif c == 2:
            try:
                dur = float(item.text().replace(",", "."))
                if dur <= 0:
                    raise ValueError
                self.pasos[i]["duracion"] = dur
            except ValueError:
                self.avisar("La duración tiene que ser un número de segundos mayor que 0.")
        self._llenar_fila(i, self.pasos[i])
        self._marcar()

    def _al_seleccionar_paso(self):
        i = self._fila_actual()
        if i is None or self.ocupado:
            return
        p = self.pasos[i]
        self.e.poner(p["posiciones"], p.get("manos"))

    def guardar(self):
        if not self.pasos:
            self.avisar("La rutina no tiene pasos: «Añadir paso» primero.")
            return
        if self.ruta is None:
            nombre = self.pedir_nombre()
            if not nombre:
                return
            base = ed.nombre_base(nombre)
            ruta = self.poses_dir / f"{ed.siguiente_numero(self.poses_dir)}_{base}.json"
            datos = ed.datos_rutina_nueva(base, copy.deepcopy(self.pasos), "editor_gui.py (interfaz gráfica)")
        else:
            if self.ruta.name == POSE_PROTEGIDA:
                self.avisar(f"{POSE_PROTEGIDA} es la postura por defecto del robot: no se modifica.")
                return
            if not self.confirmar(f"¿Sobrescribir {self.ruta.name}?"):
                return
            ruta = self.ruta
            datos = dict(self.original)
            datos["pasos"] = copy.deepcopy(self.pasos)
            datos["numero_pasos"] = len(self.pasos)
            datos["fecha_modificacion"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.poses_dir.mkdir(parents=True, exist_ok=True)
        ruta.write_text(json.dumps(datos, indent=2, ensure_ascii=False), encoding="utf-8")
        self.ruta, self.original = ruta, datos
        self.llenar_rutinas(seleccionar=ruta)
        self._marcar(False)
        self.registrar(f"Guardada {ruta.name}. En el robot: volver a copiar la carpeta (rsync, ver README).")

    def reset(self):
        if self.modificada and not self.confirmar("Reset descarta los cambios sin guardar de la rutina. ¿Seguir?"):
            return
        self.postura_segura()
        if self.ruta is not None:
            _, self.pasos = ed.leer_rutina(str(self.ruta), self.poses_dir)
        else:
            self.pasos = []
        self._llenar_tabla()
        self._marcar(False)
        self.registrar("Reset: pose segura, " + (f"{self.ruta.name} recargada del archivo." if self.ruta
                                                  else "rutina nueva vacía."))

    # ================================================================ pasos
    def sobrescribir_paso(self):
        i = self._fila_actual()
        if i is None:
            self.avisar("Selecciona en la tabla el paso a sobrescribir.")
            return
        viejo = self.pasos[i]
        self.pasos[i] = self.e.paso_desde_visor(viejo["nombre"], viejo["duracion"])
        self._llenar_fila(i, self.pasos[i])
        self._marcar()
        self.registrar(f"Paso {i + 1} «{viejo['nombre']}» = postura actual.")

    def anadir_paso(self):
        i = self._fila_actual()
        destino = len(self.pasos) if i is None else i + 1
        self.pasos.insert(destino, self.e.paso_desde_visor(f"Paso {len(self.pasos) + 1}", DUR_PASO_NUEVO))
        self._llenar_tabla()
        self.tabla.blockSignals(True)
        self.tabla.selectRow(destino)
        self.tabla.blockSignals(False)
        self._marcar()
        lineas = mod.informe(self.e.revisar())
        self.registrar(f"Añadido el paso {destino + 1}." + (" Tiene colisiones: " + "; ".join(lineas) if lineas else ""))

    def borrar_paso(self):
        i = self._fila_actual()
        if i is None:
            self.avisar("Selecciona en la tabla el paso a borrar.")
            return
        borrado = self.pasos.pop(i)
        self._llenar_tabla()
        self._marcar()
        self.registrar(f"Borrado el paso {i + 1} «{borrado['nombre']}».")

    def mover_paso(self, delta):
        i = self._fila_actual()
        if i is None or not 0 <= i + delta < len(self.pasos):
            return
        self.pasos[i], self.pasos[i + delta] = self.pasos[i + delta], self.pasos[i]
        self._llenar_tabla()
        self.tabla.blockSignals(True)
        self.tabla.selectRow(i + delta)
        self.tabla.blockSignals(False)
        self._marcar()

    # ================================================================ postura y gestos
    def postura_cero(self):
        self.e.poner({j: 0.0 for j in ed.JUNTAS}, {lado: gm.gesto("abierta") for lado in gm.LADOS})

    def postura_segura(self):
        if not self.e.segura:
            self.avisar(f"No hay {POSE_PROTEGIDA} en {self.poses_dir}.")
            return
        self.e.poner(self.e.segura["posiciones"], self.e.segura["manos"])

    def espejo(self, lado):
        self.e.espejo(lado, self.espejo_manos.isChecked())

    def _mover_brazo(self, j, rad):
        if not self.ocupado:
            self.e.poner({j: rad})

    def _mover_mano(self, lado, clave, rad):
        if not self.ocupado:
            q = self.e.leer_manos()[lado]
            q[clave] = rad
            self.e.poner({}, {lado: q})

    def gesto(self, nombre):
        lado = self.combo_lado.currentData()
        lados = list(gm.LADOS) if lado == "ambas" else [lado]
        objetivo = gm.gesto(nombre)

        def trabajo():
            brazos, manos = self.e.leer(), self.e.leer_manos()
            peor = self.e.animar(brazos, brazos, manos, {l: objetivo for l in lados}, DUR_GESTO)
            return f"Mano {self.combo_lado.currentText().lower()}: {nombre}. Camino: {self.e.resumen(peor).strip()}"
        self.lanzar(trabajo)

    # ================================================================ animaciones (en un hilo)
    def lanzar(self, trabajo):
        if self.ocupado:
            return
        self.ocupado = True
        self.e.detener = False
        self._habilitar(False)

        def correr():
            try:
                mensaje = trabajo() or ""
            except Exception as ex:                       # que el hilo no muera en silencio
                mensaje = f"[ERROR] {type(ex).__name__}: {ex}"
            self.puente.fin_trabajo.emit(mensaje)
        self.hilo = threading.Thread(target=correr, daemon=True)
        self.hilo.start()

    def _recorrer(self, lista, inicio):
        """Anima la lista [(fila de la tabla o -1, paso)] desde `inicio` (paso o None = postura actual)."""
        e = self.e
        if inicio:
            e.poner(inicio["posiciones"], inicio.get("manos"))
        brazos, manos = e.leer(), e.leer_manos()
        for fila, p in lista:
            if e.detener or e.terminar:
                return "Detenido."
            self.puente.paso_inicio.emit(fila, p["nombre"])
            fin_b = {int(k): v for k, v in p["posiciones"].items()}
            fin_m = {lado: gm.normalizar(q, manos[lado]) for lado, q in (p.get("manos") or {}).items()}
            peor = e.animar(brazos, fin_b, manos, fin_m, p["duracion"])
            self.puente.paso_fin.emit(fila, peor, p["nombre"])
            brazos = {j: fin_b.get(j, brazos[j]) for j in ed.JUNTAS}
            manos = {**manos, **fin_m}
        return "Detenido." if e.detener else ""

    def ejecutar(self):
        if not self.pasos:
            self.avisar("La rutina no tiene pasos.")
            return
        lista = list(enumerate(copy.deepcopy(self.pasos)))
        segura = self.e.segura
        if segura and not ed.es_pose_segura(self.pasos[-1], segura):
            lista.append((-1, {**segura, "nombre": "pose segura (automática)"}))
        self.registrar(f"Ejecutando {self.ruta.name if self.ruta else 'la rutina nueva'}"
                       f"{' desde la pose segura' if segura else ''}:")
        self.lanzar(lambda: self._recorrer(lista, segura) or "Fin de la rutina"
                    + (", en la pose segura." if segura else "."))

    def animar_paso(self):
        i = self._fila_actual()
        if i is None:
            self.avisar("Selecciona en la tabla el paso a animar.")
            return
        inicio = self.pasos[i - 1] if i > 0 else self.e.segura
        self.lanzar(lambda: self._recorrer([(i, copy.deepcopy(self.pasos[i]))], inicio))

    def detener(self):
        self.e.detener = True

    def _paso_inicio(self, fila, nombre):
        if fila >= 0:
            self.tabla.blockSignals(True)
            self.tabla.selectRow(fila)
            self.tabla.blockSignals(False)

    def _paso_fin(self, fila, peor, nombre):
        celda = texto_colision(peor)
        if fila >= 0:
            self._llenar_fila(fila, self.pasos[fila], celda)
        etiqueta = f"{fila + 1:02d}. {nombre}" if fila >= 0 else nombre
        self.registrar(f"  -> {etiqueta}: {self.e.resumen(peor).strip()}")

    def _fin_trabajo(self, mensaje):
        self.ocupado = False
        self.e.detener = False
        self._habilitar(True)
        if mensaje:
            self.registrar(mensaje)

    def _habilitar(self, si):
        for w in (self.combo, self.b_ejecutar, self.b_guardar, self.tabla, self.caja_sliders, self.combo_lado,
                  *self.botones_pasos, *self.botones_postura, *self.botones_gestos):
            w.setEnabled(si)
        self.b_detener.setEnabled(not si)

    # ================================================================ estado periódico
    def _refrescar_estado(self):
        hallazgos = self.e.revisar()
        if not self.ocupado:
            self.e.resaltar(hallazgos)
        lineas = lineas_colisiones(hallazgos)
        if lineas != self.ultimas_colisiones:
            self.ultimas_colisiones = lineas
            self.lista_colisiones.clear()
            if not lineas:
                self.lista_colisiones.addItem("Sin colisiones.")
            for texto, cuerpos, choque in lineas:
                item = QListWidgetItem(texto)
                item.setToolTip(cuerpos)
                item.setBackground(COLOR_CHOQUE if choque else COLOR_CERCA)
                self.lista_colisiones.addItem(item)
        brazos, manos = self.e.leer(), self.e.leer_manos()
        for fila, que in self.filas:
            fila.mostrar(brazos[que[1]] if que[0] == "brazo" else manos[que[1]][que[2]])

    # ================================================================ cierre
    def closeEvent(self, ev):
        if self.modificada and not self.confirmar("Hay cambios sin guardar en la rutina. ¿Salir igualmente?"):
            ev.ignore()
            return
        self.e.detener = self.e.terminar = True
        if self.hilo is not None:
            self.hilo.join(timeout=3.0)
        self.temporizador_vista.stop()
        self.temporizador_estado.stop()
        self.vista.cerrar()
        ev.accept()


def main():
    ap = argparse.ArgumentParser(description="Interfaz gráfica del editor de poses del H1-2 con manos.")
    ap.add_argument("--poses", default=str(ed.POSES_DEFECTO), help="carpeta de las rutinas")
    ap.add_argument("--margen", type=float, default=mod.MARGEN_DEFECTO,
                    help="m: avisar también de lo que quede a menos de esto sin tocarse (0 = solo choques)")
    ap.add_argument("--captura", help=argparse.SUPPRESS)          # pruebas: guardar la ventana en un PNG
    ap.add_argument("--salir-tras", type=float, help=argparse.SUPPRESS)   # pruebas: cerrar a los N s
    a = ap.parse_args()
    app = QApplication(sys.argv[:1])
    ventana = VentanaEditor(Path(a.poses).expanduser().resolve(), a.margen)
    ventana.show()
    if a.salir_tras:
        def salir():
            if a.captura:
                ventana.grab().save(a.captura)
            ventana.modificada = False
            ventana.close()
        QTimer.singleShot(int(a.salir_tras * 1000), salir)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
