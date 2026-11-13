#!/usr/bin/env python3
"""Fotos de la presentación: fotogramas de los vídeos de ../videos/ (con ffmpeg) y de la cámara IR de las
tiradas (../datos/), recortados a la proporción de su hueco y guardados en imagenes/fotos/ a 3 px por bp.

    python3 graficas/recortar_fotos.py            # todas (desde presentacion/)
    python3 graficas/recortar_fotos.py portada    # solo algunas

RECORTES: nombre → (fuente, instante en s, centro del recorte (x, y) y alto que se usa, en fracción del
fotograma, y hueco en bp (ancho, alto)). La fuente es «video:NivelN» o «ir:<carpeta de la tirada>»; en las
tiradas el instante se cuenta desde la primera orden (como en los registros).
"""

import csv
import os
import subprocess
import sys
import tempfile

import cv2

AQUI = os.path.dirname(os.path.abspath(__file__))
PRES = os.path.dirname(AQUI)
SEGUIDOR = os.path.dirname(PRES)
SALIDA = os.path.join(PRES, "imagenes", "fotos")
PX_POR_BP = 3

RECORTES = {
    # portada: el H1-2 en la recta del nivel 1, con la barra delante
    "portada": ("video:Nivel1", 2.5, (0.50, 0.49, 0.775), (392, 540)),
    # nivel 1
    "n1_lejos": ("video:Nivel1", 1.0, (0.50, 0.50, 1.00), (140, 249)),
    "n1_barra": ("video:Nivel1", 9.5, (0.50, 0.50, 1.00), (140, 249)),
    # nivel 2
    "n2_curva": ("video:Nivel2", 12.0, (0.55, 0.55, 0.90), (290, 182)),
    "n2_barra": ("video:Nivel2", 26.4, (0.55, 0.55, 0.90), (290, 182)),
    # nivel 3
    "n3_s": ("video:Nivel3", 6.0, (0.45, 0.55, 0.90), (290, 163)),
    "n3_segundo": ("video:Nivel3", 27.6, (0.40, 0.55, 0.90), (290, 163)),
    "n3_final": ("video:Nivel3", 42.0, (0.60, 0.55, 0.90), (250, 141)),
    # nivel 4: el giro en la esquina
    "n4_a": ("video:Nivel4", 5.4, (0.40, 0.50, 0.80), (212, 119)),
    "n4_b": ("video:Nivel4", 8.9, (0.55, 0.50, 0.80), (212, 119)),
    "n4_c": ("video:Nivel4", 12.5, (0.45, 0.50, 0.80), (212, 119)),
    "n4_d": ("video:Nivel4", 21.4, (0.55, 0.50, 0.80), (212, 119)),
    # cierre
    "g1": ("video:Nivel1", 1.0, (0.50, 0.29, 0.36), (207, 130)),
    "g2": ("video:Nivel2", 18.0, (0.62, 0.45, 0.80), (207, 130)),
    "g3": ("video:Nivel3", 16.5, (0.55, 0.45, 0.85), (207, 130)),
    "g4": ("video:Nivel4", 21.4, (0.55, 0.48, 0.85), (207, 130)),
    # cámara IR (640 × 480)
    "ir_n3_segundo": ("ir:tirada_20261107_104116_nivel3_escala1", 22.0, (0.5, 0.5, 1.0), (200, 150)),
    "ir_luz": ("ir:tirada_20261110_042852_nivel3_escala1", 34.0, (0.5, 0.5, 1.0), (200, 150)),
    "ir_oscuro": ("ir:tirada_20261110_044157_nivel3_escala1", 34.0, (0.5, 0.5, 1.0), (200, 150)),
    "ir_esquina": ("ir:tirada_20261110_053202_nivel4_escala0.5", 8.5, (0.5, 0.5, 1.0), (176, 132)),
    "ir_tras_giro": ("ir:tirada_20261110_053202_nivel4_escala0.5", 17.4, (0.5, 0.5, 1.0), (176, 132)),
}


def fotograma_video(nombre, t):
    ruta = os.path.join(SEGUIDOR, "videos", nombre + "_implementacion.MOV")
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "f.png")
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", str(t), "-i", ruta, "-frames:v", "1", out], check=True)
        return cv2.imread(out)


def fotograma_ir(carpeta, t):
    d = os.path.join(SEGUIDOR, "datos", carpeta)
    with open(os.path.join(d, "ordenes.csv")) as f:
        t0 = float(next(csv.DictReader(f))["t"])
    with open(os.path.join(d, "fotogramas.csv")) as f:
        filas = [r for r in csv.DictReader(f) if r["ir"]]
    r = min(filas, key=lambda r: abs(float(r["t_rx"]) - t0 - t))
    return cv2.imread(os.path.join(d, r["ir"]))


def recortar(img, cx, cy, alto_rel, w_bp, h_bp):
    H, W = img.shape[:2]
    h = min(H, alto_rel * H)
    w = h * w_bp / h_bp
    if w > W:
        w, h = W, W * h_bp / w_bp
    x0 = int(round(min(max(cx * W - w / 2, 0), W - w)))
    y0 = int(round(min(max(cy * H - h / 2, 0), H - h)))
    rec = img[y0:y0 + int(round(h)), x0:x0 + int(round(w))]
    return cv2.resize(rec, (w_bp * PX_POR_BP, h_bp * PX_POR_BP), interpolation=cv2.INTER_AREA)


def main():
    os.makedirs(SALIDA, exist_ok=True)
    pedidas = sys.argv[1:]
    for nombre, (fuente, t, (cx, cy, hr), (w, h)) in RECORTES.items():
        if pedidas and nombre not in pedidas:
            continue
        tipo, cual = fuente.split(":", 1)
        img = fotograma_video(cual, t) if tipo == "video" else fotograma_ir(cual, t)
        cv2.imwrite(os.path.join(SALIDA, nombre + ".jpg"), recortar(img, cx, cy, hr, w, h),
                    [cv2.IMWRITE_JPEG_QUALITY, 88])
        print("  ", nombre)


if __name__ == "__main__":
    main()
