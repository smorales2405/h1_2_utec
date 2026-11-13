#!/usr/bin/env python3
"""Comprueba, sobre el main.pdf compilado, que nada se salga de las diapositivas ni se pise.

Uso:  python3 verificar_margenes.py [main]     (tras compilar dos veces, desde esta carpeta)

Lee la caja de cada palabra con `pdftotext -bbox` y la de cada imagen del registro `main.layout`
(lo escribe \\Imagen al compilar), y comprueba:

  1. que ninguna palabra ni imagen se salga de la página (con 6 bp de margen);
  2. que nada pise el logo de la cabecera ni el pie de página en las diapositivas de contenido;
  3. que ninguna palabra pise parcialmente una imagen (el texto que queda entero dentro de una
     figura es el de la propia gráfica o una etiqueta puesta encima a propósito);
  4. que no haya palabras solapadas entre sí;
  5. que no haya imágenes solapadas entre sí.

Termina con código 1 si encuentra algún problema.
"""
import re
import subprocess
import sys
from pathlib import Path

AQUI = Path(__file__).resolve().parent
W, H, MARGEN = 960, 540, 6
LOGO = (36, 14, 96, 52)
PIE = (36, 512, 924, 530)
SIN_CABECERA = {1, 23}            # portada y cierre
A_SANGRE = {"portada.jpg"}         # fotos que llegan al borde a propósito
JOB = sys.argv[1] if len(sys.argv) > 1 else "main"


def palabras():
    html = subprocess.run(["pdftotext", "-bbox", str(AQUI / f"{JOB}.pdf"), "-"],
                          capture_output=True, text=True, check=True).stdout
    out = []
    for pg in re.split(r"<page ", html)[1:]:
        ws = re.findall(r'<word xMin="([\d.]+)" yMin="([\d.]+)" xMax="([\d.]+)" '
                        r'yMax="([\d.]+)">([^<]*)</word>', pg)
        out.append([(float(a), float(b), float(c), float(d), t) for a, b, c, d, t in ws])
    return out


def imagenes():
    out = {}
    for linea in (AQUI / f"{JOB}.layout").read_text().splitlines():
        p, x, y, w, h, f = linea.split(";")
        x, y, w, h = map(float, (x, y, w, h))
        out.setdefault(int(p), []).append((x, y, x + w, y + h, Path(f).name))
    return out


def corta(a, b):
    return max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(0.0, min(a[3], b[3]) - max(a[1], b[1]))


def area(a):
    return max(1e-6, (a[2] - a[0]) * (a[3] - a[1]))


def dentro(a, b, tol=0.5):
    return a[0] >= b[0] - tol and a[1] >= b[1] - tol and a[2] <= b[2] + tol and a[3] <= b[3] + tol


def main():
    pals, imgs = palabras(), imagenes()
    problemas = 0
    for i, ws in enumerate(pals, start=1):
        ims = imgs.get(i, [])
        msgs = []
        pagina = (MARGEN, MARGEN, W - MARGEN, H - MARGEN)
        cabecera = i not in SIN_CABECERA
        for im in ims:
            if im[4] not in A_SANGRE and not dentro(im, pagina):
                msgs.append(f"imagen fuera de la página: {im[4]} {tuple(round(v) for v in im[:4])}")
            if cabecera and im[4] != "utec_logo1.png" and corta(im, LOGO) > 1:
                msgs.append(f"imagen pisa el logo: {im[4]}")
            if cabecera and corta(im, PIE) > 1:
                msgs.append(f"imagen pisa el pie de página: {im[4]}")
        for a in range(len(ims)):
            for b in range(a + 1, len(ims)):
                if corta(ims[a], ims[b]) > 1:
                    msgs.append(f"imágenes solapadas: {ims[a][4]} / {ims[b][4]}")
        libres = []
        for w in ws:
            caja = w[:4]
            if not dentro(caja, pagina):
                msgs.append(f"texto fuera de la página: «{w[4]}» {tuple(round(v) for v in caja)}")
            if any(dentro(caja, im) for im in ims):
                continue
            for im in ims:
                if corta(caja, im) > 0.15 * area(caja):
                    msgs.append(f"texto pisa la imagen {im[4]}: «{w[4]}»")
            if cabecera and corta(caja, LOGO) > 1:
                msgs.append(f"texto pisa el logo: «{w[4]}»")
            libres.append(w)
        if cabecera:
            pie = [w for w in libres if w[1] >= PIE[1] - 2]
            cuerpo = [w for w in libres if w[3] < PIE[1] + 4 and w not in pie]
            for w in cuerpo:
                if w[3] > PIE[1] + 1:
                    msgs.append(f"texto baja hasta el pie de página: «{w[4]}» y={w[3]:.0f}")
        for a in range(len(libres)):
            for b in range(a + 1, len(libres)):
                ca, cb = libres[a][:4], libres[b][:4]
                if corta(ca, cb) > 0.25 * min(area(ca), area(cb)):
                    msgs.append(f"palabras solapadas: «{libres[a][4]}» / «{libres[b][4]}»")
        if msgs:
            problemas += len(msgs)
            print(f"── diapositiva {i}")
            for m in dict.fromkeys(msgs):
                print("   ", m)
    print(f"\n{len(pals)} diapositivas · " + ("SIN PROBLEMAS" if not problemas else f"{problemas} PROBLEMAS"))
    sys.exit(1 if problemas else 0)


if __name__ == "__main__":
    main()
