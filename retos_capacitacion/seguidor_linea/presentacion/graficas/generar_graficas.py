#!/usr/bin/env python3
"""Gráficas de la presentación, a partir de los registros de ../datos/ (tiradas, escalones) y del
simulador del seguidor. Cada una se exporta en PDF al tamaño en bp de su hueco en la diapositiva,
así que no se deforma ni cambia el tamaño de letra.

    python3 graficas/generar_graficas.py                 # todas (desde presentacion/)
    python3 graficas/generar_graficas.py fig_nivel4_giro # solo algunas
"""

import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
PRES = os.path.dirname(AQUI)
SEGUIDOR = os.path.dirname(PRES)
DATOS = os.path.join(SEGUIDOR, "datos")
sys.path.insert(0, SEGUIDOR)

for f in ("Carlito-Regular.ttf", "Carlito-Bold.ttf"):
    font_manager.fontManager.addfont(os.path.join(PRES, "fuentes", f))

GRIS, CIAN, GRIS_CLARO, GRIS_MEDIO = "#4E4B4C", "#00E1FF", "#E4E4E4", "#8A8788"
CIAN_LINEA = "#00B8D9"      # el cian UTEC sobre blanco, un punto más oscuro para trazos finos
plt.rcParams.update({
    "font.family": "Carlito", "font.size": 12, "axes.labelsize": 12, "axes.titlesize": 13,
    "xtick.labelsize": 11, "ytick.labelsize": 11, "legend.fontsize": 11,
    "axes.edgecolor": GRIS, "axes.labelcolor": "black", "xtick.color": GRIS, "ytick.color": GRIS,
    "axes.spines.top": False, "axes.spines.right": False, "axes.linewidth": 0.8,
    "legend.frameon": False, "pdf.fonttype": 42,
})


def figura(ancho_bp, alto_bp):
    return plt.figure(figsize=(ancho_bp / 72.0, alto_bp / 72.0))


def guardar(fig, nombre):
    fig.savefig(os.path.join(AQUI, nombre + ".pdf"))
    plt.close(fig)
    print("  ", nombre)


def leer_csv(ruta):
    with open(ruta) as f:
        return list(csv.DictReader(f))


def tirada(nombre):
    return os.path.join(DATOS, "tirada_" + nombre)


# ── 09 · escalón de vyaw andando ─────────────────────────────────────────
def fig_escalon():
    filas = leer_csv(os.path.join(DATOS, "escalon_20261003_081312_vx0.2_a0.3", "lowstate.csv"))
    t = np.array([float(r["t"]) for r in filas])
    cmd = np.array([float(r["vyaw_cmd"]) for r in filas])
    gz = np.array([float(r["gz"]) for r in filas])
    t -= t[0]
    k = (t > 16) & (t < 33)
    t, cmd, gz = t[k] - 16, cmd[k], gz[k]
    n = int(round(0.71 / np.median(np.diff(t))))            # media centrada de un periodo de paso
    suave = np.convolve(gz, np.ones(n) / n, mode="same")
    fig = figura(430, 235)
    ax = fig.add_axes([0.12, 0.19, 0.85, 0.74])
    ax.plot(t, gz, color=GRIS_CLARO, lw=0.8, label="gz medido (IMU)")
    ax.plot(t, suave, color=CIAN_LINEA, lw=2.2, label="gz, media de un paso")
    ax.step(t, cmd, where="post", color=GRIS, lw=1.8, label="vyaw mandada")
    ax.set_xlabel("tiempo (s)")
    ax.set_ylabel("velocidad de giro (rad/s)")
    ax.set_ylim(-0.75, 0.75)
    ax.axhline(0, color=GRIS_MEDIO, lw=0.6)
    ax.legend(loc="lower left", ncol=1)
    ax.annotate("retardo efectivo\n0.31 s", xy=(5.9, 0.16), xytext=(1.0, 0.47), fontsize=11,
                arrowprops=dict(arrowstyle="->", color=GRIS, lw=0.9))
    guardar(fig, "fig_escalon")


# ── 13 · lazo cerrado simulado en los cuatro niveles ─────────────────────
def fig_simulacion():
    from seguidor import config
    from seguidor.simulador import simular
    cfg = config.cargar()
    ancho, alto = 464, 196
    fig = figura(ancho, alto)
    datos = []
    for nivel in (1, 2, 3, 4):
        rs = [simular(cfg, nivel, 1.0, des, 0.0, +1) for des in (10.0, -10.0)]
        p = rs[0]["pista_xy"]
        datos.append((nivel, p, rs))
    m = 0.35                                            # margen alrededor de cada pista (m)
    ext = [(p[:, 0].min() - m, p[:, 0].max() + m, p[:, 1].min() - m, p[:, 1].max() + m) for _, p, _ in datos]
    hueco = 14                                          # bp entre paneles
    escala = (ancho - 12 - hueco * 3) / sum(e[1] - e[0] for e in ext)      # bp por metro, igual en todos
    escala = min(escala, (alto - 52) / max(e[3] - e[2] for e in ext))
    x = 6
    for (nivel, p, rs), (x0, x1, y0, y1) in zip(datos, ext):
        w, h = (x1 - x0) * escala, (y1 - y0) * escala
        ax = fig.add_axes([x / ancho, 30 / alto, w / ancho, h / alto])
        ax.plot(p[:, 0], p[:, 1], color=GRIS_CLARO, lw=7, solid_capstyle="butt")
        for r, col, rot in ((rs[0], CIAN_LINEA, "salida +10°"), (rs[1], GRIS, "salida −10°")):
            c = r["camino"]
            ax.plot(c[:, 1], c[:, 2], color=col, lw=1.4, label=rot if nivel == 1 else None)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.axis("off")
        fig.text((x + w / 2) / ancho, 8 / alto, f"Nivel {nivel}", ha="center", fontweight="bold", fontsize=12)
        x += w + hueco
    # escala de 1 m
    ax = fig.add_axes([0, 0, 1, 1], facecolor="none")
    ax.axis("off")
    ax.set_xlim(0, ancho)
    ax.set_ylim(0, alto)
    ax.plot([ancho - 8 - escala, ancho - 8], [alto - 10, alto - 10], color="black", lw=2)
    ax.text(ancho - 8 - escala / 2, alto - 16, "1 m", ha="center", va="top", fontsize=10)
    fig.legend(loc="upper left", bbox_to_anchor=(0.0, 1.0), ncol=2, fontsize=10)
    guardar(fig, "fig_simulacion")


# ── 15 · nivel 2: sesgo del ángulo según la ventana del ajuste ───────────
def fig_nivel2_ajuste():
    # docs/RESULTADOS.md, «La S de la salida»: error del ángulo al acercarse a la curva (IMU de referencia)
    ajustes = ["Parábola de\ntoda la vista", "Reajuste\nhasta 1.2 m", "Reajuste\nhasta 1.0 m", "Reajuste\nhasta 0.8 m"]
    bajo = [24, 18, 6, -1.5]
    alto = [29, 25, 13, 1.5]
    fig = figura(400, 205)
    ax = fig.add_axes([0.13, 0.27, 0.84, 0.68])
    x = np.arange(4)
    cols = [GRIS, GRIS, GRIS, CIAN]
    ax.bar(x, np.array(alto) - np.array(bajo), bottom=bajo, color=cols, width=0.55,
           edgecolor=["none", "none", "none", "black"], linewidth=0.8)
    for xi, b, a in zip(x, bajo, alto):
        txt = f"±{a:.1f}°" if b < 0 else f"+{b:.0f}…+{a:.0f}°"
        ax.text(xi, a + 1.2, txt, ha="center", va="bottom", fontsize=12, fontweight="bold")
    ax.set_xticks(x, ajustes)
    ax.set_ylabel("error del ángulo (°)")
    ax.set_ylim(-4, 35)
    ax.axhline(0, color=GRIS_MEDIO, lw=0.6)
    guardar(fig, "fig_nivel2_ajuste")


# ── 16 · nivel 3: cuánto del segundo semicírculo recorre cada tirada ─────
TIRADAS_N3 = [
    ("20261107_091703", 0.5), ("20261107_092003", 0.5), ("20261107_094454", 0.5),
    ("20261107_100628", 1.0), ("20261107_100742", 1.0), ("20261107_103954", 1.0), ("20261107_104116", 1.0),
    ("20261107_113050", 1.0), ("20261107_113222", 1.0), ("20261107_113528", 1.0),
    ("20261110_041348", 1.0), ("20261110_041539", 1.0), ("20261110_042852", 1.0), ("20261110_043054", 1.0),
    ("20261110_043803", 1.0), ("20261110_044157", 1.0),
]
# (índice de la primera tirada con el cambio, rótulo)
CAMBIOS_N3 = [(0, "media\nescala"), (2, "pausa\npara girar"), (3, "escala 1"), (5, "pausa con\nlínea perdida"),
              (7, "muestreo,\nrumbo, vx"), (10, "búsqueda\nen arco"), (12, "hueco\n40 cm"), (14, "hueco y\nsombra")]


def progreso_s(nombre, escala):
    d = tirada(f"{nombre}_nivel3_escala{'0.5' if escala == 0.5 else '1'}")
    o = leer_csv(os.path.join(d, "ordenes.csv"))
    r = json.load(open(os.path.join(d, "resumen.json")))
    y = np.degrees(np.unwrap(np.radians([float(x["yaw_deg"]) for x in o])))
    k = [i for i, x in enumerate(o) if x["estado"] in ("SEGUIMIENTO", "LINEA_PERDIDA")]
    dd = y[k[0]:k[-1] + 1] - y[k[0]]
    i = int(np.argmin(dd))
    segundo = min(180.0, max(0.0, float(dd[i:].max() - dd[i])))
    return segundo, r["estado_final"]


def fig_nivel3_progreso():
    vals = [progreso_s(n, e) for n, e in TIRADAS_N3]
    fig = figura(560, 250)
    ax = fig.add_axes([0.085, 0.13, 0.905, 0.60])
    x = np.arange(len(vals))
    cols = [CIAN if fin == "FIN" else GRIS for _, fin in vals]
    ax.bar(x, [v for v, _ in vals], color=cols, width=0.66, edgecolor=["black" if c == CIAN else "none" for c in cols],
           linewidth=0.7)
    ax.axhline(180, color=GRIS_MEDIO, lw=0.8, ls="--")
    ax.set_ylim(0, 190)
    ax.set_yticks([0, 45, 90, 135, 180])
    ax.set_ylabel("2.º semicírculo (°)")
    ax.set_xticks(x, [n[9:15] for n, _ in TIRADAS_N3], rotation=90, fontsize=9)
    ax.set_xlim(-0.6, len(vals) - 0.4)
    ax.tick_params(axis="x", length=0, pad=2)
    for j, (i, txt) in enumerate(CAMBIOS_N3):
        if i > 0:
            ax.axvline(i - 0.5, color=GRIS_MEDIO, lw=0.7)
        fin = CAMBIOS_N3[j + 1][0] if j + 1 < len(CAMBIOS_N3) else len(vals)
        ax.text((i + fin - 1) / 2, 196, txt, ha="center", va="bottom", fontsize=10, color="black",
                transform=ax.transData, clip_on=False)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(facecolor=CIAN, edgecolor="black", linewidth=0.7, label="FIN en la barra"),
                       Patch(facecolor=GRIS, label="PARADA: línea perdida 4.5 s")],
              loc="upper left", bbox_to_anchor=(0.0, 0.94), ncol=1, fontsize=10)
    guardar(fig, "fig_nivel3_progreso")


# ── 17 · nivel 3: confianza de la percepción con luz y a oscuras ─────────
def fig_nivel3_luz():
    casos = [("20261110_042852", "luces encendidas", GRIS_CLARO),
             ("20261110_043803", "sombra en el tramo final", GRIS_MEDIO),
             ("20261110_044157", "toda la zona a oscuras", CIAN)]
    tramos = [(0, 8, "recta y\n1.er semicírculo"), (8, 16, "1.er\nsemicírculo"), (16, 30, "2.º\nsemicírculo"),
              (30, 45, "recta final\ny barra")]
    fig = figura(400, 215)
    ax = fig.add_axes([0.12, 0.25, 0.76, 0.62])
    w = 0.26
    for j, (nombre, rot, col) in enumerate(casos):
        d = tirada(nombre + "_nivel3_escala1")
        t0 = float(leer_csv(os.path.join(d, "ordenes.csv"))[0]["t"])
        m = leer_csv(os.path.join(d, "medidas.csv"))
        tt = np.array([float(x["t"]) - t0 for x in m])
        c = np.array([float(x["confianza"]) for x in m])
        med = [float(np.median(c[(tt >= a) & (tt < b)])) for a, b, _ in tramos]
        ax.bar(np.arange(4) + (j - 1) * w, med, width=w, color=col, edgecolor="black", linewidth=0.6, label=rot)
    ax.set_xticks(np.arange(4), [r for *_, r in tramos], fontsize=10)
    ax.set_ylabel("confianza (mediana)")
    ax.set_ylim(0, 1.0)
    ax.axhline(0.45, color=GRIS, lw=0.8, ls="--")
    ax.text(3.5, 0.45, " para seguir:\n 0.45", ha="left", va="center", fontsize=10, color=GRIS, clip_on=False)
    ax.legend(loc="lower left", bbox_to_anchor=(-0.02, 1.0), ncol=3, fontsize=10, handlelength=1.2,
              columnspacing=0.9)
    guardar(fig, "fig_nivel3_luz")


# ── 18 · nivel 4: dónde queda la línea nueva durante el giro de 90° ──────
TIRADAS_N4 = [("20261110_050558", "escala 0.5", "sin adelanto", GRIS),
              ("20261110_052855", "escala 0.5", "adelanto 0.3 m", CIAN_LINEA),
              ("20261110_053202", "escala 0.5", "adelanto 0.3 m", CIAN_LINEA),
              ("20261110_053907", "escala 1", "adelanto 0.3 m", "black")]


def giro_esquina(nombre, escala):
    """(s desde que empieza el giro, distancia de los pies a la línea nueva en m, + = pasado) y la duración."""
    d = tirada(f"{nombre}_nivel4_escala{'0.5' if escala == 'escala 0.5' else '1'}")
    o = leer_csv(os.path.join(d, "ordenes.csv"))
    t0 = float(o[0]["t"])
    giro = [x for x in o if x["estado"] == "GIRO_ESQUINA" and x["enviada"] == "1" and abs(float(x["vyaw"])) > 0]
    ti, tf = float(giro[0]["t"]) - t0, float(giro[-1]["t"]) - t0
    pts = []
    for q in leer_csv(os.path.join(d, "medidas.csv")):
        tt = float(q["t"]) - t0
        if not (ti + 2.0 <= tt <= tf + 3.0) or not q["y"] or float(q["confianza"]) < 0.4:
            continue
        th = math.radians(float(q["theta_deg"]))
        if abs(math.degrees(th)) > 30:
            continue                      # la línea nueva aún sale de lado: el ajuste no vale
        dp = (float(q["y"]) - 0.13 * math.tan(th)) * math.cos(th)    # pies (13 cm detrás) → línea; + izquierda
        pts.append((tt - ti, -dp))
    return np.array(pts), tf - ti


def fig_nivel4_giro():
    fig = figura(440, 166)
    ax = fig.add_axes([0.13, 0.25, 0.85, 0.56])
    ax.axhspan(-0.10, 0.10, color=GRIS_CLARO, alpha=0.7, lw=0)
    ax.text(2.95, 0.072, "pies a ±10 cm de la línea", fontsize=10, color=GRIS)
    vistos, fines = set(), []
    for nombre, esc, rot, col in TIRADAS_N4:
        p, dur = giro_esquina(nombre, esc)
        fines.append(dur)
        lab = f"{rot}, {esc}"
        ax.plot(p[:, 0], p[:, 1], "o-", color=col, ms=2.6, lw=1.2,
                label=None if lab in vistos else lab)
        vistos.add(lab)
    ax.axvspan(min(fines), max(fines), color=GRIS_MEDIO, alpha=0.25, lw=0)
    ax.text(max(fines) + 0.1, 0.235, "acaba el giro (90°)", fontsize=10, color=GRIS, va="top")
    ax.set_xlim(2.8, 10.6)
    ax.set_ylim(-0.12, 0.47)
    ax.set_xlabel("tiempo desde que empieza el giro (s)")
    ax.set_ylabel("pasado de la línea (m)")
    ax.legend(loc="lower left", bbox_to_anchor=(-0.02, 1.0), ncol=3, fontsize=10, handlelength=1.4,
              columnspacing=0.8)
    guardar(fig, "fig_nivel4_giro")


# ── 19 · tiradas registradas por nivel ───────────────────────────────────
def fig_tiradas():
    cuenta = {}
    for d in sorted(os.listdir(DATOS)):
        if not d.startswith("tirada_") or "simulacro" in d:
            continue
        p = os.path.join(DATOS, d, "resumen.json")
        if not os.path.exists(p):
            continue
        r = json.load(open(p))
        n = r["nivel"]
        tot, ok = cuenta.get(n, (0, 0))
        cuenta[n] = (tot + 1, ok + (r["estado_final"] == "FIN"))
    fig = figura(300, 200)
    ax = fig.add_axes([0.14, 0.16, 0.84, 0.70])
    niv = sorted(cuenta)
    tot = np.array([cuenta[n][0] for n in niv])
    ok = np.array([cuenta[n][1] for n in niv])
    ax.bar(niv, ok, color=CIAN, edgecolor="black", linewidth=0.6, width=0.6, label="FIN")
    ax.bar(niv, tot - ok, bottom=ok, color=GRIS, width=0.6, label="PARADA")
    for n, a, b in zip(niv, tot, ok):
        ax.text(n, a + 0.3, f"{b}/{a}", ha="center", va="bottom", fontsize=12, fontweight="bold")
    ax.set_xticks(niv, [f"Nivel {n}" for n in niv])
    ax.set_ylabel("tiradas")
    ax.set_ylim(0, max(tot) + 3)
    ax.legend(loc="lower left", bbox_to_anchor=(-0.02, 1.0), ncol=2, fontsize=10)
    guardar(fig, "fig_tiradas")
    print("      ", {n: cuenta[n] for n in niv})


TODAS = [fig_escalon, fig_simulacion, fig_nivel2_ajuste, fig_nivel3_progreso, fig_nivel3_luz, fig_nivel4_giro,
         fig_tiradas]

if __name__ == "__main__":
    pedidas = sys.argv[1:]
    for f in TODAS:
        if not pedidas or f.__name__ in pedidas:
            f()
