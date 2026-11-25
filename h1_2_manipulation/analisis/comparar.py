#!/usr/bin/env python3
"""Compara los dos brazos del banco: cuantas veces sostuvo cada uno el objeto (Fisher exacto) y como lo hizo (mediana
[min-max] y Mann-Whitney de cada medida). Lee el resumen.json (o, si el banco no acabo, ensayos.csv) de una carpeta.

    python3 analisis/comparar.py datos/banco_20261125_101500_cubo
    python3 analisis/comparar.py --ultima                 # la carpeta de banco mas reciente de datos/
    python3 analisis/comparar.py A B --juntar             # varias tandas juntas (solo si se pide)

Regla del plan del regulador (§8): se comparan los brazos DENTRO de una tanda, intercalados; nunca contra un numero
de otra tanda o de otro dia (dos tandas del mismo dedo difirieron un factor 2 por la deriva del cero). Por eso cada
carpeta se informa por separado salvo --juntar.
Las medidas del brazo firmware salen del lazo en modo observador (las mismas lecturas, sin mandar nada).
"""

import argparse
import csv
import glob
import itertools
import json
import math
import os
import statistics
import sys

AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

MEDIDAS = [
    ("t_cerrar_s", "tiempo de cierre (s)"),
    ("pico_apriete_cierre_gf", "pico del apriete al cerrar (gf)"),
    ("apriete_al_cerrar_gf", "apriete al acabar de cerrar (gf)"),
    ("apriete_transporte_min_gf", "apriete minimo en el transporte (gf)"),
    ("apriete_sostener_gf", "apriete sostenido (gf)"),
    ("caida_sostener_pct", "caida del apriete sostenido (%)"),
    ("mano_desde_el_plan_mm", "mano apartada del plan al cerrar (mm)"),
    ("desplazamiento_mm", "objeto desplazado al devolverlo (mm)"),
    ("giro_deg", "objeto girado al devolverlo (grados)"),
    ("sitio_mm", "objeto apartado de su sitio al empezar (mm)"),
]


def fisher(a, b, c, d):
    """p bilateral del test exacto de Fisher para [[a, b], [c, d]] (sostenido / no, por brazo)."""
    n1, n2, k = a + b, c + d, a + c
    n = n1 + n2

    def p(x):
        return math.comb(n1, x) * math.comb(n2, k - x) / math.comb(n, k)
    obs = p(a)
    return min(1.0, sum(p(x) for x in range(max(0, k - n2), min(k, n1) + 1) if p(x) <= obs * (1 + 1e-9)))


def mann_whitney(x, y):
    """p bilateral de Mann-Whitney: exacta por permutaciones si caben (<= 200000), si no normal con correccion."""
    x, y = list(x), list(y)
    n1, n2 = len(x), len(y)
    if not n1 or not n2:
        return None
    todos = sorted(x + y)

    def rangos(v):
        r = {}
        i = 0
        while i < len(todos):
            j = i
            while j + 1 < len(todos) and todos[j + 1] == todos[i]:
                j += 1
            r[todos[i]] = (i + j) / 2 + 1
            i = j + 1
        return [r[a] for a in v]
    R = rangos(todos)
    u_obs = sum(rangos(x)) - n1 * (n1 + 1) / 2
    media = n1 * n2 / 2
    if math.comb(n1 + n2, n1) <= 200000:
        d_obs = abs(u_obs - media)
        cuenta = tot = 0
        for idx in itertools.combinations(range(n1 + n2), n1):
            u = sum(R[i] for i in idx) - n1 * (n1 + 1) / 2
            tot += 1
            cuenta += abs(u - media) >= d_obs - 1e-9
        return cuenta / tot
    sd = math.sqrt(n1 * n2 * (n1 + n2 + 1) / 12)
    z = (abs(u_obs - media) - 0.5) / sd if sd else 0.0
    return math.erfc(z / math.sqrt(2))


def _num(v):
    try:
        return None if v in (None, "") else float(v)
    except (TypeError, ValueError):
        return None


def _fmt(v):
    if not v:
        return "-"
    return f"{statistics.median(v):.0f} [{min(v):.0f}-{max(v):.0f}]" if max(abs(x) for x in v) >= 10 else \
        f"{statistics.median(v):.2f} [{min(v):.2f}-{max(v):.2f}]"


def tabla(ensayos):
    """El informe de un banco (lista de ensayos, como los apunta agarre/banco.py)."""
    brazos = sorted({r["brazo"] for r in ensayos}, key=lambda b: (b != "regulador", b))
    objetos = sorted({r["objeto"] for r in ensayos})
    lin = [f"Banco: {', '.join(objetos)}; {len(ensayos)} ensayos"]
    cuenta = {}
    for b in brazos:
        rs = [r for r in ensayos if r["brazo"] == b]
        sost = sum(1 for r in rs if r["resultado"] == "sostenido")
        fall = sum(1 for r in rs if r["resultado"] == "agarre_fallido")
        esc = [r.get("fase_escape") or "?" for r in rs if r["resultado"] == "escapado"]
        cuenta[b] = (sost, len(rs) - sost)
        lin.append(f"  {b:10s} {sost:2d} de {len(rs):2d} sostenidos ({100 * sost / max(len(rs), 1):.0f} %)"
                   + (f"; {fall} sin agarre" if fall else "")
                   + (f"; escapados en {', '.join(sorted(esc))}" if esc else ""))
    if len(brazos) == 2:
        (a, b_), (c, d) = cuenta[brazos[0]], cuenta[brazos[1]]
        lin.append(f"  Fisher exacto (sostenido o no, {brazos[0]} frente a {brazos[1]}): p = {fisher(a, b_, c, d):.3f}")
    lin.append("")
    lin.append(f"  {'medida':40s}" + "".join(f"{b:>22s}" for b in brazos) + ("   p (Mann-Whitney)" if len(brazos) == 2 else ""))
    for clave, texto in MEDIDAS:
        vals = {b: [x for x in (_num(r.get(clave)) for r in ensayos if r["brazo"] == b) if x is not None] for b in brazos}
        if not any(vals.values()):
            continue
        p = mann_whitney(*[vals[b] for b in brazos]) if len(brazos) == 2 and all(vals.values()) else None
        lin.append(f"  {texto:40s}" + "".join(f"{_fmt(vals[b]):>22s}" for b in brazos)
                   + (f"   {p:.3f}" if p is not None else ""))
    eventos = {b: [sum(int(_num(r.get(k)) or 0) for r in ensayos if r["brazo"] == b)
                   for k in ("resbalones", "ceden_breves", "escapes_detectados", "guardas")] for b in brazos}
    lin.append("  eventos del lazo (resbalones / cedes breves / escapes detectados / guardas): "
               + "; ".join(f"{b} {'/'.join(map(str, v))}" for b, v in eventos.items()))
    return "\n".join(lin)


def leer(carpeta):
    r = os.path.join(carpeta, "resumen.json")
    if os.path.exists(r):
        with open(r) as fh:
            e = json.load(fh).get("ensayos")
        if e:
            return e
    c = os.path.join(carpeta, "ensayos.csv")
    if os.path.exists(c):
        with open(c) as fh:
            return list(csv.DictReader(fh))
    sys.exit(f"{carpeta}: sin ensayos (resumen.json ni ensayos.csv)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("carpetas", nargs="*")
    ap.add_argument("--ultima", action="store_true")
    ap.add_argument("--juntar", action="store_true", help="varias tandas en una tabla (el plan lo desaconseja)")
    a = ap.parse_args()
    carpetas = list(a.carpetas)
    if a.ultima:
        todas = sorted(glob.glob(os.path.join(AQUI, "datos", "banco_*")))
        if not todas:
            sys.exit("no hay carpetas de banco en datos/")
        carpetas.append(todas[-1])
    if not carpetas:
        ap.error("una carpeta del banco o --ultima")
    if a.juntar:
        print(tabla([r for c in carpetas for r in leer(c)]))
        return
    for c in carpetas:
        print(f"== {os.path.basename(os.path.normpath(c))}")
        print(tabla(leer(c)))
        print()


if __name__ == "__main__":
    main()
