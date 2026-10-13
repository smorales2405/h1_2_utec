#!/usr/bin/env python3
"""Tiradas simuladas del lazo cerrado (seguidor/simulador.py), sin robot: estimacion, control y
supervisor de verdad sobre un modelo de la marcha medido y las pistas del PDF.

    python3 herramientas/simular.py --nivel 1                       # +-10 grados, a media escala
    python3 herramientas/simular.py --nivel 1 2 3 4 --escala 1 --figura
    python3 herramientas/simular.py --nivel 1 --tapar 3 6            # camara tapada de 3 a 6 s
    python3 herramientas/simular.py --nivel 1 --marcha deriva=0.05 retardo_s=0.3

Exito = FIN, la puntera entre 0 y 30 cm mas alla del borde cercano de la barra y el centro de los
pies a menos de 20 cm de la linea (con los pies a +-10 cm, el pie no sale de 30 cm).
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config  # noqa: E402
from seguidor.simulador import simular  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="Tiradas simuladas del seguidor (sin robot)")
    ap.add_argument("--nivel", type=int, nargs="+", default=[1])
    ap.add_argument("--escala", type=float, default=0.5)
    ap.add_argument("--desalineado", type=float, nargs="+", default=[10.0, -10.0], help="grados, + a la izquierda")
    ap.add_argument("--lateral", type=float, default=0.0, help="m de la linea al salir, + a la izquierda")
    ap.add_argument("--sentido", type=int, nargs="+", default=[1, -1], help="curvas a la izquierda (+1) o derecha (-1)")
    ap.add_argument("--tapar", type=float, nargs=2, action="append", default=[], metavar=("T0", "T1"))
    ap.add_argument("--marcha", nargs="*", default=[], help="cambios del modelo de marcha, p. ej. deriva=0.05")
    ap.add_argument("--semillas", type=int, default=1)
    ap.add_argument("--figura", action="store_true", help="guardar datos/simulacion_nivel<N>.png")
    args = ap.parse_args()

    cfg = config.cargar()
    marcha = {k: float(v) for k, v in (c.split("=") for c in args.marcha)}
    resultados = []
    for nivel in args.nivel:
        sentidos = args.sentido if nivel > 1 else [1]
        for sentido in sentidos:
            for des in args.desalineado:
                for semilla in range(args.semillas):
                    r = simular(cfg, nivel, args.escala, des, args.lateral, sentido, marcha, semilla, args.tapar)
                    resultados.append(r)
                    print(f"nivel {nivel} sentido {sentido:+d} salida {des:+5.1f} grados semilla {semilla}: "
                          f"{'EXITO' if r['exito'] else 'fallo'} · {r['estado_final']} ({r['motivo']}) · "
                          f"{r['tiempo_s']:.1f} s · error medio {100 * r['error_medio_m']:.1f} cm, max "
                          f"{100 * r['error_max_m']:.1f} cm · pasada {100 * r['pasada_m']:+.1f} cm · "
                          f"suavidad vyaw {r['suavidad_vyaw']:.3f} rad/s")
                    if not r["exito"]:
                        for tr in r["transiciones"]:
                            print(f"     {tr[0]:6.2f} s  {tr[1]} -> {tr[2]}: {tr[3]}")
    n_ok = sum(r["exito"] for r in resultados)
    print(f"\n{n_ok}/{len(resultados)} con exito")

    if args.figura:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for nivel in args.nivel:
            rs = [r for r in resultados if r["nivel"] == nivel]
            fig, ejes = plt.subplots(1, 3, figsize=(16, 5))
            for r in rs:
                etiqueta = f"salida {r['desalineado_deg']:+.0f} grados, sentido {r['sentido']:+d}"
                ejes[0].plot(r["pista_xy"][:, 0], r["pista_xy"][:, 1], "k-", lw=3, alpha=0.15)
                ejes[0].plot(r["camino"][:, 1], r["camino"][:, 2], lw=1, label=etiqueta)
                s = r["serie"]
                ejes[1].plot(s["t"], 100 * s["error"], lw=1, label=etiqueta)
                ejes[2].plot(s["t"], s["vyaw"], lw=1, label=f"vyaw, {etiqueta}")
                ejes[2].plot(s["t"], s["vx"], lw=1, ls="--")
            ejes[0].set_aspect("equal")
            ejes[0].set_title(f"{rs[0]['pista']} (escala {args.escala})")
            ejes[0].legend(fontsize=7)
            ejes[1].set_ylabel("centro de los pies a la linea (cm)")
            ejes[1].axhline(20, color="r", lw=0.6, ls=":")
            ejes[1].axhline(-20, color="r", lw=0.6, ls=":")
            ejes[2].set_ylabel("vyaw (rad/s) · vx (m/s, a trazos)")
            for ax in ejes[1:]:
                ax.set_xlabel("s")
            fig.tight_layout()
            ruta = os.path.join(config.ruta_datos(cfg), f"simulacion_nivel{nivel}.png")
            os.makedirs(os.path.dirname(ruta), exist_ok=True)
            fig.savefig(ruta, dpi=90)
            print(f"Figura: {ruta}")


if __name__ == "__main__":
    main()
