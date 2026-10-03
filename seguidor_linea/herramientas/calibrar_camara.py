#!/usr/bin/env python3
"""Hito 1: geometria de la camara con el robot de pie en FSM 201. Solo lectura: no manda marcha.

Ajusta el plano del suelo (preguntas 6.1.2 y 6.1.3 del PDF) sobre la mediana temporal de la
profundidad de todas las capturas, con los puntos hasta --zmax: altura, inclinacion y roll de la
camara IR izquierda, mas el roll y el pitch de la IMU del torso en ese momento (la referencia
para corregir el balanceo al andar).

Con el EMISOR ENCENDIDO: sin el, la profundidad de este suelo tiene de 3 a 15 cm de ruido entre
0.5 y 3 m y la inclinacion sale entre 50 y 53 grados segun los puntos que se usen (2026-10-03).

El plano solo no basta: el 2026-10-03 daba 50.46 grados y con eso la puntera quedaba a 0.26 m de
la vertical de la camara, donde se tendria que ver en la imagen y no se veia. La inclinacion se
fija con dos tramos de cinta transversales de 60 cm medidos con cinta desde la puntera de los
pies: la barra de fin (--puntera-barra) y una marca mas cerca (--puntera-marca, p. ej. a 1 m).
La inclinacion es la que separa en el suelo sus bordes cercanos lo mismo que la cinta; la marca
da ademas x_puntera_m (vertical de la camara -> puntera). Con --yaw mide el yaw de la camara
respecto del cuerpo: el robot tiene que estar alineado con la linea.

    ./camara_servidor.sh --profundidad --emisor
    ./ejecutar.sh herramientas/calibrar_camara.py --puntera-barra 3.58 --puntera-marca 1.00 --altura 1.65 --yaw
La marca se retira despues: el seguidor la tomaria por la barra de fin.

Escribe config/geometria.yaml y deja en datos/calibracion_<fecha>/ el resumen JSON, la imagen IR
con las distancias del suelo dibujadas y la profundidad, para el informe.
"""

import argparse
import math
import os
import sys
import time
import warnings

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config, geometria, registro  # noqa: E402
from seguidor.calibracion import ajustar_recta, buscar_tramos, inclinacion_por_marcas, puntos_de_linea  # noqa: E402
from seguidor.fuentes import FuenteZmq, prof_en_metros  # noqa: E402
from seguidor.geometria import Intrinsecos, ModeloSuelo  # noqa: E402
from seguidor.robot import FSM_MARCHA, Robot  # noqa: E402

DISTANCIAS_M = (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0)


def dibujar_distancias(ir, modelo, puntos=None, tramos=()):
    img = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
    ys = np.linspace(-3.0, 3.0, 121)
    for x in DISTANCIAS_M:
        u, v = modelo.suelo_a_pixel(np.full_like(ys, x), ys)
        ok = np.isfinite(u) & (u >= 0) & (u < img.shape[1]) & (v >= 0) & (v < img.shape[0])
        if ok.sum() < 2:
            continue
        pts = np.stack([u[ok], v[ok]], axis=1).astype(np.int32)
        cv2.polylines(img, [pts], False, (0, 200, 255), 1, cv2.LINE_AA)
        dcha = pts[np.argmax(pts[:, 0])]   # etiqueta en el extremo derecho, dentro de la imagen
        cv2.putText(img, f"{x:g} m", (int(dcha[0]) - 52, int(dcha[1]) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1, cv2.LINE_AA)
    for t in tramos:
        cv2.line(img, (t["u0"], t["fila"]), (t["u1"], t["fila"]), (255, 0, 255), 2)
    xs = np.linspace(0.2, 4.0, 60)
    for y in (-0.3, 0.0, 0.3):   # eje del robot y la tolerancia de 30 cm de la seccion 9
        u, v = modelo.suelo_a_pixel(xs, np.full_like(xs, y))
        ok = np.isfinite(u) & (u >= 0) & (u < img.shape[1]) & (v >= 0) & (v < img.shape[0])
        if ok.sum() >= 2:
            pts = np.stack([u[ok], v[ok]], axis=1).astype(np.int32)
            cv2.polylines(img, [pts], False, (80, 255, 80) if y == 0 else (60, 140, 60), 1, cv2.LINE_AA)
    for _, _, u, v in puntos or []:
        cv2.circle(img, (int(u), int(v)), 2, (255, 80, 80), -1)
    return img


def main():
    ap = argparse.ArgumentParser(description="Calibracion de la camara sobre el suelo (Hito 1)")
    ap.add_argument("--capturas", type=int, default=30)
    ap.add_argument("--zmax", type=float, default=3.0,
                    help="m: solo los puntos de profundidad hasta aqui (mas lejos el error crece mucho)")
    ap.add_argument("--puntera-barra", type=float, default=None, metavar="D",
                    help="m medidos con cinta de la puntera de los pies al borde cercano de la barra")
    ap.add_argument("--puntera-marca", type=float, default=None, metavar="D",
                    help="m medidos con cinta de la puntera al borde cercano de una marca transversal de "
                         "60 cm puesta mas cerca que la barra: con las dos se mide la inclinacion")
    ap.add_argument("--altura", type=float, default=None, metavar="H",
                    help="m medidos con cinta del suelo al centro del cristal de la camara: sustituye a la "
                         "de la profundidad (que el 2026-10-03 salio un 1.5 %% alta)")
    ap.add_argument("--yaw", action="store_true",
                    help="medir el yaw de la camara: robot alineado con la linea")
    ap.add_argument("--permitir-sin-emisor", action="store_true",
                    help="calibrar aunque el emisor este apagado (profundidad mucho peor)")
    ap.add_argument("--no-guardar", action="store_true", help="no escribir config/geometria.yaml")
    ap.add_argument("--sin-fsm", action="store_true", help="no exigir FSM 201 (solo para pruebas)")
    args = ap.parse_args()

    cfg = config.cargar()
    robot = Robot(cfg["red"]["iface"])
    if not robot.esperar_lowstate(3.0):
        sys.exit("No llega rt/lowstate.")
    fsm, _ = robot.leer_fsm()
    print(f"FSM = {fsm}")
    if fsm not in FSM_MARCHA and not args.sin_fsm:
        sys.exit("La calibracion se hace con el robot de pie en FSM 201 (L2+UP), que es como va a andar.")

    fuente = FuenteZmq(cfg["red"]["zmq_fotogramas"], solo_ultimo=True)
    profs, irs, por_captura, imus, ultimo = [], [], [], [], None
    print(f"{args.capturas} capturas (robot quieto, nadie delante de la camara)...")
    while len(profs) < args.capturas:
        f = fuente.siguiente(timeout_s=3.0)
        if f is None:
            sys.exit("No llegan fotogramas: ./camara_servidor.sh --profundidad --emisor")
        if f.prof is None:
            sys.exit("El servidor no manda profundidad: ./camara_servidor.sh --profundidad --emisor")
        if not f.emisor and not args.permitir_sin_emisor:
            sys.exit("Emisor apagado: sin el, la profundidad de este suelo tiene 3-15 cm de ruido.\n"
                     "  ./camara_servidor.sh --profundidad --emisor   (o --permitir-sin-emisor)")
        intr_prof = Intrinsecos.desde_dict(f.meta["prof"])
        z = prof_en_metros(f)
        n, d, _ = geometria.ajustar_plano(geometria.nube(z, intr_prof, zmax=args.zmax), semilla=len(profs))
        altura, incl, roll = geometria.pose_de_plano(n, d)
        por_captura.append((altura, math.degrees(incl), math.degrees(roll)))
        profs.append(np.where(z > 0, z, np.nan))
        irs.append(f.ir.astype(np.float32))
        imu = robot.imu()
        imus.append((math.degrees(imu.roll), math.degrees(imu.pitch)))
        ultimo = f
        time.sleep(0.1)   # capturas repartidas en ~3 s, no seguidas
    fuente.cerrar()

    # Ajuste final sobre la mediana temporal: quita el ruido de cada captura, no el sesgo
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)   # pixeles sin dato en todas las capturas
        prof_med = np.nan_to_num(np.nanmedian(np.stack(profs), axis=0), nan=0.0)
    ir_med = np.median(np.stack(irs), axis=0).astype(np.uint8)
    n, d, frac = geometria.ajustar_plano(geometria.nube(prof_med, intr_prof, zmax=args.zmax))
    altura_prof, incl_plano, roll = geometria.pose_de_plano(n, d)
    altura = args.altura if args.altura is not None else altura_prof
    roll_deg = math.degrees(roll)
    desv = np.std(np.array(por_captura), axis=0)
    imu_roll, imu_pitch = np.mean(np.array(imus), axis=0)
    intr_ir = Intrinsecos.desde_dict(ultimo.meta["ir"])

    print(f"\naltura (profundidad): {altura_prof:.3f} m   (desv. entre capturas {1000 * desv[0]:.1f} mm)")
    if args.altura is not None:
        print(f"altura (cinta)      : {args.altura:.3f} m -> se usa esta; la profundidad da "
              f"{100 * (altura_prof / args.altura - 1):+.1f} %")
    print(f"inclinacion (plano) : {math.degrees(incl_plano):.2f} grados bajo la horizontal   (desv. {desv[1]:.2f})")
    print(f"roll                : {roll_deg:+.2f} grados   (desv. {desv[2]:.2f})")
    print(f"puntos en el plano  : {100 * frac:.0f} % (hasta {args.zmax:g} m, emisor "
          f"{'encendido' if ultimo.emisor else 'APAGADO'})")
    print(f"IMU del torso       : roll {imu_roll:+.2f}, pitch {imu_pitch:+.2f} grados (referencia)")
    resumen = {"fecha": time.strftime("%Y-%m-%d %H:%M:%S"), "fsm": fsm, "capturas": len(profs),
               "emisor": ultimo.emisor, "zmax_m": args.zmax, "altura_m": altura,
               "altura_profundidad_m": altura_prof, "altura_cinta_m": args.altura,
               "inclinacion_plano_deg": math.degrees(incl_plano), "roll_deg": roll_deg,
               "desv_altura_mm": 1000 * desv[0], "desv_inclinacion_deg": desv[1], "desv_roll_deg": desv[2],
               "fraccion_plano": frac, "imu_roll_ref_deg": imu_roll, "imu_pitch_ref_deg": imu_pitch}

    # Tramos de cinta transversales: el mas lejano es la barra; con dos, el mas cercano es la marca
    modelo_plano = ModeloSuelo(intr_ir, altura, incl_plano, roll)
    tramos = buscar_tramos(ir_med, modelo_plano)
    barra = tramos[-1] if tramos and tramos[-1]["x_m"] > 2.0 else None
    marca = tramos[0] if len(tramos) >= 2 else None
    incl, fuente_incl = incl_plano, "plano"
    if args.puntera_barra is not None and args.puntera_marca is not None:
        if barra is None or marca is None:
            print(f"\nse ven {len(tramos)} tramo(s) transversales: hacen falta la barra y la marca")
        else:
            sep = args.puntera_barra - args.puntera_marca
            incl_m = inclinacion_por_marcas(modelo_plano, marca, barra, sep)
            if incl_m is None:
                print(f"\nninguna inclinacion entre 40 y 60 grados separa las marcas {sep:.2f} m")
            else:
                incl, fuente_incl = incl_m, "marcas"
                print(f"inclinacion (marcas): {math.degrees(incl_m):.2f} grados: la que separa la marca y la barra "
                      f"{sep:.2f} m, como la cinta ({math.degrees(incl_m - incl_plano):+.2f} respecto del plano)")
                resumen.update(inclinacion_marcas_deg=math.degrees(incl_m), separacion_marcas_m=sep)
    modelo = ModeloSuelo(intr_ir, altura, incl, roll)
    tramos = buscar_tramos(ir_med, modelo)   # mismas filas, distancias con la inclinacion elegida
    barra = tramos[-1] if tramos and tramos[-1]["x_m"] > 2.0 else None
    marca = tramos[0] if len(tramos) >= 2 else None
    incl_deg = math.degrees(incl)
    cerca_ir, lejos_ir = modelo.alcance()
    fov_ir = intr_ir.fov_deg()
    print(f"inclinacion usada   : {incl_deg:.2f} grados (de {fuente_incl})")
    print(f"IR  FOV H {fov_ir[0]:.1f} V {fov_ir[1]:.1f} grados -> suelo de {cerca_ir:.2f} a {lejos_ir:.2f} m")
    resumen.update(inclinacion_deg=incl_deg, inclinacion_fuente=fuente_incl, fov_ir_deg=fov_ir,
                   alcance_ir_m=[cerca_ir, lejos_ir], tramos=tramos)
    if "color" in ultimo.meta:
        intr_c = Intrinsecos.desde_dict(ultimo.meta["color"])
        cerca_c, lejos_c = ModeloSuelo(intr_c, altura, incl, roll).alcance()
        fov_c = intr_c.fov_deg()
        print(f"color FOV H {fov_c[0]:.1f} V {fov_c[1]:.1f} grados -> suelo de {cerca_c:.2f} a {lejos_c:.2f} m "
              "(aprox.: se ignoran los 15 mm entre la camara de color y la IR)")
        resumen.update(fov_color_deg=fov_c, alcance_color_m=[cerca_c, lejos_c])

    for nombre, t in (("marca", marca), ("barra de fin", barra)):
        if t is not None:
            print(f"{nombre:13s}: borde cercano a {t['x_m']:.2f} m de la vertical de la camara, "
                  f"largo aparente {t['largo_m']:.2f} m (mide 0.60)")
    if barra is None:
        print("barra de fin  : no se ve")
    x_puntera = cfg["geometria"].get("x_puntera_m")
    if marca is not None and args.puntera_marca is not None:      # la marca cercana depende menos de la inclinacion
        x_puntera = marca["x_m"] - args.puntera_marca
    elif barra is not None and args.puntera_barra is not None:
        x_puntera = barra["x_m"] - args.puntera_barra
    if x_puntera is not None and (args.puntera_marca is not None or args.puntera_barra is not None):
        print(f"puntera a {x_puntera:+.2f} m de la vertical de la camara -> el suelo se ve desde "
              f"{cerca_ir - x_puntera:+.2f} m por delante de la puntera")
        if cerca_ir < x_puntera:
            print("  AVISO: con esto las punteras deberian verse abajo en la imagen; si no se ven, "
                  "la inclinacion esta mal (hace falta la marca)")
        resumen.update(puntera_barra_m=args.puntera_barra, puntera_marca_m=args.puntera_marca, x_puntera_m=x_puntera)

    yaw_deg = cfg["geometria"].get("yaw_deg", 0.0)
    puntos = None
    yaw_medido = False
    if args.yaw:
        puntos = puntos_de_linea(ir_med, modelo)
        if len(puntos) < 20:
            print(f"\n--yaw: solo {len(puntos)} puntos de linea; no se mide el yaw")
        else:
            a, b, usados = ajustar_recta(puntos)
            yaw_deg, yaw_medido = -math.degrees(math.atan(b)), True
            print(f"\nlinea vista: y = {a:+.3f} m {b:+.4f}*x ({usados} puntos) -> "
                  f"yaw de la camara {yaw_deg:+.2f} grados, linea a {100 * a:+.1f} cm del eje")
            print("  (vale solo si el robot esta alineado con la linea)")
            resumen.update(linea_a_m=a, linea_b=b, linea_puntos=usados, yaw_deg=yaw_deg)

    carpeta = registro.carpeta_tirada(config.ruta_datos(cfg), "calibracion")
    cv2.imwrite(os.path.join(carpeta, "ir.png"), ir_med)   # mediana cruda: para repetir los detectores sin robot
    cv2.imwrite(os.path.join(carpeta, "ir_distancias.png"), dibujar_distancias(ir_med, modelo, puntos, tramos))
    cv2.imwrite(os.path.join(carpeta, "profundidad.png"),
                cv2.applyColorMap(cv2.convertScaleAbs(ultimo.prof, alpha=0.06), cv2.COLORMAP_JET))
    registro.guardar_json(os.path.join(carpeta, "resumen.json"), resumen)
    print(f"\nDatos: {carpeta}/ (resumen.json, ir.png, ir_distancias.png, profundidad.png)")

    if args.no_guardar:
        print("--no-guardar: config/geometria.yaml no se toca")
        return
    geo = {"calibrada": True, "fecha": resumen["fecha"], "camara": "IR izquierda",
           "emisor_en_calibracion": bool(ultimo.emisor),
           "altura_m": round(float(altura), 4), "altura_fuente": "cinta" if args.altura is not None else "profundidad",
           "inclinacion_deg": round(incl_deg, 3),
           "inclinacion_fuente": fuente_incl, "inclinacion_plano_deg": round(math.degrees(incl_plano), 3),
           "roll_deg": round(roll_deg, 3), "yaw_deg": round(float(yaw_deg), 3),
           "yaw_medido": yaw_medido or bool(cfg["geometria"].get("yaw_medido", False)),
           "imu_roll_ref_deg": round(float(imu_roll), 3), "imu_pitch_ref_deg": round(float(imu_pitch), 3),
           "x_puntera_m": None if x_puntera is None else round(float(x_puntera), 3),
           "alcance_ir_m": [round(cerca_ir, 3), round(lejos_ir, 3)], "capturas": len(profs)}
    config.guardar_geometria(geo)
    print(f"Escrito {config.RUTA_GEOMETRIA}")


if __name__ == "__main__":
    main()
