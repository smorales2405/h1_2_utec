#!/usr/bin/env python3
"""Graba un dataset para desarrollar la percepcion sin robot (fase "Conjunto de datos" del plan).

Guarda los fotogramas que publica camara_servidor.py y rt/lowstate a ~100 Hz, los dos con
time.monotonic() del PC2. NO manda ninguna orden: el robot lo lleva un operador con wasd.sh (o
con el mando) en otra terminal. El joystick queda en lowstate.csv; las teclas de wasd no.

    ./camara_servidor.sh                      (IR sin emisor, como se va a andar)
    ./ejecutar.sh herramientas/grabar_dataset.py --nombre nivel1_a [--duracion 300] [--cada 1]
    # en otra terminal:  ssh -t unitree@192.168.0.143 ~/robotics40/wasd.sh --vx 0.2 --vyaw 0.3

Para con Ctrl+C o al cumplir --duracion. Deja datos/dataset_<fecha>_<nombre>/ con el formato de
seguidor/fuentes.py (fotogramas.csv, meta.json, ir/), lowstate.csv y resumen.json.
IR en PNG sin perdidas: ~200 KB por fotograma, ~6 MB/s a 30 fps.
"""

import argparse
import math
import os
import shutil
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config, registro  # noqa: E402
from seguidor.fuentes import EscritorDataset, FuenteZmq  # noqa: E402
from seguidor.robot import Robot  # noqa: E402

LIBRE_MIN_GB = 5.0


def envolver(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser(description="Graba fotogramas e IMU con la misma hora (no mueve el robot)")
    ap.add_argument("--nombre", required=True, help="p. ej. nivel1_a, nivel3_sombra")
    ap.add_argument("--duracion", type=float, default=300.0, help="s como mucho")
    ap.add_argument("--cada", type=int, default=1, help="guardar 1 de cada N fotogramas (2 = 15 fps)")
    ap.add_argument("--nota", default="", help="texto libre para el resumen (luz, pista, quien llevo el robot)")
    args = ap.parse_args()

    cfg = config.cargar()
    carpeta = registro.carpeta_tirada(config.ruta_datos(cfg), "dataset", args.nombre)
    libre = shutil.disk_usage(carpeta).free / 1e9
    if libre < LIBRE_MIN_GB:
        sys.exit(f"Solo quedan {libre:.1f} GB libres en el disco.")

    robot = Robot(cfg["red"]["iface"])
    if not robot.esperar_lowstate(3.0):
        sys.exit("No llega rt/lowstate.")
    fsm, _ = robot.leer_fsm()
    reg_low = registro.RegistroLowstate(os.path.join(carpeta, "lowstate.csv"), cfg["registro"]["lowstate_cada"],
                                        lambda: {"estado": "grabando"})
    robot.al_lowstate(reg_low.al_recibir)

    fuente = FuenteZmq(cfg["red"]["zmq_fotogramas"], solo_ultimo=False, cola=120)
    primero = fuente.siguiente(timeout_s=3.0)
    if primero is None:
        reg_low.cerrar()
        sys.exit("No llegan fotogramas: ./camara_servidor.sh")
    flujos = [k for k in ("ir", "color", "prof") if getattr(primero, k) is not None]
    print(f"FSM {fsm} · flujos {flujos} · emisor {'ENCENDIDO (puntos sobre la cinta)' if primero.emisor else 'apagado'}"
          f" · {libre:.0f} GB libres · guardando 1 de cada {args.cada}")
    print(f"Grabando en {carpeta}  (Ctrl+C para acabar)")

    salir = {"si": False}
    signal.signal(signal.SIGINT, lambda *_: salir.__setitem__("si", True))
    signal.signal(signal.SIGTERM, lambda *_: salir.__setitem__("si", True))

    escritor = EscritorDataset(carpeta, cola=120)
    yaw0 = robot.imu().yaw
    t0 = time.monotonic()
    recibidos, perdidos, sin_fotograma = 0, 0, 0
    ultimo_n, t_linea = None, 0.0
    f = primero
    while not salir["si"] and time.monotonic() - t0 < args.duracion:
        if f is None:
            sin_fotograma += 1
            print("\n  (sin fotogramas en 1 s: ./camara_servidor.sh estado)")
        else:
            if ultimo_n is not None:
                perdidos += max(0, f.n - ultimo_n - 1)
            ultimo_n = f.n
            if recibidos % args.cada == 0:
                escritor.escribir(f)
            recibidos += 1
        ahora = time.monotonic()
        if ahora - t_linea > 1.0:
            t_linea = ahora
            imu, m = robot.imu(), robot.mando()
            rumbo = math.degrees(envolver(imu.yaw - yaw0))
            sys.stdout.write(f"\r{ahora - t0:6.1f} s · {escritor.escritos:5d} guardados · "
                             f"{recibidos / max(1e-6, ahora - t0):4.1f} fps · {escritor.descartados} descartados · "
                             f"{perdidos} perdidos · rumbo {rumbo:+6.1f} · roll {math.degrees(imu.roll):+4.1f} "
                             f"pitch {math.degrees(imu.pitch):+4.1f}{' · JOYSTICK' if m.ejes_activos() else ''}   ")
            sys.stdout.flush()
        f = fuente.siguiente(timeout_s=1.0)

    duracion = time.monotonic() - t0
    print("\nCerrando (escribiendo lo que queda en la cola)...")
    fuente.cerrar()
    escritor.cerrar()
    reg_low.cerrar()
    resumen = {"nombre": args.nombre, "nota": args.nota, "inicio": os.path.basename(carpeta), "fsm": fsm,
               "duracion_s": round(duracion, 2), "recibidos": recibidos, "guardados": escritor.escritos,
               "descartados_por_disco": escritor.descartados, "perdidos_en_camara": perdidos,
               "segundos_sin_fotograma": sin_fotograma, "fps_medio": round(recibidos / max(1e-6, duracion), 2),
               "cada": args.cada, "flujos": flujos, "emisor": primero.emisor, "meta_camara": primero.meta,
               "geometria": cfg["geometria"], "giro_total_deg": round(math.degrees(envolver(robot.imu().yaw - yaw0)), 1)}
    registro.guardar_json(os.path.join(carpeta, "resumen.json"), resumen)
    print(f"{escritor.escritos} fotogramas guardados en {duracion:.0f} s, {escritor.descartados} descartados "
          f"por disco, {perdidos} perdidos en la camara.\nDatos: {carpeta}/")


if __name__ == "__main__":
    main()
