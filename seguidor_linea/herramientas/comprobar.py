#!/usr/bin/env python3
"""Comprobacion previa, SOLO LECTURA: robot (rt/lowstate, FSM, motores, IMU, mando) y camara
(fotogramas por ZMQ). No crea el LocoClient de marcha ni publica nada en el DDS.

    ./ejecutar.sh herramientas/comprobar.py
    ./ejecutar.sh herramientas/comprobar.py --sin-camara

Sale con codigo 1 si algo impide una tirada (sin lowstate, motores en fallo, FSM distinta de
201/204, inclinacion, mando tocado o camara sin fotogramas).
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config, mando  # noqa: E402
from seguidor.fuentes import FuenteZmq  # noqa: E402
from seguidor.robot import FSM_MARCHA, Robot  # noqa: E402


def comprobar_robot(cfg, problemas):
    robot = Robot(cfg["red"]["iface"])
    if not robot.esperar_lowstate(3.0):
        problemas.append(f"no llega rt/lowstate por {robot.iface}")
        return
    n0, t0 = robot.n_lowstate, time.monotonic()
    time.sleep(2.0)
    hz = (robot.n_lowstate - n0) / (time.monotonic() - t0)
    msg, _ = robot.ultimo()
    imu = robot.imu()
    fallos = robot.motores_en_fallo()
    m = robot.mando()
    print(f"rt/lowstate   : {hz:.0f} Hz, mode_machine={msg.mode_machine}, mode_pr={msg.mode_pr}")
    print(f"motores       : {'27/27 sin fallo' if not fallos else f'EN FALLO {fallos}'}, T max {robot.temp_max()} C")
    print(f"IMU           : roll {math.degrees(imu.roll):+.1f}, pitch {math.degrees(imu.pitch):+.1f}, "
          f"yaw {math.degrees(imu.yaw):+.1f} grados")
    print(f"mando         : botones {mando.pulsados(m) or 'ninguno'}, ejes lx {m.lx:+.2f} ly {m.ly:+.2f} "
          f"rx {m.rx:+.2f} ry {m.ry:+.2f}")
    if hz < 400:
        problemas.append(f"rt/lowstate a {hz:.0f} Hz (se esperan ~500)")
    if fallos:
        problemas.append(f"motores en fallo {fallos}")
    lim = cfg["supervisor"]["inclinacion_max_deg"]
    if max(abs(math.degrees(imu.roll)), abs(math.degrees(imu.pitch))) > lim:
        problemas.append(f"inclinacion por encima de {lim} grados")
    if m.ejes_activos():
        problemas.append("un eje del mando no esta a cero: el joystick manda sobre Move")
    boton = cfg["supervisor"].get("boton_parada")
    print(f"boton parada  : {boton or 'sin elegir (supervisor.boton_parada en el YAML)'}")

    fsm, texto = robot.leer_fsm()
    print(f"FSM           : {fsm}")
    if fsm not in FSM_MARCHA:
        problemas.append(f"FSM {fsm}: para andar tiene que ser {FSM_MARCHA} (L2+UP en el mando)"
                         + ("" if fsm is not None else f" — {texto[:120]}"))


def comprobar_camara(cfg, problemas, segundos=2.0):
    fuente = FuenteZmq(cfg["red"]["zmq_fotogramas"], solo_ultimo=False, cola=120)
    primero = fuente.siguiente(timeout_s=3.0)
    if primero is None:
        problemas.append("no llegan fotogramas: ./camara_servidor.sh (y ./camara_servidor.sh log)")
        return
    n, perdidos, edades = 1, 0, []
    ultimo = primero
    t0 = time.monotonic()
    while time.monotonic() - t0 < segundos:
        f = fuente.siguiente(timeout_s=1.0)
        if f is None:
            problemas.append("la camara dejo de dar fotogramas")
            break
        edades.append(time.monotonic() - f.t_rx)
        perdidos += max(0, f.n - ultimo.n - 1)
        ultimo, n = f, n + 1
    fps = (n - 1) / max(1e-6, ultimo.t_rx - primero.t_rx)
    flujos = [k for k in ("ir", "color", "prof") if getattr(ultimo, k) is not None]
    ir = ultimo.meta.get("ir", {})
    print(f"camara        : {fps:.1f} fps, flujos {flujos}, emisor {'si' if ultimo.emisor else 'no'}, "
          f"{perdidos} fotogramas perdidos")
    if edades:
        print(f"retraso ZMQ   : mediana {1000 * sorted(edades)[len(edades) // 2]:.1f} ms (servidor -> este proceso)")
    print(f"intrinsecos IR: fx {ir.get('fx', 0):.1f} fy {ir.get('fy', 0):.1f} cx {ir.get('cx', 0):.1f} cy {ir.get('cy', 0):.1f}")
    if fps < 0.8 * cfg["camara"]["fps"]:
        problemas.append(f"camara a {fps:.1f} fps (se esperan {cfg['camara']['fps']})")
    fuente.cerrar()


def main():
    ap = argparse.ArgumentParser(description="Comprobacion previa del seguidor (solo lectura)")
    ap.add_argument("--sin-camara", action="store_true")
    ap.add_argument("--sin-robot", action="store_true")
    args = ap.parse_args()
    cfg = config.cargar()
    problemas = []
    if not args.sin_robot:
        comprobar_robot(cfg, problemas)
    if not args.sin_camara:
        comprobar_camara(cfg, problemas)
    geo = cfg["geometria"]
    print(f"geometria     : {'calibrada' if geo.get('calibrada') else 'SIN CALIBRAR'} "
          f"(altura {geo['altura_m']:.3f} m, inclinacion {geo['inclinacion_deg']:.1f} grados)")
    if problemas:
        print("\nNO APTO:\n  - " + "\n  - ".join(problemas))
        sys.exit(1)
    print("\nAPTO: nada impide la tirada desde el lado del software. L2+B en la mano.")


if __name__ == "__main__":
    main()
