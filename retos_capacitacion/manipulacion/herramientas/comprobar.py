#!/usr/bin/env python3
"""Comprobacion previa a cada tirada (seccion 10 del PDF), SOLO LECTURA: no mueve nada ni escribe en las manos.

    ./ejecutar.sh herramientas/comprobar.py
Mira: rt/lowstate (frecuencia, motores, mode_machine, IMU), FSM 201, la camara (flujos, fps, extrinseco), las
dos manos por Modbus (angulos, fuerzas, errores; con la mano abierta y quieta deberian ser ~1000 y < 100 gf), y
que existan la calibracion, el montaje, las poses de los brazos y la tabla de objetos.
"""

import math
import os
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from manip import calibracion as cb  # noqa: E402
from manip import config  # noqa: E402
from manip import mano as mm  # noqa: E402

BIEN, MAL, AVISO = "  bien ", "  MAL  ", "  aviso"


def leer_mano(ip, puerto):
    from pymodbus.client import ModbusTcpClient
    c = ModbusTcpClient(ip, port=puerto, timeout=1)
    if not c.connect():
        return None
    try:
        sal = {}
        for nombre, a in (("angulo", mm.ANGLE_ACT), ("fuerza", mm.FORCE_ACT), ("error", mm.ERROR), ("estado", mm.ESTADO),
                          ("force_set", mm.FORCE_SET)):
            if a in mm.BLOQUES_BYTE:            # un byte por DOF en 3 registros
                sal[nombre] = mm.bytes_de(c.read_holding_registers(a, 3, 1).registers)
                continue
            r = c.read_holding_registers(a, 6, 1)
            sal[nombre] = list(struct.unpack(">6h", struct.pack(">6H", *r.registers)))
        return sal
    finally:
        c.close()


def main():
    cfg = config.cargar()
    malos = 0
    print("Robot")
    from manip.robot import FSM_VALIDA, Robot
    robot = Robot(cfg["red"]["iface"])
    if not robot.esperar_lowstate(3.0):
        print(MAL, "no llega rt/lowstate")
        sys.exit(1)
    n0 = robot.n_lowstate
    time.sleep(1.0)
    hz = robot.n_lowstate - n0
    e = robot.estado()
    print(BIEN if hz > 300 else MAL, f"rt/lowstate a {hz} Hz, mode_machine {e.mode_machine}")
    malos += hz <= 300
    print(BIEN if not e.motores_en_fallo else MAL, f"motores en fallo: {e.motores_en_fallo or 'ninguno'}")
    malos += bool(e.motores_en_fallo)
    print(BIEN, f"IMU roll {math.degrees(e.imu.roll):+.1f}, pitch {math.degrees(e.imu.pitch):+.1f}, yaw {math.degrees(e.imu.yaw):+.1f} grados")
    fsm, _ = robot.leer_fsm()
    print(BIEN if fsm == FSM_VALIDA else MAL, f"FSM {fsm} (tiene que ser {FSM_VALIDA})")
    malos += fsm != FSM_VALIDA
    print("Camara")
    from manip.fuentes import FuenteZmq
    f0 = FuenteZmq(cfg["red"]["zmq_fotogramas"], solo_ultimo=False, cola=60)
    primero = f0.fresco(timeout_s=5.0)
    if primero is None:
        print(MAL, "no llegan fotogramas: ./camara_servidor.sh")
        malos += 1
    else:
        t0, n = time.monotonic(), 0
        while time.monotonic() - t0 < 2.0:
            n += f0.siguiente(1.0) is not None
        flujos = [k for k in ("ir", "color", "prof") if getattr(primero, k) is not None]
        print(BIEN if {"color", "prof"} <= set(flujos) else MAL, f"flujos {flujos}, {n / 2:.0f} fps, emisor {primero.emisor}")
        malos += not ({"color", "prof"} <= set(flujos))
        print(BIEN if "prof_a_color" in primero.meta else AVISO, "extrinseco profundidad -> color en los metadatos")
    print("Manos (solo lectura)")
    for lado, ip in cfg["red"]["manos"].items():
        r = leer_mano(ip, cfg["red"]["puerto_manos"])
        if r is None:
            print(MAL, f"{lado} ({ip}): no conecta")
            malos += 1
            continue
        abierta = min(r["angulo"][:5]) > 900
        fabrica = "  (de fabrica: manipulacion.py lo baja a 800 al empezar)" if min(r["force_set"]) >= 2000 else ""
        print(BIEN if not any(r["error"]) else MAL, f"{lado} ({ip}): angulos {r['angulo']}, fuerzas {r['fuerza']} gf, "
              f"errores {r['error']}, estado {r['estado']}, FORCE_SET {r['force_set']}{fabrica}"
              f"{'' if abierta else '  (no esta abierta)'}")
        malos += any(r["error"])
    print("Configuracion")
    ruta = config.ruta_calibracion()
    if ruta:
        c = cb.cargar(ruta)
        print(BIEN if c.origen == "calibrada" else AVISO, f"calibracion {os.path.basename(ruta)}: {c.origen}, "
              f"residuo {c.residuo_px:.2f} px / {c.residuo_mm:.1f} mm con {c.n_poses} poses")
    else:
        print(AVISO, "sin calibracion camara-torso: solo --simulacro")
    for nombre in ("montaje.yaml", "objetos.yaml", "poses_brazos.yaml", "poses_calibracion.yaml"):
        existe = os.path.exists(os.path.join(config.CONFIG, nombre))
        print(BIEN if existe else MAL, f"config/{nombre}")
        malos += not existe
    asign = {k: v.get("bandeja") for k, v in config.cargar_objetos()["objetos"].items()}
    print(BIEN if all(asign.values()) else AVISO, f"bandejas asignadas: {asign}")
    print("\n" + ("Todo bien." if malos == 0 else f"{malos} problema(s): no empezar la tirada."))
    sys.exit(1 if malos else 0)


if __name__ == "__main__":
    main()
