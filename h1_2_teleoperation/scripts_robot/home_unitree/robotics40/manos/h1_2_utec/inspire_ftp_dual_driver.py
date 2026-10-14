#!/usr/bin/env python3
"""
Driver headless de las DOS manos Inspire RH56DFTP para el H1-2.

Hace de puente Modbus <-> DDS: lee el estado de cada mano por Modbus y lo
publica en DDS, y escribe en la mano los comandos que llegan por DDS.

    rt/inspire_hand/ctrl/l   <- comandos  (inspire::inspire_hand_ctrl)
    rt/inspire_hand/state/l  -> estado    (inspire::inspire_hand_state)
    rt/inspire_hand/touch/l  -> tactil    (inspire::inspire_hand_touch)   [solo TCP]
    ... e igual con /r para la mano derecha

Son exactamente los topicos que consume `Inspire_Controller_FTP` de
xr_teleoperate (teleop/robot_control/robot_hand_inspire.py), asi que este
programa es lo que hay que tener corriendo en el PC2 antes de lanzar
`teleop_hand_and_arm.py --ee inspire_ftp` en la laptop.

Dos transportes:

  tcp     Cada mano con su propia IP (Modbus TCP, puerto 6000). Es el unico
          modo que entrega los 17 sensores tactiles. Un proceso por mano.
  serial  Ambas manos en el mismo bus RS485 (/dev/ttyUSB0), distinguidas por
          device id. NO publica datos tactiles: el bus no da el ancho de banda.

Ejemplos:
    # Modbus TCP. En este H1-2 las manos cuelgan del bridge br0 (192.168.124.0/24)
    # y las IP por defecto ya llevan la lateralidad correcta (ver LEFT_IP/RIGHT_IP).
    python3 inspire_ftp_dual_driver.py --transport tcp --network eth0

    # Medido en este robot: 274 Hz por mano sin tactil, 35 Hz con tactil.
    # Para teleoperar (comandos a 100 Hz) --no-touch da holgura de sobra.

    # RS485, ambas manos en /dev/ttyUSB0 (id 2 = izquierda, id 1 = derecha)
    python3 inspire_ftp_dual_driver.py --transport serial \
        --serial-port /dev/ttyUSB0 --left-id 2 --right-id 1 --network eth0

    # Solo estado, sin tactil, para maximizar la frecuencia del lazo
    python3 inspire_ftp_dual_driver.py --transport tcp --no-touch
"""
import argparse
import multiprocessing as mp
import signal
import sys
import time

# ── Asignacion izquierda/derecha ─────────────────────────────────────────────
# ¡OJO! En ESTE H1-2 va al REVES de los ejemplos de Unitree. Comprobado
# visualmente el 2026-09-01 moviendo un dedo a la vez y preguntando cual se
# movia (perspectiva del propio robot):
#
#     192.168.124.210  ->  mano DERECHA   ->  topicos rt/inspire_hand/*/r
#     192.168.124.211  ->  mano IZQUIERDA ->  topicos rt/inspire_hand/*/l
#
# Las dos manos reportan HAND_ID = 1 y no hay registro Modbus que distinga
# lateralidad, asi que esto SOLO se puede verificar mirando. Si se cambia una
# mano de sitio, hay que repetir la comprobacion:
#     python3 hand_test.py wiggle <ip> --dof 3
# Invertirlo espeja la teleoperacion: la mano izquierda del operador moveria
# la derecha del robot.
LEFT_IP = "192.168.124.211"
RIGHT_IP = "192.168.124.210"

# Registros de estado. Menos registros => lazo mas rapido.
# El controlador FTP de xr_teleoperate solo usa `angle_act`, asi que el
# conjunto minimo basta para teleoperar; el resto sirve para diagnostico.
STATES_MIN = [
    ("angle_act", 1546, 6, "short"),
    ("status", 1612, 3, "byte"),
]
STATES_FULL = [
    ("pos_act", 1534, 6, "short"),
    ("angle_act", 1546, 6, "short"),
    ("force_act", 1582, 6, "short"),
    ("current", 1594, 6, "short"),
    ("err", 1606, 3, "byte"),
    ("status", 1612, 3, "byte"),
    ("temperature", 1618, 3, "byte"),
]


def init_dds(network):
    """Inicializa CycloneDDS en el dominio 0, opcionalmente atado a una NIC.

    Se hace aqui a proposito. `inspire_sdkpy` trae invertida la condicion
    (llama a ChannelFactoryInitialize(0, network) cuando network es None, y a
    ChannelFactoryInitialize(0) sin la interfaz cuando SI se le pasa una), asi
    que el argumento --network se le caeria. Inicializamos por nuestra cuenta y
    a los handlers se les pasa initDDS=False.
    """
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    if network:
        ChannelFactoryInitialize(0, network)
    else:
        ChannelFactoryInitialize(0)


def worker_tcp(name, ip, port, lr, device_id, network, states, want_touch, report_every):
    """Un proceso por mano: Modbus TCP -> DDS."""
    from inspire_sdkpy import inspire_sdk

    init_dds(network)
    handler = inspire_sdk.ModbusDataHandler(
        ip=ip, port=port, LR=lr, device_id=device_id,
        states_structure=states, initDDS=False,
    )
    if not want_touch:
        # read() publica el tactil solo si use_serial es False; forzarlo a True
        # despues de construir salta ese bloque sin tocar el transporte, que ya
        # quedo creado como cliente TCP.
        handler.use_serial = True

    print(f"[{name}] listo  ip={ip}:{port}  topico=/{lr}  tactil={'si' if want_touch else 'no'}", flush=True)
    loop(handler, name, report_every)


def worker_serial(serial_port, baudrate, left_id, right_id, network, states, report_every):
    """Un solo proceso: ambas manos sobre el mismo bus RS485 -> DDS."""
    from inspire_sdkpy import inspire_sdk_double

    init_dds(network)
    handler = inspire_sdk_double.ModbusDataHandlerDouble(
        use_serial=True, serial_port=serial_port, baudrate=baudrate,
        device_id=[left_id, right_id],  # [izquierda, derecha]
        states_structure=states, initDDS=False,
    )
    print(f"[rs485] listo  {serial_port}@{baudrate}  ids L={left_id} R={right_id}  "
          f"(sin datos tactiles en RS485)", flush=True)
    loop(handler, "rs485", report_every)


def loop(handler, name, report_every):
    n, t0 = 0, time.perf_counter()
    while True:
        handler.read()          # lee por Modbus y publica el estado en DDS
        n += 1
        time.sleep(0.001)
        if report_every and n % report_every == 0:
            dt = time.perf_counter() - t0
            print(f"[{name}] {n / dt:6.1f} Hz  ({n} lecturas en {dt:.1f} s)", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--transport", choices=["tcp", "serial"], default="tcp",
                    help="tcp = una IP por mano (con tactil); serial = ambas en un bus RS485 (sin tactil)")
    ap.add_argument("--left-ip", default=LEFT_IP, help="IP Modbus TCP de la mano IZQUIERDA")
    ap.add_argument("--right-ip", default=RIGHT_IP, help="IP Modbus TCP de la mano DERECHA")
    ap.add_argument("--port", type=int, default=6000, help="puerto Modbus TCP")
    ap.add_argument("--left-id", type=int, default=2, help="device id Modbus de la mano izquierda")
    ap.add_argument("--right-id", type=int, default=1, help="device id Modbus de la mano derecha")
    ap.add_argument("--serial-port", default="/dev/ttyUSB0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--network", default=None,
                    help="NIC para DDS (p.ej. eth0). Sin esto, CycloneDDS elige sola.")
    ap.add_argument("--full-state", action="store_true",
                    help="publicar todos los registros de estado (fuerza, corriente, temperatura...) y no solo angle_act")
    ap.add_argument("--no-touch", action="store_true",
                    help="no publicar los sensores tactiles (sube la frecuencia del lazo)")
    ap.add_argument("--hand", choices=["both", "left", "right"], default="both",
                    help="arrancar solo una mano (util para diagnostico)")
    ap.add_argument("--report-every", type=int, default=500,
                    help="cada cuantas lecturas imprimir la frecuencia; 0 la silencia")
    args = ap.parse_args()

    states = STATES_FULL if args.full_state else STATES_MIN

    if args.transport == "serial":
        if args.hand != "both":
            ap.error("--hand solo aplica al transporte tcp; en RS485 el driver maneja el bus completo")
        worker_serial(args.serial_port, args.baudrate, args.left_id, args.right_id,
                      args.network, states, args.report_every)
        return

    # TCP: un proceso por mano. Cada uno abre su cliente Modbus y su DDS.
    want_touch = not args.no_touch
    specs = []
    if args.hand in ("both", "right"):
        specs.append(("derecha", args.right_ip, "r", args.right_id))
    if args.hand in ("both", "left"):
        specs.append(("izquierda", args.left_ip, "l", args.left_id))

    procs = []
    for name, ip, lr, dev in specs:
        p = mp.Process(target=worker_tcp,
                       args=(name, ip, args.port, lr, dev, args.network,
                             states, want_touch, args.report_every),
                       name=name, daemon=False)
        p.start()
        procs.append(p)
        time.sleep(0.6)   # separa las inicializaciones DDS

    def shutdown(signum, frame):
        print("\nCerrando driver...", flush=True)
        for p in procs:
            p.terminate()
        for p in procs:
            p.join(timeout=3)
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    while True:
        for p in procs:
            if not p.is_alive():
                print(f"[error] el proceso '{p.name}' murio (exitcode={p.exitcode}); "
                      f"cerrando el resto", flush=True)
                shutdown(None, None)
        time.sleep(1.0)


if __name__ == "__main__":
    main()
