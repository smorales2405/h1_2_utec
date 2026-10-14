#!/usr/bin/env python3
"""Camina hacia adelante 10 s con el LocoClient de unitree_sdk2py.

SE NIEGA A ARRANCAR si la FSM no esta en 204 (Motion). Sin eso el robot no
camina: SetVelocity se acepta y se ignora, y no avisa de nada.

Uso:  python caminar_10s.py eth0 [vx] [segundos]
      python caminar_10s.py eth0 0.15 2     <- primera prueba recomendada
"""
import re, subprocess, sys, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient

# El LocoClient de python NO tiene GetFsmId (ni ningun getter); solo los tiene
# el de C++. Leemos la FSM con el binario lee_fsm, que es de solo lectura.
LEE_FSM = "/home/unitree/robotics40/lectores_h1_2/build/lee_fsm"

def leer_fsm(iface):
    try:
        out = subprocess.run([LEE_FSM, iface], capture_output=True, text=True,
                             timeout=25).stdout
    except Exception as e:
        print("no se pudo leer la FSM:", e)
        return None
    m = re.search(r"GetFsmId\s+->\s+ret=(-?\d+)\s+valor=(-?\d+)", out)
    if not m or m.group(1) != "0":
        print("lectura de FSM fallida:\n" + out)
        return None
    return int(m.group(2))

FSM_MOTION = 204

def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    vx    = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    dur   = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

    ChannelFactoryInitialize(0, iface)
    c = LocoClient()
    c.SetTimeout(10.0)
    c.Init()

    fsm_id = leer_fsm(iface)
    print(f"FSM actual: {fsm_id}")
    if fsm_id != FSM_MOTION:
        print(f"\n*** ABORTADO: la FSM esta en {fsm_id}, hace falta {FSM_MOTION} (Motion).")
        print("    Secuencia: L2+B -> L2+UP -> bajar la cuerda hasta que los dos pies")
        print("    apoyen -> R2+X -> soltar el gancho. Sin eso no camina.")
        return 1

    print(f"\nvx={vx} m/s durante {dur} s  ->  recorrido nominal {vx*dur:.2f} m")
    print("L2+B es la parada de emergencia. Arrancando en 3 s...")
    time.sleep(3)

    t0 = time.time()
    c.SetVelocity(vx, 0.0, 0.0, dur)
    print("SetVelocity enviado; refrescando para que no expire...")
    # SetVelocity lleva su propia duracion, pero se refresca por si el
    # servicio la trunca a un valor menor.
    while time.time() - t0 < dur:
        time.sleep(0.5)
        c.SetVelocity(vx, 0.0, 0.0, 1.0)

    print("tiempo cumplido -> StopMove()")
    c.StopMove()
    time.sleep(1)
    print("parado.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
