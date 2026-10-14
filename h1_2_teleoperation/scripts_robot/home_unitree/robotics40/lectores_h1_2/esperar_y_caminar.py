#!/usr/bin/env python3
"""Espera a que la FSM llegue a 204 (Motion) y ENTONCES camina.

El disparo lo das tu con R2+X en el mando. Mientras la FSM no sea 204 no manda
nada. Si se agota la ventana, sale sin haber tocado el robot.

Uso: esperar_y_caminar.py <iface> <vx> <segundos_caminando> <ventana_s>
"""
import re, subprocess, sys, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient

LEE = "/home/unitree/robotics40/lectores_h1_2/build/lee_fsm"
MOTION = 204

def fsm(iface):
    try:
        out = subprocess.run([LEE, iface], capture_output=True, text=True,
                             timeout=25).stdout
    except Exception:
        return None
    m = re.search(r"GetFsmId\s+->\s+ret=(-?\d+)\s+valor=(-?\d+)", out)
    return int(m.group(2)) if m and m.group(1) == "0" else None

def main():
    iface  = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    vx     = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    dur    = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
    window = float(sys.argv[4]) if len(sys.argv) > 4 else 180.0

    ChannelFactoryInitialize(0, iface)
    c = LocoClient(); c.SetTimeout(10.0); c.Init()

    print(f"VIGILANTE ARMADO. Esperando FSM={MOTION} hasta {window:.0f} s.")
    print(f"Cuando pulses R2+X: vx={vx} m/s durante {dur} s (~{vx*dur:.2f} m).")
    print("L2+B para abortar en cualquier momento.\n")

    t0 = time.time()
    visto = None
    while time.time() - t0 < window:
        f = fsm(iface)
        if f != visto:
            print(f"  [{time.time()-t0:5.1f}s] FSM = {f}")
            visto = f
        if f == MOTION:
            print("\n*** 204 DETECTADO. Caminando en 3 s... ***")
            time.sleep(3)
            ini = time.time()
            c.SetVelocity(vx, 0.0, 0.0, dur)
            while time.time() - ini < dur:
                time.sleep(0.5)
                g = fsm(iface)
                if g != MOTION:
                    print(f"  !! la FSM salio de Motion ({g}) -> StopMove y aborto")
                    c.StopMove(); return 2
                c.SetVelocity(vx, 0.0, 0.0, 1.0)
                print(f"  caminando {time.time()-ini:4.1f}/{dur:.0f} s  FSM={g}")
            print("\ntiempo cumplido -> StopMove()")
            c.StopMove(); time.sleep(1)
            print("parado.")
            return 0
        time.sleep(2)

    print(f"\nventana agotada, la FSM nunca llego a {MOTION}. No se envio nada.")
    return 1

if __name__ == "__main__":
    sys.exit(main())
