#!/usr/bin/env python3
"""Marcha atras replicando el mecanismo de la opcion 3 del ejemplo oficial,
con vx negativo. Cada Move() es SetVelocity(..., duration=1), y dormimos 1 s
entre llamadas: N iteraciones = N segundos. Si se deja de mandar, para en 1 s.

Uso: atras.py <iface> <vx_negativo> <iteraciones>
"""
import sys, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient

iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
vx    = float(sys.argv[2]) if len(sys.argv) > 2 else -0.3
n     = int(sys.argv[3]) if len(sys.argv) > 3 else 10

ChannelFactoryInitialize(0, iface)
c = LocoClient(); c.SetTimeout(10.0); c.Init()
print(f"MARCHA ATRAS: vx={vx} m/s, {n} iteraciones de 1 s (~{abs(vx)*n:.2f} m)")
print("L2+B para abortar.\n")
for i in range(1, n + 1):
    c.Move(vx, 0.0, 0.0)          # == SetVelocity(vx,0,0, duration=1)
    print(f"  Move({vx},0,0)  {i}/{n}")
    time.sleep(1)
print("\nordenes cesadas; el robot para solo en ~1 s.")
