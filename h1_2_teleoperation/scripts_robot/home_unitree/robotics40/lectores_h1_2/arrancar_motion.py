#!/usr/bin/env python3
"""Start() -> FSM 204 (Motion). Imprime el retorno, que el ejemplo oficial tira."""
import re, subprocess, sys, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
LEE = "/home/unitree/robotics40/lectores_h1_2/build/lee_fsm"
def fsm(iface):
    out = subprocess.run([LEE, iface], capture_output=True, text=True, timeout=25).stdout
    m = re.search(r"GetFsmId\s+->\s+ret=(-?\d+)\s+valor=(-?\d+)", out)
    return int(m.group(2)) if m and m.group(1) == "0" else None
iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
ChannelFactoryInitialize(0, iface)
c = LocoClient(); c.SetTimeout(10.0); c.Init()
print("FSM antes:", fsm(iface))
r = c.Start()
print(f"Start() -> ret = {r} {'(OK)' if r == 0 else '(ERROR)'}")
for i in range(1, 6):
    time.sleep(2)
    f = fsm(iface)
    print(f"  +{i*2}s  FSM = {f}")
    if f == 204:
        print("\n*** EN MOTION (204) ***")
        break
