#!/usr/bin/env python3
"""SOLO LECTURA: pregunta al MotionSwitcher que servicio de movimiento esta activo (CheckMode).

    check_mode.py [iface]

name ''   -> Debug de verdad: el robot obedece rt/lowcmd.
name 'ai' -> el servicio de Unitree sigue activo: ignora rt/lowcmd SIN dar error, aunque nadie publique en
             rt/lowcmd (visto el 2026-09-25 con FSM 0). Para liberarlo hace falta ReleaseMode() (suelta las piernas).
No llama a ReleaseMode ni a SelectMode: no cambia nada.
"""
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

ChannelFactoryInitialize(0, sys.argv[1] if len(sys.argv) > 1 else "eth0")
c = MotionSwitcherClient()
c.SetTimeout(3.0)
c.Init()
status, result = c.CheckMode()
nombre = (result or {}).get("name")
print(f"CheckMode     -> ret={status}  {result}")
if status != 0:
    print("  sin respuesta del MotionSwitcher")
elif nombre:
    print(f"  servicio '{nombre}' ACTIVO: NO es Debug, los rt/lowcmd se ignoran (rt/arm_sdk si vale en FSM 201)")
else:
    print("  servicio liberado: modo Debug, el robot obedece rt/lowcmd (las piernas no se sostienen solas)")
