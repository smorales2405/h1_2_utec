#!/usr/bin/env python3
"""Manda una orden al teleop lanzado con --ipc:  enviar.py r (empezar) | q (salir) | estado"""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "xr_teleoperate"))
from teleop.utils.ipc import IPC_Client
c = IPC_Client(); time.sleep(1.0)
arg = sys.argv[1] if len(sys.argv) > 1 else "estado"
if arg in ("r", "q"):
    print("respuesta:", c.send_data({"r": "CMD_START", "q": "CMD_STOP"}[arg]))
    time.sleep(0.5)
print("en linea:", c.is_online(), " estado:", c.latest_state())
c.stop()
