#!/usr/bin/env python3
"""Sonda SOLO LECTURA de las manos Inspire RH56DFTP. No escribe ningun registro."""
import struct, sys
from pymodbus.client import ModbusTcpClient

REGS = [("POS_ACT", 1534, 6, "short"), ("ANGLE_ACT", 1546, 6, "short"),
        ("FORCE_ACT", 1582, 6, "short"), ("CURRENT", 1594, 6, "short"),
        ("ERROR", 1606, 3, "byte"), ("STATUS", 1612, 3, "byte"),
        ("TEMP", 1618, 3, "byte")]
DOF = ["meñique", "anular", "medio", "índice", "pulg-flex", "pulg-rot"]
STATUS = {0: "soltando", 1: "agarrando", 2: "parado por posición",
          3: "parado por fuerza", 5: "parado por corriente", 6: "atascado",
          7: "fallo actuador", 255: "error"}

def read(c, addr, n, kind, dev):
    r = c.read_holding_registers(addr, n, dev)
    if r.isError():
        return None
    if kind == "short":
        return list(struct.unpack(">" + "h" * n, struct.pack(">" + "H" * n, *r.registers)))
    out = []
    for reg in r.registers:
        out += [(reg >> 8) & 0xFF, reg & 0xFF]
    return out

for ip in sys.argv[1:]:
    print(f"\n=== Mano en {ip}:6000 ===")
    c = ModbusTcpClient(ip, port=6000, timeout=3)
    if not c.connect():
        print("  ✗ no conecta"); continue
    hid = read(c, 1000, 1, "short", 1)
    print(f"  HAND_ID (reg 1000) : {hid[0] if hid else '?'}")
    data = {}
    for name, addr, n, kind in REGS:
        data[name] = read(c, addr, n, kind, 1)
    print(f"  {'DOF':<11}" + "".join(f"{d:>12}" for d in DOF))
    for name in ("ANGLE_ACT", "POS_ACT", "FORCE_ACT", "CURRENT", "TEMP"):
        v = data[name]
        print(f"  {name:<11}" + ("".join(f"{x:>12}" for x in v[:6]) if v else "  (sin dato)"))
    err, st = data["ERROR"], data["STATUS"]
    print(f"  {'ERROR':<11}" + ("".join(f"{x:>12}" for x in err[:6]) if err else ""))
    if st:
        print(f"  {'STATUS':<11}" + "".join(f"{STATUS.get(x,x):>12}"[:12] for x in st[:6]))
    if err:
        print("  ⇒ " + ("sin fallos" if not any(err[:6]) else f"¡FALLOS! {err[:6]}"))
    if data["ANGLE_ACT"]:
        a = data["ANGLE_ACT"]
        print(f"  ⇒ apertura: {'abierta' if min(a[:4])>800 else 'cerrada' if max(a[:4])<200 else 'intermedia'}"
              f"  (0=cerrado, 1000=abierto)")
    c.close()
