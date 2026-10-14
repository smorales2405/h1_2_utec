#!/usr/bin/env python3
"""Cierra SOLO la flexion del pulgar hasta que su sensor tactil detecta contacto.

  pulgar_tactil.py <ip>

Lee las 4 zonas tactiles del pulgar (registros de inspire_hand_defaut.py:
punta 4480, yema 4498, medio 4690, base 4708), mide el reposo 1 s, y cierra el
DOF 4 a pasos de PASO unidades. Para al ver un taxel que sube mas de UMBRAL
sobre su reposo (o sobre 6x el ruido, lo que sea mayor), aprieta APRIETE mas y
se queda. Aborta si FORCE_ACT[4] > FMAX. No baja de MINIMO. No escribe SAVE.
"""
import struct, sys, time
import numpy as np
from pymodbus.client import ModbusTcpClient

ANGLE_SET, SPEED_SET, ANGLE_ACT, FORCE_ACT = 1486, 1522, 1546, 1582
HOLD, NDOF, DOF = -1, 6, 4
ZONAS = [("punta", 4480, 9), ("yema", 4498, 96), ("medio", 4690, 9), ("base", 4708, 96)]
PALMA = ("palma", 4900, 112)
PASO, DT, APRIETE, MINIMO = 10, 0.15, 20, 100
UMBRAL, FMAX = 60, 800

c = ModbusTcpClient(sys.argv[1], port=6000, timeout=3)
if not c.connect(): sys.exit("no conecta")
def regs(a, n, signo=True):
    r = c.read_holding_registers(a, n, 1)
    f = "h" if signo else "H"
    return np.array(struct.unpack(f">{n}{f}", struct.pack(f">{n}H", *r.registers)), dtype=float)
def tactil():
    return np.concatenate([regs(a, n, False) for _, a, n in ZONAS])
def mover(v):
    x = [HOLD] * NDOF; x[DOF] = int(v)
    c.write_registers(ANGLE_SET, [i & 0xFFFF for i in x], 1)

a0 = regs(ANGLE_ACT, 6); f0 = regs(FORCE_ACT, 6)
print(f"inicial ANGLE={a0.astype(int).tolist()} FORCE={f0.astype(int).tolist()}", flush=True)
muestras = []
for _ in range(10):
    muestras.append(tactil()); time.sleep(0.1)
M = np.array(muestras); base = M.mean(0); ruido = M.std(0).max()
umbral = max(UMBRAL, 6 * ruido)
palma0 = regs(*PALMA[1:], signo=False)
print(f"tactil pulgar en reposo: max={base.max():.0f} media={base.mean():.1f} ruido={ruido:.1f} -> umbral +{umbral:.0f}", flush=True)
print(f"palma en reposo: max={palma0.max():.0f} suma={palma0.sum():.0f} (si la caja apoya en la palma, sube)", flush=True)

c.write_registers(SPEED_SET, [150] * NDOF, 1)
objetivo = a0[DOF]; motivo = None
while objetivo - PASO >= MINIMO:
    objetivo -= PASO
    mover(objetivo); time.sleep(DT)
    t = tactil(); d = t - base; f = regs(FORCE_ACT, 6)[DOF]
    if f > FMAX:
        motivo = f"FUERZA {f:.0f} gf > {FMAX}"; objetivo = min(1000, objetivo + 150); mover(objetivo); break
    if d.max() > umbral:
        k = int(d.argmax()); ini = 0
        for nombre, _, n in ZONAS:
            if k < ini + n:
                motivo = f"contacto tactil en '{nombre}' (taxel {k-ini}, +{d.max():.0f}), fuerza {f:.0f} gf"; break
            ini += n
        objetivo = max(MINIMO, objetivo - APRIETE); mover(objetivo); break
time.sleep(1.2)
a = regs(ANGLE_ACT, 6); f = regs(FORCE_ACT, 6); t = tactil() - base
print(f"parada: {motivo or 'llego al minimo sin contacto'}", flush=True)
print(f"final ANGLE={a.astype(int).tolist()} FORCE={f.astype(int).tolist()}", flush=True)
ini = 0
for nombre, _, n in ZONAS:
    z = t[ini:ini+n]; ini += n
    print(f"  pulgar {nombre:6s}: taxeles activos {int((z > umbral).sum()):3d}/{n}  max +{z.max():.0f}", flush=True)
p = regs(*PALMA[1:], signo=False) - palma0
print(f"  palma       : taxeles activos {int((p > umbral).sum()):3d}/112  max +{p.max():.0f}", flush=True)
c.close()
