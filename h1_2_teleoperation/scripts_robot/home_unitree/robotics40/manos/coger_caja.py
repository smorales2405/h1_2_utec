#!/usr/bin/env python3
"""Agarre de una caja con UNA mano Inspire RH56DFTP, parando cada dedo al tocar.

  coger_caja.py preforma <ip>   mano abierta y pulgar girado enfrente de los dedos
  coger_caja.py cerrar   <ip>   cierra dedos + flexion del pulgar; cada DOF se
                                congela al pasar CONTACTO gf y aprieta APRIETE mas
  coger_caja.py soltar   <ip>   abre dedos y pulgar (la rotacion se queda)

Protecciones (como hand_test.py de h1_2_utec): SPEED_SET bajo, -1 (no-op) en los
DOF que no se mueven, aborto y apertura si |fuerza| > FMAX, nunca escribe SAVE.
DOF: 0 menique, 1 anular, 2 medio, 3 indice, 4 pulgar-flex, 5 pulgar-rot.
0 = cerrado, 1000 = abierto.
"""
import struct, sys, time
from pymodbus.client import ModbusTcpClient

ANGLE_SET, SPEED_SET, ANGLE_ACT, FORCE_ACT, ERROR_REG = 1486, 1522, 1546, 1582, 1606
HOLD, OPEN, NDOF = -1, 1000, 6
VEL = 150
CONTACTO = 250     # gf: a partir de aqui el dedo ha tocado la caja
APRIETE = 60       # unidades de angulo que se cierra de mas tras el contacto
FMAX = 1000        # gf: aborto
ROT_OPUESTO = 100  # pulgar-rot: girado enfrente de los dedos

c = None
def leer(a):
    r = c.read_holding_registers(a, NDOF, 1)
    return list(struct.unpack(">6h", struct.pack(">6H", *r.registers)))
def escribir(a, v):
    c.write_registers(a, [int(x) & 0xFFFF for x in v], 1)
def mover(d):  # d = {dof: angulo}
    v = [HOLD] * NDOF
    for k, x in d.items(): v[k] = int(x)
    escribir(ANGLE_SET, v)
def errores():
    r = c.read_holding_registers(ERROR_REG, 3, 1); o = []
    for x in r.registers: o += [(x >> 8) & 0xFF, x & 0xFF]
    return o[:NDOF]
def estado(t):
    print(f"  {t}: ANGLE={leer(ANGLE_ACT)} FORCE={leer(FORCE_ACT)} ERR={errores()}", flush=True)

def rampa_simple(dofs, objetivo, pasos=15, dt=0.12):
    ini = leer(ANGLE_ACT)
    for i in range(1, pasos + 1):
        mover({d: ini[d] + (objetivo - ini[d]) * i / pasos for d in dofs})
        time.sleep(dt)
    time.sleep(1.5)

def cerrar():
    dofs = [0, 1, 2, 3, 4]
    ini = leer(ANGLE_ACT)
    obj = {d: 0 for d in dofs}
    activos = set(dofs); congelado = {}
    pasos, dt = 40, 0.10
    for i in range(1, pasos + 1):
        f = leer(FORCE_ACT); a = leer(ANGLE_ACT)
        if max(abs(x) for x in f[:5]) > FMAX:
            print(f"  ABORTO: fuerza {f} > {FMAX} gf -> abriendo", flush=True)
            mover({d: OPEN for d in dofs}); return False
        for d in list(activos):
            if f[d] > CONTACTO:
                congelado[d] = max(0, a[d] - APRIETE)
                activos.discard(d)
                print(f"  contacto DOF {d}: angulo {a[d]}, fuerza {f[d]} gf -> se queda en {congelado[d]}", flush=True)
        cmd = {d: ini[d] + (obj[d] - ini[d]) * i / pasos for d in activos}
        cmd.update(congelado)
        mover(cmd)
        if not activos:
            break
        time.sleep(dt)
    if activos:
        print(f"  sin contacto en DOF {sorted(activos)}: llegaron al cierre total", flush=True)
    time.sleep(1.5)
    return True

if __name__ == "__main__":
    modo, ip = sys.argv[1], sys.argv[2]
    c = ModbusTcpClient(ip, port=6000, timeout=3)
    if not c.connect(): sys.exit(f"no conecta con {ip}")
    estado("inicial")
    escribir(SPEED_SET, [VEL] * NDOF)
    if modo == "preforma":
        rampa_simple([0, 1, 2, 3, 4], OPEN)
        rampa_simple([5], ROT_OPUESTO)
    elif modo == "cerrar":
        cerrar()
    elif modo == "soltar":
        rampa_simple([4], OPEN); rampa_simple([0, 1, 2, 3], OPEN)
    estado("final")
    c.close()
