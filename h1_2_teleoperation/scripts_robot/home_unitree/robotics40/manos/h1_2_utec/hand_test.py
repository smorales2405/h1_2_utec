#!/usr/bin/env python3
"""
Prueba controlada de las manos Inspire RH56DFTP. Movimientos lentos, un DOF a la
vez, con vigilancia de fuerza y restauracion del estado inicial.

Salvaguardas:
  * SPEED_SET bajo (por defecto 150/1000) antes de mover nada. La caracterizacion
    de este modelo mostro que el sobreimpulso de fuerza lo domina la velocidad de
    cierre, asi que se mueve despacio a proposito.
  * ANGLE_SET se escribe con -1 en todos los DOF que no se tocan: -1 es no-op y
    deja ese actuador exactamente como esta.
  * Vigilancia de |FORCE_ACT| del DOF en movimiento. Al superar el umbral se
    aborta y se manda abrir ese dedo.
  * El cierre nunca llega al tope: se para en `--target` (500 por defecto).
  * Al terminar, por Ctrl-C o por excepcion, el DOF vuelve a donde estaba.
  * Nunca se escribe el registro SAVE (1005): ningun cambio queda persistido.

Subcomandos:
    read      solo lectura, no mueve nada
    wiggle    mueve UN dedo: abierto -> --target -> abierto, N veces
    sweep     recorre los 6 DOF uno por uno, para identificar el orden
    open      abre todos los dedos despacio
    forceclb  abre la mano y tara el cero del sensor de fuerza (reg 1009)

Ejemplos:
    python3 hand_test.py read   192.168.124.210
    python3 hand_test.py wiggle 192.168.124.210 --dof 3 --cycles 3
    python3 hand_test.py sweep  192.168.124.210
    python3 hand_test.py open   192.168.124.211 --dof 1,2
"""
import argparse
import struct
import sys
import time

from pymodbus.client import ModbusTcpClient

ANGLE_SET, SPEED_SET = 1486, 1522
ANGLE_ACT, POS_ACT, FORCE_ACT, CURRENT = 1546, 1534, 1582, 1594
ERROR_REG = 1606
FORCE_CLB = 1009        # escribir 1 = tarar el cero de fuerza; exige palma abierta y libre
NDOF = 6
OPEN, HOLD = 1000, -1          # 1000 = extremo abierto; -1 = no-op

DOF_NAME = ["meñique", "anular", "medio", "índice", "pulgar-flexión", "pulgar-rotación"]


class Hand:
    def __init__(self, ip, port=6000, dev=1, timeout=3):
        self.ip, self.dev = ip, dev
        self.c = ModbusTcpClient(ip, port=port, timeout=timeout)
        if not self.c.connect():
            raise SystemExit(f"✗ no conecta con {ip}:{port}")

    def _read(self, addr, n=NDOF):
        r = self.c.read_holding_registers(addr, n, self.dev)
        if r.isError():
            raise RuntimeError(f"error leyendo {addr}")
        return list(struct.unpack(">" + "h" * n, struct.pack(">" + "H" * n, *r.registers)))

    def _write(self, addr, values):
        r = self.c.write_registers(addr, [int(v) & 0xFFFF for v in values], self.dev)
        if r.isError():
            raise RuntimeError(f"error escribiendo {addr}")

    angles = property(lambda s: s._read(ANGLE_ACT))
    forces = property(lambda s: s._read(FORCE_ACT))
    pos = property(lambda s: s._read(POS_ACT))
    current = property(lambda s: s._read(CURRENT))

    def errors(self):
        r = self.c.read_holding_registers(ERROR_REG, 3, self.dev)
        out = []
        for reg in r.registers:
            out += [(reg >> 8) & 0xFF, reg & 0xFF]
        return out[:NDOF]

    def set_speed(self, speed):
        self._write(SPEED_SET, [speed] * NDOF)

    def move(self, dof, value):
        """Comanda un solo DOF; el resto queda intacto gracias al -1."""
        v = [HOLD] * NDOF
        v[dof] = value
        self._write(ANGLE_SET, v)

    def force_clb(self):
        """Tara el cero del sensor de fuerza. La mano debe estar ABIERTA y sin
        tocar nada: el firmware toma la lectura actual como cero."""
        self._write(FORCE_CLB, [1])

    def close(self):
        self.c.close()


def show(h, titulo):
    print(f"\n  {titulo}")
    print(f"    {'':<16}" + "".join(f"{n[:9]:>11}" for n in DOF_NAME))
    for etiqueta, val in (("ANGLE_ACT", h.angles), ("POS_ACT", h.pos),
                          ("FORCE_ACT (gf)", h.forces), ("CURRENT", h.current),
                          ("ERROR", h.errors())):
        print(f"    {etiqueta:<16}" + "".join(f"{v:>11}" for v in val))


def ramp(h, dof, desde, hasta, pasos, dt, fmax):
    """Lleva un DOF de `desde` a `hasta` en rampa, vigilando la fuerza."""
    for i in range(1, pasos + 1):
        objetivo = int(desde + (hasta - desde) * i / pasos)
        h.move(dof, objetivo)
        time.sleep(dt)
        f = h.forces[dof]
        if abs(f) > fmax:
            print(f"    ⚠ ABORTO: |FORCE_ACT[{dof}]| = {abs(f)} gf > {fmax} gf — abriendo")
            h.move(dof, OPEN)
            time.sleep(1.0)
            return False
    return True


def wiggle(h, dof, target, cycles, fmax, speed):
    inicial = h.angles[dof]
    print(f"\n  ▶ Moviendo SOLO el DOF {dof} = {DOF_NAME[dof]}")
    print(f"    parte de ANGLE_ACT={inicial}, va a {target} y vuelve a {OPEN}, {cycles} vez/veces")
    print(f"    velocidad {speed}/1000 (lenta) · aborta si |fuerza| > {fmax} gf")
    try:
        for k in range(cycles):
            print(f"    ciclo {k+1}/{cycles}: cerrando  ...", flush=True)
            if not ramp(h, dof, OPEN, target, 12, 0.10, fmax):
                return
            time.sleep(0.6)
            print(f"    ciclo {k+1}/{cycles}: abriendo  ...", flush=True)
            if not ramp(h, dof, target, OPEN, 12, 0.10, fmax):
                return
            time.sleep(0.6)
        print(f"    fuerza final: {h.forces[dof]} gf   ANGLE_ACT: {h.angles[dof]}")
    finally:
        h.move(dof, OPEN if inicial > 500 else inicial)
        time.sleep(0.8)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["read", "wiggle", "sweep", "open", "forceclb"])
    ap.add_argument("ip")
    ap.add_argument("--dof", default="3", help="DOF a mover (0-5), o lista '1,2'")
    ap.add_argument("--target", type=int, default=500, help="cierre maximo, 0-1000 (500 = medio)")
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--speed", type=int, default=150, help="SPEED_SET 0-1000; bajo = lento y seguro")
    ap.add_argument("--fmax", type=int, default=800, help="umbral de aborto por fuerza, gf")
    ap.add_argument("--device-id", type=int, default=1)
    a = ap.parse_args()

    dofs = [int(x) for x in a.dof.split(",")]
    for d in dofs:
        if not 0 <= d < NDOF:
            ap.error(f"--dof fuera de rango: {d}")
    if not 0 <= a.target <= 1000:
        ap.error("--target debe estar en 0..1000")

    h = Hand(a.ip, dev=a.device_id)
    print(f"=== Mano {a.ip} ===")
    try:
        show(h, "estado inicial")
        if a.cmd == "read":
            return

        h.set_speed(a.speed)
        print(f"\n  SPEED_SET = {a.speed}/1000 en los 6 DOF (solo en RAM, no se guarda)")

        if a.cmd == "forceclb":
            print("\n  ▶ abriendo los 6 DOF antes de tarar (la tara toma la lectura actual como cero)")
            for d in range(NDOF):
                ramp(h, d, h.angles[d], OPEN, 12, 0.10, 3000)
            time.sleep(1.0)
            print(f"    fuerzas antes de tarar: {h.forces}")
            print("  ▶ escribiendo FORCE_CLB (reg 1009) = 1 ...")
            h.force_clb()
            time.sleep(2.5)
            print(f"    fuerzas despues de tarar: {h.forces}")
        elif a.cmd == "open":
            for d in dofs:
                print(f"  ▶ abriendo DOF {d} = {DOF_NAME[d]} (fuerza actual {h.forces[d]} gf)")
                ramp(h, d, h.angles[d], OPEN, 15, 0.12, a.fmax)
                time.sleep(0.5)
        elif a.cmd == "wiggle":
            for d in dofs:
                wiggle(h, d, a.target, a.cycles, a.fmax, a.speed)
        elif a.cmd == "sweep":
            for d in range(NDOF):
                wiggle(h, d, a.target, 1, a.fmax, a.speed)
                time.sleep(1.2)

        show(h, "estado final")
    except KeyboardInterrupt:
        print("\n  interrumpido — abriendo los DOF tocados")
        for d in dofs:
            h.move(d, OPEN)
        time.sleep(1.0)
    finally:
        h.close()


if __name__ == "__main__":
    sys.exit(main())
