#!/usr/bin/env python3
"""Ordenes sueltas a las manos Inspire, con la clase Mano de caja_cuadrado.py (SPEED_SET 150, rampas).

    mano.py leer         izq|der|ambas   angulos, fuerzas, err, status y temperatura (solo lectura)
    mano.py puno         izq|der|ambas   pulgar fuera, dedos a 0, pulgar a 500 (aborta si algo aprieta)
    mano.py abrir        izq|der|ambas   pulgar y dedos abiertos (la rotacion del pulgar no se toca)
    mano.py borrar-error izq|der|ambas   escribe 1 en el registro 1004 (p. ej. tras un err 5 / status 7)

Los registros se leen de uno en uno: leer 1546..1620 de un bloque devuelve basura (sesion 12).
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import caja_cuadrado as cc  # noqa: E402

IPS = {"izq": "192.168.124.211", "der": "192.168.124.210"}
DOF = ["menique", "anular", "medio", "indice", "pulgar", "rot_pulgar"]


def leer(m):
    ang, fza = m.angulos().astype(int).tolist(), m.fuerzas().astype(int).tolist()
    bytes3 = lambda a: [int(x) & 0xFF for r in m.regs(a, 3, False).astype(int) for x in (r >> 8, r)]
    err, sta, tmp = bytes3(1606), bytes3(1612), bytes3(1618)
    for i, n in enumerate(DOF):
        print(f"    {n:10s} angulo {ang[i]:5d}  fuerza {fza[i]:5d} gf  err {err[i]:3d}  status {sta[i]:3d}  {tmp[i]:3d} C")


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ("leer", "puno", "abrir", "borrar-error"):
        print(__doc__); sys.exit(1)
    orden, cual = sys.argv[1], sys.argv[2]
    manos = ["izq", "der"] if cual == "ambas" else [cual]
    for lado in manos:
        if lado not in IPS:
            sys.exit(f"mano desconocida: {lado} (izq, der o ambas)")
        print(f"[{lado}] {IPS[lado]}: {orden}", flush=True)
        if orden == "leer":
            # sin Mano(): su __init__ escribe SPEED_SET, y leer no debe escribir nada
            import threading
            from pymodbus.client import ModbusTcpClient
            m = cc.Mano.__new__(cc.Mano)
            m.c, m.cerrojo = ModbusTcpClient(IPS[lado], port=6000, timeout=3), threading.Lock()
            if not m.c.connect():
                sys.exit(f"no conecta con la mano {IPS[lado]}")
        else:
            m = cc.Mano(IPS[lado])
        try:
            if orden == "leer":
                leer(m)
            elif orden == "puno":
                m.puno()
            elif orden == "abrir":
                m.soltar()
            elif orden == "borrar-error":
                m.escribir(1004, [1])
            if orden != "leer":
                leer(m)
        finally:
            m.c.close()


if __name__ == "__main__":
    main()
