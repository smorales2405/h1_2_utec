#!/usr/bin/env python3
"""Prueba de las dos manos Inspire RH56DFTP del H1-2: abrir y cerrar durante 15 s, registrando todo.

Cada mano en su hilo, con su cliente Modbus TCP (.211 izquierda, .210 derecha). Secuencia por mano:
  1. abrir flexion del pulgar (DOF 4)      2. abrir los 4 dedos (0-3)
  3. rotacion del pulgar a 986 (DOF 5), ya con los dedos abiertos (asi no choca)
  4. ciclos durante 15 s: cerrar dedos -> cerrar pulgar a 500 -> abrir pulgar -> abrir dedos
  5. terminar en puno (dedos 0, pulgar 500, rotacion 986)
Seguridad: FORCE_SET 1000 gf mientras dura (se restaura el valor original), corte si un dedo pasa de 800 gf
(abre ese dedo y aborta la mano), velocidad 400 (se restaura).
Registro: lecturas separadas por muestra (angulo 1546, fuerza 1582, corriente 1594, err 1606, status 1612, temp 1618).\nOJO: un read de bloque 1546..1620 devuelve basura (no es contiguo en el firmware): fuerzas de 11306 gf en las dos manos.
Salida: manos_<fecha>_{izq,der}.csv y un resumen por pantalla.
"""
import csv, struct, sys, threading, time
from pymodbus.client import ModbusTcpClient

ANGLE_SET, FORCE_SET, SPEED_SET = 1486, 1498, 1522
FMAX, F_LIMITE, VEL = 800, 1000, 400
T_CICLOS, PULGAR_ROT, PULGAR_CERRADO = 15.0, 986, 500
MANOS = (("izq", "192.168.124.211"), ("der", "192.168.124.210"))
FECHA = time.strftime("%Y%m%d_%H%M%S")


def s16(regs):
    return list(struct.unpack(f">{len(regs)}h", struct.pack(f">{len(regs)}H", *regs)))


class Mano:
    def __init__(self, nombre, ip):
        self.nombre, self.c = nombre, ModbusTcpClient(ip, port=6000, timeout=2)
        if not self.c.connect():
            raise RuntimeError(f"{nombre}: no conecta con {ip}")
        self.orden = [-1] * 6
        self.filas, self.eventos, self.t0 = [], [], time.time()
        self.force_orig = s16(self.c.read_holding_registers(FORCE_SET, 6, 1).registers)
        self.speed_orig = s16(self.c.read_holding_registers(SPEED_SET, 6, 1).registers)
        self.c.write_registers(FORCE_SET, [F_LIMITE] * 6, 1)
        self.c.write_registers(SPEED_SET, [VEL] * 6, 1)

    def evento(self, txt):
        self.eventos.append((round(time.time() - self.t0, 2), txt))

    def leer(self, fase):
        rd = lambda a, n: self.c.read_holding_registers(a, n, 1).registers
        ang, fza, cor = s16(rd(1546, 6)), s16(rd(1582, 6)), s16(rd(1594, 6))
        b = lambda a: list(struct.pack(">3H", *rd(a, 3)))
        err, sta, tmp = b(1606), b(1612), b(1618)
        self.filas.append([round(time.time() - self.t0, 3), fase] + self.orden + ang + fza + cor + err + sta + tmp)
        return ang, fza

    def mandar(self, dofs, valor):
        v = [-1] * 6
        for d in dofs:
            v[d] = int(valor); self.orden[d] = int(valor)
        self.c.write_registers(ANGLE_SET, [x & 0xFFFF for x in v], 1)

    def mover(self, dofs, valor, fase, espera=1.2, vigilar=(0, 1, 2, 3)):
        """Manda y registra hasta que llegan (o pasa `espera`). Corta por fuerza."""
        self.mandar(dofs, valor)
        t = time.time()
        while time.time() - t < espera:
            ang, fza = self.leer(fase)
            malos = [d for d in vigilar if fza[d] > FMAX]
            if malos:
                self.mandar(malos, 1000)
                raise RuntimeError(f"{self.nombre}: dedos {malos} con {[fza[d] for d in malos]} gf en '{fase}'")
            if all(abs(ang[d] - valor) < 15 for d in dofs) and time.time() - t > 0.3:
                break
            time.sleep(0.01)

    def restaurar(self):
        try:
            self.c.write_registers(FORCE_SET, [x & 0xFFFF for x in self.force_orig], 1)
            self.c.write_registers(SPEED_SET, [x & 0xFFFF for x in self.speed_orig], 1)
        finally:
            self.c.close()


def prueba(nombre, ip, res):
    m = None
    try:
        m = Mano(nombre, ip)
        m.leer("inicio"); m.evento(f"inicio angulos {m.filas[-1][8:14]}")
        m.mover([4], 1000, "abrir_pulgar")
        m.mover([0, 1, 2, 3], 1000, "abrir_dedos", 2.0)
        m.mover([5], PULGAR_ROT, "rotar_pulgar", 2.0, vigilar=(0, 1, 2, 3, 4))
        m.evento(f"preparada: {m.filas[-1][8:14]}")
        n, t = 0, time.time()
        while time.time() - t < T_CICLOS:
            m.mover([0, 1, 2, 3], 0, "cerrar_dedos", 1.5)
            m.mover([4], PULGAR_CERRADO, "cerrar_pulgar", 1.0, vigilar=(0, 1, 2, 3, 4))
            m.mover([4], 1000, "abrir_pulgar", 1.0)
            m.mover([0, 1, 2, 3], 1000, "abrir_dedos", 1.5)
            n += 1
        m.evento(f"{n} ciclos en {time.time() - t:.1f} s")
        m.mover([0, 1, 2, 3], 0, "puno_dedos", 1.5)
        m.mover([4], PULGAR_CERRADO, "puno_pulgar", 1.0, vigilar=(0, 1, 2, 3, 4))
        for _ in range(20):
            m.leer("final"); time.sleep(0.02)
        m.evento(f"final angulos {m.filas[-1][8:14]}")
        res[nombre] = ("ok", n)
    except Exception as e:
        res[nombre] = ("PARADA", str(e))
        if m: m.evento(f"PARADA: {e}")
    finally:
        if m:
            m.restaurar()
            with open(f"manos_{FECHA}_{nombre}.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "fase"] + [f"cmd{i}" for i in range(6)] + [f"ang{i}" for i in range(6)]
                           + [f"fza{i}" for i in range(6)] + [f"cor{i}" for i in range(6)]
                           + [f"err{i}" for i in range(6)] + [f"sta{i}" for i in range(6)] + [f"tmp{i}" for i in range(6)])
                w.writerows(m.filas)
            for ev in m.eventos:
                print(f"[{nombre}] {ev[0]:6.2f} s  {ev[1]}", flush=True)
            print(f"[{nombre}] {len(m.filas)} muestras -> manos_{FECHA}_{nombre}.csv", flush=True)


res = {}
hilos = [threading.Thread(target=prueba, args=(n, ip, res)) for n, ip in MANOS]
[h.start() for h in hilos]; [h.join() for h in hilos]
print("RESULTADO:", res)
