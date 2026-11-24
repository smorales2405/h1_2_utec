"""Minimal standalone Modbus I/O for the Inspire RH56DFTP characterization.

No PyQt. Single client; block reads/writes with signed int16 decoding, mirroring
the proven pattern in core/hand_connection.py but kept OUTSIDE the GUI per the
characterization rules. Reused by exp1/exp2 (exp0 stays self-contained).

Register block start addresses come from the manual (RH56DFTP). Each block holds
6 shorts (one per DOF); DOF k is element [k] of the read/written list. Reading a
whole 6-short block at the start address and indexing the DOF is the pattern
that is verified to work on this hand.
"""
from __future__ import annotations

import csv
import struct
import time

from pymodbus.client import ModbusTcpClient, ModbusSerialClient

# ── Register map (block start addr; 6 shorts each) ────────────────────────
ANGLE_SET = 1486   # W    -1 (no-op / hold), 0-1000
FORCE_SET = 1498   # W    0-3000  (g, force threshold)
SPEED_SET = 1522   # W    0-1000
POS_ACT   = 1534   # R    0-2000  (actuator position, fine resolution)
ANGLE_ACT = 1546   # R    0-1000
FORCE_ACT = 1582   # R    -4000..4000  (g, signed)
CURRENT   = 1594   # R    0-2000  (mA)
TEMP      = 1618   # R    temperatura por actuador: 6 regs (1618..1623), 1 temp/reg, 0-100 C
#                        (manual sec. 2.6.19 dice "6 bytes" pero la lectura cruda de 1618..1623
#                         da 6 valores plausibles -> 1 valor/registro, NO byte-empaquetado.
#                         1618 meñique, 1619 ring, 1620 medio, 1621 indice, 1622/1623 pulgar)

NDOF = 6
G_TO_N = 9.80665 / 1000.0   # grams-force -> Newton

# ── Rango físico por DOF (manual RH56DFTP, secc. 2.6.11, p. 21) ───────────
DOF_NAMES = [
    "Meñique", "Anular", "Medio", "Índice",
    "Pulgar (flex.)", "Pulgar (rot.)",
]

# Rango angular absoluto de cada DOF, tal como lo publica el manual.
DOF_DEG_RANGE = [
    (20.0, 176.0),   # 0-3: dedos            "20°-176°"
    (20.0, 176.0),
    (20.0, 176.0),
    (20.0, 176.0),
    (-13.0, 70.0),   # 4: flexión del pulgar "-13°-70°"
    (90.0, 165.0),   # 5: rotación del pulgar "90°-165°"
]

# Correspondencia (grados en ANGLE_SET=0, grados en ANGLE_SET=1000).
# Regla única, confirmada en los 6 DOF: **ANGLE_SET 1000 = extremo ABIERTO =
# ángulo MAYOR del rango; ANGLE_SET 0 = extremo CERRADO = ángulo MENOR.**
# Evidencia:
#   · dedos      — el manual dice «ANGLE_ACT(3) = 1000 (i.e. fully open)» y las
#                  campañas del Exp 1 lo verifican (1000 abre, 0 cierra, POS_ACT
#                  crece al cerrar) → 1000 = extendido = 176°.
#   · pulgar rot.— medido con pose_check.py (2026-08-25): ANGLE_SET 1000 deja la
#                  rotación ABIERTA (lateral) y 0 la lleva al tope de cierre
#                  (oposición). En la figura del manual (p. 22/27) β se mide
#                  desde el plano metacarpiano: 90° = pulgar perpendicular al
#                  plano = máxima OPOSICIÓN; 165° = casi tendido en el plano =
#                  ABIERTO. Luego 1000 = 165° y 0 = 90°.
#   · pulgar flex.— misma regla + la figura del manual (p. 21/27): θ crece al
#                  extender, luego 1000 = 70° (extendido) y 0 = -13° (flexionado).
#                  Confírmalo de vista en la fase P0.2 del RUNBOOK_pulgar.
#
#   ⚠ Interfaz/core/angle_converter.py usa la dirección INVERTIDA (0 = abierto)
#     y un rango distinto para la flexión del pulgar (53.6° vs los 70° del
#     manual). Esa tabla solo afecta a los grados que MUESTRA la GUI, no al
#     control; no se replica aquí.
DOF_DEG_ENDPOINTS = [
    (20.0, 176.0),   # 0
    (20.0, 176.0),   # 1
    (20.0, 176.0),   # 2
    (20.0, 176.0),   # 3: verificado en la campaña del Exp 1
    (-13.0, 70.0),   # 4: flexión del pulgar   (0 = flexionado, 1000 = extendido)
    (90.0, 165.0),   # 5: rotación del pulgar  (0 = opuesto/cerrado, 1000 = abierto)
]


def reg_to_deg(reg, dof):
    """ANGLE_SET/ANGLE_ACT (0-1000) -> grados, o None si la dirección no está medida."""
    ep = DOF_DEG_ENDPOINTS[dof]
    if ep is None:
        return None
    a0, a1000 = ep
    return round(a0 + max(0, min(1000, reg)) / 1000.0 * (a1000 - a0), 1)


def deg_to_reg(deg, dof):
    """Grados -> ANGLE_SET (0-1000). Falla si la dirección del DOF no está medida."""
    ep = DOF_DEG_ENDPOINTS[dof]
    if ep is None:
        lo, hi = DOF_DEG_RANGE[dof]
        raise ValueError(
            f"DOF {dof} ({DOF_NAMES[dof]}): el manual da el rango {lo}°-{hi}° pero NO "
            f"qué extremo de ANGLE_SET corresponde a cada ángulo. Mídelo con "
            f"`pose_check.py --dof {dof}` y usa el valor de registro directamente "
            f"(p. ej. --hold {dof}:0), o completa DOF_DEG_ENDPOINTS en hand_modbus.py.")
    a0, a1000 = ep
    return int(max(0, min(1000, round((deg - a0) / (a1000 - a0) * 1000.0))))


def fmt_angle(reg, dof):
    """'ANGLE_SET 0 ≈ 176.0°' o, si los grados no están medidos, solo el registro."""
    d = reg_to_deg(reg, dof)
    return f"ANGLE_SET {reg}" + (f" ≈ {d}°" if d is not None else " (grados sin medir)")


# ── Mapa medido POS_ACT ↔ ANGLE_SET ───────────────────────────────────────
# La relación NO es lineal (el actuador comprime su avance al flexionar), así
# que interpolar entre dos anclas introduce error. `pose_check.py --csv` deja la
# tabla completa; estas funciones la usan por tramos.

def load_pos_angle_map(path):
    """CSV de pose_check.py -> [(angle_set, pos_act)] ordenado por POS, o None."""
    pts = []
    try:
        with open(path, newline='') as f:
            for r in csv.DictReader(f):
                try:
                    pts.append((int(r['angle_set']), int(r['pos_act'])))
                except (TypeError, ValueError, KeyError):
                    continue
    except OSError:
        return None
    pts.sort(key=lambda t: t[1])
    return pts if len(pts) >= 2 else None


def pos_to_angle(pts, pos):
    """POS_ACT -> ANGLE_SET por interpolación lineal a tramos (extrapola en los bordes)."""
    if not pts or len(pts) < 2:
        return None
    if pos <= pts[0][1]:
        (a0, p0), (a1, p1) = pts[0], pts[1]
    elif pos >= pts[-1][1]:
        (a0, p0), (a1, p1) = pts[-2], pts[-1]
    else:
        for (a0, p0), (a1, p1) in zip(pts, pts[1:]):
            if p0 <= pos <= p1:
                break
    if p1 == p0:
        return None
    return int(max(0, min(1000, round(a0 + (pos - p0) * (a1 - a0) / (p1 - p0)))))


# ── DOF fijados ("hold") ──────────────────────────────────────────────────
# Un experimento sobre un DOF puede exigir que otro quede ANCLADO en un ángulo
# (p. ej. caracterizar la flexión del pulgar (DOF 4) con la rotación (DOF 5)
# fija en su ángulo máximo). El ancla se RE-AFIRMA en cada escritura de
# ANGLE_SET: cuesta cero (el bloque de 6 shorts se escribe igual) y hace que el
# ancla sobreviva a cualquier apertura global del script.

def parse_hold(spec):
    """'5:0' o '5:165d' (grados) o varios separados por coma -> {dof: angle_reg}."""
    hold = {}
    for part in (spec or '').split(','):
        part = part.strip()
        if not part:
            continue
        d_txt, sep, v_txt = part.partition(':')
        if not sep or not v_txt.strip():
            raise ValueError(f"--hold: entrada inválida '{part}' (usa DOF:ANGLE o DOF:GRADOSd)")
        d = int(d_txt)
        if not (0 <= d < NDOF):
            raise ValueError(f"--hold: DOF {d} fuera de rango 0..{NDOF-1}")
        v_txt = v_txt.strip()
        if v_txt.lower().endswith('d'):
            reg = deg_to_reg(float(v_txt[:-1]), d)
        else:
            reg = int(v_txt)
            if not (0 <= reg <= 1000):
                raise ValueError(f"--hold: ANGLE {reg} fuera de rango 0..1000 (DOF {d})")
        hold[d] = reg
    return hold


def describe_hold(hold):
    if not hold:
        return '—'
    return ' · '.join(f"DOF {d} ({DOF_NAMES[d]}) = {fmt_angle(a, d)}"
                      for d, a in sorted(hold.items()))


def angle_vector(dof, value, hold=None):
    """Bloque ANGLE_SET que mueve `dof`, re-afirma los `hold` y deja el resto en -1."""
    v = [-1] * NDOF
    for d, a in (hold or {}).items():
        v[d] = a
    v[dof] = value          # el DOF bajo prueba siempre manda
    return v


def open_vector(open_angle, hold=None):
    """Bloque ANGLE_SET que abre TODOS los dedos salvo los `hold` (que se mantienen)."""
    v = [open_angle] * NDOF
    for d, a in (hold or {}).items():
        v[d] = a
    return v


def hold_only_vector(hold):
    """Bloque ANGLE_SET que solo comanda los `hold` (el resto queda como esté)."""
    v = [-1] * NDOF
    for d, a in (hold or {}).items():
        v[d] = a
    return v


def preposition_hold(hand, hold, band=8, timeout_s=4.0, speed=300, settle_s=0.15):
    """Lleva los DOF anclados a su ángulo y espera a que `ANGLE_ACT` los alcance.

    Debe correrse UNA vez al inicio de cada sesión: fija la postura de referencia
    del experimento. Devuelve (ok, {dof: angle_act_medido}).
    """
    if not hold:
        return True, {}
    hand.write_block(SPEED_SET, [speed] * NDOF)
    hand.write_block(ANGLE_SET, hold_only_vector(hold))
    t0 = time.perf_counter()
    last = {}
    while time.perf_counter() - t0 < timeout_s:
        a = hand.read_block(ANGLE_ACT)
        if a is not None:
            last = {d: a[d] for d in hold}
            if all(abs(a[d] - hold[d]) <= band for d in hold):
                time.sleep(settle_s)
                a2 = hand.read_block(ANGLE_ACT)
                if a2 is not None:
                    last = {d: a2[d] for d in hold}
                return True, last
        time.sleep(0.02)
    return False, last


def report_hold(hand, hold, band=8, timeout_s=4.0, speed=300):
    """preposition_hold + impresión legible (grados incluidos). Devuelve ok."""
    if not hold:
        return True
    print(f"Anclando DOF fijados: {describe_hold(hold)}")
    ok, act = preposition_hold(hand, hold, band, timeout_s, speed)
    for d in sorted(hold):
        a = act.get(d)
        if a is None:
            print(f"  DOF {d}: ANGLE_ACT no leído")
        else:
            print(f"  DOF {d} ({DOF_NAMES[d]}): ANGLE_ACT={a}"
                  + (f" ≈ {reg_to_deg(a, d)}°" if reg_to_deg(a, d) is not None else "")
                  + f"  (objetivo {fmt_angle(hold[d], d)})")
    if not ok:
        print("  ⚠ el ancla NO asentó dentro del timeout: verifica que el DOF pueda "
              "alcanzar ese ángulo (¿tope mecánico o colisión?) antes de seguir.")
    return ok


class HandModbus:
    """Single-client Modbus wrapper: signed block reads and writes."""

    def __init__(self, client, device_id=1):
        self.client = client
        self.device_id = device_id

    # ── Connection ────────────────────────────────────────────────────
    @classmethod
    def open_tcp(cls, ip, port, device_id=1, timeout=1.0):
        c = ModbusTcpClient(ip, port=port, timeout=timeout)
        return cls(c, device_id) if c.connect() else None

    @classmethod
    def open_serial(cls, port, baud, device_id=1, timeout=1.0):
        c = ModbusSerialClient(port=port, baudrate=baud, timeout=timeout)
        return cls(c, device_id) if c.connect() else None

    # ── Block I/O ─────────────────────────────────────────────────────
    def read_block(self, addr, count=NDOF):
        """Read `count` regs as signed int16. Returns list[int] or None."""
        try:
            r = self.client.read_holding_registers(addr, count, self.device_id)
        except Exception:
            return None
        if r.isError():
            return None
        packed = struct.pack('>' + 'H' * count, *r.registers)
        return list(struct.unpack('>' + 'h' * count, packed))

    def read_temps(self):
        """Temperatura de los 6 actuadores (C, 0-100) o None.

        El bloque TEMP llega en DOS formatos segun el transporte, verificado en
        esta unidad:
          · RS-485  -> 1 temperatura por registro (1618..1623). Las campanas
                       serial registraron 42-44 C asi.
          · TCP     -> byte-empaquetado, 2 temperaturas por registro y los 3
                       ultimos a cero: 1618=0x1818 -> 24 C y 24 C. Es el formato
                       que describe el manual (sec. 2.6.19, "6 bytes").
        No hay flag: se decide por el VALOR. Una temperatura valida cabe en
        0-100, asi que cualquier registro por encima de 100 solo puede ser un par
        empaquetado. Decidirlo por transporte seria fragil; por rango, no.
        """
        raw = self.read_block(TEMP, NDOF)
        if raw is None:
            return None
        if all(0 <= v <= 100 for v in raw):
            return raw
        out = []
        for v in raw[:NDOF // 2]:
            u = v & 0xFFFF
            out.extend((u & 0xFF, (u >> 8) & 0xFF))
        return out

    def write_block(self, addr, values):
        """Write signed ints as uint16 (so -1 -> 0xFFFF). Returns bool ok."""
        regs = [int(v) & 0xFFFF for v in values]
        try:
            r = self.client.write_registers(addr, regs, self.device_id)
        except Exception:
            return False
        return not r.isError()

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass
