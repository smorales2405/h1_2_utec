#!/usr/bin/env python3
"""Pruebas que NO necesitan robot. Comprueban lo que se puede comprobar sin él.

    source scripts/env.sh
    python3 scripts/99_selftest.py

Cubre: la tabla de articulaciones contra el URDF, el CRC tabulado contra la
implementación literal, las métricas contra señales de las que se conoce la
respuesta, y la carga de gains.yaml. Si esto pasa y aun así el robot no se
mueve, el problema está en el robot o en el canal, no aquí.
"""
from __future__ import annotations

import math
import random
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from h1_2_joint_control import config as cfg
from h1_2_joint_control import crc as crcmod
from h1_2_joint_control import metrics as mt
from h1_2_joint_control import trajectories as tr
from h1_2_joint_control.joints import (ARM_INDICES, ARM_SDK_INDICES, BY_INDEX,
                                       BY_NAME, JOINTS, resolve)

URDF = Path.home() / "humanoid_ws/src/h1_2_utec/h1_2_description/urdf/h1_2_handless.urdf"

_fails: list[str] = []


def check(name: str, ok: bool, detail: str = ""):
    print(f"  {'✔' if ok else '✗'} {name}" + (f"   {detail}" if detail else ""))
    if not ok:
        _fails.append(name)


def t_joints():
    print("\nTabla de articulaciones")
    check("27 motores comandables", len(JOINTS) == 27, f"{len(JOINTS)}")
    check("índices 0..26 sin huecos",
          [j.idx for j in JOINTS] == list(range(27)))
    check("14 motores de brazo, 13..26", ARM_INDICES == tuple(range(13, 27)))
    check("arm_sdk cede 15 (14 brazo + cintura)",
          len(ARM_SDK_INDICES) == 15 and 12 in ARM_SDK_INDICES)
    check("resolve('arms') == los 14", resolve("arms") == list(ARM_INDICES))
    check("resolve por grupo", len(resolve("wrist")) == 6)
    check("resolve rechaza lo desconocido",
          _raises(lambda: resolve("no_existe"), ValueError))
    check("clamp respeta los topes",
          BY_NAME["L_wrist_pitch"].clamp(10.0) == BY_NAME["L_wrist_pitch"].q_max
          and BY_NAME["L_wrist_pitch"].clamp(-10.0) == BY_NAME["L_wrist_pitch"].q_min)

    if URDF.exists():
        import xml.etree.ElementTree as ET
        lim = {}
        for j in ET.parse(URDF).getroot().findall("joint"):
            l = j.find("limit")
            if l is not None:
                lim[j.get("name")] = (float(l.get("lower")), float(l.get("upper")),
                                      float(l.get("effort")), float(l.get("velocity")))
        bad = [j.name for j in JOINTS
               if j.urdf in lim and not _close(lim[j.urdf],
                                               (j.q_min, j.q_max, j.tau_max, j.dq_max))]
        missing = [j.name for j in JOINTS if j.urdf not in lim]
        check("topes coinciden con el URDF", not bad and not missing,
              f"{len(lim)} articulaciones leídas" if not bad and not missing
              else f"discrepan: {bad}  no encontradas: {missing}")
    else:
        check("URDF disponible para contrastar", False, f"no está {URDF}")


def t_crc():
    print("\nCRC de Unitree")
    random.seed(1234)
    ok = True
    for _ in range(200):
        w = [random.getrandbits(32) for _ in range(random.randint(1, 60))]
        if crcmod.crc32_words(w) != crcmod.crc32_reference(w):
            ok = False
            break
    check("tabulado == referencia (200 casos aleatorios)", ok)
    check("una palabra a cero",
          crcmod.crc32_words([0]) == crcmod.crc32_reference([0]))
    try:
        from unitree_hg.msg import LowCmd
    except ImportError:
        check("mensajes unitree_hg disponibles", False,
              "falta 'source scripts/env.sh'")
        return
    m = LowCmd()
    buf = crcmod.serialize(m)
    check("serializa 1000 bytes", len(buf) == crcmod.CRC_BYTES, f"{len(buf)}")
    # 4262932383 es lo que daba la implementación literal de
    # test_mandar_modificado.py, ya validada contra el robot real.
    check("CRC del mensaje vacío == el histórico",
          crcmod.set_crc(m) == 4262932383, str(crcmod.set_crc(m)))
    m.motor_cmd[16].q = 0.5
    m.motor_cmd[16].kp = 50.0
    check("CRC cambia al cambiar el mensaje", crcmod.set_crc(m) == 655003232)

    # tau va ANTES de kp: si el layout estuviera mal, intercambiarlos daría
    # el mismo CRC. Esta prueba lo detectaría.
    a, b = LowCmd(), LowCmd()
    a.motor_cmd[16].tau, a.motor_cmd[16].kp = 1.0, 2.0
    b.motor_cmd[16].tau, b.motor_cmd[16].kp = 2.0, 1.0
    check("el orden tau/kp/kd importa (layout correcto)",
          crcmod.set_crc(a) != crcmod.set_crc(b))

    n, t0 = 200, time.perf_counter()
    for _ in range(n):
        crcmod.set_crc(m)
    dt = (time.perf_counter() - t0) / n
    check("rápido para 250 Hz", dt < 0.002,
          f"{dt*1e3:.2f} ms/mensaje -> techo {1/dt:.0f} Hz")


def t_metrics():
    print("\nMétricas")
    fs = 250.0
    t = np.arange(0, 4.0, 1 / fs)
    rms, peak = mt._hf_rms(np.sin(2 * np.pi * 20 * t), fs, 8.0)
    check("energía de alta frecuencia: seno de 20 Hz, A=1",
          abs(rms - 0.7071) < 0.02 and abs(peak - 20.0) < 0.5,
          f"rms={rms:.4f} (esperado 0.707), pico={peak:.1f} Hz")
    rms2, _ = mt._hf_rms(np.sin(2 * np.pi * 2 * t), fs, 8.0)
    check("un seno de 2 Hz no cuenta como temblor", rms2 < 0.01, f"rms={rms2:.5f}")

    # Escalón de un sistema de 2º orden del que se conoce la respuesta:
    # zeta = 0.5 -> sobreimpulso teórico exp(-pi*z/sqrt(1-z^2)) = 16.3 %
    wn, z, amp = 2 * np.pi * 2.0, 0.5, 0.2
    t_step, tt = 0.5, np.arange(0, 4.0, 1 / fs)
    wd = wn * math.sqrt(1 - z * z)
    resp = np.where(tt < t_step, 0.0, 0.0)
    tau_ = np.clip(tt - t_step, 0, None)
    resp = amp * (1 - np.exp(-z * wn * tau_) *
                  (np.cos(wd * tau_) + z / math.sqrt(1 - z * z) * np.sin(wd * tau_)))
    resp[tt < t_step] = 0.0

    class S:
        pass
    samples = []
    for k, tk in enumerate(tt):
        s = S()
        s.t, s.weight = float(tk), 1.0
        s.q_des = {0: amp if tk >= t_step else 0.0}
        s.q = {0: float(resp[k])}
        s.dq = {0: 0.0}
        s.tau = {0: 0.0}
        samples.append(s)
    st = mt.step_response(samples, 0, 100.0, 2.0, t_step)
    check("sobreimpulso de un 2º orden con zeta=0.5",
          abs(st.overshoot - 0.163) < 0.02,
          f"{st.overshoot*100:.1f} % (teórico 16.3 %)")
    check("error final ~0 sin gravedad", abs(st.steady_error) < 1e-3,
          f"{st.steady_error:.2e}")
    check("cuenta oscilaciones sin inventárselas", 1 <= st.oscillations <= 6,
          f"{st.oscillations}")


def t_trajectories():
    print("\nTrayectorias")
    q, dq = tr.step(0.2, 0.5)(0.4)
    check("escalón: 0 antes del salto", q == 0.0)
    check("escalón: amplitud después", tr.step(0.2, 0.5)(0.6)[0] == 0.2)
    f = tr.smooth_step(0.2, 0.3, 0.5)
    check("escalón suave: velocidad nula en los extremos",
          abs(f(0.5)[1]) < 1e-9 and abs(f(0.8)[1]) < 1e-9)
    check("escalón suave: llega a la amplitud", abs(f(0.9)[0] - 0.2) < 1e-9)
    # la velocidad del seno debe ser la derivada de su posición
    s = tr.sine(0.1, 0.5)
    h = 1e-5
    num = (s(1.0 + h)[0] - s(1.0 - h)[0]) / (2 * h)
    check("seno: dq es la derivada de q", abs(num - s(1.0)[1]) < 1e-4,
          f"numérica={num:.5f} analítica={s(1.0)[1]:.5f}")
    c = tr.chirp(0.1, 0.2, 3.0, 10.0)
    num = (c(4.0 + h)[0] - c(4.0 - h)[0]) / (2 * h)
    check("chirp: dq es la derivada de q", abs(num - c(4.0)[1]) < 1e-3)
    check("zero_velocity anula dq", tr.zero_velocity(s)(1.0)[1] == 0.0)


def t_config():
    print("\nConfiguración")
    g = cfg.load()
    check("gains.yaml carga", g is not None, f"conjunto activo '{g.set_name}'")
    check("todos los brazos tienen ganancia",
          all(g.for_index(i)[0] > 0 for i in ARM_INDICES))
    off, xr = cfg.load("official_arm_sdk"), cfg.load("xr_teleoperate")
    check("el codo difiere entre oficial y xr_teleoperate",
          off.for_index(16) == (50.0, 1.0) and xr.for_index(16) == (140.0, 3.0),
          f"oficial {off.for_index(16)}  xr {xr.for_index(16)}")
    check("las piernas caen en legs_hold", off.for_index(3) == (300.0, 5.0))
    check("los tobillos tienen su excepción", off.for_index(4) == (140.0, 3.0))
    check("conjunto inexistente da error",
          _raises(lambda: cfg.load("no_existe"), KeyError))
    s = g.safety
    check("topes de seguridad razonables",
          0 < s.tau_abort_fraction <= 1 and s.temperature_abort > 50,
          f"par<{s.tau_abort_fraction*100:.0f}%  T<{s.temperature_abort:.0f}°C")


def _raises(fn, exc):
    try:
        fn()
    except exc:
        return True
    except Exception:
        return False
    return False


def _close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def main() -> int:
    print("Pruebas sin robot\n" + "=" * 60)
    t_joints()
    t_crc()
    t_metrics()
    t_trajectories()
    t_config()
    print("\n" + "=" * 60)
    if _fails:
        print(f"✗ {len(_fails)} fallo(s): " + ", ".join(_fails))
        return 1
    print("✔ todo correcto.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
