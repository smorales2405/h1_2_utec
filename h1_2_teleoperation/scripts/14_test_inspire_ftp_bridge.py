#!/usr/bin/env python3
"""
Prueba del puente FTP de las manos del simulador, SIN arrancar Isaac Sim.

Ejercita `dds/inspire_ftp_dds.py` (el parche de este repo a unitree_sim_isaaclab)
contra el `inspire_sdkpy` de verdad, que es lo que usa `Inspire_Controller_FTP`
de xr_teleoperate. Comprueba cuatro cosas:

  1. El IDL vendorizado (dds/inspire_ftp_idl.py) y el de inspire_sdkpy son el
     MISMO tipo para Cyclone DDS: se publica con uno y se lee con el otro.
  2. El estado del simulador sale por rt/inspire_hand/state/{l,r} con la escala
     correcta (0..1000, 1000 = abierto) y la mano correcta en cada topico.
  3. Un comando en rt/inspire_hand/ctrl/{l,r} vuelve a angulos articulares de
     Isaac Lab, con la lateralidad correcta y sin tocar la otra mano.
  4. `angle_set = -1` se respeta como no-op del protocolo RH56.

Se ejecuta en el entorno "tv" (tiene cyclonedds 0.10.2, unitree_sdk2py e
inspire_sdkpy) o en "unitree_sim_env". No hace falta GPU ni el robot.

    conda activate tv
    python scripts/14_test_inspire_ftp_bridge.py
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SIM = ROOT / "unitree_sim_isaaclab"

if not (SIM / "dds" / "inspire_ftp_dds.py").exists():
    sys.exit(f"ERROR: no encuentro {SIM}/dds/inspire_ftp_dds.py — ¿esta aplicado el parche?")

# El puente importa "dds.*", asi que unitree_sim_isaaclab tiene que estar en el path
sys.path.insert(0, str(SIM))

from unitree_sdk2py.core.channel import (            # noqa: E402
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber,
)
from inspire_sdkpy import inspire_dds                # noqa: E402
from inspire_sdkpy.inspire_hand_defaut import get_inspire_hand_ctrl  # noqa: E402

from dds.inspire_ftp_dds import InspireFTPDDS        # noqa: E402

# Rangos articulares del modelo Inspire en Isaac Lab, por DOF (ver el puente)
JOINT_RANGE = [(0.0, 1.7)] * 4 + [(0.0, 0.5), (-0.1, 1.3)]

fallos = []
BRIDGE = None


OK_MARK = "\033[32m✔\033[0m"
NO_MARK = "\033[31m✗\033[0m"


def check(desc, cond, detalle=""):
    marca = OK_MARK if cond else NO_MARK
    print(f"  {marca} {desc}" + (f"   {detalle}" if detalle else ""))
    if not cond:
        fallos.append(desc)


def norm(val, lo, hi):
    return min(1.0, max(0.0, (hi - val) / (hi - lo)))


def denorm(n, lo, hi):
    return (1.0 - min(1.0, max(0.0, n))) * (hi - lo) + lo


def main():
    # El simulador habla en el dominio 1 (el robot real usa el 0)
    ChannelFactoryInitialize(1)

    global BRIDGE
    bridge = BRIDGE = InspireFTPDDS()
    assert bridge.setup_publisher(), "no se pudieron crear los publicadores"
    assert bridge.setup_subscriber(), "no se pudieron crear los suscriptores"

    # Lector de estado con el IDL REAL de inspire_sdkpy: si esto lee lo que
    # publica el IDL vendorizado, los dos tipos son intercambiables.
    estado = {"l": None, "r": None}
    subs = {}
    for lado in ("l", "r"):
        s = ChannelSubscriber(f"rt/inspire_hand/state/{lado}", inspire_dds.inspire_hand_state)
        s.Init((lambda ld: (lambda msg: estado.__setitem__(ld, msg)))(lado), 10)
        subs[lado] = s

    pubs = {lado: ChannelPublisher(f"rt/inspire_hand/ctrl/{lado}", inspire_dds.inspire_hand_ctrl)
            for lado in ("l", "r")}
    for p in pubs.values():
        p.Init()

    time.sleep(0.5)   # descubrimiento DDS

    # ---------------------------------------------------------------- estado
    # Angulos distintos en cada DOF para que un cruce izquierda/derecha o un
    # desorden de dedos salte a la vista.
    #   indices 0..5  = mano DERECHA,  6..11 = mano IZQUIERDA
    pos_der = [0.00, 0.17, 0.85, 1.70, 0.25, 0.60]
    pos_izq = [1.70, 0.85, 0.17, 0.00, 0.50, -0.10]
    positions = pos_der + pos_izq
    esperado_der = [round(norm(v, *JOINT_RANGE[i]) * 1000) for i, v in enumerate(pos_der)]
    esperado_izq = [round(norm(v, *JOINT_RANGE[i]) * 1000) for i, v in enumerate(pos_izq)]

    print("\n== 1. Estado: Isaac Lab -> rt/inspire_hand/state/{l,r} ==")
    bridge.write_inspire_state(positions, [0.0] * 12, [0.0] * 12)
    for _ in range(50):
        bridge.dds_publisher()
        time.sleep(0.02)
        if estado["l"] is not None and estado["r"] is not None:
            break

    check("llega estado por rt/inspire_hand/state/r", estado["r"] is not None)
    check("llega estado por rt/inspire_hand/state/l", estado["l"] is not None)
    if estado["r"] is None or estado["l"] is None:
        return
    check("el IDL vendorizado lo lee inspire_sdkpy sin cambios", True)

    got_der = list(estado["r"].angle_act)
    got_izq = list(estado["l"].angle_act)
    check("angle_act de la mano DERECHA", got_der == esperado_der, f"{got_der} == {esperado_der}")
    check("angle_act de la mano IZQUIERDA", got_izq == esperado_izq, f"{got_izq} == {esperado_izq}")
    check("dedo abierto -> 1000", got_der[0] == 1000, f"DOF0 derecha = {got_der[0]}")
    check("dedo cerrado -> 0", got_der[3] == 0, f"DOF3 derecha = {got_der[3]}")
    check("pos_act replica angle_act", list(estado["r"].pos_act) == got_der)
    check("sin fuerza simulada (force_act = 0)", all(f == 0 for f in estado["r"].force_act))

    # --------------------------------------------------------------- comando
    print("\n== 2. Comando: rt/inspire_hand/ctrl/{l,r} -> Isaac Lab ==")
    cmd = get_inspire_hand_ctrl()
    cmd.angle_set = [1000, 800, 500, 0, 250, 700]
    cmd.mode = 0b0001
    pubs["l"].Write(cmd)

    for _ in range(50):
        time.sleep(0.02)
        c = bridge.get_inspire_hand_command()
        if c and any(abs(v) > 1e-9 for v in c["positions"][6:12]):
            break
    c = bridge.get_inspire_hand_command()
    check("el comando llega al puente", c is not None and "positions" in c)
    if not c:
        return

    esperado_cmd = [denorm(v / 1000.0, *JOINT_RANGE[i]) for i, v in enumerate(cmd.angle_set)]
    obtenido_cmd = c["positions"][6:12]
    ok = all(abs(a - b) < 1e-6 for a, b in zip(obtenido_cmd, esperado_cmd))
    check("ctrl/l -> DOF 6..11 (mano izquierda)", ok,
          f"{[round(v,3) for v in obtenido_cmd]} == {[round(v,3) for v in esperado_cmd]}")

    # la mano derecha no ha recibido nada: sigue en su valor de reposo (abierta)
    reposo_der = [denorm(1.0, *JOINT_RANGE[i]) for i in range(6)]
    ok = all(abs(a - b) < 1e-6 for a, b in zip(c["positions"][0:6], reposo_der))
    check("ctrl/l NO toca la mano derecha", ok, f"{[round(v,3) for v in c['positions'][0:6]]}")

    # ------------------------------------------------------------ no-op (-1)
    print("\n== 3. No-op del protocolo RH56 (angle_set = -1) ==")
    cmd2 = get_inspire_hand_ctrl()
    cmd2.angle_set = [-1, -1, -1, 900, -1, -1]   # solo el DOF 3 (indice)
    cmd2.mode = 0b0001
    pubs["l"].Write(cmd2)

    objetivo_dof3 = denorm(0.9, *JOINT_RANGE[3])
    for _ in range(50):
        time.sleep(0.02)
        c = bridge.get_inspire_hand_command()
        if c and abs(c["positions"][6 + 3] - objetivo_dof3) < 1e-6:
            break
    c = bridge.get_inspire_hand_command()
    check("el DOF comandado se mueve", abs(c["positions"][6 + 3] - objetivo_dof3) < 1e-6,
          f"DOF3 izquierda = {c['positions'][6+3]:.4f}, esperado {objetivo_dof3:.4f}")
    sin_tocar = [i for i in (0, 1, 2, 4, 5)]
    ok = all(abs(c["positions"][6 + i] - esperado_cmd[i]) < 1e-6 for i in sin_tocar)
    check("los DOF con -1 conservan el valor anterior", ok,
          f"{[round(c['positions'][6+i], 3) for i in sin_tocar]}")

    # ------------------------------------------------------------------ fin
    print()
    if fallos:
        print(f"\033[31m{len(fallos)} COMPROBACION(ES) FALLIDAS:\033[0m")
        for f in fallos:
            print(f"  - {f}")
        sys.exit(1)
    print("\033[32m=== PUENTE FTP DEL SIMULADOR: TODO OK ===\033[0m")


if __name__ == "__main__":
    try:
        main()
    finally:
        # SharedMemoryManager crea segmentos en /dev/shm que sobreviven al
        # proceso si no se cierran.
        for shm in (getattr(BRIDGE, "input_shm", None), getattr(BRIDGE, "output_shm", None)):
            if shm is not None:
                try:
                    shm.cleanup()
                except Exception:
                    pass
