#!/usr/bin/env python3
"""
Prueba de lazo cerrado del canal DDS de las manos Inspire FTP, SIN hardware.

Levanta un servicio de manos falso (el papel que hace el PC2) y contra el corre
el `Inspire_Controller_FTP` real de xr_teleoperate, alimentado con landmarks de
mano sinteticos. Valida, en la laptop y antes de tener el robot:

  * los tipos IDL de inspire_sdkpy viajan por CycloneDDS
  * los nombres de topico del controlador y del servicio coinciden
  * el retargeting DexPilot produce angulos y la normalizacion los deja en 0..1000
  * el controlador lee de vuelta `angle_act` publicado por el servicio

Todo el trafico DDS queda confinado a loopback (CYCLONEDDS_URI), asi que no se
mezcla con ningun robot que haya en la red.

    python 04_test_inspire_dds_loopback.py
"""
import multiprocessing as mp
import os
import sys
from pathlib import Path
import tempfile
import time

REPO = str(Path(__file__).resolve().parent.parent / "xr_teleoperate")

# CycloneDDS confinado a loopback: sin multicast y con un unico peer local.
CYCLONE_XML = """<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface address="127.0.0.1"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>
      <Peers><Peer address="localhost"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""


def mock_hand_service(ready, received, stop):
    """Hace de PC2: publica estado de ambas manos y recoge los comandos."""
    from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                             ChannelPublisher, ChannelSubscriber)
    from inspire_sdkpy import inspire_dds
    import inspire_sdkpy.inspire_hand_defaut as default

    ChannelFactoryInitialize(0)

    pubs, subs = {}, {}
    for lr in ("l", "r"):
        p = ChannelPublisher(f"rt/inspire_hand/state/{lr}", inspire_dds.inspire_hand_state)
        p.Init()
        pubs[lr] = p
        s = ChannelSubscriber(f"rt/inspire_hand/ctrl/{lr}", inspire_dds.inspire_hand_ctrl)
        s.Init()
        subs[lr] = s

    ready.set()
    while not stop.is_set():
        for lr in ("l", "r"):
            st = default.get_inspire_hand_state()
            st.angle_act = [500] * 6          # mano a medio cerrar
            pubs[lr].Write(st)
            msg = subs[lr].Read()
            if msg is not None:
                received[lr] = (list(msg.angle_set), int(msg.mode))
        time.sleep(0.01)


def main():
    xml = tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False)
    xml.write(CYCLONE_XML)
    xml.close()
    os.environ["CYCLONEDDS_URI"] = f"file://{xml.name}"

    mgr = mp.Manager()
    received, ready, stop = mgr.dict(), mgr.Event(), mgr.Event()
    svc = mp.Process(target=mock_hand_service, args=(ready, received, stop), daemon=True)
    svc.start()
    if not ready.wait(timeout=20):
        print("✗ el servicio de manos falso no arranco"); return 1
    print("✔ servicio de manos falso publicando en rt/inspire_hand/state/{l,r}")

    sys.path.insert(0, REPO)
    os.chdir(os.path.join(REPO, "teleop"))          # hand_retargeting usa rutas relativas
    import numpy as np
    from multiprocessing import Array, Lock
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP

    ChannelFactoryInitialize(0)

    left_in = Array("d", 75, lock=True)
    right_in = Array("d", 75, lock=True)
    lock = Lock()
    state_out = Array("d", 12, lock=False)
    action_out = Array("d", 12, lock=False)

    # 25 landmarks x 3 de una mano ABIERTA (dedos extendidos a lo largo de +y)
    hand = np.zeros((25, 3))
    for i in range(25):
        hand[i] = [0.02 * (i % 5), 0.02 * (i // 5), 0.0]
    left_in[:] = hand.flatten()
    right_in[:] = hand.flatten()

    ctrl = Inspire_Controller_FTP(left_in, right_in, lock, state_out, action_out)
    time.sleep(4)

    rc = 0
    print()
    if not received:
        print("✗ el servicio no recibio ningun comando en rt/inspire_hand/ctrl/*"); rc = 1
    for lr in ("l", "r"):
        if lr not in received:
            print(f"✗ mano '{lr}': sin comandos recibidos"); rc = 1; continue
        angles, mode = received[lr]
        rango_ok = len(angles) == 6 and all(0 <= a <= 1000 for a in angles)
        modo_ok = mode == 0b0001
        print(f"{'✔' if rango_ok and modo_ok else '✗'} mano '{lr}': angle_set={angles} mode={mode:#06b}"
              f"  (esperado 6 valores en 0..1000, mode=0b0001 control por angulo)")
        rc |= 0 if (rango_ok and modo_ok) else 1

    with lock:
        estado = list(state_out)
    esperado = [0.5] * 12          # angle_act 500 / 1000 normalizado
    estado_ok = all(abs(a - b) < 1e-6 for a, b in zip(estado, esperado))
    print(f"{'✔' if estado_ok else '✗'} realimentacion de estado leida por el controlador: {estado}"
          f"  (esperado {esperado})")
    rc |= 0 if estado_ok else 1

    stop.set()
    svc.join(timeout=3)
    os.unlink(xml.name)
    print("\n" + ("✔ CADENA DDS DE LAS MANOS INSPIRE FTP: OK" if rc == 0
                  else "✗ la prueba fallo, revisa arriba"))
    return rc


if __name__ == "__main__":
    sys.exit(main())
