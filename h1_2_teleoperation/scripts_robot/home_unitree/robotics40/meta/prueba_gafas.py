#!/usr/bin/env python3
"""Prueba SOLO de gafas: Vuer + imagen de la D435i + seguimiento de manos. NO usa DDS ni mueve el robot.

Crea el TeleVuerWrapper igual que teleop_hand_and_arm.py, le pasa la imagen del teleimager y cada
segundo imprime si llegan datos de las Quest y donde estan las munecas (en el marco del robot).
"""
import os, sys, time, numpy as np
TELEOP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "xr_teleoperate", "teleop")
sys.path.insert(0, os.path.dirname(TELEOP)); os.chdir(TELEOP)
# Precarga PIL y TODOS sus plugins antes de que televuer haga fork() del proceso de Vuer: si no, el hijo
# hereda el cerrojo de importacion tomado por otro hilo y se bloquea al importar PngImagePlugin en el
# primer JPEG que manda a las gafas (visto con py-spy el 2026-09-25: Vuer al 0 % y sockets sin leer).
from PIL import Image as _PIL_Image; _PIL_Image.init()
from teleimager.image_client import ImageClient
from televuer import TeleVuerWrapper

img = ImageClient(host="127.0.0.1", request_bgr=True)
cfg = img.get_cam_config()["head_camera"]
tv = TeleVuerWrapper(use_hand_tracking=True, binocular=cfg["binocular"], img_shape=cfg["image_shape"],
                     display_mode="immersive", zmq=cfg["enable_zmq"], webrtc=cfg["enable_webrtc"],
                     webrtc_url=None, arm_reference_mode="head_yaw")
print("Vuer listo en :8012. Ctrl-C para salir.", flush=True)
t_rep, n_img = time.time(), 0
try:
    while True:
        f = img.get_head_frame()
        if f is not None and f.bgr is not None:
            tv.render_to_xr(f.bgr); n_img += 1
        d = tv.get_tele_data()
        if time.time() - t_rep >= 1.0:
            L, R = d.left_wrist_pose[:3, 3], d.right_wrist_pose[:3, 3]
            print(f"datos XR={d.motion_data_ready}  img {n_img}/s  muneca izq {np.round(L,2)}  der {np.round(R,2)}  "
                  f"dedos izq {np.round(np.asarray(d.left_hand_pos).reshape(-1,3)[[4,9,14,19,24]].tolist() if np.size(d.left_hand_pos)>=75 else [],2).tolist()}", flush=True)
            t_rep, n_img = time.time(), 0
        time.sleep(1/30)
except KeyboardInterrupt:
    pass
