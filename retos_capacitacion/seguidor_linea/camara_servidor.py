#!/usr/bin/env python3
"""Servidor de la D435i para el seguidor de linea. Se lanza con camara_servidor.sh (sudo).

Abre la camara una sola vez y publica cada fotograma por ZMQ (PUB en 127.0.0.1) para los
procesos del usuario: el seguidor, la grabacion del dataset y la calibracion. Es un proceso
aparte porque pyrealsense2 exige sudo y, como root, el DDS de Unitree no arranca. Por eso aqui
no se importa nada de unitree_sdk2py.

Ademas sirve un mosaico de depuracion (IR y color, a media resolucion) en http://<robot>:5000.
Si la camara deja de dar fotogramas la reabre; tras tres fallos seguidos, hardware_reset
(como ~/robotics40/camara.py).
"""

import argparse
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from seguidor import config, fotograma_zmq  # noqa: E402
from seguidor.mensajes import Fotograma  # noqa: E402

MJPEG_CADA_S = 0.1   # el mosaico de depuracion va a 10 Hz como mucho


def intrinsecos(perfil):
    i = perfil.as_video_stream_profile().get_intrinsics()
    return {"fx": i.fx, "fy": i.fy, "cx": i.ppx, "cy": i.ppy, "ancho": i.width, "alto": i.height,
            "modelo": str(i.model), "coefs": list(i.coeffs)}


class Servidor:
    def __init__(self, args):
        self.args = args
        self.salir = threading.Event()
        self._ctx = zmq.Context.instance()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.setsockopt(zmq.SNDHWM, 4)
        self._pub.setsockopt(zmq.LINGER, 0)
        self._pub.bind(args.zmq)
        self._jpeg = None
        self._num_jpeg = 0
        self._cond = threading.Condition()
        self.clientes = 0
        self.publicados = 0

    def _abrir(self):
        a = self.args
        cfg = rs.config()
        cfg.enable_stream(rs.stream.infrared, 1, a.ancho, a.alto, rs.format.y8, a.fps)
        if a.color:
            cfg.enable_stream(rs.stream.color, a.ancho, a.alto, rs.format.bgr8, a.fps)
        if a.profundidad:
            cfg.enable_stream(rs.stream.depth, a.ancho, a.alto, rs.format.z16, a.fps)
        pipeline = rs.pipeline()
        perfil = pipeline.start(cfg)
        sensor = perfil.get_device().first_depth_sensor()
        sensor.set_option(rs.option.emitter_enabled, 1.0 if a.emisor else 0.0)
        meta = {"ir": intrinsecos(perfil.get_stream(rs.stream.infrared, 1)),
                "escala_prof": sensor.get_depth_scale(), "fps": a.fps, "emisor": a.emisor}
        if a.color:
            meta["color"] = intrinsecos(perfil.get_stream(rs.stream.color))
        if a.profundidad:
            meta["prof"] = intrinsecos(perfil.get_stream(rs.stream.depth))
        return pipeline, meta

    def captura(self):
        fallos = 0
        t_jpeg = 0.0
        while not self.salir.is_set():
            pipeline = None
            try:
                print(f"[CAMARA] abriendo la D435i: IR{' + color' if self.args.color else ''}"
                      f"{' + profundidad' if self.args.profundidad else ''}, emisor "
                      f"{'encendido' if self.args.emisor else 'apagado'}, {self.args.fps} fps", flush=True)
                pipeline, meta = self._abrir()
                print("[CAMARA] en marcha", flush=True)
                fallos = 0
                t_stats, n_stats = time.monotonic(), 0
                while not self.salir.is_set():
                    frames = pipeline.wait_for_frames(5000)
                    ir = frames.get_infrared_frame(1)
                    if not ir:
                        continue
                    f = Fotograma(n=ir.get_frame_number(), t_cam=frames.get_timestamp() / 1000.0,
                                  t_rx=time.monotonic(), emisor=self.args.emisor, meta=meta,
                                  ir=np.asanyarray(ir.get_data()))
                    if self.args.color:
                        c = frames.get_color_frame()
                        f.color = np.asanyarray(c.get_data()) if c else None
                    if self.args.profundidad:
                        d = frames.get_depth_frame()
                        f.prof = np.asanyarray(d.get_data()) if d else None
                    self._pub.send_multipart(fotograma_zmq.codificar(f), copy=True)
                    self.publicados += 1
                    n_stats += 1
                    if self.clientes and f.t_rx - t_jpeg > MJPEG_CADA_S:
                        t_jpeg = f.t_rx
                        self._mosaico(f)
                    if f.t_rx - t_stats > 10.0:
                        print(f"[CAMARA] {n_stats / (f.t_rx - t_stats):.1f} fps, {self.publicados} publicados, "
                              f"{self.clientes} navegador(es)", flush=True)
                        t_stats, n_stats = f.t_rx, 0
            except Exception as e:
                fallos += 1
                print(f"[CAMARA] error ({fallos}): {e}", flush=True)
            finally:
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
            if self.salir.is_set():
                break
            if fallos >= 3:
                self._reset()
                fallos = 0
            else:
                time.sleep(2)
        print("[CAMARA] cerrada", flush=True)

    def _reset(self):
        try:
            for dev in rs.context().query_devices():
                print("[CAMARA] hardware_reset", flush=True)
                dev.hardware_reset()
            time.sleep(8)
        except Exception as e:
            print(f"[CAMARA] reset fallido: {e}", flush=True)

    def _mosaico(self, f: Fotograma):
        partes = [cv2.cvtColor(f.ir, cv2.COLOR_GRAY2BGR)]
        if f.color is not None:
            partes.append(f.color)
        img = np.hstack(partes)
        img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self._cond:
                self._jpeg, self._num_jpeg = buf.tobytes(), self._num_jpeg + 1
                self._cond.notify_all()

    def flujo_mjpeg(self):
        self.clientes += 1
        visto = -1
        try:
            while not self.salir.is_set():
                with self._cond:
                    if not self._cond.wait_for(lambda: self._num_jpeg != visto and self._jpeg is not None,
                                               timeout=10):
                        continue
                    jpeg, visto = self._jpeg, self._num_jpeg
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
        finally:
            self.clientes -= 1


def main():
    cfg = config.cargar()
    cam = cfg["camara"]
    ap = argparse.ArgumentParser(description="Servidor de la D435i para el seguidor de linea (root)")
    ap.add_argument("--zmq", default=cfg["red"]["zmq_fotogramas"])
    ap.add_argument("--fps", type=int, default=cam["fps"])
    ap.add_argument("--ancho", type=int, default=cam["ancho"])
    ap.add_argument("--alto", type=int, default=cam["alto"])
    ap.add_argument("--color", action=argparse.BooleanOptionalAction, default=cam["color"])
    ap.add_argument("--profundidad", action=argparse.BooleanOptionalAction, default=cam["profundidad"])
    ap.add_argument("--emisor", action=argparse.BooleanOptionalAction, default=cam["emisor"])
    ap.add_argument("--puerto-video", type=int, default=cam["puerto_video"], help="0 = sin mosaico")
    args = ap.parse_args()

    srv = Servidor(args)
    hilo = threading.Thread(target=srv.captura, daemon=True)

    def terminar(*_):
        srv.salir.set()
        hilo.join(timeout=6)
        os._exit(0)

    signal.signal(signal.SIGTERM, terminar)
    signal.signal(signal.SIGINT, terminar)
    hilo.start()
    print(f"[CAMARA] publicando en {args.zmq}", flush=True)

    if args.puerto_video:
        from flask import Flask, Response
        app = Flask(__name__)

        @app.route("/")
        def video():
            return Response(srv.flujo_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

        print(f"[CAMARA] mosaico en http://<robot>:{args.puerto_video}", flush=True)
        app.run(host="0.0.0.0", port=args.puerto_video, debug=False, threaded=True)
    else:
        srv.salir.wait()


if __name__ == "__main__":
    main()
