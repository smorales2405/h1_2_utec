#!/usr/bin/env python3
"""Streaming en vivo de los CUATRO flujos de la D435i por HTTP (MJPEG).

Profundidad (coloreada), RGB, infrarrojo 1 y infrarrojo 2, en una sola pagina.
Se abre desde cualquier navegador de la LAN:  http://192.168.123.164:8080

Requiere sudo (pyrealsense2 no ve el dispositivo como usuario normal).
Si la camara no da fotogramas, hace hardware_reset() y reintenta.

Uso:  sudo /home/unitree/teleop_venv/bin/python stream_d435.py [--puerto 8080] [--fps 30]
"""
import argparse, socket, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pyrealsense2 as rs

FLUJOS = ("color", "depth", "ir1", "ir2")
TITULOS = {"color": "RGB", "depth": "Profundidad",
           "ir1": "Infrarrojo 1 (izq)", "ir2": "Infrarrojo 2 (der)"}

ultimo = {k: None for k in FLUJOS}      # JPEG ya codificado
info = {"fps": 0.0, "centro": 0.0, "validos": 0.0, "n": 0, "estado": "arrancando"}
lock = threading.Lock()
parar = threading.Event()


def abrir(fps):
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, fps)
    cfg.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, fps)
    p = rs.pipeline()
    perfil = p.start(cfg)
    return p, perfil


def capturar(fps):
    """Hilo de captura. Reintenta con hardware_reset si no llegan fotogramas."""
    for intento in range(4):
        try:
            pipe, perfil = abrir(fps)
        except Exception as e:
            info["estado"] = f"no abre: {e}"
            time.sleep(3)
            continue
        escala = perfil.get_device().first_depth_sensor().get_depth_scale()
        col = rs.colorizer()
        col.set_option(rs.option.color_scheme, 0)   # 0 = Jet, se lee mucho mejor
        try:
            pipe.wait_for_frames(8000)
        except Exception:
            info["estado"] = "sin fotogramas -> hardware_reset"
            try:
                pipe.stop()
            except Exception:
                pass
            try:
                list(rs.context().query_devices())[0].hardware_reset()
            except Exception:
                pass
            time.sleep(9)
            continue

        info["estado"] = "en marcha"
        t0, n0 = time.time(), 0
        while not parar.is_set():
            try:
                fr = pipe.wait_for_frames(5000)
            except Exception as e:
                info["estado"] = f"se corto: {e}"
                break
            d = fr.get_depth_frame()
            c = fr.get_color_frame()
            i1 = fr.get_infrared_frame(1)
            i2 = fr.get_infrared_frame(2)
            if not (d and c and i1 and i2):
                continue

            dz = np.asanyarray(d.get_data())
            imgs = {
                "color": np.asanyarray(c.get_data()),
                "depth": np.asanyarray(col.colorize(d).get_data()),
                "ir1": cv2.cvtColor(np.asanyarray(i1.get_data()), cv2.COLOR_GRAY2BGR),
                "ir2": cv2.cvtColor(np.asanyarray(i2.get_data()), cv2.COLOR_GRAY2BGR),
            }
            n0 += 1
            dt = time.time() - t0
            if dt >= 0.5:
                info["fps"] = n0 / dt
                t0, n0 = time.time(), 0
            v = dz[dz > 0]
            info["centro"] = float(d.get_distance(320, 240))
            info["validos"] = 100.0 * v.size / dz.size
            info["n"] = int(d.get_frame_number())

            nuevos = {}
            for k, im in imgs.items():
                ok, buf = cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    nuevos[k] = buf.tobytes()
            with lock:
                ultimo.update(nuevos)
        try:
            pipe.stop()
        except Exception:
            pass
        return
    info["estado"] = "imposible abrir la camara"


PAGINA = """<!doctype html><html lang="es"><head><meta charset="utf-8">
<title>D435i del H1-2</title>
<style>
 :root{--bg:#14161a;--fg:#e8eaed;--mut:#9aa0a6;--bd:#2a2e35}
 body{margin:0;background:var(--bg);color:var(--fg);
      font:14px/1.5 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
 header{padding:14px 18px;border-bottom:1px solid var(--bd);display:flex;
        gap:18px;align-items:baseline;flex-wrap:wrap}
 h1{font-size:16px;margin:0;font-weight:600}
 .m{color:var(--mut);font-variant-numeric:tabular-nums}
 .g{display:grid;grid-template-columns:repeat(2,1fr);gap:12px;padding:12px}
 @media(max-width:900px){.g{grid-template-columns:1fr}}
 figure{margin:0;border:1px solid var(--bd);border-radius:10px;overflow:hidden;background:#000}
 figcaption{padding:7px 11px;background:#1b1e24;color:var(--mut);font-size:12px}
 img{display:block;width:100%;height:auto}
</style></head><body>
<header><h1>Intel RealSense D435i &mdash; H1-2</h1>
<span class="m" id="s">conectando&hellip;</span></header>
<div class="g">
 <figure><img src="/f/color"><figcaption>RGB</figcaption></figure>
 <figure><img src="/f/depth"><figcaption>Profundidad (coloreada)</figcaption></figure>
 <figure><img src="/f/ir1"><figcaption>Infrarrojo 1 (izquierdo)</figcaption></figure>
 <figure><img src="/f/ir2"><figcaption>Infrarrojo 2 (derecho)</figcaption></figure>
</div>
<script>
setInterval(async()=>{try{const r=await fetch('/info');const j=await r.json();
 document.getElementById('s').textContent =
  j.estado+' \\u00b7 '+j.fps.toFixed(1)+' fps \\u00b7 centro '+j.centro.toFixed(3)+
  ' m \\u00b7 '+j.validos.toFixed(1)+'% validos \\u00b7 #'+j.n;}catch(e){}},500);
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            b = PAGINA.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
            return
        if self.path == "/info":
            import json
            b = json.dumps(info).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
            return
        if self.path.startswith("/f/"):
            k = self.path[3:]
            if k not in FLUJOS:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=fr")
            self.end_headers()
            try:
                while not parar.is_set():
                    with lock:
                        j = ultimo.get(k)
                    if j is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--fr\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(j)).encode()
                                     + b"\r\n\r\n" + j + b"\r\n")
                    time.sleep(0.033)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        self.send_error(404)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", type=int, default=8080)
    ap.add_argument("--fps", type=int, default=30)
    a = ap.parse_args()

    threading.Thread(target=capturar, args=(a.fps,), daemon=True).start()
    ip = socket.gethostbyname(socket.gethostname())
    srv = ThreadingHTTPServer(("0.0.0.0", a.puerto), H)
    srv.daemon_threads = True
    print(f"streaming en  http://192.168.123.164:{a.puerto}/   (y http://{ip}:{a.puerto}/)")
    print("Ctrl-C para parar")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        parar.set()
        srv.shutdown()
        print("parado.")


if __name__ == "__main__":
    sys.exit(main())
