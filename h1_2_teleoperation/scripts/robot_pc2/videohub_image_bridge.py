#!/usr/bin/env python3
"""
Puente entre el servicio de vídeo del H1-2 y `xr_teleoperate`.

La D435i de la cabeza está cableada a PC1 (el computador de locomoción), al que
no tenemos acceso por SSH, así que `teleimager` no puede abrirla directamente:
no hay ningún dispositivo de vídeo en el bus USB del PC2. Pero el robot expone
su cámara por DDS a través del servicio `videohub`, que responde a cualquiera en
el dominio 0 sin credenciales.

Este programa hace de `teleimager-server` sin necesitar la cámara en local:

    videohub (DDS, PC1)  --JPEG 1920x1080-->  [este puente]  --ZMQ-->  laptop
                                                             --REP-->  cam_config

Habla el mismo protocolo que `teleimager`, así que el `ImageClient` de
`xr_teleoperate` funciona SIN modificar una sola línea del repositorio:

  * ZMQ REP en el puerto 60000 → responde el `cam_config` en JSON
  * ZMQ PUB en el puerto 55555 → publica los fotogramas JPEG en crudo

Reutiliza las clases del propio `teleimager` (`ZMQ_Responser`,
`ZMQ_PublisherManager`) para que el formato no se desvíe del original.

Medido en este robot: el servicio responde en ~6.4 ms, pero **los fotogramas
nuevos llegan a ~15 Hz** (el resto de las respuestas son repeticiones del
último). Por eso el sondeo va por defecto al doble de esa tasa.

Uso, en el PC2:

    ~/teleop_venv/bin/python ~/videohub_image_bridge.py --network eth0

Y en la laptop, sin cambios:

    ./scripts/03_launch_teleop.sh
"""
import argparse
import sys
import time

import cv2
import numpy as np


def construir_config(alto, ancho, fps, zmq_port):
    """cam_config con la misma forma que `cam_config_server.yaml` de teleimager.

    `image_shape` DEBE coincidir con el tamaño real de los fotogramas: televuer
    reserva memoria compartida exactamente de ese tamaño y escribe encima.
    Las cámaras de muñeca van deshabilitadas: este robot no las tiene.
    """
    return {
        "webrtc": {"bitrate": {"min": 2000000, "default": 5000000, "max": 12000000},
                   "gop_length": 60},
        "head_camera": {
            "enable_zmq": True,
            "zmq_port": zmq_port,
            # WebRTC apagado: lo sirve teleimager desde la cámara local, y aquí
            # la fuente es DDS. La laptop recibe por ZMQ y renderiza con Vuer.
            "enable_webrtc": False,
            "webrtc_port": 60001,
            "webrtc_codec": "h264",
            "type": "videohub",
            "image_shape": [alto, ancho],
            # La D435i es monocular: una sola imagen, no dos lado a lado.
            "binocular": False,
            "fps": fps,
            "video_id": None,
            "serial_number": None,
            "physical_path": None,
        },
        "left_wrist_camera":  {"enable_zmq": False, "zmq_port": 55556, "enable_webrtc": False,
                               "webrtc_port": 60002, "webrtc_codec": "h264", "type": "uvc",
                               "image_shape": [480, 640], "binocular": False, "fps": 30,
                               "video_id": None, "serial_number": None, "physical_path": None},
        "right_wrist_camera": {"enable_zmq": False, "zmq_port": 55557, "enable_webrtc": False,
                               "webrtc_port": 60003, "webrtc_codec": "h264", "type": "uvc",
                               "image_shape": [480, 640], "binocular": False, "fps": 30,
                               "video_id": None, "serial_number": None, "physical_path": None},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--width", type=int, default=1280, help="ancho publicado (la fuente da 1920)")
    ap.add_argument("--height", type=int, default=720, help="alto publicado (la fuente da 1080)")
    ap.add_argument("--fps", type=int, default=15, help="fps declarados en el cam_config")
    ap.add_argument("--poll-hz", type=float, default=30.0,
                    help="sondeos por segundo al videohub; el doble de la tasa de fotogramas nuevos")
    ap.add_argument("--quality", type=int, default=80, help="calidad JPEG 1-100")
    ap.add_argument("--zmq-port", type=int, default=55555)
    ap.add_argument("--config-port", type=int, default=60000)
    ap.add_argument("--network", default=None, help="NIC para DDS (eth0 en el PC2)")
    ap.add_argument("--passthrough", action="store_true",
                    help="publicar el JPEG original 1920x1080 sin reescalar ni recomprimir")
    ap.add_argument("--report-every", type=int, default=150, help="0 lo silencia")
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
    from teleimager.image_client import ZMQ_PublisherManager, ZMQ_Responser

    if a.passthrough:
        a.width, a.height = 1920, 1080

    if a.network:
        ChannelFactoryInitialize(0, a.network)
    else:
        ChannelFactoryInitialize(0)

    video = VideoClient()
    video.SetTimeout(2.0)
    video.Init()

    code, primera = video.GetImageSample()
    if code != 0 or not primera:
        print(f"✗ el servicio videohub no responde (code={code}). "
              f"¿Está el robot encendido y en el mismo dominio DDS?")
        return 1
    origen = cv2.imdecode(np.frombuffer(bytes(primera), np.uint8), cv2.IMREAD_COLOR)
    print(f"✔ videohub responde: fuente {origen.shape[1]}x{origen.shape[0]}, "
          f"{len(primera)/1024:.0f} KB por fotograma")

    cam_config = construir_config(a.height, a.width, a.fps, a.zmq_port)
    responder = ZMQ_Responser(cam_config, host="0.0.0.0", port=a.config_port)
    publisher = ZMQ_PublisherManager.get_instance()

    print(f"✔ cam_config servido en tcp://0.0.0.0:{a.config_port} (REP)")
    print(f"✔ fotogramas publicados en tcp://0.0.0.0:{a.zmq_port} (PUB) "
          f"a {a.width}x{a.height}" + (" sin recomprimir" if a.passthrough else
                                       f", JPEG q={a.quality}"))
    print(f"  sondeando el videohub a {a.poll_hz:.0f} Hz. Ctrl-C para salir.\n")

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), a.quality]
    intervalo = 1.0 / a.poll_hz
    n = nuevos = fallos = 0
    ultimo = None
    t0 = time.perf_counter()
    siguiente = time.monotonic()

    try:
        while True:
            code, data = video.GetImageSample()
            if code != 0 or not data:
                fallos += 1
            else:
                crudo = bytes(data)
                n += 1
                if crudo != ultimo:
                    nuevos += 1
                    ultimo = crudo
                if a.passthrough:
                    publisher.publish(crudo, a.zmq_port)
                else:
                    img = cv2.imdecode(np.frombuffer(crudo, np.uint8), cv2.IMREAD_COLOR)
                    if img is None:
                        fallos += 1
                    else:
                        if (img.shape[1], img.shape[0]) != (a.width, a.height):
                            img = cv2.resize(img, (a.width, a.height), interpolation=cv2.INTER_AREA)
                        ok, jpg = cv2.imencode(".jpg", img, encode_params)
                        if ok:
                            publisher.publish(jpg.tobytes(), a.zmq_port)
                        else:
                            fallos += 1

            if a.report_every and n and n % a.report_every == 0:
                dt = time.perf_counter() - t0
                print(f"  publicados {n/dt:5.1f} Hz · fotogramas nuevos {nuevos/dt:5.1f} Hz"
                      f" · fallos {fallos}", flush=True)

            siguiente += intervalo
            dormir = siguiente - time.monotonic()
            if dormir > 0:
                time.sleep(dormir)
            else:
                siguiente = time.monotonic()
    except KeyboardInterrupt:
        print("\ncerrando puente...")
    finally:
        responder.stop()
        publisher.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
