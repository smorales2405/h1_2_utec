#!/usr/bin/env python3
"""
Vigila la aparicion de la camara de cabeza en el PC2 y, en cuanto la detecta,
imprime todo lo que hace falta para configurar teleimager.

Pensado para tenerlo corriendo mientras se revisa el cable USB: reporta el
cambio en el momento en que el dispositivo enumera, sin tener que adivinar.

    ~/teleop_venv/bin/python ~/wait_for_camera.py            # vigila indefinidamente
    ~/teleop_venv/bin/python ~/wait_for_camera.py --once     # comprueba una vez y sale

Que mira, en este orden:
  1. USB: cualquier vendor 8086 (Intel) o 32902, que es como enumera la D435i
  2. /dev/video*  (la D435i expone varios nodos: color, depth, IR, metadatos)
  3. pyrealsense2: enumeracion nativa, que da el numero de serie — el
     identificador que teleimager necesita en cam_config_server.yaml
"""
import argparse
import glob
import subprocess
import sys
import time

INTEL_VENDORS = ("8086", "32902")


def usb_intel():
    """Dispositivos USB de Intel presentes (la D435i es 8086:0b3a)."""
    try:
        salida = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return []
    return [l for l in salida.splitlines()
            if any(f"ID {v}:" in l for v in INTEL_VENDORS) or "realsense" in l.lower()]


def video_nodes():
    return sorted(glob.glob("/dev/video*"))


def realsense_devices():
    """(serie, nombre, firmware) de cada RealSense que vea pyrealsense2."""
    try:
        import pyrealsense2 as rs
    except ImportError:
        return None                      # sin pyrealsense2 instalado
    try:
        devs = []
        for d in rs.context().query_devices():
            devs.append((
                d.get_info(rs.camera_info.serial_number),
                d.get_info(rs.camera_info.name),
                d.get_info(rs.camera_info.firmware_version),
            ))
        return devs
    except Exception as e:
        print(f"  (pyrealsense2 fallo: {e})")
        return []


def snapshot():
    return usb_intel(), video_nodes(), realsense_devices()


def informe(usb, video, rs_devs):
    print(f"  USB Intel/RealSense : {usb if usb else 'ninguno'}")
    print(f"  /dev/video*         : {video if video else 'ninguno'}")
    if rs_devs is None:
        print("  pyrealsense2        : NO INSTALADO — instalalo desde el bundle:")
        print("      ~/teleop_venv/bin/pip install --no-index "
              "--find-links ~/pc2_offline/wheels pyrealsense2")
    elif rs_devs:
        for s, n, fw in rs_devs:
            print(f"  RealSense           : {n}  serie={s}  firmware={fw}")
    else:
        print("  pyrealsense2        : instalado, pero no ve ninguna camara")


def receta(rs_devs):
    if not rs_devs:
        return
    serie, nombre, _ = rs_devs[0]
    print("\n" + "=" * 70)
    print(f"  ✔ CAMARA DETECTADA: {nombre}  (serie {serie})")
    print("=" * 70)
    print("""
Pon esto en la seccion head_camera de ~/teleimager/cam_config_server.yaml.
La D435i es MONOCULAR: binocular debe ir en false (una camara binocular manda
las dos imagenes lado a lado en una sola de 480x1280; esta no).

head_camera:
  enable_zmq: true
  zmq_port: 55555
  enable_webrtc: true
  webrtc_port: 60001
  webrtc_codec: h264
  type: realsense
  image_shape: [480, 640]      # sube a [720, 1280] si la red lo aguanta
  binocular: false
  fps: 30
  video_id: null
  serial_number: {serie}
  physical_path: null

Luego, en el PC2:
    ~/teleop_venv/bin/teleimager-server

Y desde la laptop:
    ./scripts/03_launch_teleop.sh
""".replace("{serie}", serie))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--once", action="store_true", help="comprobar una vez y salir")
    ap.add_argument("--interval", type=float, default=2.0, help="segundos entre sondeos")
    a = ap.parse_args()

    print("Estado actual:")
    usb, video, rs_devs = snapshot()
    informe(usb, video, rs_devs)
    if rs_devs:
        receta(rs_devs)
        return 0
    if a.once:
        print("\n  ✗ No hay camara conectada al PC2.")
        return 1

    print(f"\nVigilando cada {a.interval:.0f} s. Conecta o revisa el cable USB de la D435i.")
    print("Ctrl-C para salir.\n")
    anterior = (usb, video, rs_devs or [])
    try:
        while True:
            time.sleep(a.interval)
            actual = snapshot()
            if (actual[0], actual[1], actual[2] or []) != anterior:
                print(f"[{time.strftime('%H:%M:%S')}] CAMBIO detectado:")
                informe(*actual)
                print()
                if actual[2]:
                    receta(actual[2])
                    return 0
                anterior = (actual[0], actual[1], actual[2] or [])
    except KeyboardInterrupt:
        print("\ninterrumpido")
        return 1


if __name__ == "__main__":
    sys.exit(main())
