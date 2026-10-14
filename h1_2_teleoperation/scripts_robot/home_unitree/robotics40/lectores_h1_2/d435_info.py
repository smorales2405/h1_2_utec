#!/usr/bin/env python3
"""Extrae informacion de la RealSense D435/D435i: profundidad, RGB y los DOS infrarrojos.

Solo lee. No cambia la configuracion persistente de la camara.

Uso:  d435_info.py [--frames N] [--guardar DIR] [--no-imu]
"""
import argparse, sys, time

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("pyrealsense2 no esta disponible en este interprete")
import numpy as np

FS = {rs.format.z16: "z16", rs.format.y8: "y8", rs.format.bgr8: "bgr8",
      rs.format.rgb8: "rgb8", rs.format.yuyv: "yuyv"}

def cab(t):
    print("\n" + "=" * 68); print(t); print("=" * 68)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--guardar", default=None, help="directorio donde volcar los PNG")
    ap.add_argument("--no-imu", action="store_true")
    a = ap.parse_args()

    cab("1. DISPOSITIVOS")
    ctx = rs.context()
    devs = list(ctx.query_devices())
    if not devs:
        print("  NINGUNA CAMARA DETECTADA.")
        print("  Comprueba: cable USB3, 'lsusb | grep 8086' (D435i = 8086:0b3a),")
        print("  y que exista /dev/video*.")
        return 1
    d = devs[0]
    for k in ("name", "serial_number", "firmware_version", "product_line",
              "usb_type_descriptor", "physical_port"):
        try:
            print(f"  {k:22s}: {d.get_info(getattr(rs.camera_info, k))}")
        except Exception:
            pass

    cab("2. SENSORES Y PERFILES DISPONIBLES")
    quiere = {"Depth", "Color", "Infrared 1", "Infrared 2", "Infrared"}
    for s in d.query_sensors():
        nom = s.get_info(rs.camera_info.name)
        print(f"\n  --- sensor: {nom} ---")
        vistos = {}
        for p in s.get_stream_profiles():
            if not p.is_video_stream_profile():
                continue
            v = p.as_video_stream_profile()
            st = p.stream_name()
            if st.split()[0] not in {x.split()[0] for x in quiere}:
                continue
            clave = (st, v.width(), v.height(), FS.get(p.format(), str(p.format())))
            vistos.setdefault(clave, []).append(p.fps())
        for (st, w, h, f), fps in sorted(vistos.items()):
            print(f"    {st:12s} {w:5d}x{h:<5d} {f:6s} fps: {sorted(set(fps))}")

    cab("3. ARRANCANDO LOS CUATRO FLUJOS")
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
    imu = False
    if not a.no_imu:
        try:
            cfg.enable_stream(rs.stream.accel)
            cfg.enable_stream(rs.stream.gyro)
            imu = True
        except Exception:
            pass
    pipe = rs.pipeline()
    perfil = pipe.start(cfg)
    print("  pipeline arrancado" + (" (con IMU)" if imu else " (sin IMU)"))

    try:
        ds = perfil.get_device().first_depth_sensor().get_depth_scale()
        print(f"  escala de profundidad: {ds} m por unidad")

        cab("4. INTRINSECOS Y EXTRINSECOS")
        for nombre, st, idx in (("depth", rs.stream.depth, 0), ("color", rs.stream.color, 0),
                                ("infrared1", rs.stream.infrared, 1),
                                ("infrared2", rs.stream.infrared, 2)):
            try:
                vp = perfil.get_stream(st, idx).as_video_stream_profile()
                i = vp.get_intrinsics()
                print(f"  {nombre:10s} {i.width}x{i.height}  fx={i.fx:.2f} fy={i.fy:.2f} "
                      f"ppx={i.ppx:.2f} ppy={i.ppy:.2f}  modelo={i.model}")
                print(f"             coefs={[round(c,5) for c in i.coeffs]}")
            except Exception as e:
                print(f"  {nombre}: {e}")
        try:
            e = perfil.get_stream(rs.stream.depth).get_extrinsics_to(perfil.get_stream(rs.stream.color))
            print(f"  extrinsecos depth->color: t={[round(x,5) for x in e.translation]} m")
            base = perfil.get_device().first_depth_sensor().get_option(rs.option.stereo_baseline)
            print(f"  linea base estereo: {base:.3f} mm")
        except Exception as e:
            print(f"  extrinsecos: {e}")

        cab(f"5. CAPTURANDO {a.frames} FOTOGRAMAS")
        al = rs.align(rs.stream.color)
        n = 0; t0 = time.time(); ult = None
        while n < a.frames:
            fr = pipe.wait_for_frames(10000)
            ult = fr; n += 1
        dt = time.time() - t0
        print(f"  {n} fotogramas en {dt:.2f} s -> {n/dt:.1f} fps reales")

        fa = al.process(ult)
        dep = fa.get_depth_frame(); col = fa.get_color_frame()
        ir1 = ult.get_infrared_frame(1); ir2 = ult.get_infrared_frame(2)

        cab("6. CONTENIDO DE CADA FLUJO")
        dz = np.asanyarray(dep.get_data())
        val = dz[dz > 0]
        print(f"  PROFUNDIDAD  {dz.shape} {dz.dtype}  fotograma #{dep.get_frame_number()}")
        if val.size:
            print(f"     validos: {100*val.size/dz.size:5.1f} %   "
                  f"min {val.min()*ds:.3f} m   mediana {np.median(val)*ds:.3f} m   "
                  f"max {val.max()*ds:.3f} m")
            print(f"     distancia en el centro: {dep.get_distance(dz.shape[1]//2, dz.shape[0]//2):.3f} m")
        else:
            print("     *** ni un pixel valido: tapada, demasiado cerca, o sin emisor ***")
        cz = np.asanyarray(col.get_data())
        print(f"  RGB          {cz.shape} {cz.dtype}  fotograma #{col.get_frame_number()}  "
              f"BGR medio {[round(float(cz[:,:,i].mean()),1) for i in range(3)]}")
        for nom, f in (("INFRARROJO 1 (izq)", ir1), ("INFRARROJO 2 (der)", ir2)):
            z = np.asanyarray(f.get_data())
            print(f"  {nom} {z.shape} {z.dtype}  fotograma #{f.get_frame_number()}  "
                  f"intensidad media {z.mean():.1f}  min {z.min()} max {z.max()}")

        if imu:
            cab("7. IMU")
            for st, nom, uni in ((rs.stream.accel, "acelerometro", "m/s2"),
                                 (rs.stream.gyro, "giroscopo", "rad/s")):
                try:
                    v = ult.first_or_default(st).as_motion_frame().get_motion_data()
                    print(f"  {nom:14s} x={v.x:+8.4f} y={v.y:+8.4f} z={v.z:+8.4f}  {uni}")
                except Exception as e:
                    print(f"  {nom}: {e}")

        if a.guardar:
            import os, cv2
            os.makedirs(a.guardar, exist_ok=True)
            dn = cv2.applyColorMap(cv2.convertScaleAbs(dz, alpha=0.03), cv2.COLORMAP_JET)
            cv2.imwrite(f"{a.guardar}/depth.png", dn)
            cv2.imwrite(f"{a.guardar}/color.png", cz)
            cv2.imwrite(f"{a.guardar}/ir1.png", np.asanyarray(ir1.get_data()))
            cv2.imwrite(f"{a.guardar}/ir2.png", np.asanyarray(ir2.get_data()))
            np.save(f"{a.guardar}/depth_raw.npy", dz)
            print(f"\n  guardado en {a.guardar}/ : depth.png color.png ir1.png ir2.png depth_raw.npy")
    finally:
        pipe.stop()
        print("\npipeline parado.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
