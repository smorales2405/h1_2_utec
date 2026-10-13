#!/usr/bin/env python3
"""Seguidor de linea del H1-2 (reto R40-RT-H1_2-0001). MUEVE EL ROBOT, salvo con --simulacro.

    ./seguidor_linea.sh --nivel 1 --simulacro     # Hito 3: todo el lazo sin Move; el operador lleva el robot
    ./seguidor_linea.sh --nivel 1                 # primeras tiradas de cada nivel, a la mitad (--escala 0.5)
    ./seguidor_linea.sh --nivel 1 --escala 1      # despues de una tirada limpia

Bloques (seccion 5 del PDF): fuente (camara_servidor.py por ZMQ) -> percepcion -> estimacion (con la
IMU) -> control -> supervisor, el unico que llama a Move y StopMove (seguidor/*.py). Ordenes a 20 Hz,
cada Move dura 1 s: si este proceso muere, el robot para solo.

Antes: ./camara_servidor.sh en otra terminal (emisor apagado), robot de pie en FSM 201 en el cuadro
de inicio, L2+B en la mano del operador y nadie en la franja de 1.5 m. Sin --simulacro pide escribir
SEGUIR (hace falta ssh -t). Paran el programa: la tecla ESPACIO en este terminal, el vigilante
(inclinacion > 20 grados, motor en fallo, rt/lowstate o camara sin datos, joystick tocado, boton de
parada), la FSM distinta de 201, la linea perdida mas de t_perdida_s, Ctrl+C y cualquier excepcion.
Todas mandan velocidad 0 (StopMove) y la repiten 1.5 s: el robot deja de andar y de girar y se queda de
pie, en FSM 201. L2+B del mando sigue siendo la parada de emergencia.

En simulacro recorre los mismos estados y registra la orden que mandaria, pero no crea el LocoClient.
El joystick no para (el operador lleva el robot con el) y la estima de avance usa la vx que se habria
mandado, asi que la distancia a la barra a ciegas no es la real.

Datos: datos/tirada_<fecha>_nivel<N>[_simulacro]/ con fotogramas.csv + ir/ (1 de cada --cada),
lowstate.csv (100 Hz), medidas.csv (cada fotograma), ordenes.csv (cada ciclo) y resumen.json. Tiene
el formato de un dataset: evaluar_percepcion.py y reproducir.py lo leen.
"""

import argparse
import math
import os
import signal
import sys
import threading
import time
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from seguidor import config, registro  # noqa: E402
from seguidor.control import Control  # noqa: E402
from seguidor.estimacion import Estimador, HistorialImu  # noqa: E402
from seguidor.fuentes import EscritorDataset, FuenteZmq  # noqa: E402
from seguidor.geometria import Intrinsecos  # noqa: E402
from seguidor.percepcion import Percepcion  # noqa: E402
from seguidor.robot import FSM_MARCHA, Robot  # noqa: E402
from seguidor.supervisor import Supervisor  # noqa: E402
from seguidor.teclado import Teclado  # noqa: E402
from seguidor.vigilante import Vigilante  # noqa: E402

COLS_MEDIDAS = ["t", "t_proceso", "n", "ms", "confianza", "usada", "y", "theta_deg", "kappa", "obj_x", "obj_y",
                "x_ini", "x_fin", "barra", "esquina_x", "esquina_sentido", "polaridad", "n_franjas",
                "roll_lento_deg", "pitch_lento_deg", "estado"]
COLS_ORDENES = ["t", "estado", "motivo", "enviada", "vx", "vy", "vyaw", "confianza", "edad_s", "y", "theta_deg",
                "kappa", "obj_x", "obj_y", "extrapolado", "alfa_deg", "curvatura", "dist_fin", "esquina", "v_est",
                "rumbo_ref_deg", "yaw_deg", "edad_fotograma", "racha"]


def deg(x):
    return math.degrees(x) if x is not None and math.isfinite(x) else math.nan


def main():
    cfg = config.cargar()
    ap = argparse.ArgumentParser(description="Seguidor de linea del H1-2 (MUEVE EL ROBOT salvo con --simulacro)")
    ap.add_argument("--nivel", type=int, required=True, choices=(1, 2, 3, 4))
    ap.add_argument("--escala", type=float, default=cfg["limites"]["escala_primeras"],
                    help="fraccion de vx y vyaw maximas (las primeras tiradas de cada nivel, 0.5)")
    ap.add_argument("--simulacro", action="store_true", help="todo el lazo sin mandar ningun Move")
    ap.add_argument("--cada", type=int, default=2, help="guardar 1 de cada N fotogramas (0 = ninguno)")
    ap.add_argument("--nota", default="")
    args = ap.parse_args()
    if not 0.0 < args.escala <= 1.0:
        sys.exit("--escala tiene que estar en (0, 1]")
    sup_cfg = cfg["supervisor"]
    geo = cfg["geometria"]
    if not geo.get("calibrada"):
        sys.exit("La geometria de la camara no esta calibrada (herramientas/calibrar_camara.py).")

    # --- robot ------------------------------------------------------------------------------------------
    robot = Robot(cfg["red"]["iface"])
    if not robot.esperar_lowstate(3.0):
        sys.exit("No llega rt/lowstate.")
    imu = HistorialImu()
    robot.al_lowstate(imu.al_lowstate)
    fsm, texto = robot.leer_fsm()
    print(f"FSM {fsm} · motores en fallo {robot.motores_en_fallo() or 'ninguno'} · T max {robot.temp_max()} C")
    if fsm not in FSM_MARCHA:
        if not args.simulacro:
            sys.exit(f"La FSM tiene que ser {FSM_MARCHA} (L2+UP en el mando). Leido: {texto[:200]}")
        print(f"  (simulacro: FSM {fsm}, se sigue igual)")
    vigilante = Vigilante(robot, cfg, joystick_para=not args.simulacro)
    motivo = vigilante.motivo_parada()
    if motivo:
        sys.exit(f"No se empieza: {motivo}")

    # --- camara ----------------------------------------------------------------------------------------
    fuente = FuenteZmq(cfg["red"]["zmq_fotogramas"], solo_ultimo=True)
    f = fuente.siguiente(timeout_s=3.0)
    if f is None or f.ir is None:
        sys.exit("No llegan fotogramas IR: ./camara_servidor.sh (y ./camara_servidor.sh log)")
    if f.emisor:
        print("AVISO: el emisor IR esta encendido (puntos sobre la cinta). Se anda con el emisor apagado.")
    meta_camara = f.meta
    per = Percepcion(cfg["percepcion"], Intrinsecos.desde_dict(f.meta["ir"]), geo)
    estimador = Estimador(cfg, imu, geo.get("x_puntera_m"))
    control = Control(cfg, args.escala)
    sup = Supervisor(cfg, robot, vigilante, control, estimador, nivel=args.nivel, simulacro=args.simulacro)

    # vista previa: 1.5 s de linea (tambien llena la postura lenta de la IMU)
    confs = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.5:
        f = fuente.siguiente(timeout_s=0.5)
        if f is not None:
            m = per.procesar(f, *estimador.postura(f.t_rx))
            confs.append(m.confianza)
    print(f"Linea: confianza mediana {np.median(confs) if confs else 0:.2f}; ultima medida y {100 * m.y:+.1f} cm, "
          f"angulo {deg(m.theta):+.1f} grados, polaridad {per.detalle.polaridad:+d}"
          + (f", barra a {m.barra_fin:.2f} m" if m.barra_fin is not None else ""))

    # --- registro ----------------------------------------------------------------------------------------
    sufijo = f"nivel{args.nivel}" + ("_simulacro" if args.simulacro else f"_escala{args.escala:g}")
    carpeta = registro.carpeta_tirada(config.ruta_datos(cfg), "tirada", sufijo)
    cerrojo = threading.Lock()
    ctx = {"estado": "ESPERA", "vx": 0.0, "vy": 0.0, "vyaw": 0.0}

    def contexto():
        with cerrojo:
            return dict(ctx)

    reg_low = registro.RegistroLowstate(os.path.join(carpeta, "lowstate.csv"), cfg["registro"]["lowstate_cada"], contexto)
    robot.al_lowstate(reg_low.al_recibir)
    medidas = registro.CsvSimple(os.path.join(carpeta, "medidas.csv"), COLS_MEDIDAS)
    ordenes = registro.CsvSimple(os.path.join(carpeta, "ordenes.csv"), COLS_ORDENES)
    escritor = EscritorDataset(carpeta) if args.cada > 0 else None

    def cerrar_registros():
        reg_low.cerrar()
        medidas.cerrar()
        ordenes.cerrar()
        if escritor:
            escritor.cerrar()

    # --- confirmacion --------------------------------------------------------------------------------------
    print(f"\nNivel {args.nivel} · escala {args.escala} (vx max {control.vx_max:.2f} m/s, vyaw max "
          f"{control.vyaw_max:.2f} rad/s) · ordenes a {sup_cfg['hz_ordenes']} Hz · datos en {carpeta}/")
    if args.simulacro:
        print("  SIMULACRO: no se manda ningun Move. Lleva el robot con el joystick; tapar la camara debe acabar en PARADA.")
    else:
        print("==================================================================")
        print("  ESTO MUEVE EL ROBOT: sigue la linea hasta la barra de fin.")
        print("  L2 + B en el mando es la parada de emergencia. Tenlo en la mano.")
        print("  ESPACIO en este terminal: parada (velocidad 0, el robot se queda de pie).")
        print("  Nadie en la franja de 1.5 m de la pista. Joysticks a cero y sin tocar.")
        print("==================================================================")
        if not sys.stdin.isatty():
            cerrar_registros()
            sys.exit("Hace falta un terminal para confirmar: ssh -t (o ./seguidor_linea.sh desde la PC)")
        robot.habilitar_marcha()
        try:
            if input(f"  Escribe {sup_cfg['palabra']} para empezar: ").strip() != sup_cfg["palabra"]:
                print("Cancelado.")
                cerrar_registros()
                return
        except (EOFError, KeyboardInterrupt):
            cerrar_registros()
            return

    parar = {"motivo": None}
    signal.signal(signal.SIGINT, lambda *_: parar.__setitem__("motivo", "Ctrl+C"))
    signal.signal(signal.SIGTERM, lambda *_: parar.__setitem__("motivo", "SIGTERM"))

    # FSM en segundo plano: distinta de 201/204 -> parada (en simulacro solo se avisa)
    fin_hilo = threading.Event()

    def vigilar_fsm():
        while not fin_hilo.wait(sup_cfg["fsm_cada_s"]):
            valor, _ = robot.leer_fsm()
            if valor is not None and valor not in FSM_MARCHA:
                if args.simulacro:
                    print(f"\n  (simulacro: FSM {valor})")
                else:
                    sup.aviso_externo = f"FSM {valor} (distinta de {FSM_MARCHA})"
    threading.Thread(target=vigilar_fsm, daemon=True).start()

    # mientras se escribia SEGUIR no se leia la camara y las colas de ZMQ se quedaron con fotogramas viejos:
    # fuera, y empezar con uno reciente (2026-10-07: si no, el vigilante paraba por "camara sin fotogramas")
    f = fuente.fresco()
    if f is None:
        cerrar_registros()
        sys.exit("No llegan fotogramas recientes de la camara: ./camara_servidor.sh estado (y log)")

    # --- lazo -------------------------------------------------------------------------------------------------
    periodo = 1.0 / sup_cfg["hz_ordenes"]
    ultimo_foto = f.t_rx
    ms, n_fotos = [], 0
    t_pantalla = 0.0
    inicio = time.monotonic()
    # ESPACIO: parada del operador. El supervisor la ve en la siguiente vuelta del lazo (< 50 ms). Se abre
    # justo antes del try: el finally devuelve el terminal a su modo normal
    teclado = Teclado(lambda: parar.__setitem__("motivo", "tecla ESPACIO (parada del operador)"))
    print("  ESPACIO = parar" if teclado.activo else "  (sin terminal: la tecla ESPACIO no funciona; Ctrl+C si)")
    sup.empezar(inicio)
    t_tick = inicio
    teclado.abrir()
    try:
        while not sup.terminado:
            if parar["motivo"]:
                sup.detener(time.monotonic(), parar["motivo"])
                break
            f = fuente.siguiente(timeout_s=max(0.0, t_tick - time.monotonic()))
            if f is not None and f.ir is not None:
                if escritor and n_fotos % args.cada == 0:
                    escritor.escribir(f)
                n_fotos += 1
                roll, pitch = estimador.postura(f.t_rx)
                m = per.procesar(f, roll, pitch)
                usada = estimador.medida(m)
                ultimo_foto = f.t_rx
                ms.append(m.ms)
                medidas.escribir({
                    "t": m.t, "t_proceso": time.monotonic(), "n": f.n, "ms": m.ms, "confianza": m.confianza,
                    "usada": usada, "y": m.y, "theta_deg": deg(m.theta), "kappa": m.kappa, "obj_x": m.objetivo[0],
                    "obj_y": m.objetivo[1], "x_ini": m.alcance[0], "x_fin": m.alcance[1],
                    "barra": m.barra_fin if m.barra_fin is not None else math.nan,
                    "esquina_x": m.esquina[0] if m.esquina else math.nan, "esquina_sentido": m.esquina[1] if m.esquina else "",
                    "polaridad": per.detalle.polaridad, "n_franjas": m.n_franjas,
                    "roll_lento_deg": deg(roll), "pitch_lento_deg": deg(pitch), "estado": sup.estado})
            ahora = time.monotonic()
            if ahora < t_tick:
                continue
            t_tick = max(t_tick + periodo, ahora + 0.5 * periodo)
            dec = sup.paso(ahora, ahora - ultimo_foto)
            o, d = dec.orden_enviada, dec.detalle
            est = d.get("est")
            with cerrojo:
                ctx.update(estado=dec.estado, vx=o.vx, vy=o.vy, vyaw=o.vyaw)
            fila = {"t": ahora, "estado": dec.estado, "motivo": dec.motivo, "enviada": d.get("enviada", False),
                    "vx": o.vx, "vy": o.vy, "vyaw": o.vyaw, "alfa_deg": deg(d.get("alfa")), "curvatura": d.get("curvatura", math.nan),
                    "yaw_deg": deg(robot.imu().yaw), "edad_fotograma": ahora - ultimo_foto}
            if est is not None:
                fila.update(confianza=est.confianza, edad_s=est.edad_s, y=est.y, theta_deg=deg(est.theta), kappa=est.kappa,
                            obj_x=est.objetivo[0], obj_y=est.objetivo[1], extrapolado=est.extrapolado,
                            dist_fin=est.dist_fin if est.dist_fin is not None else math.nan,
                            esquina=f"{est.esquina[0]:.3f}/{est.esquina[1]:+d}" if est.esquina else "",
                            v_est=est.v, rumbo_ref_deg=deg(est.rumbo_ref), racha=est.racha)
            ordenes.escribir(fila)
            if ahora - t_pantalla > 0.25:
                t_pantalla = ahora
                linea = f"\r{ahora - inicio:6.1f} s {dec.estado:13s} vx {o.vx:4.2f} vyaw {o.vyaw:+5.2f}"
                if est is not None and est.hay_linea:
                    linea += (f" · conf {est.confianza:4.2f} edad {min(est.edad_s, 99):4.1f} s · y {100 * est.y:+5.1f} cm "
                              f"ang {deg(est.theta):+5.1f}")
                    if est.dist_fin is not None:
                        linea += f" · barra {est.dist_fin:+5.2f} m"
                sys.stdout.write(linea + "   ")
                sys.stdout.flush()
    except Exception as e:
        traceback.print_exc()
        sup.detener(time.monotonic(), f"excepcion: {e!r}")
    finally:
        fin_hilo.set()
        if not sup.terminado:
            sup.detener(time.monotonic(), parar["motivo"] or "fin del programa")
        with cerrojo:
            ctx.update(estado=sup.estado, vx=0.0, vy=0.0, vyaw=0.0)
        # mantener el robot quieto y de pie: velocidad 0 cada 0.1 s mientras se registra como para
        t_fin = time.monotonic() + 1.5
        while time.monotonic() < t_fin:
            try:
                sup.quieto()
            except Exception as e:
                print(f"\nNo se pudo mandar velocidad 0: {e}")
                break
            time.sleep(0.1)
        teclado.cerrar()
        cerrar_registros()
        fuente.cerrar()

    duracion = time.monotonic() - inicio
    print(f"\n\n{sup.estado}: {sup.motivo}")
    for t, de, a, mot in sup.transiciones:
        print(f"  {t:7.2f} s  {de} -> {a}: {mot}")
    ms = np.array(ms) if ms else np.array([math.nan])
    resumen = {
        "nivel": args.nivel, "escala": args.escala, "simulacro": args.simulacro, "nota": args.nota,
        "fsm_inicial": fsm, "estado_final": sup.estado, "motivo": sup.motivo, "duracion_s": round(duracion, 2),
        "transiciones": sup.transiciones, "tiempo_por_estado_s": {k: round(v, 2) for k, v in sup.tiempos.items()},
        "moves_enviados": sup.ordenes_enviadas, "parada_con_espacio": teclado.pulsada, "fotogramas": n_fotos, "guardados": escritor.escritos if escritor else 0,
        "percepcion_ms": {"mediana": float(np.nanmedian(ms)), "p95": float(np.nanpercentile(ms, 95)),
                          "max": float(np.nanmax(ms))},
        "meta_camara": meta_camara, "geometria": geo, "config": cfg,
        "pendiente": "medir con cinta la distancia de la puntera a la barra al parar" if sup.estado == "FIN" else "",
    }
    registro.guardar_json(os.path.join(carpeta, "resumen.json"), resumen)
    print(f"\n{sup.ordenes_enviadas} Move enviados · {n_fotos} fotogramas · percepcion {resumen['percepcion_ms']['mediana']:.1f} ms"
          f" (p95 {resumen['percepcion_ms']['p95']:.1f}) · {duracion:.1f} s\nDatos: {carpeta}/")


if __name__ == "__main__":
    main()
