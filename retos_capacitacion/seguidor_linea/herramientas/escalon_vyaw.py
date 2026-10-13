#!/usr/bin/env python3
"""Respuesta del H1-2 a escalones de vyaw (preguntas 6.3.1 y 6.3.2 del PDF). MUEVE EL ROBOT.

En lazo abierto, para ver la respuesta pura del control de marcha de Unitree, con vx constante:
  base     vyaw = 0    --base s
  escalon  vyaw = +A   --escalon s
  vuelta   vyaw = 0    --vuelta s   (sirve de base al siguiente escalon)
  escalon  vyaw = -A   ...          (el signo alterna para volver al rumbo de partida)
Sobre el yaw de la IMU mide la deriva de la base, la velocidad de regimen y la ganancia, el
retardo efectivo y el tiempo hasta 2 grados (seguidor/analisis.py). Cada Move queda en
ordenes.csv con lo que tardo el RPC (una parte de la latencia de 6.3.2).

    ./ejecutar.sh herramientas/escalon_vyaw.py --simulacro            # todo menos los Move
    ./ejecutar.sh herramientas/escalon_vyaw.py --vx 0 --amplitud 0.3
    ./ejecutar.sh herramientas/escalon_vyaw.py --vx 0.2 --amplitud 0.3 --base 2 --escalon 2 --vuelta 2

Antes: robot de pie en FSM 201, espacio libre (con vx > 0 avanza vx * duracion: el plan lo dice
antes de empezar), L2+B en la mano, nadie toca el joystick (tocarlo para la prueba y cuenta como
intervencion). Va por ssh -t: pide escribir ESCALON. Ante cualquier motivo del vigilante, Ctrl+C
o una excepcion manda StopMove; ademas cada Move dura 1 s, asi que si el programa muere el robot
para solo. Datos: datos/escalon_<fecha>_<...>/ lowstate.csv (100 Hz), ordenes.csv y resumen.json.
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

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import config, registro  # noqa: E402
from seguidor.analisis import respuesta_escalon  # noqa: E402
from seguidor.geometria import yaw_de  # noqa: E402
from seguidor.robot import FSM_MARCHA, Robot  # noqa: E402
from seguidor.vigilante import Vigilante  # noqa: E402

PALABRA = "ESCALON"


class Parada(Exception):
    pass


def envolver(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser(description="Escalones de vyaw en lazo abierto (MUEVE EL ROBOT)")
    ap.add_argument("--vx", type=float, default=0.0, help="m/s durante toda la prueba (0 = en el sitio)")
    ap.add_argument("--amplitud", type=float, default=0.3, help="rad/s del escalon")
    ap.add_argument("--escalones", type=int, default=2, help="escalones, alternando +A y -A")
    ap.add_argument("--base", type=float, default=3.0, help="s con vyaw = 0 antes del primero")
    ap.add_argument("--escalon", type=float, default=3.0, help="s de cada escalon")
    ap.add_argument("--vuelta", type=float, default=3.0, help="s con vyaw = 0 despues de cada escalon")
    ap.add_argument("--espacio", type=float, default=3.0, help="m libres por delante: no se empieza si avanzaria mas")
    ap.add_argument("--simulacro", action="store_true", help="no manda ningun Move: lee, registra y vigila")
    args = ap.parse_args()

    cfg = config.cargar()
    lim = cfg["limites"]
    hz = cfg["supervisor"]["hz_ordenes"]
    if not 0.0 <= args.vx <= lim["vx_max"]:
        sys.exit(f"--vx tiene que estar entre 0 y {lim['vx_max']} m/s")
    if not 0.0 < args.amplitud <= lim["vyaw_max"]:
        sys.exit(f"--amplitud tiene que estar entre 0 y {lim['vyaw_max']} rad/s")
    if args.escalones < 1 or min(args.base, args.escalon, args.vuelta) < 1.0:
        sys.exit("hace falta al menos un escalon y fases de 1 s o mas")
    total = args.base + args.escalones * (args.escalon + args.vuelta)
    avance = args.vx * total
    if avance > args.espacio:
        sys.exit(f"avanzaria {avance:.1f} m y --espacio es {args.espacio} m: menos vx, fases mas cortas o menos escalones")

    robot = Robot(cfg["red"]["iface"])
    if not robot.esperar_lowstate(3.0):
        sys.exit("No llega rt/lowstate.")
    fsm, texto = robot.leer_fsm()
    print(f"FSM {fsm} · motores en fallo {robot.motores_en_fallo() or 'ninguno'} · T max {robot.temp_max()} C")
    if fsm not in FSM_MARCHA and not args.simulacro:
        sys.exit(f"La FSM tiene que ser {FSM_MARCHA} (L2+UP en el mando). Leido: {texto[:200]}")
    vigilante = Vigilante(robot, cfg)
    motivo = vigilante.motivo_parada()
    if motivo:
        sys.exit(f"No se empieza: {motivo}")

    print(f"\nPlan: vx {args.vx} m/s; base {args.base} s, luego {args.escalones} escalon(es) de "
          f"{args.amplitud} rad/s x {args.escalon} s ({math.degrees(args.amplitud * args.escalon):.0f} grados cada uno, "
          f"alternando el sentido) con {args.vuelta} s de vuelta.\n      {total:.0f} s en total, "
          f"avanza ~{avance:.1f} m. Ordenes a {hz} Hz.")

    sufijo = f"vx{args.vx:g}_a{args.amplitud:g}" + ("_simulacro" if args.simulacro else "")
    carpeta = registro.carpeta_tirada(config.ruta_datos(cfg), "escalon", sufijo)
    cerrojo = threading.Lock()
    estado = {"estado": "espera", "vx": 0.0, "vy": 0.0, "vyaw": 0.0}

    def contexto():
        with cerrojo:
            return dict(estado)

    def poner(**kw):
        with cerrojo:
            estado.update(kw)

    muestras = []   # (t, yaw, gz) de cada rt/lowstate, para el analisis

    def guardar_muestra(msg, t):
        muestras.append((t, yaw_de(msg.imu_state.quaternion), msg.imu_state.gyroscope[2]))

    reg_low = registro.RegistroLowstate(os.path.join(carpeta, "lowstate.csv"), cfg["registro"]["lowstate_cada"], contexto)
    robot.al_lowstate(reg_low.al_recibir)
    robot.al_lowstate(guardar_muestra)
    ordenes = registro.CsvSimple(os.path.join(carpeta, "ordenes.csv"),
                                 ["t_envio", "t_vuelta", "estado", "escalon", "vx", "vy", "vyaw"])

    if args.simulacro:
        print("\n  SIMULACRO: no se manda ningun Move.")
    else:
        print("==================================================================")
        print("  ESTO MUEVE EL ROBOT: escalones de giro" + (f" andando a {args.vx} m/s" if args.vx else " en el sitio"))
        print("  L2 + B en el mando es la parada de emergencia. Tenlo en la mano.")
        print("  Tocar el joystick para la prueba.")
        print("==================================================================")
        if not sys.stdin.isatty():
            sys.exit("Hace falta un terminal para confirmar: ssh -t")
        robot.habilitar_marcha()
        try:
            if input(f"  Escribe {PALABRA} para empezar: ").strip() != PALABRA:
                print("Cancelado.")
                reg_low.cerrar()
                ordenes.cerrar()
                return
        except (EOFError, KeyboardInterrupt):
            reg_low.cerrar()
            ordenes.cerrar()
            return

    parar = {"si": False}
    signal.signal(signal.SIGINT, lambda *_: parar.__setitem__("si", True))
    signal.signal(signal.SIGTERM, lambda *_: parar.__setitem__("si", True))

    fases = [("base", 0.0, args.base, 0)]
    for k in range(1, args.escalones + 1):
        fases += [("escalon", args.amplitud if k % 2 else -args.amplitud, args.escalon, k),
                  ("vuelta", 0.0, args.vuelta, k)]
    periodo = 1.0 / hz
    yaw0 = robot.imu().yaw
    hechas, rpc = [], []
    motivo_final = "completado"
    t_prueba = time.monotonic()
    try:
        for nombre, vyaw, duracion, k in fases:
            poner(estado=nombre, vx=args.vx, vy=0.0, vyaw=vyaw)
            t_ini = time.monotonic()
            while time.monotonic() - t_ini < duracion:
                ciclo = time.monotonic()
                if parar["si"]:
                    raise Parada("Ctrl+C")
                motivo = vigilante.motivo_parada()
                if motivo:
                    raise Parada(motivo)
                t_envio = time.monotonic()
                if not args.simulacro:
                    robot.mover(args.vx, 0.0, vyaw)   # dura 1 s: si esto muere, el robot para solo
                t_vuelta = time.monotonic()
                rpc.append(t_vuelta - t_envio)
                ordenes.escribir({"t_envio": t_envio, "t_vuelta": t_vuelta, "estado": nombre, "escalon": k,
                                  "vx": args.vx, "vy": 0.0, "vyaw": vyaw})
                sys.stdout.write(f"\r{nombre:8s} {k}  {time.monotonic() - t_ini:4.1f}/{duracion:.0f} s  vyaw {vyaw:+.2f}"
                                 f"  rumbo {math.degrees(envolver(robot.imu().yaw - yaw0)):+6.1f} grados   ")
                sys.stdout.flush()
                resto = periodo - (time.monotonic() - ciclo)
                if resto > 0:
                    time.sleep(resto)
            hechas.append((nombre, k, t_ini, time.monotonic(), vyaw))
    except Parada as e:
        motivo_final = str(e)
        print(f"\nPARADA: {e}")
    except Exception as e:
        motivo_final = f"excepcion: {e!r}"
        traceback.print_exc()
    finally:
        if robot.marcha_habilitada:
            try:
                robot.parar()
            except Exception as e:
                print(f"No se pudo mandar StopMove: {e}")
        poner(estado="fin", vx=0.0, vy=0.0, vyaw=0.0)
        time.sleep(1.5)   # registrar la parada
        reg_low.cerrar()
        ordenes.cerrar()

    datos = np.array(list(muestras))
    resultados = []
    if not args.simulacro and len(datos):
        for i, (nombre, k, t_ini, t_fin, vyaw) in enumerate(hechas):
            if nombre != "escalon" or i == 0:
                continue
            nombre_b, _, b_ini, b_fin, _ = hechas[i - 1]
            if nombre_b == "vuelta":
                # al principio de la vuelta el robot aun esta parando el giro anterior: solo la 2a mitad
                b_ini = (b_ini + b_fin) / 2
            try:
                r = respuesta_escalon(datos[:, 0], datos[:, 1], b_ini, t_ini, t_fin, vyaw)
            except ValueError as e:
                print(f"escalon {k}: {e}")
                continue
            r["escalon"] = k
            resultados.append(r)
        if resultados:
            print("\nescalon  orden     regimen   ganancia  retardo  hasta 2 grados  deriva base")
            for r in resultados:
                print(f"  {r['escalon']:4d}  {r['amplitud']:+.2f}    {r['vel_regimen']:+.3f}    {r['ganancia']:5.2f}"
                      f"     {r['retardo']:5.2f} s   {r['t_umbral']:5.2f} s        {math.degrees(r['deriva']):+.1f} grados/s")
    rpc_ms = 1000 * np.array(rpc) if rpc else np.array([math.nan])
    resumen = {"args": vars(args), "fsm": fsm, "motivo_parada": motivo_final,
               "duracion_s": round(time.monotonic() - t_prueba, 2), "fases_completas": len(hechas),
               "fases_previstas": len(fases), "giro_final_deg": round(math.degrees(envolver(robot.imu().yaw - yaw0)), 1),
               "rpc_move_ms": {"mediana": float(np.median(rpc_ms)), "p95": float(np.percentile(rpc_ms, 95)),
                               "max": float(np.max(rpc_ms))},
               "escalones": resultados}
    registro.guardar_json(os.path.join(carpeta, "resumen.json"), resumen)
    print(f"\nMotivo de fin: {motivo_final}. RPC de Move: mediana {resumen['rpc_move_ms']['mediana']:.1f} ms, "
          f"max {resumen['rpc_move_ms']['max']:.1f} ms.\nDatos: {carpeta}/")


if __name__ == "__main__":
    main()
