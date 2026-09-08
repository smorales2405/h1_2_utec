#!/usr/bin/env python3
"""Temporización del lazo (fase F0). NO mueve el robot.

Publica en el tópico inerte de pruebas —el que nadie escucha— y mide dos cosas
a la vez:

  * el `dt` de nuestro lazo de control, que es lo que determina la fase real
    con la que llegan las consignas;
  * el intervalo de llegada de `/lowstate`, que es lo que determina si la `q`
    con la que se calculan todas las métricas está fresca o rancia.

La segunda es la que puede morder: si el flujo de estado pierde muestras, la
posición registrada está vieja y **todas** las métricas salen sesgadas, sin que
nada lo delate.

Por qué el jitter preocupa menos de lo que parece en este paquete: la
trayectoria se evalúa contra tiempo de reloj (`func(now - t0)` con
`time.monotonic()`), no contra un contador de ciclos. Un ciclo tarde sigue
publicando la consigna correcta para ese instante, así que el jitter añade
ruido de muestreo y no error de fase en el comando.

Uso:
    source scripts/env.sh
    python3 scripts/10_loop_timing.py                 # 250 y 500 Hz, 120 s cada uno
    python3 scripts/10_loop_timing.py --seconds 300   # los 5 min del protocolo
    python3 scripts/10_loop_timing.py --rates 250     # solo una frecuencia
"""
from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_hg.msg import LowCmd, LowState

from h1_2_joint_control import config as cfg
from h1_2_joint_control import recorder as rec
from h1_2_joint_control.crc import set_crc
from h1_2_joint_control.joints import NUM_CMD_MOTOR

TOPIC_DRY = "/h1_2_joint_control/dry_run"
T_STATE = 1.0 / 500.0            # el robot publica /lowstate a 500 Hz


class Sonda(Node):
    def __init__(self):
        super().__init__("h1_2_loop_timing")
        self.llegadas: list[float] = []
        self.ultimo = 0.0
        self.pub = self.create_publisher(LowCmd, TOPIC_DRY, 10)
        self.create_subscription(LowState, "/lowstate", self._cb,
                                 qos_profile_sensor_data)
        self.mode_machine = 0

    def _cb(self, msg: LowState):
        t = time.perf_counter()
        self.llegadas.append(t)
        self.ultimo = t
        self.mode_machine = int(msg.mode_machine)


def resumen(v, escala=1000.0):
    a = np.asarray(v) * escala
    return dict(n=len(a), mediana=float(np.median(a)),
                p95=float(np.percentile(a, 95)), p99=float(np.percentile(a, 99)),
                maximo=float(a.max()), minimo=float(a.min()))


def corrida(sonda, rate, seconds, con_registro, gravedad=None):
    """Un bloque de medida. Devuelve (stats de dt, stats de /lowstate, extras)."""
    T = 1.0 / rate
    msg = LowCmd()
    msg.mode_pr = 0
    msg.mode_machine = sonda.mode_machine
    for i in range(NUM_CMD_MOTOR):
        msg.motor_cmd[i].mode = 1
        msg.motor_cmd[i].kp = 0.0      # inerte de todas formas: nadie escucha
        msg.motor_cmd[i].kd = 0.0

    registro = [] if con_registro else None
    sonda.llegadas.clear()
    dts: list[float] = []
    # Antigüedad del estado en el instante en que el lazo lo usa. ES LA MÉTRICA
    # QUE IMPORTA, y no el recuento de huecos en la llegada: un lazo que en
    # cada ciclo lee la ÚLTIMA posición conocida no sufre por perderse mensajes
    # intermedios, sufre si el más reciente es viejo.
    edades: list[float] = []
    t0 = time.perf_counter()
    nxt = t0
    tarde = 0
    n = int(seconds * rate)
    for k in range(n):
        nxt += T
        edades.append(time.perf_counter() - sonda.ultimo)
        set_crc(msg)
        sonda.pub.publish(msg)
        if gravedad is not None:
            gravedad()
        if registro is not None:
            registro.append((time.perf_counter() - t0, msg.crc))
        s = nxt - time.perf_counter()
        if s > 0:
            time.sleep(s)
        else:
            tarde += 1
            nxt = time.perf_counter()
        dts.append(time.perf_counter())
    dur = time.perf_counter() - t0

    dt = np.diff(dts)
    llegadas = list(sonda.llegadas)
    gaps = np.diff(llegadas) if len(llegadas) > 2 else np.array([T_STATE])
    perdidas = int(np.sum(gaps > 2 * T_STATE))
    ed = np.asarray(edades) * 1000.0
    extras = dict(
        edad_mediana=float(np.median(ed)), edad_p99=float(np.percentile(ed, 99)),
        edad_max=float(ed.max()),
        duracion=dur, ciclos=n, tarde=tarde,
        hz_real=n / dur, deriva_ms=(dur - n * T) * 1000.0,
        hz_state=(len(llegadas) - 1) / dur if len(llegadas) > 1 else 0.0,
        perdidas=perdidas,
        registro=len(registro) if registro is not None else 0,
    )
    return resumen(dt), resumen(gaps), extras


def linea(nombre, d, g, e, T_ms):
    print(f"  {nombre:<24} {d['mediana']:>6.3f} {d['p99']:>6.3f} {d['maximo']:>7.3f} "
          f"{d['p99']/T_ms:>6.2f} {e['tarde']*100/e['ciclos']:>6.2f}% "
          f"{e['hz_state']:>7.0f} {e['edad_mediana']:>7.2f} {e['edad_p99']:>7.2f} "
          f"{e['edad_max']:>7.2f} {0.5*e['edad_max']:>8.2f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=120.0,
                    help="duración de cada bloque (el protocolo pide 300)")
    ap.add_argument("--rates", default="250,500",
                    help="frecuencias a medir, separadas por comas")
    ap.add_argument("--no-gravity", action="store_true",
                    help="no medir el bloque con g(q) dentro del ciclo")
    a = ap.parse_args()
    rates = [float(x) for x in a.rates.split(",") if x.strip()]

    rclpy.init()
    sonda = Sonda()
    ex = SingleThreadedExecutor()
    ex.add_node(sonda)
    hilo = threading.Thread(target=ex.spin, daemon=True)
    hilo.start()

    try:
        t0 = time.monotonic()
        while not sonda.llegadas and time.monotonic() - t0 < 15:
            time.sleep(0.05)
        if not sonda.llegadas:
            print("✗ no llega /lowstate. Con el robot desconectado esta fase solo\n"
                  "  puede medir el suelo del sistema, no la carga real.", file=sys.stderr)
            return 1
        print(f"\n  /lowstate llegando, mode_machine={sonda.mode_machine}")
        print(f"  {len(rates)} frecuencia(s) × {a.seconds:.0f} s"
              + ("" if a.no_gravity else " + un bloque con g(q) en el ciclo"))
        total = a.seconds * (len(rates) + (0 if a.no_gravity else 1) + 1)
        print(f"  duración aproximada: {total/60:.1f} min\n")

        grav = None
        if not a.no_gravity:
            try:
                import pinocchio as pin
                urdf = (Path.home() / "humanoid_ws/src/h1_2_utec/h1_2_description"
                        / "urdf/h1_2.urdf")
                mdl = pin.buildModelFromUrdf(str(urdf))
                dat = mdl.createData()
                qn = pin.neutral(mdl)
                grav = lambda: pin.computeGeneralizedGravity(mdl, dat, qn)
                print("  modelo de gravedad cargado para el bloque extra\n")
            except Exception as exc:
                print(f"  (sin bloque de gravedad: {exc})\n")
                grav = None

        print(f"  {'':<24} {'---- dt del lazo ----':^21} {'':>6} {'':>7} "
              f"{'-- edad del estado --':^24}")
        print(f"  {'bloque':<24} {'med':>6} {'p99':>6} {'max':>7} "
              f"{'p99/T':>6} {'tarde':>7} {'state':>7} {'med':>7} {'p99':>7} "
              f"{'max':>7} {'err máx':>8}")
        print(f"  {'':<24} {'ms':>6} {'ms':>6} {'ms':>7} "
              f"{'':>6} {'':>7} {'Hz':>7} {'ms':>7} {'ms':>7} {'ms':>7} {'mrad':>8}")
        print("  " + "─" * 108)

        filas = []
        for r in rates:
            d, g, e = corrida(sonda, r, a.seconds, con_registro=False)
            linea(f"{r:.0f} Hz", d, g, e, 1000.0 / r)
            filas.append((f"{r:.0f} Hz", d, g, e))

        r0 = rates[0]
        d, g, e = corrida(sonda, r0, a.seconds, con_registro=True)
        linea(f"{r0:.0f} Hz + registro", d, g, e, 1000.0 / r0)
        filas.append((f"{r0:.0f} Hz + registro", d, g, e))

        if grav is not None:
            d, g, e = corrida(sonda, r0, a.seconds, con_registro=False, gravedad=grav)
            linea(f"{r0:.0f} Hz + g(q)", d, g, e, 1000.0 / r0)
            filas.append((f"{r0:.0f} Hz + g(q)", d, g, e))

        print()
        peor_edad = max(f[3]["edad_max"] for f in filas)
        err_max = 0.5 * peor_edad          # mrad a 0.5 rad/s
        print(f"  Criterio literal del protocolo (dt p99 ≤ 1.2·T, cero perdidas): "
              f"{'cumple' if all(f[1]['p99'] <= 1.2*1000.0/rates[0] for f in filas if f[0].startswith(f'{rates[0]:.0f}')) and all(f[3]['perdidas']==0 for f in filas) else 'NO CUMPLE'}")
        print(f"  Criterio corregido (edad del estado): peor caso {peor_edad:.2f} ms")
        print(f"    -> a 0.5 rad/s introduce {err_max:.2f} mrad de error aparente,")
        print(f"       contra errores medidos de 5 a 32 mrad. "
              f"{'ACEPTABLE' if err_max < 2.0 else 'REVISAR'}")
        print(f"\n  Por qué el criterio literal no es el bueno para este sistema:")
        print(f"    · la trayectoria se evalúa contra reloj, no contra contador de")
        print(f"      ciclos, así que un ciclo tarde publica la consigna CORRECTA;")
        print(f"    · el lazo lee la última posición conocida, así que perderse")
        print(f"      mensajes intermedios de /lowstate es inocuo. Lo que sesga es")
        print(f"      que el más reciente sea viejo, y eso es la edad.")

        stamp = rec.stamp()
        for nombre, d, g, e in filas:
            rec.append_index({"stamp": stamp, "test": "loop_timing", "channel": "dry_run",
                              "gains": "", "rate_hz": nombre, "csv": "",
                              "dt_mediana_ms": d["mediana"], "dt_p95_ms": d["p95"],
                              "dt_p99_ms": d["p99"], "dt_max_ms": d["maximo"],
                              "tarde_pct": 100 * e["tarde"] / e["ciclos"],
                              "deriva_ms": e["deriva_ms"], "hz_state": e["hz_state"],
                              "state_p99_ms": g["p99"], "perdidas": e["perdidas"]})
        print(f"\n  guardado en {rec.INDEX}")
        return 0
    finally:
        ex.shutdown()
        hilo.join(timeout=2.0)
        sonda.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
