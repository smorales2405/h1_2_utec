#!/usr/bin/env python3
"""Levanta el brazo DERECHO del robot REAL y registra el movimiento.

Es la contraparte de `h1_2_algoritms/fk_right_arm_raise.py`, que hace lo mismo
sobre el simulador MuJoCo. Genera un CSV **con el mismo esquema de columnas**,
así que los dos se superponen directamente y se puede ver si la simulación se
parece al robot.

Qué hace, en orden
------------------
1. Toma el control y lleva **los dos brazos a 0°**, por los tres tramos que no
   topan con la envolvente de autocolisión. El ensayo tiene que partir siempre
   de la misma postura o no es comparable con nada.
2. Recorre una trayectoria quíntica —velocidad y aceleración nulas en los
   extremos— desde q=0 hasta `q_goal`, en los 7 motores del brazo derecho.
   Es la MISMA trayectoria y los mismos valores por defecto que el ensayo del
   simulador.
3. Registra a `rate` Hz: consigna y medida de posición, velocidad y par de las
   7 articulaciones, más la pose del efector final calculada por cinemática
   directa tanto de la consigna como de la medida.
4. Devuelve los brazos a la postura de reposo y suelta el control.

Todo en UN proceso: si se soltara el control entre medias, la gravedad se
llevaría los codos antes de que el siguiente comando enganchara.

La cinemática sale del URDF de `h1_2_inspire_description`, y coincide con la DH
del simulador a 0.00 mm y 0.0000° (ver `fk.py`). Sin esa comprobación previa,
comparar los efectores mediría la diferencia entre las dos cinemáticas.

Uso
---
    ros2 run h1_2_arm_control debug_mode --ros-args -p action:=enter
    ros2 run h1_2_arm_control fk_right_arm_raise

    # comparar con el simulador:
    ros2 run h1_2_arm_control plot_fk_raise <csv_real> \
        --compare <csv_sim> --labels "real,simulación"

Parámetros
----------
    q_goal        pose final [rad], 7 valores (por defecto, la del simulador)
    t_rise        duración de la subida [s]
    t_hold        sostenido al final [s]
    return_home   además de subir, vuelve a 0° (ciclo completo)
    rate          frecuencia de registro [Hz]
    csv_path      dónde dejar el CSV
    skip_init     no llevar a 0° primero (solo si YA está ahí; no es lo normal)

CON EL ROBOT COLGADO DEL ARNÉS.
"""
import csv as _csv
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

from ._node_base import ArmNode, ejecuta
from .fk import ArmFK
from .joints import ARM_INDICES, BY_INDEX, BY_NAME
from .postures import to_rest, to_zero

# El mismo orden que `h1_2_algoritms.joint_limits.ARM_JOINT_NAMES["right"]`,
# que es el que espera la FK y el que llevan las columnas del CSV.
RIGHT = ["R_shoulder_pitch", "R_shoulder_roll", "R_shoulder_yaw", "R_elbow",
         "R_wrist_roll", "R_wrist_pitch", "R_wrist_yaw"]
CORTO = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
         "wrist_roll", "wrist_pitch", "wrist_yaw"]

# Pose final por defecto: la MISMA del ensayo en simulación, para que los dos
# CSV sean comparables sin tocar nada.
#   shoulder_pitch = -1.40  levanta el brazo hacia adelante
#   shoulder_roll  = -0.25  lo separa un poco del torso
#   elbow          = +0.70  codo ligeramente flexionado
DEFAULT_Q_GOAL = [-1.40, -0.25, 0.30, 0.70, 0.35, 0.20, 0.40]

CABECERA = (["t"]
            + [f"q_des_{n}" for n in CORTO] + [f"q_meas_{n}" for n in CORTO]
            + [f"dq_des_{n}" for n in CORTO] + [f"dq_meas_{n}" for n in CORTO]
            + [f"tau_meas_{n}" for n in CORTO]
            + ["ee_des_x", "ee_des_y", "ee_des_z",
               "ee_des_qw", "ee_des_qx", "ee_des_qy", "ee_des_qz",
               "ee_meas_x", "ee_meas_y", "ee_meas_z",
               "ee_meas_qw", "ee_meas_qx", "ee_meas_qy", "ee_meas_qz"])


def quintic(s):
    """Perfil quíntico 0->1, velocidad y aceleración nulas en los extremos.

    Idéntico al del ensayo en simulación. Devuelve (posición, derivada
    respecto de `s` normalizado)."""
    s = min(max(s, 0.0), 1.0)
    return (10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5,
            30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4)


class FkRightArmRaise(ArmNode):
    def __init__(self, nombre="h1_2_fk_right_arm_raise"):
        super().__init__(nombre, {
            "q_goal": DEFAULT_Q_GOAL,
            "t_rise": 4.0,
            "t_hold": 2.0,
            "return_home": False,
            "rate": 100.0,
            "csv_path": "",
            "skip_init": False,
        })

    # ---------------------------------------------------------- trayectoria
    def q_ref(self, t):
        """(q_des, dq_des) de los 7 en el instante t. Igual que en simulación."""
        d = self.q_goal                       # q_home = 0, así que d = q_goal
        if t < self.t_rise:                                   # subida
            s, ds = quintic(t / self.t_rise)
            return s * d, (ds / self.t_rise) * d
        if t < self.t_rise + self.t_hold or not self.return_home:
            return d.copy(), np.zeros(7)
        t2 = t - (self.t_rise + self.t_hold)
        if t2 < self.t_rise:                                  # bajada
            s, ds = quintic(t2 / self.t_rise)
            return d - s * d, -(ds / self.t_rise) * d
        return np.zeros(7), np.zeros(7)                       # sostenido abajo

    # ----------------------------------------------------------------- run
    def run(self) -> int:
        q_goal = np.asarray(self.p("q_goal"), dtype=float)
        if q_goal.size != 7:
            print(f"  q_goal debe tener 7 elementos, tiene {q_goal.size}")
            return 1
        self.t_rise = float(self.p("t_rise"))
        self.t_hold = float(self.p("t_hold"))
        self.return_home = bool(self.p("return_home"))
        rate = float(self.p("rate"))
        idx = [BY_NAME[n].idx for n in RIGHT]

        # Saturar contra los topes articulares, y avisar en vez de callarlo.
        fk = ArmFK("right")
        gains = None
        with self.cliente() as cli:
            cli.wait_for_state()
            gains = cli.gains
            rec = []
            for k, i in enumerate(idx):
                lo, hi = gains.limits(i)
                v = min(max(float(q_goal[k]), lo), hi)
                if abs(v - q_goal[k]) > 1e-9:
                    print(f"  ⚠ {BY_INDEX[i].name}: {math.degrees(q_goal[k]):+.1f}° "
                          f"fuera de límites, saturado a {math.degrees(v):+.1f}°")
                q_goal[k] = v
            self.q_goal = q_goal

            t_end = self.t_rise + self.t_hold
            if self.return_home:
                t_end += self.t_rise + self.t_hold

            print(f"\n  Brazo DERECHO, {len(idx)} articulaciones")
            print(f"  {'articulación':<20}{'0°':>8}{'meta':>10}")
            for k, i in enumerate(idx):
                print(f"    {BY_INDEX[i].name:<18}{0.0:>7.1f}°"
                      f"{math.degrees(q_goal[k]):>9.1f}°")
            print(f"\n  subida {self.t_rise:.1f} s, sostenido {self.t_hold:.1f} s"
                  + (", vuelta a 0°" if self.return_home else "")
                  + f"  (total {t_end:.1f} s, registro a {rate:.0f} Hz)")
            self._aviso_par(cli, idx, q_goal)

            # ---- 1. los dos brazos a 0° ----
            cli.engage()
            if not self.p("skip_init"):
                print("\n── llevando los dos brazos a 0° ─────────────────────")
                to_zero(cli, speed=float(self.p("speed")))
                peor = max(abs(cli.q(i)) for i in ARM_INDICES)
                print(f"  peor desviación de 0°: {math.degrees(peor):.2f}°")
            else:
                print("\n  skip_init: se mide desde donde esté el brazo.")

            # ---- 2. la trayectoria ----
            print("\n── ensayo ───────────────────────────────────────────")
            for k, i in enumerate(idx):
                cli.set_trajectory(
                    i, (lambda k: (lambda t: (self.q_ref(t)[0][k],
                                              self.q_ref(t)[1][k])))(k),
                    q_base=0.0)
            t0 = time.monotonic()
            dt = 1.0 / rate
            siguiente = t0
            while True:
                t = time.monotonic() - t0
                if t >= t_end:
                    break
                q_des = np.array([cli.q_des(i) for i in idx])
                dq_des = np.array([cli.dq_des(i) for i in idx])
                q_m = np.array([cli.q(i) for i in idx])
                dq_m = np.array([cli.dq(i) for i in idx])
                tau_m = np.array([cli.tau(i) for i in idx])
                rec.append([t] + q_des.tolist() + q_m.tolist()
                           + dq_des.tolist() + dq_m.tolist() + tau_m.tolist()
                           + fk.pose(q_des).tolist() + fk.pose(q_m).tolist())
                siguiente += dt
                cli.sleep(max(0.0, siguiente - time.monotonic()))
            cli.clear_trajectory()
            cli.wait_all_settled(idx)
            print(f"  {len(rec)} muestras en {t:.1f} s "
                  f"({len(rec)/max(t,1e-9):.1f} Hz reales)")

            # ---- 3. a reposo ----
            print("\n── devolviendo a reposo ─────────────────────────────")
            reposo = to_rest(cli, speed=float(self.p("speed")))
            if cli.collision_clamps:
                print(f"  ⚠ autocolisión: consigna recortada en "
                      f"{cli.collision_clamps} ciclos")
            print(f"  {cli.loop_health()}")
            cli.release(home_to=reposo)

        return self._guarda(rec, fk)

    # ------------------------------------------------------------ auxiliares
    def _aviso_par(self, cli, idx, q_goal):
        """El par de gravedad en la meta, contra el umbral de aborto.

        Levantar el brazo al frente es la postura cara: el hombro pasa de
        sostener casi nada a sostener el brazo entero en voladizo. Más vale
        saber cuánto margen queda ANTES de arrancar que descubrirlo con un
        aborto a mitad del ensayo.
        """
        if cli.gravity is None:
            return
        q27 = np.zeros(27)
        for k, i in enumerate(idx):
            q27[i] = q_goal[k]
        try:
            t = cli.gravity.tau(q27)
        except Exception:
            return
        print(f"\n  {'articulación':<20}{'gravedad en la meta':>21}{'margen':>10}")
        for i in idx:
            v = abs(t.get(i, 0.0))
            tope = cli.gains.safety.tau_abort_fraction * BY_INDEX[i].tau_max
            aviso = "   ⚠ poco margen" if tope - v < 8.0 else ""
            print(f"    {BY_INDEX[i].name:<18}{v:>19.1f} N{tope - v:>9.1f} N{aviso}")

    def _guarda(self, rec, fk):
        if not rec:
            print("\n  no se registró ninguna muestra.")
            return 1
        ruta = self.p("csv_path")
        if not ruta:
            base = os.path.join(os.path.expanduser("~"), "resultados_brazo_derecho")
            os.makedirs(base, exist_ok=True)
            ruta = os.path.join(
                base, f"real_right_arm_raise_"
                      f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        with open(ruta, "w", newline="") as f:
            w = _csv.writer(f)
            w.writerow(CABECERA)
            w.writerows(rec)
        d = np.array(rec)
        i_qd, i_qm = 1, 8
        err = np.degrees(np.abs(d[:, i_qm:i_qm + 7] - d[:, i_qd:i_qd + 7]))
        print(f"\n  CSV -> {ruta}")
        print(f"\n  {'articulación':<20}{'err medio':>11}{'err máx':>10}")
        for k, n in enumerate(CORTO):
            print(f"    {n:<18}{err[:, k].mean():>10.2f}°{err[:, k].max():>9.2f}°")
        ee_d, ee_m = d[:, 36:39], d[:, 43:46]
        dist = np.linalg.norm(ee_m - ee_d, axis=1) * 1000
        print(f"\n  efector: desvío medio {dist.mean():.1f} mm, "
              f"máximo {dist.max():.1f} mm")
        print(f"\n  Para comparar con el simulador:\n"
              f"    ros2 run h1_2_arm_control plot_fk_raise {ruta} \\\n"
              f"        --compare <csv_del_simulador> --labels \"real,simulación\"")
        return 0


def main(args=None):
    sys.exit(ejecuta(FkRightArmRaise, "h1_2_fk_right_arm_raise", args))


if __name__ == "__main__":
    main()
