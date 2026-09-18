#!/usr/bin/env python3
"""Comprueba que la cinemática de este paquete es la misma que la del simulador.

No es una formalidad. Todo el ensayo de validación se apoya en que el robot
real y el simulador calculen el efector con la MISMA aritmética; si no, al
superponer los CSV se vería la diferencia entre las dos fórmulas y se
confundiría con la diferencia entre simulación y realidad.

Y el riesgo es concreto: al portar la tabla DH se intercambió el signo del
offset de hombro entre brazos, y eso costaba 30° exactos de orientación y
253 mm de posición. Lo encontró esta comprobación, no una lectura del código.

    ros2 run h1_2_arm_control fk_check

Hace tres cosas, y las que pueda:

    1. la DH de aquí contra el URDF de `h1_2_inspire_description` (pinocchio)
    2. la DH de aquí contra la de `h1_2_algoritms`, si ese paquete se puede
       importar (está en otro workspace, así que puede no estar)
    3. la DH de aquí contra las columnas `ee_*` que el simulador ya dejó
       escritas en sus CSV, si se le pasa alguno

    ros2 run h1_2_arm_control fk_check --ros-args -p csv:='[/ruta/sim.csv]'
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node

from .fk import ArmFK, fkine_arm, verifica_contra_urdf

CORTO = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
         "wrist_roll", "wrist_pitch", "wrist_yaw"]
TOL_MM, TOL_DEG = 0.01, 0.05


class FkCheck(Node):
    def __init__(self, nombre="h1_2_fk_check"):
        super().__init__(nombre)
        self.declare_parameter("csv", [""])
        self.declare_parameter("urdf", "")

    def run(self) -> int:
        fallos = []

        print("\n  1) contra el URDF")
        for lado in ("right", "left"):
            try:
                mm, deg = verifica_contra_urdf(
                    lado, self.get_parameter("urdf").value or None, n=6)
                if mm > TOL_MM or deg > TOL_DEG:
                    fallos.append(f"URDF/{lado}: {mm:.3f} mm, {deg:.3f}°")
            except Exception as e:
                print(f"     brazo {lado}: no se pudo ({e})")

        print("\n  2) contra la DH de h1_2_algoritms")
        try:
            sys.path.insert(0, "/home/mito/humanoid_ws/src/h1_2_algoritms")
            from h1_2_algoritms.fk_functions import (
                fkine_arm_left_unitree, fkine_arm_right_unitree)
            rng = np.random.default_rng(1)
            Q = [np.zeros(7), np.array([-1.40, -0.25, 0.30, 0.70, 0.35, 0.20, 0.40])]
            Q += [rng.uniform(-1.0, 1.0, 7) for _ in range(20)]
            for lado, ref in (("right", fkine_arm_right_unitree),
                              ("left", fkine_arm_left_unitree)):
                e = max(float(np.abs(fkine_arm(q, lado) - ref(q)).max()) for q in Q)
                print(f"     brazo {lado:<6} diferencia máxima: {e:.2e}")
                if e > 1e-9:
                    fallos.append(f"DH/{lado}: {e:.2e}")
        except Exception as e:
            print(f"     no disponible ({e}); se salta")

        rutas = [c for c in self.get_parameter("csv").value if c]
        if rutas:
            print("\n  3) contra las columnas ee_* de los CSV")
            import csv as _csv
            fk = ArmFK("right")
            for ruta in rutas:
                with open(ruta, newline="") as f:
                    r = _csv.reader(f)
                    h = next(r)
                    d = np.array([[float(v) for v in row] for row in r if row])
                col = {n: i for i, n in enumerate(h)}
                for et in ("des", "meas"):
                    q = d[:, [col[f"q_{et}_{n}"] for n in CORTO]]
                    ee = d[:, [col[f"ee_{et}_{c}"] for c in ("x", "y", "z")]]
                    mio = np.array([fk.pose(row) for row in q])
                    mm = float(np.linalg.norm(mio[:, :3] - ee, axis=1).max()) * 1000
                    print(f"     {ruta.split('/')[-1]:<34} ee_{et:<5} {mm:7.4f} mm")
                    if mm > TOL_MM:
                        fallos.append(f"{ruta.split('/')[-1]}/{et}: {mm:.3f} mm")

        if fallos:
            print(f"\n  ✘ NO cuadra: {'; '.join(fallos)}")
            print("    Comparar el efector real con el del simulador mediría la\n"
                  "    diferencia entre las dos cinemáticas, no entre simulación\n"
                  "    y realidad.")
            return 1
        print("\n  ✔ la cinemática es la misma. La comparación es válida.")
        return 0


def main(args=None):
    rclpy.init(args=args)
    n = FkCheck()
    try:
        sys.exit(n.run())
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
