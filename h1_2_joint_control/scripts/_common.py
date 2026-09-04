"""Trozos compartidos por los scripts: argumentos, avisos y construcción del cliente."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from h1_2_joint_control import config as cfg          # noqa: E402
from h1_2_joint_control.client import H12Client       # noqa: E402
from h1_2_joint_control.joints import BY_INDEX, BY_NAME  # noqa: E402

BANNER = """
╔══════════════════════════════════════════════════════════════════════════╗
║  Esto MUEVE UN ROBOT REAL de 70 kg. Antes de continuar:                  ║
║                                                                          ║
║   · El H1-2 debe estar COLGADO DEL ARNÉS o firmemente sujeto.            ║
║   · Nadie dentro del alcance de los brazos.                              ║
║   · Mando a mano. PARO DE EMERGENCIA: L2 + B.                            ║
║   · Ctrl-C aquí devuelve el brazo a su sitio y suelta el control.        ║
╚══════════════════════════════════════════════════════════════════════════╝
"""


def add_common_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--channel", choices=["arm_sdk", "lowcmd"], default="arm_sdk",
                    help="arm_sdk (por defecto, el robot conserva las piernas) o "
                         "lowcmd (SOLO en modo debug, robot colgado)")
    ap.add_argument("--gains", default=None,
                    help="conjunto de config/gains.yaml (por defecto, el 'active')")
    ap.add_argument("--kp", type=float, default=None, help="sobrescribe kp")
    ap.add_argument("--kd", type=float, default=None, help="sobrescribe kd")
    ap.add_argument("--rate", type=float, default=250.0,
                    help="Hz del lazo de publicación (xr_teleoperate usa 250)")
    ap.add_argument("--yes", action="store_true",
                    help="no pedir confirmación (para automatizar barridos)")
    ap.add_argument("--tag", default="", help="etiqueta para el nombre del CSV")
    ap.add_argument("--dry-run", action="store_true",
                    help="publicar en un tópico que nadie escucha: ejercita todo "
                         "el software sin que el robot pueda moverse")
    ap.add_argument("--weight", type=float, default=1.0,
                    help="autoridad máxima de arm_sdk, de 0 a 1. 0 = publica en "
                         "el tópico bueno pero sin mandar; 0.3 = manda flojo")


def confirm(args, extra: str = "") -> None:
    if getattr(args, "dry_run", False):
        print("\n  PRUEBA EN SECO: se publica en un tópico que nadie escucha.")
        print("  El robot no puede moverse. Solo se valida el software.\n")
        return
    if getattr(args, "weight", 1.0) <= 0.0:
        print("\n  Peso 0: se publica en el canal bueno pero con autoridad nula.")
        print("  El robot conserva el mando de los brazos.\n")
    print(BANNER)
    if extra:
        print(extra)
    if args.yes:
        print("  (--yes: se continúa sin preguntar)\n")
        return
    try:
        r = input("  Escribe 'si' para continuar: ").strip().lower()
    except EOFError:
        r = ""
    if r not in ("si", "sí", "s", "yes", "y"):
        raise SystemExit("  cancelado.")


def build_client(args, controlled: list[int], verbose: bool = True) -> H12Client:
    gains = cfg.load(args.gains)
    if args.kp is not None or args.kd is not None:
        for i in controlled:
            kp, kd = gains.for_index(i)
            gains.set_index(i, args.kp if args.kp is not None else kp,
                            args.kd if args.kd is not None else kd)
    return H12Client(controlled=controlled, gains=gains, channel=args.channel,
                     rate_hz=args.rate, verbose=verbose,
                     dry_run=getattr(args, "dry_run", False),
                     max_weight=getattr(args, "weight", 1.0))


def joint_index(spec: str) -> int:
    if spec in BY_NAME:
        return BY_NAME[spec].idx
    if spec.isdigit() and int(spec) in BY_INDEX:
        return int(spec)
    raise SystemExit(
        f"articulación desconocida: '{spec}'.\n  Válidas: "
        + ", ".join(BY_NAME))


def describe(idx: int, gains) -> str:
    j = BY_INDEX[idx]
    kp, kd = gains.for_index(idx)
    sat = j.tau_max / kp if kp else float("inf")
    return (f"  {j.name}  (motor {j.idx}, {j.urdf})\n"
            f"    topes URDF : [{j.q_min:+.3f}, {j.q_max:+.3f}] rad "
            f"= [{math.degrees(j.q_min):+.0f}°, {math.degrees(j.q_max):+.0f}°]\n"
            f"    par máximo : {j.tau_max:.1f} Nm     velocidad máx: {j.dq_max:.1f} rad/s\n"
            f"    ganancias  : kp={kp:.1f}  kd={kd:.2f}   "
            f"-> satura el motor con {sat:.3f} rad ({math.degrees(sat):.1f}°) de error")
