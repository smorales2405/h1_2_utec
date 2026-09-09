"""Trozos compartidos por los scripts: argumentos, avisos y construcción del cliente."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from h1_2_joint_control import config as cfg          # noqa: E402
from h1_2_joint_control.client import H12Client       # noqa: E402
from h1_2_joint_control.joints import ARM_INDICES, BY_INDEX, BY_NAME  # noqa: E402

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
    ap.add_argument("--no-posture", action="store_true",
                    help="no llevar el robot a la postura de ensayo de "
                         "config/gains.yaml (hombros separados del cuerpo) "
                         "antes de medir")
    ap.add_argument("--kp-scale", type=float, default=1.0,
                    help="multiplica kp de las articulaciones vigiladas. Con 0.5 "
                         "es la prueba de humo de la compensación de gravedad: "
                         "con tau_ff el brazo debe sostenerse igual")
    ap.add_argument("--gravity-ff", action="store_true",
                    help="compensar la gravedad con el modelo identificado. "
                         "Sin esto, el PD solo genera el par de sostenimiento a "
                         "costa de un error permanente tau_g/kp que no se va")
    ap.add_argument("--gravity-at", choices=["meas", "des"], default="meas",
                    help="dónde evaluar g(): en la posición medida (por defecto) "
                         "o en la consigna, como pide el protocolo. En la "
                         "consigna se adelanta durante el movimiento")
    ap.add_argument("--gravity-frac", type=float, default=0.5,
                    help="tope de tau_ff, como fracción del par máximo")
    ap.add_argument("--direction", choices=["auto", "positive", "negative"],
                    default="auto",
                    help="sentido del ensayo. auto = hacia donde queda más "
                         "recorrido articular, que en los brazos especulares "
                         "del H1-2 evita empujar contra el torso")
    ap.add_argument("--dry-run", action="store_true",
                    help="publicar en un tópico que nadie escucha: ejercita todo "
                         "el software sin que el robot pueda moverse")
    ap.add_argument("--weight", type=float, default=1.0,
                    help="autoridad máxima de arm_sdk, de 0 a 1. 0 = publica en "
                         "el tópico bueno pero sin mandar; 0.3 = manda flojo")
    ap.add_argument("--legs", choices=["free", "damp", "hold"], default="free",
                    help="solo con --channel lowcmd: qué hacer con las piernas. "
                         "free = kp=kd=0, igual que el servicio en reposo (por "
                         "defecto, y lo correcto con el robot colgado); "
                         "damp = kd=2 para que no se bamboleen; "
                         "hold = kp=300, rígidas")


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


def _gravedad(args):
    """El modelo de gravedad, o None si no se ha pedido."""
    if not getattr(args, "gravity_ff", False):
        return None
    from h1_2_joint_control.gravity import GravityModel
    return GravityModel()


def require_debug_mode(args) -> None:
    """Con `--channel lowcmd`, negarse a arrancar si el servicio sigue activo.

    Publicar en `/lowcmd` mientras el controlador de alto nivel también lo hace
    es exactamente el problema que este paquete existe para evitar. Antes que
    soltar el servicio por nuestra cuenta —que es una decisión con
    consecuencias físicas— se para y se dice qué hacer.
    """
    if args.channel != "lowcmd" or getattr(args, "dry_run", False):
        return
    from h1_2_joint_control.motion_switcher import MotionSwitcher
    ms = MotionSwitcher()
    name = ms.active_mode()
    ms.close()
    if name is None:
        raise SystemExit("  el servicio motion_switcher no responde; no se puede "
                         "comprobar si /lowcmd está libre. Abortado.")
    if name:
        raise SystemExit(
            f"  El controlador de alto nivel '{name}' sigue activo y publica en\n"
            f"  /lowcmd a 500 Hz. Si publicamos ahí a la vez, el motor recibirá\n"
            f"  consignas alternas y la articulación no seguirá su referencia.\n\n"
            f"  Con el robot COLGADO DEL ARNÉS:\n"
            f"      python3 scripts/06_debug_mode.py enter\n"
            f"  y al terminar la sesión:\n"
            f"      python3 scripts/06_debug_mode.py exit")


def build_client(args, controlled: list[int], verbose: bool = True) -> H12Client:
    require_debug_mode(args)
    gains = cfg.load(args.gains)
    if args.kp is not None or args.kd is not None:
        for i in controlled:
            kp, kd = gains.for_index(i)
            gains.set_index(i, args.kp if args.kp is not None else kp,
                            args.kd if args.kd is not None else kd)
    escala = getattr(args, "kp_scale", 1.0)
    if escala != 1.0:
        for i in controlled:
            kp, kd = gains.for_index(i)
            gains.set_index(i, kp * escala, kd)
    return H12Client(controlled=controlled, gains=gains, channel=args.channel,
                     rate_hz=args.rate, verbose=verbose,
                     dry_run=getattr(args, "dry_run", False),
                     max_weight=getattr(args, "weight", 1.0),
                     legs_policy=getattr(args, "legs", "free"),
                     gravity=_gravedad(args),
                     gravity_frac=getattr(args, "gravity_frac", 0.5),
                     gravity_at=getattr(args, "gravity_at", "meas"))


def joint_index(spec: str) -> int:
    if spec in BY_NAME:
        return BY_NAME[spec].idx
    if spec.isdigit() and int(spec) in BY_INDEX:
        return int(spec)
    raise SystemExit(
        f"articulación desconocida: '{spec}'.\n  Válidas: "
        + ", ".join(BY_NAME))


def pick_amplitude(idx: int, q0: float, amp: float, limits: tuple[float, float],
                   direction: str = "auto") -> tuple[float, str]:
    """Elige el sentido y la amplitud del ensayo. Devuelve (amp, comentario).

    Con `direction="auto"` se va hacia donde queda MÁS recorrido articular. No
    es un capricho: en el H1-2 los brazos son especulares, así que un `+0.12`
    que separa el brazo izquierdo del cuerpo mete el derecho CONTRA el torso.
    Medido: R_shoulder_roll con +0.12 rad choca, el error se queda en 6.3° y el
    par llega a 29.6 Nm (saltó la protección). En el sentido contrario, el
    mismo ensayo da 1.67° de error y 4.3 Nm.

    El recorrido libre no equivale exactamente a «lejos del cuerpo», pero en la
    práctica separa los dos casos y evita empujar contra un tope.

    `limits` son los topes EFECTIVOS, ya combinados: los del URDF con su margen
    y los blandos de autocolisión. Vienen de `Gains.limits(idx)`.
    """
    lo, hi = limits
    up_ok, down_ok = (q0 + abs(amp)) <= hi, (q0 - abs(amp)) >= lo
    room_up, room_down = hi - q0, q0 - lo

    if direction == "positive":
        want = +1
    elif direction == "negative":
        want = -1
    else:
        want = +1 if room_up >= room_down else -1

    if want > 0 and up_ok:
        return abs(amp), ""
    if want < 0 and down_ok:
        return -abs(amp), ""
    # el sentido preferido no cabe: probar el otro
    if up_ok:
        return abs(amp), "(sentido invertido: no cabía)"
    if down_ok:
        return -abs(amp), "(sentido invertido: no cabía)"
    room = max(min(room_up, room_down), 0.0)
    return math.copysign(room * 0.8, want), "(amplitud recortada por el tope)"


def apply_test_posture(cli, args, gains) -> None:
    """Lleva el robot a la postura de ensayo, salvo que se pida lo contrario."""
    if getattr(args, "no_posture", False):
        print("  --no-posture: se mide desde la postura en que estuviera el robot.")
        return
    if not gains.test_posture():
        return
    cli.go_to_test_posture()
    # A partir de aquí `q0` deja de ser «donde estaba al arrancar» para las
    # articulaciones movidas: los ensayos parten de la postura de ensayo. El
    # `q0` original se conserva en el cliente para devolver el robot al final.


def describe(idx: int, gains) -> str:
    j = BY_INDEX[idx]
    kp, kd = gains.for_index(idx)
    sat = j.tau_max / kp if kp else float("inf")
    lo, hi = gains.limits(idx)
    lineas = [
        f"  {j.name}  (motor {j.idx}, {j.urdf})",
        f"    topes URDF : [{math.degrees(j.q_min):+7.1f}°, {math.degrees(j.q_max):+7.1f}°]"
        f"   (final de carrera mecánico)",
        f"    efectivos  : [{math.degrees(lo):+7.1f}°, {math.degrees(hi):+7.1f}°]"
        + ("   ← recortado por autocolisión (soft_limits_deg)"
           if gains.has_soft_limit(idx) else
           f"   (URDF menos {gains.safety.joint_limit_margin:.2f} rad de guarda)"),
        f"    par máximo : {j.tau_max:.1f} Nm     velocidad máx: {j.dq_max:.1f} rad/s",
        f"    ganancias  : kp={kp:.1f}  kd={kd:.2f}   "
        f"-> satura el motor con {sat:.3f} rad ({math.degrees(sat):.1f}°) de error",
    ]
    return "\n".join(lineas)


def relaja_topes_de_ensayo(gains, log) -> None:
    """Quita los topes de autocolisión de los `shoulder_roll`, solo para ir a 0°.

    Los topes de `soft_limits_deg` (±10°) y la tabla
    `shoulder_roll_vs_elbow_deg` (±5° con el codo flexionado) están puestos
    para los ENSAYOS, donde una articulación barre sola y hace falta margen
    porque la trayectoria pasa por muchas posturas.

Las dos posturas fijas de la teleoperación no son barridos:

    * la **cero**, verificada FÍSICAMENTE por el operador el 2026-09-09 —los
      siete ángulos de cada brazo a 0°, sin colisión—, y que además el modelo
      ya admitía: con codo y pitch a 0° el `shoulder_roll` llega a −10° sin
      chocar (tabla de 08_PLAN.md §2.2);
    * la de **reposo**, que es donde el brazo cuelga por su propio peso, ±5°
      medidos con el robot en el arnés. Acabar exactamente ahí hace que soltar
      las ganancias no mueva nada.

    Se relaja solo en esos dos scripts. El resto conserva los topes.
    """
    rolls = [i for i in ARM_INDICES if BY_INDEX[i].name.endswith("shoulder_roll")]
    for i in rolls:
        gains._soft.pop(i, None)
    gains._cond_pares = {}
    log("  topes de autocolisión de los shoulder_roll relajados para esta "
        "postura\n  (verificada físicamente por el operador; ver la docstring)")
