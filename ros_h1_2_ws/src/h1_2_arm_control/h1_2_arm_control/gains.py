"""Carga de `config/gains.yaml`."""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from .joints import BY_INDEX, BY_NAME

def _config_path() -> Path:
    """Dónde está `gains.yaml`.

    Instalado con `colcon`, la configuración vive en el `share/` del paquete y
    NO junto al código. Se busca ahí primero, y si el paquete no está instalado
    —se está ejecutando del árbol fuente— se cae al directorio de al lado.

    `H12_GAINS` lo fuerza, que es lo que hace falta para probar unas ganancias
    distintas sin reinstalar.
    """
    import os
    v = os.environ.get("H12_GAINS")
    if v:
        return Path(v)
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("h1_2_arm_control")) / "config" / "gains.yaml"
        if p.exists():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "config" / "gains.yaml"


CONFIG_PATH = _config_path()


@dataclass
class Safety:
    tau_abort_fraction: float = 0.70
    temperature_abort: float = 80.0
    joint_limit_margin: float = 0.05
    max_ref_velocity: float = 1.0
    lowstate_timeout: float = 0.5


class Gains:
    """Ganancias kp/kd por índice de motor, con sus valores por defecto."""

    def __init__(self, raw: dict, set_name: str):
        self.raw = raw
        self.set_name = set_name
        if set_name not in raw["sets"]:
            raise KeyError(
                f"conjunto de ganancias '{set_name}' no existe. "
                f"Disponibles: {', '.join(raw['sets'])}"
            )
        self._joints = dict(raw["sets"][set_name]["joints"])
        self.description = raw["sets"][set_name].get("description", "")
        self._legs = raw.get("legs_hold", {})
        safety_raw = {k: v for k, v in raw.get("safety", {}).items()
                      if k != "description"}
        self.safety = Safety(**safety_raw)

        # Postura de ensayo y topes blandos van en GRADOS en el YAML; aquí
        # dentro todo es radianes, como el resto del paquete.
        # Postura de reposo: dónde dejar los brazos al TERMINAR, y por dónde
        # pasar para llegar. No es la de ensayo al revés: importa el orden.
        _rest = raw.get("rest_posture_deg", {}) or {}
        self._reposo = {
            BY_NAME[n].idx: math.radians(float(v))
            for n, v in (_rest.get("joints", {}) or {}).items() if n in BY_NAME
        }
        self._reposo_roll_salida = math.radians(
            float(_rest.get("roll_salida_deg", 10.0)))

        # Tope de hombro condicionado al codo. La autocolisión no es un número
        # fijo: depende de dónde esté el resto del brazo.
        cond = raw.get("shoulder_roll_vs_elbow_deg", {}) or {}
        self._cond_pares = {
            BY_NAME[r].idx: BY_NAME[e].idx
            for r, e in (cond.get("depende_de", {}) or {}).items()
            if r in BY_NAME and e in BY_NAME
        }
        self._cond_tabla = [(math.radians(float(a)), math.radians(float(b)))
                            for a, b in (cond.get("tabla", []) or [])]
        self._cond_extra = math.radians(float(cond.get("extra_si_muneca", 0.0)))

        self._direction = {
            BY_NAME[n].idx: str(v)
            for n, v in (raw.get("test_direction", {}).get("joints", {}) or {}).items()
            if n in BY_NAME
        }
        self._soft = {}
        for n, lim in (raw.get("soft_limits_deg", {}).get("joints", {}) or {}).items():
            if n not in BY_NAME:
                continue
            lo = lim.get("min")
            hi = lim.get("max")
            self._soft[BY_NAME[n].idx] = (
                math.radians(float(lo)) if lo is not None else None,
                math.radians(float(hi)) if hi is not None else None,
            )

    # -- consulta ---------------------------------------------------------
    def for_index(self, idx: int) -> tuple[float, float]:
        """(kp, kd) del motor `idx`. Para piernas, las de `legs_hold`."""
        name = BY_INDEX[idx].name
        if name in self._joints:
            g = self._joints[name]
            return float(g["kp"]), float(g["kd"])
        ov = self._legs.get("overrides", {})
        g = ov.get(name) or self._legs.get("default", {"kp": 0.0, "kd": 0.0})
        return float(g["kp"]), float(g["kd"])

    # -- postura de ensayo y topes ---------------------------------------
    def rest_posture(self) -> dict[int, float]:
        """{índice: rad} donde dejar los brazos al terminar la sesión."""
        return dict(self._reposo)

    @property
    def rest_roll_exit(self) -> float:
        """|shoulder_roll| al que salir ANTES de estirar el codo, en rad."""
        return self._reposo_roll_salida

    def limits(self, idx: int) -> tuple[float, float]:
        """Topes efectivos de una articulación, en rad.

        Es la intersección de dos cosas distintas:

        * los del URDF, que son el final de carrera MECÁNICO, y se estrechan
          con `safety.joint_limit_margin` porque llegar ahí es un golpe;
        * los blandos de `soft_limits_deg`, que son de AUTOCOLISIÓN y se
          aplican tal cual: ya llevan dentro el margen que se decidió.
        """
        j = BY_INDEX[idx]
        m = self.safety.joint_limit_margin
        lo, hi = j.q_min + m, j.q_max - m
        soft_lo, soft_hi = self._soft.get(idx, (None, None))
        if soft_lo is not None:
            lo = max(lo, soft_lo)
        if soft_hi is not None:
            hi = min(hi, soft_hi)
        return lo, hi

    def has_soft_limit(self, idx: int) -> bool:
        return idx in self._soft or idx in self._cond_pares

    # -- tope de hombro condicionado al codo ------------------------------
    def elbow_of(self, idx_roll: int) -> int | None:
        """Índice del codo del que depende el tope de ese hombro."""
        return self._cond_pares.get(idx_roll)

    def roll_min_abs(self, idx_roll: int, q_elbow: float,
                     wrist_motion: bool = False) -> float | None:
        """|shoulder_roll| mínimo, en rad, para ese ángulo de codo.

        Interpola linealmente la tabla de `shoulder_roll_vs_elbow_deg` y
        satura en los extremos. Devuelve None si esa articulación no tiene
        regla condicionada.
        """
        if idx_roll not in self._cond_pares or not self._cond_tabla:
            return None
        t = sorted(self._cond_tabla)
        if q_elbow <= t[0][0]:
            v = t[0][1]
        elif q_elbow >= t[-1][0]:
            v = t[-1][1]
        else:
            v = t[-1][1]
            for (x0, y0), (x1, y1) in zip(t, t[1:]):
                if x0 <= q_elbow <= x1:
                    a = (q_elbow - x0) / (x1 - x0) if x1 > x0 else 0.0
                    v = y0 + a * (y1 - y0)
                    break
        return v + (self._cond_extra if wrist_motion else 0.0)

    def limits_dynamic(self, idx: int, q_elbow: float | None = None,
                       wrist_motion: bool = False) -> tuple[float, float]:
        """Topes efectivos teniendo en cuenta la postura del codo.

        Cuando hay regla condicionada, ésta **sustituye** al tope fijo de
        `soft_limits_deg`, no se intersecta con él. Si se intersectaran, el
        fijo de ±10° ganaría siempre y el condicional no serviría de nada
        justo en el caso que lo motiva: con el codo flexionado el hombro PUEDE
        acercarse hasta ±5°. El fijo queda como respaldo para cuando no se
        conoce el ángulo del codo.

        El signo lo da el lado: el roll positivo separa el brazo izquierdo del
        cuerpo y el negativo separa el derecho.
        """
        if q_elbow is None or idx not in self._cond_pares:
            return self.limits(idx)
        m_abs = self.roll_min_abs(idx, q_elbow, wrist_motion)
        if m_abs is None:
            return self.limits(idx)
        # se parte de los topes del URDF con su guarda, sin el tope fijo
        j = BY_INDEX[idx]
        m = self.safety.joint_limit_margin
        lo, hi = j.q_min + m, j.q_max - m
        if BY_INDEX[idx].side == "left":
            lo = max(lo, m_abs)
        else:
            hi = min(hi, -m_abs)
        return lo, hi

    def cond_pairs(self) -> dict[int, int]:
        """{índice de hombro: índice de codo del que depende su tope}."""
        return dict(self._cond_pares)

    def max_elbow_for_roll(self, idx_roll: int, q_roll: float,
                           wrist_motion: bool = False) -> float:
        """Ángulo de codo más grande admisible con el hombro donde está.

        Es la inversa de `roll_min_abs`. Hace falta porque la restricción
        acopla las dos articulaciones: si el hombro está a 6° y el codo se
        está estirando hacia 80°, quien hay que frenar es el CODO, no el
        hombro. Sin esto, la protección solo miraría a uno de los dos.
        """
        if idx_roll not in self._cond_pares or not self._cond_tabla:
            return float("inf")
        objetivo = abs(q_roll) - (self._cond_extra if wrist_motion else 0.0)
        t = sorted(self._cond_tabla)
        if objetivo <= t[0][1]:
            return t[0][0]
        if objetivo >= t[-1][1]:
            return float("inf")
        for (x0, y0), (x1, y1) in zip(t, t[1:]):
            if y0 <= objetivo <= y1:
                a = (objetivo - y0) / (y1 - y0) if y1 > y0 else 0.0
                return x0 + a * (x1 - x0)
        return t[-1][0]

    def _violacion(self, i_roll: int, q_roll: float, q_elbow: float,
                   wrist_motion: bool) -> float:
        """Cuánto se sale del tope esa pareja, en rad. 0 = dentro."""
        lo, hi = self.limits_dynamic(i_roll, q_elbow, wrist_motion)
        return max(lo - q_roll, q_roll - hi, 0.0)

    def enforce_pairs(self, q_des, q_prev=None, traj_joints=(),
                      wrist_motion: bool = False, commanded=None) -> int:
        """Corrige `q_des` in situ para que ninguna pareja hombro-codo choque.

        Dos decisiones de diseño que no son obvias:

        **No corrige hacia la zona buena, solo impide empeorar.** Un portero
        que "arregle" la postura ORDENA un movimiento que nadie ha pedido, y
        eso es peligroso: el robot en reposo tiene los hombros a ±5° con el
        brazo estirado, que ya incumple la regla de ±10°, y un portero
        corrector movería los dos brazos nada más enganchar. Así que se
        compara la violación nueva con la anterior y solo se congela si
        aumenta. Acercarse al tope desde fuera siempre se permite: es lo que
        hace un cambio de postura al llevar los hombros de ±5° a ±18°.

        **Frena a quien se mueve.** Si el codo se estira hacia 80° con el
        hombro a 6°, hay que parar el CODO. Frenar siempre al hombro dejaría
        pasar un ensayo de codo que se salta la restricción.

        Devuelve cuántas correcciones ha hecho.
        """
        n = 0
        for i_roll, i_elb in self._cond_pares.items():
            if commanded is not None and (i_roll not in commanded
                                          or i_elb not in commanded):
                continue
            d_new = self._violacion(i_roll, float(q_des[i_roll]),
                                    float(q_des[i_elb]), wrist_motion)
            if d_new <= 1e-9:
                continue
            d_old = (self._violacion(i_roll, float(q_prev[i_roll]),
                                     float(q_prev[i_elb]), wrist_motion)
                     if q_prev is not None else 0.0)
            if d_new <= d_old + 1e-9:
                continue                      # no empeora: se deja pasar
            n += 1
            if i_elb in traj_joints and i_roll not in traj_joints:
                q_des[i_elb] = (float(q_prev[i_elb]) if q_prev is not None
                                else self.max_elbow_for_roll(
                                    i_roll, float(q_des[i_roll]), wrist_motion))
            else:
                q_des[i_roll] = (float(q_prev[i_roll]) if q_prev is not None
                                 else self.clamp(i_roll, float(q_des[i_roll])))
        return n

    def pair_ok(self, idx_roll: int, q_roll: float, q_elbow: float,
                wrist_motion: bool = False) -> bool:
        """¿Es admisible esa pareja (hombro, codo)?

        Se comprueba sobre la PAREJA y no sobre el hombro solo porque la
        restricción los acopla: mover el codo de 0° a 80° con el hombro a 5°
        es tan inadmisible como lo contrario, y quien se mueve puede ser
        cualquiera de los dos.
        """
        lo, hi = self.limits_dynamic(idx_roll, q_elbow, wrist_motion)
        return lo - 1e-9 <= q_roll <= hi + 1e-9

    def clamp(self, idx: int, q: float) -> float:
        lo, hi = self.limits(idx)
        return min(max(q, lo), hi)

    def __repr__(self) -> str:
        return f"<Gains '{self.set_name}' con {len(self._joints)} articulaciones>"

    # -- persistencia -----------------------------------------------------
def load(set_name: str | None = None, path: Path | None = None) -> Gains:
    """Carga un conjunto de ganancias de `gains.yaml`.

    Sin argumentos toma el conjunto que el propio fichero marca como `active`.
    """
    path = Path(path) if path else _config_path()
    if not path.exists():
        raise SystemExit(
            f"no encuentro las ganancias en {path}.\n"
            f"  ¿Has hecho `colcon build` y `source install/setup.bash`?\n"
            f"  Se puede forzar la ruta con:  export H12_GAINS=/ruta/a/gains.yaml")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    return Gains(raw, set_name or raw.get("active", "official_arm_sdk"))


def joint_or_die(spec: str) -> int:
    """Un único índice de motor a partir de un nombre o número."""
    if spec in BY_NAME:
        return BY_NAME[spec].idx
    if spec.isdigit() and int(spec) in BY_INDEX:
        return int(spec)
    raise SystemExit(f"articulación desconocida: '{spec}'")
