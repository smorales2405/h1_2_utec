"""Carga de `config/gains.yaml`."""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from .joints import BY_INDEX, BY_NAME

PKG_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PKG_ROOT / "config" / "gains.yaml"
LOG_DIR = PKG_ROOT / "logs"


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
        self._posture = {
            BY_NAME[n].idx: math.radians(float(v))
            for n, v in (raw.get("test_posture_deg", {}).get("joints", {}) or {}).items()
            if n in BY_NAME
        }
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

    def set_index(self, idx: int, kp: float, kd: float) -> None:
        self._joints[BY_INDEX[idx].name] = {"kp": float(kp), "kd": float(kd)}

    def as_dict(self) -> dict:
        return copy.deepcopy(self._joints)

    # -- postura de ensayo y topes ---------------------------------------
    def test_posture(self) -> dict[int, float]:
        """{índice: ángulo en rad} al que llevar el robot antes de medir."""
        return dict(self._posture)

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
        return idx in self._soft

    def direction(self, idx: int, requested: str = "auto") -> str:
        """Sentido del ensayo. Lo que pide el usuario manda; si pide `auto` y
        la articulación tiene preferencia en el YAML, se usa esa."""
        if requested != "auto":
            return requested
        return self._direction.get(idx, "auto")

    def clamp(self, idx: int, q: float) -> float:
        lo, hi = self.limits(idx)
        return min(max(q, lo), hi)

    def __repr__(self) -> str:
        return f"<Gains '{self.set_name}' con {len(self._joints)} articulaciones>"

    # -- persistencia -----------------------------------------------------
    def write_into(self, target_set: str, path: Path = CONFIG_PATH,
                   note: str | None = None) -> None:
        """Guarda estas ganancias en `sets[target_set].joints` de gains.yaml.

        Sustituye SOLO las líneas de ese bloque, no el fichero entero. Pasar
        por `yaml.safe_dump` sería más corto pero se llevaría por delante todos
        los comentarios, y en este fichero los comentarios son media
        documentación: de dónde sale cada conjunto de ganancias, por qué los
        topes blandos no llevan margen, qué significa cada sentido de ensayo.

        Deja copia previa en `<fichero>.bak`.
        """
        original = path.read_text(encoding="utf-8")
        lineas = original.splitlines(keepends=True)

        def sangria(l: str) -> int:
            return len(l) - len(l.lstrip(" "))

        # 1) localizar `sets:`, 2) dentro, el conjunto, 3) dentro, `joints:`
        i_sets = next((k for k, l in enumerate(lineas)
                       if l.rstrip("\n") == "sets:"), None)
        if i_sets is None:
            raise KeyError("no encuentro la clave 'sets:' en " + str(path))

        i_set = None
        for k in range(i_sets + 1, len(lineas)):
            l = lineas[k]
            if l.strip() and sangria(l) == 0:
                break                              # se acabó el bloque `sets:`
            if l.strip().rstrip(":") == target_set and l.strip().endswith(":"):
                i_set = k
                break
        if i_set is None:
            raise KeyError(f"el conjunto '{target_set}' no está en {path}")

        base = sangria(lineas[i_set])
        i_joints = None
        for k in range(i_set + 1, len(lineas)):
            l = lineas[k]
            if l.strip() and sangria(l) <= base:
                break                              # se acabó este conjunto
            if l.strip() == "joints:":
                i_joints = k
                break
        if i_joints is None:
            raise KeyError(f"'{target_set}' no tiene bloque 'joints:'")

        sang_j = sangria(lineas[i_joints])
        fin = len(lineas)
        for k in range(i_joints + 1, len(lineas)):
            l = lineas[k]
            if l.strip() and sangria(l) <= sang_j:
                fin = k
                break

        ancho = max((len(n) for n in self._joints), default=0)
        nuevas = [f"{' ' * (sang_j + 2)}{n + ':':<{ancho + 1}} "
                  f"{{kp: {g['kp']}, kd: {g['kd']}}}\n"
                  for n, g in self._joints.items()]

        path.with_suffix(path.suffix + ".bak").write_text(original, encoding="utf-8")
        path.write_text("".join(lineas[:i_joints + 1] + nuevas + lineas[fin:]),
                        encoding="utf-8")


def load(set_name: str | None = None, path: Path = CONFIG_PATH) -> Gains:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    return Gains(raw, set_name or raw.get("active", "official_arm_sdk"))


def joint_or_die(spec: str) -> int:
    """Un único índice de motor a partir de un nombre o número."""
    if spec in BY_NAME:
        return BY_NAME[spec].idx
    if spec.isdigit() and int(spec) in BY_INDEX:
        return int(spec)
    raise SystemExit(f"articulación desconocida: '{spec}'")
