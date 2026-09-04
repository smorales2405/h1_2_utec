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

    def clamp(self, idx: int, q: float) -> float:
        lo, hi = self.limits(idx)
        return min(max(q, lo), hi)

    def __repr__(self) -> str:
        return f"<Gains '{self.set_name}' con {len(self._joints)} articulaciones>"

    # -- persistencia -----------------------------------------------------
    def write_into(self, target_set: str, path: Path = CONFIG_PATH,
                   note: str | None = None) -> None:
        """Guarda estas ganancias en `sets[target_set]` de gains.yaml.

        Reescribe el fichero entero con yaml.safe_dump, así que se pierden los
        comentarios de las secciones tocadas. Se hace copia previa en
        `<fichero>.bak`.
        """
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        entry = raw["sets"].setdefault(target_set, {})
        entry["joints"] = self.as_dict()
        if note:
            entry["description"] = note
        path.with_suffix(path.suffix + ".bak").write_text(
            path.read_text(encoding="utf-8"), encoding="utf-8")
        path.write_text(
            yaml.safe_dump(raw, sort_keys=False, allow_unicode=True,
                           default_flow_style=False),
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
