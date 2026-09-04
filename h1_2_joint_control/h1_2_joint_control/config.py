"""Carga de `config/gains.yaml`."""
from __future__ import annotations

import copy
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
        self.safety = Safety(**raw.get("safety", {}))

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
