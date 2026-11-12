#!/usr/bin/env python3
"""Software coupling of the Inspire RH56DFTP hands: 6 commands -> 12 joint angles per hand.

The URDF's <mimic> can only couple two joints linearly (q2 = k * q1), and the real hand does
not move that way: its distal phalanges lag at mid travel and catch up at the end, and the
thumb's three joints follow different curves. Here every joint follows its own curve,
measured on photos of the real hand, as a function of the command:

    s = 1 - ANGLE_SET/1000        0 = open, 1 = closed (ANGLE_ACT reads on the same scale)

The curves live in config/hand_coupling.yaml (written by tools/fit_hand_coupling.py) and are
read by linear interpolation between its 21 nodes. Both hands share them. Every curve starts at
q = 0: the URDF zero of every hand joint is the open hand (ANGLE_SET 1000).

DOF order, as in the SDK registers ANGLE_SET / ANGLE_ACT:
    0 little, 1 ring, 2 middle, 3 index, 4 thumb_bend, 5 thumb_rot

    from hand_coupling import HandCoupling
    hc = HandCoupling()
    q = hc.joints_from_angle_set([1000, 1000, 500, 500, 1000, 0], "left")
    # {'left_little_1_joint': 0.0, ..., 'left_thumb_swing_joint': 1.2}
    hc.angle_set_from_joints(q, "left")          # -> [1000, 1000, 500, 500, 1000, 0]

Command line:
    python3 hand_coupling.py 1000 1000 500 500 1000 0 [--side right]   the 12 joints
    python3 hand_coupling.py --table                                   the curves every 125
    python3 hand_coupling.py --test                                    self-test
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import yaml

DOFS = ("little", "ring", "middle", "index", "thumb_bend", "thumb_rot")
SIDES = ("left", "right")
CONFIG_NAME = "hand_coupling.yaml"


def default_config() -> Path:
    """config/hand_coupling.yaml of this package: next to the sources (also through a
    --symlink-install), or in the installed share directory."""
    here = Path(__file__).resolve().parents[1] / "config" / CONFIG_NAME
    if here.exists():
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("h1_2_inspire_description")) / "config" / CONFIG_NAME
    except Exception as exc:        # noqa: BLE001 - any failure means the same: not found
        raise FileNotFoundError(f"{CONFIG_NAME} not found next to {__file__} nor in the package share") from exc


def joint_name(side: str, suffix: str) -> str:
    if side not in SIDES:
        raise ValueError(f"side must be 'left' or 'right', not {side!r}")
    return f"{side}_{suffix}_joint"


def s_from_angle_set(angle_set: Sequence[float]) -> np.ndarray:
    """ANGLE_SET (or ANGLE_ACT), 0..1000 -> s, 0..1. Out-of-range values are clipped."""
    return np.clip(1.0 - np.asarray(angle_set, float) / 1000.0, 0.0, 1.0)


def angle_set_from_s(s: Sequence[float]) -> list[int]:
    return [int(round(1000.0 * (1.0 - float(v)))) for v in np.clip(np.asarray(s, float), 0.0, 1.0)]


class HandCoupling:
    """The coupling curves of config/hand_coupling.yaml."""

    def __init__(self, config: str | Path | None = None):
        path = Path(config) if config else default_config()
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.config = path
        self.s = np.asarray(data["s"], float)
        self.curves = {k: np.asarray(v, float) for k, v in data["curves"].items()}
        self.limits = {k: (float(v[0]), float(v[1])) for k, v in data["limits"].items()}
        names = [d["name"] for d in data["dofs"]]
        if tuple(names) != DOFS:
            raise ValueError(f"{path}: dofs must be {DOFS}, found {tuple(names)}")
        # per DOF: [(joint suffix, curve)], actuated joint first
        self.dof_joints = [list(d["joints"].items()) for d in data["dofs"]]
        for k, v in self.curves.items():
            if len(v) != len(self.s) or np.any(np.diff(v) <= 0.0):
                raise ValueError(f"{path}: curve {k} must have one value per node and grow strictly")
        self.suffixes = [j for joints in self.dof_joints for j, _ in joints]
        self.actuated = [joints[0][0] for joints in self.dof_joints]

    # ------------------------------------------------------------------ forward: s -> joints
    def joints_from_s(self, s: Sequence[float] | Mapping[str, float], side: str) -> dict[str, float]:
        """s of the 6 DOF (SDK order, or {dof name: s}) -> {URDF joint name: q} for the 12 joints."""
        s6 = self._six(s)
        out = {}
        for k, joints in enumerate(self.dof_joints):
            for suffix, curve in joints:
                out[joint_name(side, suffix)] = float(np.interp(s6[k], self.s, self.curves[curve]))
        return out

    def joints_from_angle_set(self, angle_set: Sequence[float], side: str) -> dict[str, float]:
        """ANGLE_SET or ANGLE_ACT of one hand (6 values, SDK order) -> the 12 joints."""
        return self.joints_from_s(s_from_angle_set(angle_set), side)

    def derivatives(self, s: Sequence[float] | Mapping[str, float], side: str) -> dict[str, tuple[int, float]]:
        """{URDF joint name: (DOF index, dq/ds)}: how each joint moves with its DOF's s. With a
        Jacobian J over the 12 joints, the Jacobian over the 6 DOF is J_s[:, k] = sum over the
        joints of DOF k of J[:, joint] * dq/ds (the slope of the segment s falls in)."""
        s6 = self._six(s)
        out = {}
        for k, joints in enumerate(self.dof_joints):
            i = int(np.clip(np.searchsorted(self.s, s6[k], side="right") - 1, 0, len(self.s) - 2))
            ds = self.s[i + 1] - self.s[i]
            for suffix, curve in joints:
                c = self.curves[curve]
                out[joint_name(side, suffix)] = (k, float((c[i + 1] - c[i]) / ds))
        return out

    # ------------------------------------------------------------------ inverse: joints -> s
    def s_from_joints(self, q: Mapping[str, float], side: str) -> list[float]:
        """s of the 6 DOF from the ACTUATED joints in `q` (full URDF names, or suffixes such as
        'index_1'). The coupled joints in `q`, if any, are ignored. A missing actuated joint is
        an error. Angles beyond the curve's range give s = 0 or 1."""
        out = []
        for joints in self.dof_joints:
            suffix, curve = joints[0]
            full = joint_name(side, suffix)
            if full in q:
                value = q[full]
            elif suffix in q:
                value = q[suffix]
            else:
                raise KeyError(f"missing actuated joint {full}")
            out.append(float(np.interp(float(value), self.curves[curve], self.s)))
        return out

    def angle_set_from_joints(self, q: Mapping[str, float], side: str) -> list[int]:
        """ANGLE_SET of the 6 DOF (SDK order) from the actuated joints in `q`."""
        return angle_set_from_s(self.s_from_joints(q, side))

    def couple(self, q: Mapping[str, float], side: str) -> dict[str, float]:
        """The 12 joints of one hand from its 6 actuated joints: what the coupling node publishes."""
        return self.joints_from_s(self.s_from_joints(q, side), side)

    # ------------------------------------------------------------------ MuJoCo
    def polynomial(self, suffix: str, degree: int = 4) -> list[float]:
        """Coefficients [a0, a1, ...] of q_joint ~= a0 + a1*q_act + a2*q_act^2 + ... over the curve,
        q_act being the actuated joint of the same DOF: the form of MuJoCo's
        <equality joint polycoef> (with the reference pose at q = 0). The actuated joint itself
        gives [0, 1, 0, ...]."""
        for joints in self.dof_joints:
            names = dict(joints)
            if suffix in names:
                x = self.curves[joints[0][1]]
                y = self.curves[names[suffix]]
                c = np.polynomial.polynomial.polyfit(x, y, degree)
                return [float(v) for v in c] + [0.0] * (4 - degree)
        raise KeyError(f"unknown joint suffix {suffix!r}")

    # ------------------------------------------------------------------ helpers
    def _six(self, s) -> np.ndarray:
        if isinstance(s, Mapping):
            unknown = set(s) - set(DOFS)
            if unknown:
                raise KeyError(f"unknown DOF {sorted(unknown)}; valid: {DOFS}")
            missing = [d for d in DOFS if d not in s]
            if missing:
                raise KeyError(f"missing DOF {missing}")
            s = [s[d] for d in DOFS]
        s6 = np.asarray(s, float).ravel()
        if s6.shape != (6,):
            raise ValueError(f"expected 6 values (SDK order {DOFS}), got {s6.size}")
        return np.clip(s6, 0.0, 1.0)


# ====================================================================== command line
def _table(hc: HandCoupling) -> None:
    cols = list(dict.fromkeys(c for joints in hc.dof_joints for _, c in joints))
    print("ANGLE_SET  " + "".join(f"{c:>12s}" for c in cols))
    for a in range(1000, -1, -125):
        s = 1.0 - a / 1000.0
        print(f"{a:9d}  " + "".join(f"{np.interp(s, hc.s, hc.curves[c]):12.3f}" for c in cols))


def _test(hc: HandCoupling) -> int:
    fails = []

    def check(name, ok):
        print(f"  [{'ok' if ok else 'FAIL'}] {name}")
        if not ok:
            fails.append(name)

    rng = np.random.default_rng(0)
    for side in SIDES:
        a = rng.integers(0, 1001, 6)
        q = hc.joints_from_angle_set(a, side)
        check(f"{side}: 12 joints with the side prefix", len(q) == 12 and all(k.startswith(side + "_") for k in q))
        check(f"{side}: ANGLE_SET -> joints -> ANGLE_SET round trip ({list(a)})",
              hc.angle_set_from_joints(q, side) == [int(v) for v in a])
    open_q = hc.joints_from_angle_set([1000] * 6, "left")
    shut_q = hc.joints_from_angle_set([0] * 6, "left")
    check("open hand = first node of every curve = q 0 (the URDF zero)",
          all(abs(open_q[joint_name("left", j)] - hc.curves[c][0]) < 1e-12 and hc.curves[c][0] == 0.0
              for js in hc.dof_joints for j, c in js))
    check("closed hand = last node of every curve",
          all(abs(shut_q[joint_name("left", j)] - hc.curves[c][-1]) < 1e-12 for js in hc.dof_joints for j, c in js))
    check("every joint inside its software-variant limits, open and closed",
          all(hc.limits[j][0] - 1e-9 <= qq[joint_name("left", j)] <= hc.limits[j][1] + 1e-9
              for qq in (open_q, shut_q) for j in hc.suffixes))
    check("out-of-range commands are clipped", hc.joints_from_angle_set([-50, 1200, 0, 0, 0, 0], "left")
          == hc.joints_from_angle_set([0, 1000, 0, 0, 0, 0], "left"))
    q = hc.joints_from_s([0.3] * 6, "right")
    check("couple(): the coupled joints follow the actuated ones",
          all(abs(hc.couple({k: v for k, v in q.items() if k.split("_", 1)[1][:-6] in hc.actuated}, "right")[k] - v)
              < 1e-9 for k, v in q.items()))
    s0, h = np.full(6, 0.42), 1e-6
    d = hc.derivatives(s0, "left")
    qa, qb = hc.joints_from_s(s0 - h, "left"), hc.joints_from_s(s0 + h, "left")
    check("derivatives() match finite differences", all(abs((qb[k] - qa[k]) / (2 * h) - v) < 1e-6 for k, (_, v) in d.items()))
    worst = 0.0
    for joints in hc.dof_joints:
        x = hc.curves[joints[0][1]]
        for suffix, curve in joints:
            p = hc.polynomial(suffix)
            worst = max(worst, float(np.max(np.abs(np.polynomial.polynomial.polyval(x, p) - hc.curves[curve]))))
    check(f"MuJoCo quartics within 0.01 rad of the curves (worst {worst:.4f})", worst < 0.01)
    try:
        hc.joints_from_s({"index": 0.5}, "left")
        check("a dict with missing DOF is rejected", False)
    except KeyError:
        check("a dict with missing DOF is rejected", True)
    print("all checks passed" if not fails else f"{len(fails)} check(s) FAILED")
    return 1 if fails else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="RH56DFTP software coupling: ANGLE_SET -> 12 joint angles.")
    ap.add_argument("angle_set", nargs="*", type=float, help="6 values, SDK order: little ring middle index thumb_bend thumb_rot")
    ap.add_argument("--side", choices=SIDES, default="left")
    ap.add_argument("--config", type=Path, help=f"curves file (default: config/{CONFIG_NAME} of the package)")
    ap.add_argument("--table", action="store_true", help="print the curves every 125 ANGLE_SET")
    ap.add_argument("--test", action="store_true", help="run the self-test")
    args = ap.parse_args(argv)
    hc = HandCoupling(args.config)
    if args.test:
        return _test(hc)
    if args.table:
        _table(hc)
        return 0
    if len(args.angle_set) != 6:
        ap.error("give the 6 ANGLE_SET values, or --table / --test")
    for k, v in hc.joints_from_angle_set(args.angle_set, args.side).items():
        print(f"{k:28s} {v:+.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
