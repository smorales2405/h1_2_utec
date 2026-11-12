#!/usr/bin/env python3
"""Fit the software-coupling curves of the RH56DFTP hands to the photo measurements.

    config/hand_coupling_measurements.yaml  ->  config/hand_coupling.yaml

    python3 tools/fit_hand_coupling.py              rewrite config/hand_coupling.yaml
    python3 tools/fit_hand_coupling.py --check      compare with the file on disk, write nothing
    python3 tools/fit_hand_coupling.py --plot f.png also draw curves and measurements

Every curve maps s = 1 - ANGLE_SET/1000 (0 open, 1 closed) to one joint angle in rad, and starts
at q = 0: the URDF zero of every hand joint is the open hand (ANGLE_SET 1000). The cubics are
fitted through the origin and the rotation's interpolation passes through 0 at s = 0. The tool
also reports where the measurements alone put the open pose. If that drifts more than OPEN_TOL
from 0, the URDF zero no longer matches the hand: rotate the joint origins by the drift and
subtract it from the measurements.

  - Fingers: one curve per phalanx shared by the four fingers, which agree within the
    measurement noise (rms 0.05 rad around the common curve). Weighted least-squares cubic,
    constrained to grow over [0, 1].
  - Thumb bending: one weighted cubic per joint (thumb_1, thumb_2, thumb_3), also constrained
    to grow. thumb_2 and thumb_3 are fitted free, not as multiples of thumb_1: the photos tell
    the three apart (rms 0.028 / 0.029 rad around the free curves, 0.039 / 0.058 around the
    mimic variant's 0.601 / 0.579 x thumb_1).
  - Thumb rotation: monotone cubic interpolation (PCHIP) through the nine points (the one at
    ANGLE_SET 1000 taken as 0), which are precise to 0.01 rad and flatten sharply between
    ANGLE_SET 125 and 0; a cubic would round that knee off by 0.03 rad.

Each measurement weighs 1/sigma, sigma = half the width of its 5 % cost interval, at least
SIGMA_MIN. The curves are written at 21 nodes (s every 0.05); between nodes they are read
by linear interpolation, which stays within 0.001 rad of the fitted functions.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.interpolate import PchipInterpolator
from scipy.optimize import minimize

PKG = Path(__file__).resolve().parents[1]
MEASUREMENTS = PKG / "config" / "hand_coupling_measurements.yaml"
OUTPUT = PKG / "config" / "hand_coupling.yaml"

SIGMA_MIN = 0.01                 # rad
OPEN_TOL = 0.02                  # rad: open pose of the measurements vs the URDF zero
NODES = np.round(np.linspace(0.0, 1.0, 21), 2)
CURVES = ("finger_1", "finger_2", "thumb_1", "thumb_2", "thumb_3", "thumb_swing")

# SDK order of ANGLE_SET / ANGLE_ACT (registers 0..5) -> joints of that DOF and the curve of each.
# The first joint of every DOF is the actuated one, used to invert the coupling.
DOFS = (
    ("little", (("little_1", "finger_1"), ("little_2", "finger_2"))),
    ("ring", (("ring_1", "finger_1"), ("ring_2", "finger_2"))),
    ("middle", (("middle_1", "finger_1"), ("middle_2", "finger_2"))),
    ("index", (("index_1", "finger_1"), ("index_2", "finger_2"))),
    ("thumb_bend", (("thumb_1", "thumb_1"), ("thumb_2", "thumb_2"), ("thumb_3", "thumb_3"))),
    ("thumb_rot", (("thumb_swing", "thumb_swing"),)),
)


def s_of(angle_set) -> np.ndarray:
    return 1.0 - np.asarray(angle_set, float) / 1000.0


def sigma(lo, hi) -> np.ndarray:
    return np.maximum((np.asarray(hi, float) - np.asarray(lo, float)) / 2.0, SIGMA_MIN)


def increasing_cubic(x, y, sig, through_origin=True):
    """Weighted least-squares cubic, constrained to dq/ds >= 0 on [0, 1] and, unless
    through_origin is False, to q(0) = 0. Returns (polyval coefficients, rms)."""
    x, y, sig = (np.asarray(v, float) for v in (x, y, sig))
    grid = np.linspace(0.0, 1.0, 201)
    full = (lambda c: np.append(c, 0.0)) if through_origin else (lambda c: c)
    A = np.vander(x, 4)[:, :3] if through_origin else np.vander(x, 4)
    c0 = np.linalg.lstsq(A / sig[:, None], y / sig, rcond=None)[0]
    res = minimize(lambda c: float(np.sum(((y - np.polyval(full(c), x)) / sig) ** 2)), c0, method="SLSQP",
                   constraints=[dict(type="ineq", fun=lambda c: np.polyval(np.polyder(full(c)), grid))],
                   options=dict(ftol=1e-12, maxiter=500))
    if not res.success:
        raise RuntimeError(f"constrained fit failed: {res.message}")
    c = full(res.x)
    return c, float(np.sqrt(np.mean((y - np.polyval(c, x)) ** 2)))


def fit(meas: dict) -> tuple[dict, dict, dict]:
    """{curve: values at NODES}, {curve: rms of the measurements around it (rad)},
    {curve: open pose of the measurements alone, i.e. q at s = 0 of a fit not forced through 0}."""
    curves, rms, open_q = {}, {}, {}
    # fingers: the four series pooled, plus the open pose measured on the four together
    s1, q1, w1, s2, q2, w2 = ([] for _ in range(6))
    for name, f in meas["fingers"].items():
        if name == "open_stack":
            a, q, lo, hi = f
            s1.append(float(s_of(a))), q1.append(q), w1.append(float(sigma(lo, hi)))
            continue
        for a, qa, loa, hia, qb, lob, hib in f["angles"]:
            s1.append(float(s_of(a))), q1.append(qa), w1.append(float(sigma(loa, hia)))
            s2.append(float(s_of(a))), q2.append(qb), w2.append(float(sigma(lob, hib)))
    for name, (x, y, sg) in (("finger_1", (s1, q1, w1)), ("finger_2", (s2, q2, w2))):
        c, rms[name] = increasing_cubic(x, y, sg)
        curves[name] = np.polyval(c, NODES)
        open_q[name] = float(increasing_cubic(x, y, sg, through_origin=False)[0][-1])
    # thumb bending: thumb_1, thumb_2 and thumb_3 each on their own
    tb = np.array(meas["thumb_bend"]["angles"], float)
    for k, name in enumerate(("thumb_1", "thumb_2", "thumb_3")):
        x, y, sg = s_of(tb[:, 0]), tb[:, 1 + 3 * k], sigma(tb[:, 2 + 3 * k], tb[:, 3 + 3 * k])
        c, rms[name] = increasing_cubic(x, y, sg)
        curves[name] = np.polyval(c, NODES)
        open_q[name] = float(increasing_cubic(x, y, sg, through_origin=False)[0][-1])
    # thumb rotation: through the points, the open one (ANGLE_SET 1000) taken as 0
    tr = np.array(sorted(meas["thumb_rot"]["angles"], key=lambda r: -r[0]), float)
    if tr[0, 0] != 1000:
        raise RuntimeError("thumb_rot: the measurement at ANGLE_SET 1000 is missing")
    open_q["thumb_swing"] = float(tr[0, 1])
    tr[0, 1] = 0.0
    curves["thumb_swing"] = PchipInterpolator(s_of(tr[:, 0]), tr[:, 1])(NODES)
    rms["thumb_swing"] = 0.0
    for name, v in curves.items():
        if np.any(np.diff(v) <= 0.0):
            raise RuntimeError(f"{name}: the curve does not grow strictly, it cannot be inverted")
    return {k: np.round(curves[k], 4) for k in CURVES}, rms, open_q


def limits(curves: dict) -> dict:
    """[lower, upper] of every hand joint in the software variant: from 0 (open) to the curve's
    closed value, rounded up to 0.001 rad."""
    out = {}
    for _, joints in DOFS:
        for joint, curve in joints:
            out[joint] = [0.0, math.ceil(round(curves[curve][-1] * 1000, 6)) / 1000]
    return out


def render(curves: dict, rms: dict) -> str:
    f = lambda v: f"{v:.4f}".replace("-0.0000", "0.0000")
    L = []
    w = L.append
    w("# Software coupling of the Inspire RH56DFTP hands: joint angle as a function of the command.")
    w("#")
    w("# GENERATED by tools/fit_hand_coupling.py from config/hand_coupling_measurements.yaml.")
    w("# Do not edit by hand: change the measurements and run the tool again.")
    w("#")
    w("# s = 1 - ANGLE_SET/1000: 0 = open, 1 = closed (ANGLE_ACT reads on the same scale).")
    w("# Every curve gives one joint in rad, URDF convention, at the nodes in `s`; read it by linear")
    w("# interpolation in between. Every curve starts at 0: q = 0 is the open hand. Both hands use")
    w("# the same curves: the right hand's joints are mirrored so that the same q gives the mirrored")
    w("# pose.")
    w("# Read by: urdf/inspire_hand_*.urdf.xacro (limits, coupling:=software), scripts/hand_coupling.py.")
    w("")
    w("s: [" + ", ".join(f"{x:.2f}" for x in NODES) + "]")
    w("")
    w("curves:")
    for k in CURVES:
        w(f"  {k}: [" + ", ".join(f(x) for x in curves[k]) + "]")
    w("")
    w("# rms of the measurements around each curve (rad); 0 = curve through the points")
    w("fit_rms: {" + ", ".join(f"{k}: {rms[k]:.3f}" for k in CURVES) + "}")
    w("")
    w("# SDK order of ANGLE_SET / ANGLE_ACT (index 0..5). joints: URDF joint suffix -> curve. The first")
    w("# joint of each DOF is the actuated one (the joint the inverse, joints -> ANGLE_SET, reads).")
    w("dofs:")
    for name, joints in DOFS:
        w(f"  - name: {name}")
        w("    joints: {" + ", ".join(f"{j}: {c}" for j, c in joints) + "}")
    w("")
    w("# [lower, upper] of each joint with coupling:=software: from 0 (open) to the closed value.")
    w("limits:")
    for j, (lo, hi) in limits(curves).items():
        w(f"  {j}: [{lo:.3f}, {hi:.3f}]")
    return "\n".join(L) + "\n"


def plot(path: Path, curves: dict, meas: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    blue, ink, ink2, muted = "#2a78d6", "#0b0b0b", "#52514e", "#c9c7c1"
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.6))
    marks = dict(index="o", middle="s", ring="^", little="D")
    titles = dict(finger_1="Fingers, proximal (*_1)", finger_2="Fingers, distal (*_2)", thumb_1="thumb_1",
                  thumb_2="thumb_2", thumb_3="thumb_3", thumb_swing="thumb_swing")
    tb = np.array(meas["thumb_bend"]["angles"], float)
    tr = np.array(meas["thumb_rot"]["angles"], float)
    for ax, k in zip(axes.ravel(), CURVES):
        ax.grid(True, color="#e6e5e1", lw=0.6)
        ax.spines[["top", "right"]].set_visible(False)
        if k.startswith("finger"):
            col = 1 if k == "finger_1" else 4
            for name, m in marks.items():
                a = np.array(meas["fingers"][name]["angles"], float)
                ax.errorbar(a[:, 0], a[:, col], yerr=[a[:, col] - a[:, col + 1], a[:, col + 2] - a[:, col]], fmt=m,
                            ms=5, mfc="white", mec=ink2, ecolor=muted, elinewidth=0.8, label=name)
            if k == "finger_1":
                a, q, lo, hi = meas["fingers"]["open_stack"]
                ax.errorbar([a], [q], yerr=[[q - lo], [hi - q]], fmt="*", ms=9, mfc="white", mec=ink2,
                            ecolor=muted, label="open, 4 fingers")
        elif k == "thumb_swing":
            ax.plot(tr[:, 0], tr[:, 1], "o", ms=5, mfc="white", mec=ink2, label="measured")
        else:
            c = 1 + 3 * ("thumb_1", "thumb_2", "thumb_3").index(k)
            ax.errorbar(tb[:, 0], tb[:, c], yerr=[tb[:, c] - tb[:, c + 1], tb[:, c + 2] - tb[:, c]], fmt="o", ms=5,
                        mfc="white", mec=ink2, ecolor=muted, elinewidth=0.8, label="measured")
        ax.plot(1000 * (1 - NODES), curves[k], color=blue, lw=2, label="curve")
        ax.set_xlim(1030, -30)
        ax.set_title(titles[k], loc="left", color=ink, fontsize=10)
        ax.set_xlabel("ANGLE_SET")
        ax.set_ylabel("q (rad)")
        ax.legend(frameon=False, fontsize=7.5, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="compare with config/hand_coupling.yaml, write nothing")
    ap.add_argument("--plot", type=Path, help="also save a figure of curves and measurements")
    args = ap.parse_args()
    meas = yaml.safe_load(MEASUREMENTS.read_text(encoding="utf-8"))
    curves, rms, open_q = fit(meas)
    text = render(curves, rms)
    if args.plot:
        plot(args.plot, curves, meas)
    if args.check:
        same = OUTPUT.exists() and OUTPUT.read_text(encoding="utf-8") == text
        print("config/hand_coupling.yaml is up to date" if same else "config/hand_coupling.yaml differs from the fit")
        return 0 if same else 1
    OUTPUT.write_text(text, encoding="utf-8")
    for k in CURVES:
        v = curves[k]
        print(f"{k:12s} q(500) = {v[10]:.3f}  q(closed) = {v[-1]:.3f}  rms {rms[k]:.3f}  "
              f"open pose of the measurements {open_q[k]:+.3f} rad")
    print(f"written {OUTPUT.relative_to(PKG)}")
    drift = {k: v for k, v in open_q.items() if abs(v) > OPEN_TOL}
    if drift:
        print(f"WARNING: the measured open pose is more than {OPEN_TOL} rad from q = 0 in {sorted(drift)}; "
              "the URDF zero no longer matches the hand (see the docstring)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
