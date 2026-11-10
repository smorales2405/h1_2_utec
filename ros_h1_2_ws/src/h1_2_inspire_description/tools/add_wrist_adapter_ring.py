#!/usr/bin/env python3
"""Add the Inspire hand's adapter ring to wrist_yaw_link.STL.

Unitree's mesh ends in the O56 face at x = 54 mm, with an O37.8 boss up to x = 63. A robot
with Inspire hands has an O56 ring, 10 mm thick, bolted on that face, and it is the outer
face of that ring (x = 64 mm) that the hand's flange sits on.

The input must be Unitree's original mesh:

    git show 45099e5:ros_h1_2_ws/src/h1_2_inspire_description/meshes/h1_2/wrist_yaw_link.STL > wrist.STL
    tools/add_wrist_adapter_ring.py wrist.STL meshes/h1_2/wrist_yaw_link.STL 10

Usage:  add_wrist_adapter_ring.py input.STL output.STL [length_mm=10]
"""
import struct
import sys

import numpy as np

X0, R_OUT, R_IN, N = 0.054, 0.028, 0.019, 72


def ring(length):
    a = np.linspace(0, 2 * np.pi, N, endpoint=False)
    c, s = np.cos(a), np.sin(a)
    p = lambda x, r: np.column_stack([np.full(N, x), r * c, r * s])
    e0, e1, i0, i1 = p(X0, R_OUT), p(X0 + length, R_OUT), p(X0, R_IN), p(X0 + length, R_IN)
    t = []
    for k in range(N):
        j = (k + 1) % N
        t += [(e0[k], e0[j], e1[j]), (e0[k], e1[j], e1[k])]      # outer wall
        t += [(i0[k], i1[j], i0[j]), (i0[k], i1[k], i1[j])]      # inner wall
        t += [(e1[k], e1[j], i1[j]), (e1[k], i1[j], i1[k])]      # seat face (+x)
        t += [(e0[k], i0[j], e0[j]), (e0[k], i0[k], i0[j])]      # face against the wrist (-x)
    return np.array(t, dtype=np.float32)


def main(src, dst, length=0.010):
    data = open(src, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    assert len(data) == 84 + 50 * n, "not a binary STL"
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    tris = np.frombuffer(data, dtype=dt, count=n, offset=84)
    v = tris["v"].reshape(-1, 3)
    assert abs(v[:, 0].max() - 0.063) < 1e-5, "not the original mesh (ring already there?)"
    face = v[np.abs(v[:, 0] - X0) < 1e-6]
    assert abs(np.hypot(face[:, 1], face[:, 2]).max() - R_OUT) < 1e-5

    t = ring(length)
    new = np.zeros(len(t), dtype=dt)
    new["v"] = t
    nrm = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]).astype(np.float64)
    new["n"] = (nrm / np.linalg.norm(nrm, axis=1, keepdims=True)).astype(np.float32)
    t64 = t.astype(np.float64)
    vol = float(np.einsum("ij,ij->i", t64[:, 0], np.cross(t64[:, 1], t64[:, 2])).sum() / 6)
    expected = np.pi * (R_OUT**2 - R_IN**2) * length
    assert 0.98 * expected < vol < expected, (vol, expected)       # normals point outwards

    out = np.concatenate([tris, new])
    with open(dst, "wb") as f:
        f.write(data[:80])
        f.write(struct.pack("<I", len(out)))
        f.write(out.tobytes())
    print(f"{src}\n  -> {dst}\n  triangles {n} -> {len(out)}; ring O{2*R_OUT*1000:.0f}/O{2*R_IN*1000:.0f} mm "
          f"from x = {X0*1000:.0f} to {(X0+length)*1000:.1f} mm; largest x {out['v'][..., 0].max()*1000:.2f} mm")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]) / 1000 if len(sys.argv) > 3 else 0.010)
