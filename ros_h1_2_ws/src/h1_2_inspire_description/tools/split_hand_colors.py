#!/usr/bin/env python3
"""Split the Inspire hand meshes by colour, for the visuals of the URDF.

An STL carries no colour and a URDF visual takes one material, so every link mesh of the
hands is written again as up to three pieces, one per colour, and the link declares one
<visual> per piece. The pieces add up to the link mesh; the link mesh itself stays as the
collision geometry.

    meshes/hand_left/left_index_1.STL  ->  meshes/hand_left/visual/left_index_1_black.STL
                                           meshes/hand_left/visual/left_index_1_white.STL
                                           meshes/hand_left/visual/left_index_1_silver.STL

Colours, as on the real hand: black for the flange, the palm and the rubber pads and tips
of the digits; white for the cover on the back of the hand and the shells of the digits;
silver for the ring between palm and flange, the connector and the metal brackets, rods
and hinge lugs.

How a triangle gets its colour. Each mesh is cut into smooth patches along its creases
(dihedral angle over 30 deg), which is how the CAD faces come back out of the STL: the
dorsal cover, the rubber of the palm, the silver ring and the pad and shell of each finger
are one patch each. A few patches are coloured by rule and the rest take the colour of
the neighbour they share the longest border with. The thumb has no crease between rubber
and shell, so there the rubber ends on a plane at a height read off photos of the real
hand, and the triangles that plane crosses are cut along it.

The right hand is classified in the mirror image of the left, so both come out alike. Its
digits are the left ones mirrored, so the same rules give the same answer. Its palm is
the vendor's own mesh, in which the dorsal cover runs into the body without a crease, so
the outline of the cover is taken from the left palm: every point of the right shell has
the colour of the nearest triangle of the mirrored left shell, and the triangles the
outline crosses are cut along it. Those and the ones cut on the thumb are the only
triangles of the visuals that are not triangles of the link mesh.

Usage:  split_hand_colors.py [package_dir]        (default: the package this file is in)
"""
import struct
import sys
from pathlib import Path

import numpy as np

BLACK, WHITE, SILVER = 0, 1, 2
NAMES = ("black", "white", "silver")
CREASE = np.radians(30.0)
DT = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])

FINGERS = ("index", "middle", "ring", "little")
# How far down the sides of each thumb phalanx the rubber comes, from 1 (only the pad) to -1
# (all of it). Read off photos of the real hand; the real edge is a curve, this one is not.
THUMB_RUBBER = {"thumb_1": -0.45, "thumb_2": -0.05, "thumb_3": -0.50}


def read(path):
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    assert len(data) == 84 + 50 * n, f"{path}: not a binary STL"
    return np.frombuffer(data, dtype=DT, count=n, offset=84).copy()


def write(path, tris):
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(tris)))
        f.write(tris.tobytes())


def records(v):
    """STL records for triangles given as (n, 3, 3) vertices."""
    v = np.asarray(v, dtype=np.float32).reshape(-1, 3, 3)
    out = np.zeros(len(v), dtype=DT)
    out["v"] = v
    n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0]).astype(np.float64)
    out["n"] = (n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-30)).astype(np.float32)
    return out


def cut(tris, d):
    """Split triangles by the sign of d, one value per corner (k, 3): returns the part
    where d > 0 and the rest. A triangle with corners on both sides is cut on d = 0."""
    plus, minus = (d > 0).all(1), (d <= 0).all(1)
    pos, neg = [tris[plus]], [tris[minus]]
    for t, e in zip(tris[~plus & ~minus], d[~plus & ~minus]):
        s = e > 0
        i = next(i for i in range(3) if s[i] != s[(i + 1) % 3] and s[i] != s[(i + 2) % 3])   # the odd corner
        j, k = (i + 1) % 3, (i + 2) % 3
        pj = t[i] + e[i] / (e[i] - e[j]) * (t[j] - t[i])
        pk = t[i] + e[i] / (e[i] - e[k]) * (t[k] - t[i])
        (pos if s[i] else neg).append(np.array([[t[i], pj, pk]]))
        (neg if s[i] else pos).append(np.array([[pj, t[j], t[k]], [pj, t[k], pk]]))
    return np.concatenate(pos), np.concatenate(neg)


class Mesh:
    """Triangles with their smooth patches, connected components and patch adjacency."""

    def __init__(self, v):
        self.v = np.asarray(v, dtype=np.float64)                      # (n, 3, 3), metres
        nrm = np.cross(self.v[:, 1] - self.v[:, 0], self.v[:, 2] - self.v[:, 0])
        self.area = np.linalg.norm(nrm, axis=1) / 2
        self.n = nrm / (2 * self.area[:, None] + 1e-30)
        self.c = self.v.mean(1)
        pts = np.ascontiguousarray(self.v.reshape(-1, 3).astype(np.float32))
        _, inv = np.unique(pts.view([("", pts.dtype)] * 3).ravel(), return_inverse=True)
        faces = inv.reshape(-1, 3)
        e = np.sort(np.stack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]], 1).reshape(-1, 2), 1)
        f = np.repeat(np.arange(len(faces)), 3)
        length = np.linalg.norm(self.v[f, np.tile([0, 1, 2], len(faces))]
                                - self.v[f, np.tile([1, 2, 0], len(faces))], axis=1)
        order = np.lexsort((e[:, 1], e[:, 0]))
        e, f, length = e[order], f[order], length[order]
        same = (e[1:] == e[:-1]).all(1)
        self.fa, self.fb, self.shared = f[:-1][same], f[1:][same], length[:-1][same]
        smooth = np.einsum("ij,ij->i", self.n[self.fa], self.n[self.fb]) > np.cos(CREASE)
        self.patch = self._union(self.fa[smooth], self.fb[smooth])
        self.comp = self._union(self.fa, self.fb)
        self.parea = np.bincount(self.patch, weights=self.area)

    def _union(self, a, b):
        parent = np.arange(len(self.v))

        def root(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for i, j in zip(a, b):
            ri, rj = root(i), root(j)
            if ri != rj:
                parent[max(ri, rj)] = min(ri, rj)
        _, ids = np.unique([root(i) for i in range(len(parent))], return_inverse=True)
        return ids

    def normal(self, mask):
        """Area-weighted mean normal of the triangles in mask, and how flat they are (0..1)."""
        s = (self.n[mask] * self.area[mask, None]).sum(0)
        return s / (np.linalg.norm(s) + 1e-30), np.linalg.norm(s) / (self.area[mask].sum() + 1e-30)

    def centre(self, mask):
        return (self.c[mask] * self.area[mask, None]).sum(0) / self.area[mask].sum()

    def spread(self, colour):
        """Give every uncoloured patch the colour of the coloured neighbour it shares the
        longest border with; what never touches a coloured patch ends up black."""
        colour = colour.copy()
        pa, pb = self.patch[self.fa], self.patch[self.fb]
        while True:
            ca, cb = colour[self.fa], colour[self.fb]
            votes = {}
            for p, c, w in ((pa[(ca < 0) & (cb >= 0)], cb[(ca < 0) & (cb >= 0)], self.shared[(ca < 0) & (cb >= 0)]),
                            (pb[(cb < 0) & (ca >= 0)], ca[(cb < 0) & (ca >= 0)], self.shared[(cb < 0) & (ca >= 0)])):
                for pi, ci, wi in zip(p, c, w):
                    votes.setdefault(pi, np.zeros(3))[ci] += wi
            if not votes:
                break
            for p, w in votes.items():
                colour[self.patch == p] = int(np.argmax(w))
        colour[colour < 0] = BLACK
        return colour


def pad_direction(m):
    """The way the rubber pad faces: the mean normal of its dimples, which are the many
    tiny patches of the mesh."""
    tiny = np.isin(m.patch, np.flatnonzero(m.parea < 6e-6))
    assert len(np.unique(m.patch[tiny])) > 15, "no dimples found"
    return m.normal(tiny)[0]


def big_patches(m, least):
    return [p for p in np.argsort(-m.parea) if m.parea[p] >= least]


def faces_out(m, mask):
    """Whether the triangles in mask are an outer surface of the link: most of their area
    faces away from the link's middle."""
    mid = (m.v.reshape(-1, 3).max(0) + m.v.reshape(-1, 3).min(0)) / 2
    out = np.einsum("ij,ij->i", m.c[mask] - mid, m.n[mask]) > 0
    return (m.area[mask] * out).sum() > 0.5 * m.area[mask].sum()


def digit(m, link):
    """Triangles and colours of one phalanx. Frame of a finger link: the finger runs along +y
    and its pad faces +x. Frame of a thumb link: the thumb runs along -x. The sideways axis
    is z."""
    colour = np.full(len(m.v), -1)
    if link == "thumb_swing":                                   # the metal bracket at the base of the thumb
        return m.v, np.full(len(m.v), SILVER)

    pad = pad_direction(m)
    big = big_patches(m, 2e-4)
    dimpled = max(big, key=lambda p: m.shared[((m.patch[m.fa] == p) & (m.parea[m.patch[m.fb]] < 6e-6))
                                              | ((m.patch[m.fb] == p) & (m.parea[m.patch[m.fa]] < 6e-6))].sum())
    shell = [p for p in big if p != dimpled and m.parea[p] >= 4e-4 and faces_out(m, m.patch == p)
             and m.normal(m.patch == p)[0] @ pad < -0.5]
    wrapped = m.patch == dimpled                                # the thumb: one patch wraps pad and shell

    for p in big:                                               # inner surfaces and whatever else is large
        colour[m.patch == p] = BLACK
    if shell:                                                   # a finger: pad and shell are patches of their own
        for p in shell:
            colour[m.patch == p] = WHITE
    else:
        corners = m.v[wrapped].reshape(-1, 3)
        h = corners @ pad                                       # height towards the pad, -1 (back) to 1 (pad)
        rubber = lambda q: (2 * (q @ pad) - h.max() - h.min()) / (h.max() - h.min()) - THUMB_RUBBER[link]
        tip = lambda q: corners[:, 0].min() + 0.45 * np.ptp(corners[:, 0]) - q[..., 0]
        black = rubber(m.c[wrapped]) > 0
        if link == "thumb_3":                                   # the tip is rubber all round
            black |= tip(m.c[wrapped]) > 0
        colour[wrapped] = np.where(black, BLACK, WHITE)

    # Metal: the hinge lugs that stick out of the phalanx towards the joint.
    lugs = {"thumb_1": lambda c: c[0] < -0.029, "thumb_3": lambda c: c[0] > -0.004}
    lugs.update({f"{f}_2": (lambda c: c[1] < 0.0015) for f in FINGERS})
    if link in lugs:
        for p in np.unique(m.patch):
            if m.parea[p] < 2e-4 and lugs[link](m.centre(m.patch == p)):
                colour[m.patch == p] = SILVER
    colour = m.spread(colour)
    if shell:
        return m.v, colour

    # The thumb again, now exactly: its wrapped patch cut where the rubber ends.
    black, white = cut(m.v[wrapped], rubber(m.v[wrapped]))
    if link == "thumb_3":
        more, white = cut(white, tip(white))
        black = np.concatenate([black, more])
    return (np.concatenate([m.v[~wrapped], black, white]),
            np.concatenate([colour[~wrapped], np.full(len(black), BLACK), np.full(len(white), WHITE)]))


def palm(m, cover=True):
    """Colours of the palm. Frame (left hand): the fingers point along +z, the back of the
    hand faces +y and the flange hangs below, around an axis parallel to z. Without cover,
    the dorsal cover is left black. Leaves in m.shell the triangles of the outer shell."""
    colour = np.full(len(m.v), -1)
    comps = np.unique(m.comp)
    area = {k: m.area[m.comp == k].sum() for k in comps}
    zmin = {k: m.v[m.comp == k][..., 2].min() for k in comps}
    zmax = {k: m.v[m.comp == k][..., 2].max() for k in comps}
    flange = min((k for k in comps if area[k] > 20e-4), key=lambda k: zmin[k])   # the large solid that reaches lowest
    z_top = zmax[flange] - 0.0085                                       # top face of the flange, under its neck
    by_area = sorted(comps, key=lambda k: -area[k])
    shell, frame = by_area[0], by_area[1]
    assert flange not in (shell, frame)
    z_body = zmin[shell]                                                # where the rubber of the palm ends
    m.shell = m.comp == shell

    for k in comps:
        sel = m.comp == k
        if k == flange:
            colour[sel] = BLACK
        elif zmax[k] < z_top:                                           # sits on the flange: the connector
            colour[sel] = SILVER
        elif k == shell:
            for p in np.unique(m.patch[sel]):
                if m.parea[p] >= 2e-4:
                    dorsal = (m.parea[p] >= 30e-4 and m.normal(m.patch == p)[0][1] > 0.8
                              and faces_out(m, m.patch == p))
                    colour[m.patch == p] = WHITE if cover and dorsal else BLACK
        elif k == frame:                                                # its lower end is the silver ring
            colour[sel] = np.where(m.c[sel, 2] < z_body, SILVER, BLACK)
        else:                                                           # actuators inside; brackets, rods and pins outside
            colour[sel] = BLACK if area[k] >= 20e-4 else SILVER
    return m.spread(colour)


def distance_to_triangles(p, a, b, c):
    """Distance from point p to each of the triangles (a, b, c), given as (k, 3) arrays."""
    ab, ac, ap, bp, cp = b - a, c - a, p - a, p - b, p - c
    d1, d2 = (ab * ap).sum(1), (ac * ap).sum(1)
    d3, d4 = (ab * bp).sum(1), (ac * bp).sum(1)
    d5, d6 = (ab * cp).sum(1), (ac * cp).sum(1)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    tiny = 1e-300
    q = a + ab * (vb / (va + vb + vc + tiny))[:, None] + ac * (vc / (va + vb + vc + tiny))[:, None]   # inside
    for mask, point in (                                   # the six regions outside, last one wins
            ((va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0),
             b + (c - b) * ((d4 - d3) / (d4 - d3 + d5 - d6 + tiny))[:, None]),
            ((vb <= 0) & (d2 >= 0) & (d6 <= 0), a + ac * (d2 / (d2 - d6 + tiny))[:, None]),
            ((d6 >= 0) & (d5 <= d6), c),
            ((vc <= 0) & (d1 >= 0) & (d3 <= 0), a + ab * (d1 / (d1 - d3 + tiny))[:, None]),
            ((d3 >= 0) & (d4 <= d3), b),
            ((d1 <= 0) & (d2 <= 0), a)):
        q = np.where(mask[:, None], point, q)
    return np.linalg.norm(q - p, axis=1)


class Nearest:
    """Colour of the triangle of a mesh nearest to a point, among those that face the way
    given (any, if none does nearby)."""

    def __init__(self, mesh, sel, colour, cell=0.003):
        self.v, self.n, self.colour, self.cell = mesh.v[sel], mesh.n[sel], colour[sel], cell
        lo = np.floor(self.v.min(1) / cell).astype(int)
        hi = np.floor(self.v.max(1) / cell).astype(int)
        grid = {}
        for t in range(len(self.v)):                        # every cell the triangle's box touches
            for i in range(lo[t, 0], hi[t, 0] + 1):
                for j in range(lo[t, 1], hi[t, 1] + 1):
                    for k in range(lo[t, 2], hi[t, 2] + 1):
                        grid.setdefault((i, j, k), []).append(t)
        self.grid = {k: np.array(v) for k, v in grid.items()}
        self.around = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)]
        self.all = np.arange(len(self.v))

    def __call__(self, p, n):
        k0 = np.floor(p / self.cell).astype(int)
        cand = [self.grid[k] for k in ((k0[0] + i, k0[1] + j, k0[2] + k) for i, j, k in self.around)
                if k in self.grid]
        cand = np.unique(np.concatenate(cand)) if cand else self.all
        d = distance_to_triangles(p, self.v[cand, 0], self.v[cand, 1], self.v[cand, 2])
        same = self.n[cand] @ n > 0.3
        return self.colour[cand[np.argmin(np.where(same, d, d + 1.0)) if same.any() else np.argmin(d)]]


def paint_cover(m, src, colour):
    """The shell of m repainted with the colours of the shell of src, which is the same
    surface with other triangles. A triangle whose corners come out alike keeps that
    colour; one that the outline between two colours crosses is cut along it. Returns the
    triangles that replace m.v[m.shell] and their colours."""
    look = Nearest(src, src.shell, colour)
    tris, cols = [], []

    def crossing(a, b, ca, n):                      # point of a-b where the colour changes
        for _ in range(14):
            mid = (a + b) / 2
            if look(mid, n) == ca:
                a = mid
            else:
                b = mid
        return (a + b) / 2

    for t, n in zip(m.v[m.shell], m.n[m.shell]):
        c = [look(p, n) for p in t]
        if c[0] == c[1] == c[2]:
            tris.append(t)
            cols.append(c[0])
            continue
        i = next(i for i in range(3) if c[i] != c[(i + 1) % 3] and c[i] != c[(i + 2) % 3])   # the odd corner
        a, b, d = t[i], t[(i + 1) % 3], t[(i + 2) % 3]
        pb, pd = crossing(a, b, c[i], n), crossing(a, d, c[i], n)
        tris += [np.array([a, pb, pd]), np.array([pb, b, d]), np.array([pb, d, pd])]
        cols += [c[i], c[(i + 1) % 3], c[(i + 1) % 3]]
    return np.array(tris), np.array(cols)


def split(package):
    meshes = Path(package) / "meshes"
    report = []
    for side in ("left", "right"):
        folder = meshes / f"hand_{side}"
        out = folder / "visual"
        out.mkdir(exist_ok=True)
        for old in out.glob("*.STL"):
            old.unlink()
        for src in sorted(folder.glob("*.STL")):
            tris = read(src)
            v = tris["v"].astype(np.float64)
            if side == "right":                                         # classify the mirror image
                v = v[:, ::-1] * [1.0, -1.0, 1.0]
            m = Mesh(v)
            link = "hand_base" if src.stem == "base_link" else src.stem.split("_", 1)[1]
            if link != "hand_base":
                v, colour = digit(m, link)
            elif side == "left":
                v, colour = m.v, palm(m)
                left_palm = (m, colour)
            else:                                                       # the cover's outline comes from the left palm
                colour = palm(m, cover=False)
                shell, shell_colour = paint_cover(m, *left_palm)
                v = np.concatenate([m.v[~m.shell], shell])
                colour = np.concatenate([colour[~m.shell], shell_colour])
            if side == "right":                                         # back from the mirror image
                v = v[:, ::-1] * [1.0, -1.0, 1.0]
            line = f"{side}_{link}:"
            for c, name in enumerate(NAMES):
                if (colour == c).any():
                    piece = v[colour == c]
                    write(out / f"{side}_{link}_{name}.STL", records(piece))
                    area = np.linalg.norm(np.cross(piece[:, 1] - piece[:, 0], piece[:, 2] - piece[:, 0]), axis=1).sum() / 2
                    line += f"  {name} {area * 1e4:.1f} cm2 ({len(piece)} tri)"
            report.append(line)
    return report


if __name__ == "__main__":
    package = sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
    print("\n".join(split(package)))
