#!/usr/bin/env python3
"""Shorten the wrist flange of the Inspire RH56DFTP palm mesh.

The vendor's base_link.STL carries the standard flange of the RH56DFTP-0: O50 x 30 mm,
with an aviation connector. The H1-2 carries the RH56DFTP-2, whose flange is shorter.

The flange is a separate closed solid of revolution inside the STL. This script
  - removes the slice z in [ZA, ZB] of it, which holds the whole connector recess;
  - moves what lies below (the rim chamfer and the four radial screw holes) up by
    30 mm - height;
  - rebuilds the outer wall up to the top chamfer ring, the wall of the cup and its
    floor, so the flange is still a closed solid;
  - puts the connector back, smaller and elsewhere: the vendor's O16 recess does not fit
    in a 16 mm flange, so a O14 pocket is cut in the wall halfway between the dorsal
    screw hole and the one on the little-finger side, which is where the real hand has
    it, and the vendor's connector piece goes in it scaled to O12.25.
Every other triangle of the STL is written back byte for byte.

The input must be the vendor's original mesh (30 mm flange):

    git show 45099e5:ros_h1_2_ws/src/h1_2_inspire_description/meshes/hand_left/base_link.STL > left.STL
    tools/shorten_hand_flange.py left.STL meshes/hand_left/base_link.STL 16

Usage:  shorten_hand_flange.py input.STL output.STL [height_mm=16] [--no-connector]
        11 <= height < 30; the connector needs height >= 16.
"""
import struct
import sys

import numpy as np

ORIGINAL_HEIGHT = 0.030
# Cut planes, as exact float32 values (the STL vertices are float32).
ZA = float(np.float32(-0.10146))        # m: lower plane (9.5 mm above the rim: holes included)
ZB = float(np.float32(ZA + 0.019))      # m: upper plane (1.5 mm below the flange's top face)
RING_HEIGHT = 0.0295     # m above the rim: top chamfer ring of the outer wall (r = 25)
R_OUT, R_CUP, R_BORE = 0.025, 0.019, 0.014
# Connector: a blind pocket in the outer wall, its axis radial, with the vendor's piece in it.
POCKET_R = 0.007             # m (the vendor's recess is 0.008)
VENDOR_POCKET_R = 0.008
POCKET_FLOOR = 0.022         # m from the flange axis: 3 mm deep, as the vendor's
SECTOR = np.radians(25.0)    # half-width of the stretch of wall that is re-meshed: ends on ring vertices
N_RIM = 48


def read(path):
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    assert len(data) == 84 + 50 * n, "not a binary STL"
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return data[:80], np.frombuffer(data, dtype=dt, count=n, offset=84).copy()


def write(path, header, tris):
    with open(path, "wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(tris)))
        f.write(tris.tobytes())


def weld(v):
    """Unique vertices (exact float32 equality) and faces as indices."""
    pts = np.ascontiguousarray(v.reshape(-1, 3))
    key = pts.view([("", pts.dtype)] * 3).ravel()
    _, first, inv = np.unique(key, return_index=True, return_inverse=True)
    return pts[first].astype(np.float64), inv.reshape(-1, 3)


def components(faces, nv):
    parent = np.arange(nv)

    def root(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b, c in faces:
        ra, rb, rc = root(a), root(b), root(c)
        r = min(ra, rb, rc)
        parent[ra] = parent[rb] = parent[rc] = r
    return np.array([root(i) for i in range(nv)])


def f32(a):
    """Round to float32 and return float64: the coordinates that end up in the STL."""
    return np.asarray(a, dtype=np.float64).astype(np.float32).astype(np.float64)


def cut(a, b, z):
    """Point of segment a-b on plane z. Always interpolated from the end with the lower
    z, so two neighbouring triangles get exactly the same point."""
    if (a[2], a[0], a[1]) > (b[2], b[0], b[1]):
        a, b = b, a
    t = (z - a[2]) / (b[2] - a[2])
    p = a + t * (b - a)
    p[2] = z
    return f32(p)


def clip(tri, z, keep_below):
    """Sutherland-Hodgman of one triangle against plane z. Returns triangles."""
    inside = (lambda p: p[2] <= z) if keep_below else (lambda p: p[2] >= z)
    poly = []
    for i in range(3):
        a, b = tri[i], tri[(i + 1) % 3]
        if inside(a):
            poly.append(a)
            if not inside(b):
                poly.append(cut(a, b, z))
        elif inside(b):
            poly.append(cut(a, b, z))
    return [np.array([poly[0], poly[i], poly[i + 1]]) for i in range(1, len(poly) - 1)]


def loop(points, centre):
    """Unique points (exact equality) sorted by angle around the axis."""
    u = np.unique(np.asarray(points), axis=0)
    ang = np.arctan2(u[:, 1] - centre[1], u[:, 0] - centre[0])
    order = np.argsort(ang)
    return u[order], ang[order]


def stitch(pa, aa, pb, ab):
    """Triangles between two loops sorted by angle (zipper)."""
    tris, i, j, na, nb = [], 0, 0, len(pa), len(pb)
    while i < na or j < nb:
        a0, b0 = pa[i % na], pb[j % nb]
        next_a = aa[i + 1] if i + 1 < na else aa[0] + 2 * np.pi
        next_b = ab[j + 1] if j + 1 < nb else ab[0] + 2 * np.pi
        if j >= nb or (i < na and next_a <= next_b):
            tris.append(np.array([a0, pa[(i + 1) % na], b0]))
            i += 1
        else:
            tris.append(np.array([a0, pb[(j + 1) % nb], b0]))
            j += 1
    return tris


def orient(tris, normal_at):
    """Leave every triangle with its normal along normal_at(centroid)."""
    out = []
    for t in tris:
        n = np.cross(t[1] - t[0], t[2] - t[0])
        if np.dot(n, normal_at(t.mean(0))) < 0:
            t = t[[0, 2, 1]]
        out.append(t)
    return out


def open_edges(tris):
    """Edges that do not appear exactly twice, once in each direction."""
    _, faces = weld(np.asarray(tris, dtype=np.float32))
    count = {}
    for f in faces:
        for k in range(3):
            a, b = int(f[k]), int(f[(k + 1) % 3])
            count[(a, b)] = count.get((a, b), 0) + 1
    return sum(1 for (a, b), c in count.items() if c != 1 or count.get((b, a), 0) != 1)


def volume(tris):
    t = np.asarray(tris, dtype=np.float64)
    return float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6.0)


ON_PLANE = 1e-7              # m: a vertex this close to a cutting plane is on it


def clip_dist(tri, dist, eps=ON_PLANE):
    """Part of a triangle where dist >= 0 (dist: signed distance of each vertex to a plane).
    Returns (triangles, points of the kept part that lie on the plane)."""
    inside, on = dist >= -eps, np.abs(dist) <= eps
    poly, on_plane = [], []

    def crossing(a, b, da, db):
        if tuple(a) > tuple(b):          # same point from both neighbouring triangles
            a, b, da, db = b, a, db, da
        return f32(a + da / (da - db) * (b - a))

    for i in range(3):
        j = (i + 1) % 3
        if inside[i]:
            poly.append(tri[i])
            if on[i]:
                on_plane.append(tri[i])
            elif not inside[j]:
                p = crossing(tri[i], tri[j], dist[i], dist[j])
                poly.append(p)
                on_plane.append(p)
        elif inside[j] and not on[j]:
            p = crossing(tri[i], tri[j], dist[i], dist[j])
            poly.append(p)
            on_plane.append(p)
    out = [np.array([poly[0], poly[i], poly[i + 1]]) for i in range(1, len(poly) - 1)]
    out = [t for t in out if np.linalg.norm(np.cross(t[1] - t[0], t[2] - t[0])) > 1e-13]
    return out, (on_plane if out else [])


def add_connector(flange, vendor, centre, z_lo, z_hi):
    """Cut the connector pocket in the outer wall of the flange (still a closed solid) and
    return (flange, connector piece). z_lo, z_hi: the two chamfer rings of the outer wall."""
    side = 1.0 if centre[1] > 0 else -1.0            # left hand: dorsum towards +y; right: -y
    th = side * np.radians(45.0)                     # between the dorsal hole and the little-finger one
    a_hat = np.array([np.cos(th), np.sin(th)])       # pocket axis, away from the flange axis
    t_hat = np.array([-a_hat[1], a_hat[0]])
    zc = (z_lo + z_hi) / 2
    assert (z_hi - z_lo) / 2 - POCKET_R > 2e-4, "the connector does not fit in this flange height"

    flange = np.asarray(flange, dtype=np.float64)
    xy = flange[..., :2] - centre
    delta = np.arctan2(xy @ t_hat, xy @ a_hat)       # angle of every vertex from the pocket axis
    wall = ((np.hypot(xy[..., 0], xy[..., 1]).min(1) > R_OUT - 2e-4)
            & (flange[..., 2].min(1) > z_lo - 2e-6) & (flange[..., 2].max(1) < z_hi + 2e-6))
    near = np.abs(delta).max(1) < np.pi / 2           # not the far side, where the angle wraps
    assert (np.ptp(delta[wall & near], axis=1) < np.radians(60)).all(), "a wall triangle spans too wide an angle"

    # The two planes through the flange axis that bound the re-meshed stretch of wall; the
    # distance to each is positive outside the stretch.
    normals = {s: s * np.array([-np.sin(th + s * SECTOR), np.cos(th + s * SECTOR)]) for s in (-1, 1)}
    dist = {s: xy @ n for s, n in normals.items()}
    hit = wall & near & ~(dist[-1] >= -ON_PLANE).all(1) & ~(dist[1] >= -ON_PLANE).all(1)
    kept, edge = [], {-1: [], 1: []}
    for i in np.flatnonzero(hit):
        for s in (-1, 1):
            pieces, pts = clip_dist(flange[i], dist[s][i])
            kept += pieces
            edge[s] += pts
    for s in (-1, 1):                                 # wall vertices of untouched triangles on the planes
        on = (wall & near & ~hit)[:, None] & (np.abs(dist[s]) <= ON_PLANE)
        edge[s] += list(flange[on])

    # Boundary of the hole left in the wall: the two rings between the planes and the two cut lines.
    v = flange[wall].reshape(-1, 3)
    d = np.arctan2((v[:, :2] - centre) @ t_hat, (v[:, :2] - centre) @ a_hat)
    rings = v[(np.abs(d) <= SECTOR + 1e-9) & ((np.abs(v[:, 2] - z_lo) < 2e-6) | (np.abs(v[:, 2] - z_hi) < 2e-6))]
    border = np.unique(np.vstack([rings] + [np.array(edge[s]) for s in (-1, 1)]), axis=0)
    for s in (-1, 1):
        z = np.unique(np.array(edge[s]), axis=0)[:, 2]
        assert abs(z.min() - z_lo) < 2e-6 and abs(z.max() - z_hi) < 2e-6, "cut line does not end on the rings"
    bd = np.arctan2((border[:, :2] - centre) @ t_hat, (border[:, :2] - centre) @ a_hat)
    ang_border = np.arctan2(border[:, 2] - zc, R_OUT * bd)      # polar angle in the unrolled wall
    order = np.argsort(ang_border)
    border, ang_border = border[order], ang_border[order]

    phi = -np.pi + 2 * np.pi * (np.arange(N_RIM) + 0.5) / N_RIM
    sr, tr = POCKET_R * np.cos(phi), POCKET_R * np.sin(phi)

    def at(a, s, t):                                  # pocket coordinates -> mesh coordinates
        p = np.empty((len(s), 3))
        p[:, :2] = centre + np.outer(a, a_hat) + np.outer(s, t_hat)
        p[:, 2] = zc + t
        return f32(p)

    rim = at(np.sqrt(R_OUT**2 - sr**2), sr, tr)       # on the outer wall
    bottom = at(np.full(N_RIM, POCKET_FLOOR), sr, tr)
    mid = at(np.array([POCKET_FLOOR]), np.array([0.0]), np.array([0.0]))[0]
    ang_rim = np.arctan2(tr, R_OUT * np.arcsin(sr / R_OUT))

    radial = lambda c: np.array([c[0] - centre[0], c[1] - centre[1], 0.0])
    to_axis = lambda c: -np.array([*(((c[:2] - centre) @ t_hat) * t_hat), c[2] - zc])
    skin = orient(stitch(border, ang_border, rim, ang_rim), radial)
    side_wall, floor = [], []
    for k in range(N_RIM):
        j = (k + 1) % N_RIM
        side_wall += [np.array([rim[k], rim[j], bottom[k]]), np.array([rim[j], bottom[j], bottom[k]])]
        floor.append(np.array([mid, bottom[k], bottom[j]]))
    side_wall = orient(side_wall, to_axis)
    floor = orient(floor, lambda c: np.array([a_hat[0], a_hat[1], 0.0]))

    # The vendor's connector piece, from its place on the dorsal side to the pocket.
    vendor = np.asarray(vendor, dtype=np.float64)
    av = np.array([0.0, side])                        # the vendor has it at +-90 deg
    tv = np.array([-av[1], av[0]])
    q = vendor.reshape(-1, 3)
    a = (q[:, :2] - centre) @ av
    k = POCKET_R / VENDOR_POCKET_R
    zv = (q[:, 2].max() + q[:, 2].min()) / 2
    piece = at(a - a.min() + POCKET_FLOOR, k * ((q[:, :2] - centre) @ tv), k * (q[:, 2] - zv)).reshape(-1, 3, 3)

    out = np.concatenate([flange[~hit], np.array(kept + skin + side_wall + floor)])
    return out, piece


def shorten(src, dst, height=0.016, connector=True):
    removed = ORIGINAL_HEIGHT - height
    assert 0.011 - 1e-9 <= height < ORIGINAL_HEIGHT, "height outside [11, 30) mm"
    header, tris = read(src)
    verts, faces = weld(tris["v"])
    roots = components(faces, len(verts))
    comp = roots[faces[:, 0]]

    # The flange is the component that reaches lowest. The connector is the only other
    # component that lies entirely below the flange's top face.
    flange = roots[np.argmin(verts[:, 2])]
    is_flange = comp == flange
    vf = verts[np.unique(faces[is_flange])]
    size = vf.max(0) - vf.min(0)
    assert np.allclose(size, [0.050, 0.050, 0.0385], atol=2e-4), f"unexpected flange: {size}"
    centre = (vf.max(0)[:2] + vf.min(0)[:2]) / 2
    z_rim, z_top = vf[:, 2].min(), vf[:, 2].min() + ORIGINAL_HEIGHT
    z_ring = z_rim + RING_HEIGHT
    assert abs(z_rim + 0.11096) < 2e-5, z_rim

    connectors = [r for r in np.unique(comp) if r != flange
                  and verts[np.unique(faces[comp == r])][:, 2].max() < z_top - 1e-4]
    assert len(connectors) == 1, connectors
    is_conn = comp == connectors[0]
    zc = verts[np.unique(faces[is_conn])][:, 2]
    assert ZA < zc.min() and zc.max() < ZB, "the connector does not lie inside the slice"

    radius = lambda p: np.hypot(p[..., 0] - centre[0], p[..., 1] - centre[1])
    assert np.abs(vf[:, 2] - ZA).min() > 1e-5 and np.abs(vf[:, 2] - ZB).min() > 1e-5

    up = np.array([0.0, 0.0, removed])
    below, above = [], []
    for t in tris["v"][is_flange].astype(np.float64):
        below += clip(t, ZA, True)
        above += clip(t, ZB, False)

    # Outer wall above the cut (between ZB and the chamfer ring): rebuilt by stitching.
    def is_outer_wall(t):
        return radius(t).min() > R_OUT - 2e-4 and t[:, 2].max() < z_ring + 1e-5

    above = [t for t in above if not is_outer_wall(t)]

    moved = [f32(t + up) for t in below]
    pa = np.array([q for t, s in zip(below, moved) for p, q in zip(t, s) if p[2] == ZA])
    pb = np.array([p for t in above for p in t if p[2] == ZB])
    ra, rb = radius(pa), radius(pb)
    assert ((np.abs(ra - R_OUT) < 2e-4) | (np.abs(ra - R_CUP) < 2e-4)).all(), "cut ZA not clean"
    assert (np.abs(rb - R_BORE) < 2e-4).all(), "cut ZB not clean"

    outer, ang_outer = loop(pa[ra > 0.022], centre)
    cup, ang_cup = loop(pa[ra < 0.022], centre)
    bore, ang_bore = loop(pb, centre)
    ring = vf[(np.abs(vf[:, 2] - z_ring) < 1e-5) & (np.abs(radius(vf) - R_OUT) < 1e-5)]
    ring, ang_ring = loop(ring, centre)
    assert len(ring) == 36, len(ring)

    radial = lambda c: np.array([c[0] - centre[0], c[1] - centre[1], 0.0])
    wall = orient(stitch(outer, ang_outer, ring, ang_ring), radial)
    # Wall of the cup, from the moved loop up to ZB (not needed if the loop is already at ZB).
    cup_wall = []
    if cup[0, 2] < ZB - 1e-7:
        top = cup.copy()
        top[:, 2] = ZB
        n = len(cup)
        for i in range(n):
            j = (i + 1) % n
            cup_wall += [np.array([cup[i], cup[j], top[i]]), np.array([cup[j], top[j], top[i]])]
        cup_wall = orient(cup_wall, lambda c: -radial(c))
        cup = top
    floor = orient(stitch(cup, ang_cup, bore, ang_bore), lambda c: np.array([0.0, 0.0, -1.0]))

    new = np.asarray(moved + above + wall + cup_wall + floor, dtype=np.float32)
    solid = volume(new)

    piece = np.zeros((0, 3, 3), dtype=np.float32)
    if connector:
        v = new.reshape(-1, 3).astype(np.float64)
        on = np.abs(radius(v) - R_OUT) < 1e-5
        z_lo = v[on & (np.abs(v[:, 2] - (z_rim + removed + 0.0005)) < 2e-6), 2]
        z_hi = v[on & (np.abs(v[:, 2] - z_ring) < 2e-6), 2]
        assert len(z_lo) and len(z_hi) and np.ptp(z_lo) == 0 and np.ptp(z_hi) == 0
        new, piece = add_connector(new, tris["v"][is_conn], centre, z_lo[0], z_hi[0])
        new, piece = new.astype(np.float32), piece.astype(np.float32)

    before = tris["v"][is_flange]
    assert open_edges(before) == 0, "the original flange was not a closed solid"
    n_open = open_edges(new)
    assert n_open == 0, f"{n_open} open edges left"
    v0, v1 = volume(before), volume(new)
    assert 0 < v1 < v0
    if connector:                                     # the pocket is a 3 mm deep O14 hole, and no more
        pocket = solid - v1
        assert 0.8 < pocket / (np.pi * POCKET_R**2 * 0.003) < 1.25, pocket
        assert open_edges(piece) == 0 and volume(piece) > 0

    def block_of(v):
        b = np.zeros(len(v), dtype=tris.dtype)
        b["v"] = v
        n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0]).astype(np.float64)
        length = np.linalg.norm(n, axis=1, keepdims=True)
        b["n"] = (n / np.where(length > 0, length, 1.0)).astype(np.float32)
        return b

    block = block_of(new)
    out = np.concatenate([tris[~is_flange & ~is_conn], block, block_of(piece)])
    write(dst, header, out)

    z = out["v"][..., 2]
    print(f"{src}\n  -> {dst}")
    print(f"  triangles {len(tris)} -> {len(out)}  (flange {is_flange.sum()} -> {len(block)}, "
          f"connector {is_conn.sum()} -> {len(piece)})")
    print(f"  flange axis x,y = {centre[0]*1000:+.3f}, {centre[1]*1000:+.3f} mm")
    print(f"  lowest z {z_rim*1000:.2f} -> {z.min()*1000:.2f} mm ;"
          f" flange height {(z_top - z.min())*1000:.2f} mm ; cup {(ZB - z.min())*1000:.2f} mm")
    print(f"  flange volume {v0*1e6:.2f} -> {v1*1e6:.2f} cm3 ; open edges 0")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "--no-connector"]
    shorten(args[0], args[1], float(args[2]) / 1000 if len(args) > 2 else 0.016,
            connector="--no-connector" not in sys.argv)
