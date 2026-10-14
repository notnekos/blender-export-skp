"""2D plan (top view) generation with hidden line removal.

Feature edges of every mesh instance (boundary, sharp, silhouette and loose
edges) are projected onto the XY plane.  Visibility is tested by casting rays
straight up (+Z) through a BVH of all occluding triangles; collinear and
overlapping segments are merged afterwards.
"""

import math

from mathutils import Vector
from mathutils.bvhtree import BVHTree

from .collect import ROLE_MAIN, ROLE_SAFETY, transform

UP = Vector((0.0, 0.0, 1.0))


def _edge_faces(geom):
    """{(i, j): [face indices]} using the planar polygons of the mesh."""
    out = {}
    for fi, face in enumerate(geom.faces):
        n = len(face)
        for k in range(n):
            a = face[k]
            b = face[(k + 1) % n]
            key = (a, b) if a < b else (b, a)
            out.setdefault(key, []).append(fi)
    return out


def _normal(pts):
    nx = ny = nz = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1, z1 = pts[i]
        x2, y2, z2 = pts[(i + 1) % n]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length < 1e-20:
        return (0.0, 0.0, 0.0)
    return (nx / length, ny / length, nz / length)


def _feature_edges(geom, world, feature_cos, all_edges=False):
    """Yield world-space (p1, p2) for feature edges of one instance."""
    normals = [_normal([world[i] for i in f]) for f in geom.faces]
    eps = 1e-6
    for (a, b), faces in _edge_faces(geom).items():
        keep = all_edges or len(faces) != 2
        if not keep:
            n1 = normals[faces[0]]
            n2 = normals[faces[1]]
            if geom.face_groups[faces[0]] == geom.face_groups[faces[1]]:
                continue  # diagonal inside one source polygon
            dot = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]
            s1 = 1 if n1[2] > eps else (-1 if n1[2] < -eps else 0)
            s2 = 1 if n2[2] > eps else (-1 if n2[2] < -eps else 0)
            keep = dot < feature_cos or s1 != s2
        if keep:
            yield world[a], world[b]
    for a, b in geom.loose_edges:
        yield world[a], world[b]


class _Visibility:
    def __init__(self, tris_verts, tris, size):
        self.bvh = BVHTree.FromPolygons(tris_verts, tris, all_triangles=True) if tris else None
        self.delta = max(size * 2e-5, 1e-7)
        self.eps = self.delta * 10.0

    def visible(self, p, perp):
        if self.bvh is None:
            return True
        z = p[2] + self.eps
        ray = self.bvh.ray_cast
        if ray(Vector((p[0], p[1], z)), UP)[0] is None:
            return True
        d = self.delta
        if ray(Vector((p[0] + perp[0] * d, p[1] + perp[1] * d, z)), UP)[0] is None:
            return True
        if ray(Vector((p[0] - perp[0] * d, p[1] - perp[1] * d, z)), UP)[0] is None:
            return True
        return False


def _visible_parts(vis, p1, p2, step):
    """Split a 3D edge into visible parameter intervals [(t0, t1)]."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    length2d = math.hypot(dx, dy)
    perp = (-dy / length2d, dx / length2d)
    n = max(1, min(256, int(math.ceil(length2d / step))))

    def test(t):
        return vis.visible((p1[0] + dx * t, p1[1] + dy * t, p1[2] + dz * t), perp)

    def refine(t_a, t_b, state_a):
        # binary search of the visibility change between t_a and t_b
        for _ in range(8):
            mid = (t_a + t_b) * 0.5
            if test(mid) == state_a:
                t_a = mid
            else:
                t_b = mid
        return (t_a + t_b) * 0.5

    samples = [(i + 0.5) / n for i in range(n)]
    states = [test(t) for t in samples]
    if all(states):
        return [(0.0, 1.0)]
    if not any(states):
        return []
    parts = []
    start = 0.0 if states[0] else None
    for i in range(1, n):
        if states[i] != states[i - 1]:
            t = refine(samples[i - 1], samples[i], states[i - 1])
            if states[i]:
                start = t
            else:
                parts.append((start, t))
                start = None
    if start is not None:
        parts.append((start, 1.0))
    return parts


def merge_segments(segments, tol):
    """Merge collinear overlapping 2D segments. segments: [((x1, y1), (x2, y2))]."""
    groups = {}
    for (x1, y1), (x2, y2) in segments:
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length < tol:
            continue
        ux = dx / length
        uy = dy / length
        if ux < -1e-12 or (abs(ux) <= 1e-12 and uy < 0):
            ux, uy = -ux, -uy
        angle = math.atan2(uy, ux)
        offset = ux * y1 - uy * x1  # signed distance of the line from origin
        key = (round(angle / 2e-5), round(offset / tol))
        s1 = ux * x1 + uy * y1
        s2 = ux * x2 + uy * y2
        if s1 > s2:
            s1, s2 = s2, s1
        g = groups.get(key)
        if g is None:
            g = groups[key] = [ux, uy, 0.0, 0, []]
        g[2] += offset
        g[3] += 1
        g[4].append((s1, s2))

    out = []
    for ux, uy, off_sum, count, intervals in groups.values():
        offset = off_sum / count
        nx, ny = -uy, ux  # normal: point = s*u + offset*n
        intervals.sort()
        cur_a, cur_b = intervals[0]
        merged = []
        for a, b in intervals[1:]:
            if a <= cur_b + tol:
                if b > cur_b:
                    cur_b = b
            else:
                merged.append((cur_a, cur_b))
                cur_a, cur_b = a, b
        merged.append((cur_a, cur_b))
        for a, b in merged:
            out.append(((a * ux + offset * nx, a * uy + offset * ny),
                        (b * ux + offset * nx, b * uy + offset * ny)))
    return out


def build_plan(scene_data, unit_factor, feature_angle_deg=30.0, hidden_lines=True,
               precision=1000, merge_tol=0.5, progress=None):
    """Return {(role, layer override or None): [2D segments]} in drawing units."""
    feature_cos = math.cos(math.radians(feature_angle_deg))

    # world positions per instance
    worlds = []
    for inst in scene_data.instances:
        geom = scene_data.meshes[inst.mesh]
        worlds.append([transform(inst.matrix, v) for v in geom.verts])

    # occluders: all main geometry
    tri_verts = []
    tris = []
    mn = [1e30] * 3
    mx = [-1e30] * 3
    for inst, world in zip(scene_data.instances, worlds):
        if inst.role != ROLE_MAIN:
            continue
        geom = scene_data.meshes[inst.mesh]
        base = len(tri_verts)
        tri_verts.extend(world)
        tris.extend((a + base, b + base, c + base) for a, b, c in geom.tris)
        for p in world:
            for i in range(3):
                if p[i] < mn[i]:
                    mn[i] = p[i]
                if p[i] > mx[i]:
                    mx[i] = p[i]
    if mx[0] < mn[0]:
        mn = [0.0] * 3
        mx = [1.0] * 3
    size = max(1e-6, math.sqrt(sum((mx[i] - mn[i]) ** 2 for i in range(3))))
    extent2d = max(mx[0] - mn[0], mx[1] - mn[1], 1e-6)
    step = extent2d / max(50, precision)
    min_len = extent2d * 1e-7

    vis = _Visibility(tri_verts, tris, size) if hidden_lines else None

    result = {}
    total = max(1, len(scene_data.instances))
    for n, (inst, world) in enumerate(zip(scene_data.instances, worlds)):
        if progress:
            progress(n / total)
        geom = scene_data.meshes[inst.mesh]
        key = (inst.role, inst.layer)
        bucket = result.setdefault(key, [])
        safety = inst.role == ROLE_SAFETY
        for p1, p2 in _feature_edges(geom, world, feature_cos):
            if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < min_len:
                continue
            if vis is None or safety:
                parts = [(0.0, 1.0)]
            else:
                parts = _visible_parts(vis, p1, p2, step)
            for t0, t1 in parts:
                a = (p1[0] + (p2[0] - p1[0]) * t0, p1[1] + (p2[1] - p1[1]) * t0)
                b = (p1[0] + (p2[0] - p1[0]) * t1, p1[1] + (p2[1] - p1[1]) * t1)
                bucket.append(((a[0] * unit_factor, a[1] * unit_factor),
                               (b[0] * unit_factor, b[1] * unit_factor)))

    for key in list(result):
        result[key] = merge_segments(result[key], merge_tol)
    return result, (mn, mx)
