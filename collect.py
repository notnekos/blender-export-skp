"""Collect exportable geometry from the Blender scene.

Produces unit-agnostic data (metres) consumed by the SKP and DWG writers:
    SceneData.meshes     - unique mesh geometry (shared by instances)
    SceneData.instances  - (mesh index, world matrix, object name, role)
    SceneData.materials  - Blender materials referenced by the geometry
"""

import math

import numpy as np

ROLE_MAIN = "MAIN"
ROLE_SAFETY = "SAFETY"
ROLE_EXCLUDE = "EXCLUDE"

SAFETY_KEYWORDS = ("safety", "zone", "зона", "зоны", "безопас")


class MeshGeom:
    __slots__ = ("name", "verts", "faces", "face_mats", "face_groups", "soft_edges",
                 "loose_edges", "tris", "tri_faces")

    def __init__(self, name):
        self.name = name
        self.verts = []        # [(x, y, z)] local coordinates, metres
        self.faces = []        # [tuple of vertex indices] (planar polygons)
        self.face_mats = []    # material slot -> global material index (-1 none)
        self.face_groups = []  # source polygon index (faces split from one polygon share it)
        self.soft_edges = set()
        self.loose_edges = []
        self.tris = []         # triangulation of all faces (vertex index triples)
        self.tri_faces = []    # index of the face each triangle belongs to


class Instance:
    __slots__ = ("mesh", "matrix", "name", "role", "layer")

    def __init__(self, mesh, matrix, name, role, layer):
        self.mesh = mesh
        self.matrix = matrix
        self.name = name
        self.role = role
        self.layer = layer


class SceneData:
    def __init__(self):
        self.meshes = []
        self.instances = []
        self.materials = []  # bpy.types.Material or None
        self._mat_index = {}

    def material_index(self, mat):
        if mat is None:
            return -1
        key = mat.name_full
        idx = self._mat_index.get(key)
        if idx is None:
            idx = len(self.materials)
            self.materials.append(mat)
            self._mat_index[key] = idx
        return idx


# ---------------------------------------------------------------------------


def object_role(obj):
    """Detect the plan layer role of an object (custom property or collection)."""
    prop = obj.get("dwg_role")
    if isinstance(prop, str) and prop.upper() in (ROLE_MAIN, ROLE_SAFETY, ROLE_EXCLUDE):
        return prop.upper()
    names = [obj.name] + [c.name for c in obj.users_collection]
    low = " ".join(names).lower()
    if any(k in low for k in SAFETY_KEYWORDS):
        return ROLE_SAFETY
    return ROLE_MAIN


def object_layer(obj):
    prop = obj.get("dwg_layer")
    return prop if isinstance(prop, str) and prop.strip() else None


def material_color(mat):
    """(r, g, b, a), metallic, roughness of a Blender material (sRGB 0..1)."""
    color = tuple(mat.diffuse_color)
    metallic = getattr(mat, "metallic", 0.0)
    roughness = getattr(mat, "roughness", 0.5)
    tree = getattr(mat, "node_tree", None)
    if tree is not None:
        bsdf = next((n for n in tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if bsdf is not None:
            base = bsdf.inputs.get("Base Color")
            if base is not None and not base.is_linked:
                color = tuple(base.default_value)
            alpha = bsdf.inputs.get("Alpha")
            if alpha is not None and not alpha.is_linked:
                color = color[:3] + (alpha.default_value,)
            met = bsdf.inputs.get("Metallic")
            if met is not None and not met.is_linked:
                metallic = met.default_value
            rough = bsdf.inputs.get("Roughness")
            if rough is not None and not rough.is_linked:
                roughness = rough.default_value
    # Blender stores linear colours, SketchUp / CAD expect sRGB
    rgb = tuple(_linear_to_srgb(c) for c in color[:3])
    alpha = color[3] if len(color) > 3 else 1.0
    return rgb + (alpha,), metallic, roughness


def _linear_to_srgb(c):
    c = max(0.0, min(1.0, c))
    if c <= 0.0031308:
        return c * 12.92
    return 1.055 * (c ** (1.0 / 2.4)) - 0.055


def _foreach(collection, attr, count, dtype, width=1):
    arr = np.empty(count * width, dtype=dtype)
    if count:
        collection.foreach_get(attr, arr)
    return arr.reshape(-1, width) if width > 1 else arr


def _mesh_from_object(obj_eval, name, scale, planar_tol, smooth_angle, scene):
    """Convert an evaluated object to MeshGeom (local coordinates in metres)."""
    try:
        mesh = obj_eval.to_mesh()
    except RuntimeError:
        return None
    if mesh is None:
        return None
    try:
        return _read_mesh(mesh, obj_eval, name, scale, planar_tol, smooth_angle, scene)
    finally:
        obj_eval.to_mesh_clear()


def _read_mesh(mesh, obj_eval, name, scale, planar_tol, smooth_angle, scene):
    geom = MeshGeom(name)
    nv = len(mesh.vertices)
    npoly = len(mesh.polygons)
    nloop = len(mesh.loops)
    nedge = len(mesh.edges)

    co = _foreach(mesh.vertices, "co", nv, np.float64, 3)
    geom.verts = list(map(tuple, (co * scale).tolist()))

    slot_mats = [scene.material_index(s.material) for s in obj_eval.material_slots]
    slot_mats.append(-1)  # out of range material index -> no material

    loop_vert = _foreach(mesh.loops, "vertex_index", nloop, np.int64)
    loop_edge = _foreach(mesh.loops, "edge_index", nloop, np.int64)
    starts = _foreach(mesh.polygons, "loop_start", npoly, np.int64)
    totals = _foreach(mesh.polygons, "loop_total", npoly, np.int64)
    mat_idx = _foreach(mesh.polygons, "material_index", npoly, np.int64)
    np.clip(mat_idx, 0, len(slot_mats) - 1, out=mat_idx)
    normals = _foreach(mesh.polygons, "normal", npoly, np.float64, 3)
    centers = _foreach(mesh.polygons, "center", npoly, np.float64, 3)
    poly_of_loop = np.repeat(np.arange(npoly), totals)

    # planarity of n-gons (distance of every corner from the polygon plane)
    nonplanar = np.zeros(npoly, dtype=bool)
    if npoly:
        dist = np.abs(np.einsum("ij,ij->i", co[loop_vert] - centers[poly_of_loop],
                                normals[poly_of_loop]))
        maxd = np.maximum.reduceat(dist, starts) if nloop else np.zeros(npoly)
        nonplanar = (totals > 3) & (maxd > planar_tol / scale)

    mesh.calc_loop_triangles()
    ntri = len(mesh.loop_triangles)
    tri_verts = _foreach(mesh.loop_triangles, "vertices", ntri, np.int64, 3).tolist()
    tri_poly = _foreach(mesh.loop_triangles, "polygon_index", ntri, np.int64).tolist()

    lv = loop_vert.tolist()
    st = starts.tolist()
    tt = totals.tolist()
    mats = [slot_mats[i] for i in mat_idx.tolist()]
    np_flags = nonplanar.tolist()

    faces = geom.faces
    face_mats = geom.face_mats
    face_groups = geom.face_groups
    face_of_poly = [0] * npoly
    for pi in range(npoly):
        face_of_poly[pi] = len(faces)
        if np_flags[pi]:
            face_of_poly[pi] = -1  # replaced by its triangles below
            continue
        faces.append(tuple(lv[st[pi]:st[pi] + tt[pi]]))
        face_mats.append(mats[pi])
        face_groups.append(pi)
    for tri, pi in zip(tri_verts, tri_poly):
        tri = tuple(tri)
        fi = face_of_poly[pi]
        if fi < 0:
            fi = len(faces)
            faces.append(tri)
            face_mats.append(mats[pi])
            face_groups.append(pi)
        geom.tris.append(tri)
        geom.tri_faces.append(fi)

    # edges: loose edges and soft (smooth) edges
    edge_verts = _foreach(mesh.edges, "vertices", nedge, np.int64, 2)
    counts = np.bincount(loop_edge, minlength=nedge) if nloop else np.zeros(nedge, np.int64)
    geom.loose_edges = list(map(tuple, edge_verts[counts == 0].tolist()))

    if smooth_angle > 0.0 and nloop:
        sharp = np.zeros(nedge, dtype=bool)
        attr = mesh.attributes.get("sharp_edge")
        if attr is not None and attr.domain == "EDGE" and attr.data_type == "BOOLEAN":
            attr.data.foreach_get("value", sharp)
        flat = np.zeros(npoly, dtype=bool)
        attr = mesh.attributes.get("sharp_face")
        if attr is not None and attr.domain == "FACE" and attr.data_type == "BOOLEAN":
            attr.data.foreach_get("value", flat)
        order = np.argsort(loop_edge, kind="stable")
        sorted_edges = loop_edge[order]
        first = np.searchsorted(sorted_edges, np.arange(nedge))
        two = np.nonzero((counts == 2) & ~sharp)[0]
        if len(two):
            fa = poly_of_loop[order[first[two]]]
            fb = poly_of_loop[order[first[two] + 1]]
            dots = np.einsum("ij,ij->i", normals[fa], normals[fb])
            ok = (dots >= math.cos(smooth_angle)) & ~flat[fa] & ~flat[fb]
            for a, b in edge_verts[two[ok]].tolist():
                geom.soft_edges.add((a, b) if a < b else (b, a))
    return geom


def _shares_geometry(obj):
    """True if the evaluated mesh depends only on the mesh datablock."""
    if obj.type != "MESH":
        return False
    if any(m.show_viewport or m.show_render for m in obj.modifiers):
        return False
    if obj.data.shape_keys is not None:
        return False
    return True


def collect(context, selection_only=False, visible_only=True, apply_modifiers=True,
            planar_tol=1e-5, smooth_angle=0.0, progress=None,
            role_fn=object_role, layer_fn=object_layer, collection=None):
    """Gather scene geometry. Returns SceneData (all values in metres).

    role_fn(obj) -> "MAIN" / "SAFETY" / "EXCLUDE", layer_fn(obj) -> layer name or None.
    collection: export only objects of this collection (and its children).
    """
    scene_data = SceneData()
    depsgraph = context.evaluated_depsgraph_get()
    unit_scale = context.scene.unit_settings.scale_length or 1.0
    convertible = {"MESH", "CURVE", "SURFACE", "META", "FONT", "CURVES"}

    selected = None
    if collection is not None:
        selected = {o.name_full for o in collection.all_objects}
    if selection_only:
        names = {o.name_full for o in context.selected_objects}
        selected = names if selected is None else selected & names
    cache = {}

    # object_instances items are only valid inside the loop: never store them
    total = max(1, len(depsgraph.objects))
    for n, inst in enumerate(depsgraph.object_instances):
        if progress:
            progress(min(1.0, n / total))
        obj = inst.object
        if obj.type not in convertible:
            continue
        original = obj.original
        parent = inst.parent.original if inst.is_instance and inst.parent else None
        if selected is not None:
            names = {original.name_full}
            if parent is not None:
                names.add(parent.name_full)
            if not names & selected:
                continue
        if visible_only:
            owner = parent if parent is not None else original
            try:
                if not owner.visible_get():
                    continue
            except RuntimeError:
                pass

        role = role_fn(original)
        if parent is not None and role == ROLE_MAIN:
            role = role_fn(parent)
        if role == ROLE_EXCLUDE:
            continue
        layer = layer_fn(original) or (layer_fn(parent) if parent else None)

        if inst.is_instance and obj.data is not None and not _shares_geometry(original):
            # geometry-nodes / particle instances: identify by evaluated data
            key = ("inst", obj.data.as_pointer())
            name = obj.data.name
        elif apply_modifiers and not _shares_geometry(original):
            key = ("obj", original.name_full)
            name = original.name
        else:
            mats = tuple(s.material.name_full if s.material else "" for s in obj.material_slots)
            data = original.data
            key = ("data", data.name_full if data else original.name_full, mats)
            name = data.name if data else original.name

        mesh_index = cache.get(key)
        if mesh_index is None:
            if apply_modifiers:
                source = obj
            else:
                source = original
            geom = _mesh_from_object(source, name, unit_scale, planar_tol, smooth_angle,
                                     scene_data)
            if geom is None or (not geom.faces and not geom.loose_edges):
                cache[key] = -1
                continue
            mesh_index = len(scene_data.meshes)
            scene_data.meshes.append(geom)
            cache[key] = mesh_index
        if mesh_index < 0:
            continue

        m = inst.matrix_world.copy()
        m.translation *= unit_scale
        matrix = tuple(tuple(m[r][c] for c in range(4)) for r in range(4))
        display = original.name if parent is None else "%s.%s" % (parent.name, original.name)
        scene_data.instances.append(Instance(mesh_index, matrix, display, role, layer))
    return scene_data


def transform(matrix, p):
    return (
        matrix[0][0] * p[0] + matrix[0][1] * p[1] + matrix[0][2] * p[2] + matrix[0][3],
        matrix[1][0] * p[0] + matrix[1][1] * p[1] + matrix[1][2] * p[2] + matrix[1][3],
        matrix[2][0] * p[0] + matrix[2][1] * p[1] + matrix[2][2] * p[2] + matrix[2][3],
    )
