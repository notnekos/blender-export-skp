"""Native writer for SketchUp 2021+ .skp files (no SketchUp SDK required).

File layout (reverse engineered from a SketchUp 2025 file):
    <UTF-16 "SketchUp Model" + "{version}" + "VFF" header>
    ZIP archive:
        meta/meta.dat                 - small TLV blob with model metadata
        materials/<name>/material.xml - one per material
        styles/<name>/style.xml
        model.dat                     - the model itself, a tree of TLV records

Every TLV record is: uint16 tag, uint32 payload length, payload.  Containers
simply hold a sequence of child records.  Entity ids are unsigned integers
stored little endian with the minimal number of bytes.

All geometry is stored in inches (SketchUp internal unit).
"""

import io
import os
import struct
import time
import uuid
import zipfile

_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

INCH = 0.0254  # metres per inch

# Flags of the drawing element record (tag 0x07d3)
FLAG_NORMAL = 0x06
FLAG_SOFT = 0x16  # soft + smooth edge

# First entity id free for our own objects.  Ids 1..17 are used by the
# template (style, fonts, layer, layer material, line styles).
FIRST_FREE_ID = 0x12


# ---------------------------------------------------------------------------
# low level TLV helpers
# ---------------------------------------------------------------------------

def _rec(tag, payload):
    return struct.pack("<HI", tag, len(payload)) + payload


def _node(tag, *children):
    return _rec(tag, b"".join(children))


def _uint(value):
    """Minimal little endian unsigned int (at least one byte)."""
    if value < 0x100:
        return bytes((value,))
    if value < 0x10000:
        return value.to_bytes(2, "little")
    if value < 0x1000000:
        return value.to_bytes(3, "little")
    return value.to_bytes((value.bit_length() + 7) // 8, "little")


_HI = struct.Struct("<HI")
_HIHI = struct.Struct("<HIHI")
_POINT = struct.Struct("<HI3d")
_PLANE = struct.Struct("<HI4d")
_FLAGS = struct.Struct("<HIB")
_REV0 = _FLAGS.pack(0x0FA2, 1, 0)
_REV1 = _FLAGS.pack(0x0FA2, 1, 1)


def _base_bytes(entity_id, flags, material_id=None):
    """Fast equivalent of _entity_base()."""
    idb = _uint(entity_id)
    k = len(idb)
    body = _HIHI.pack(0x05DC, 6 + k, 0x05DE, k) + idb
    if material_id is not None:
        mb = _uint(material_id)
        body += _HI.pack(0x07D1, len(mb)) + mb
    body += _FLAGS.pack(0x07D3, 1, flags)
    return _HI.pack(0x07D0, len(body)) + body


def _id_rec(tag, value):
    return _rec(tag, _uint(value))


def _u8(tag, value):
    return _rec(tag, bytes((value & 0xFF,)))


def _i32(tag, value):
    return _rec(tag, struct.pack("<i", value))


def _u32(tag, value):
    return _rec(tag, struct.pack("<I", value & 0xFFFFFFFF))


def _dbl(tag, *values):
    return _rec(tag, struct.pack("<%dd" % len(values), *values))


def _str(tag, text):
    return _rec(tag, text.encode("utf-8"))


def _parse(buf, start=0, end=None):
    """Parse a sequence of TLV records -> list of [tag, payload_bytes]."""
    if end is None:
        end = len(buf)
    out = []
    pos = start
    while pos < end:
        tag, length = struct.unpack_from("<HI", buf, pos)
        pos += 6
        out.append([tag, buf[pos:pos + length]])
        pos += length
    if pos != end:
        raise ValueError("Broken TLV stream")
    return out


def _serialize(records):
    return b"".join(_rec(t, p) for t, p in records)


def _entity_base(entity_id, flags=FLAG_NORMAL, material_id=None):
    """Drawing element record 0x07d0 (entity id, material, visibility)."""
    parts = [_node(0x05DC, _id_rec(0x05DE, entity_id))]
    if material_id is not None:
        parts.append(_id_rec(0x07D1, material_id))
    parts.append(_u8(0x07D3, flags))
    return _node(0x07D0, *parts)


def _id_list(ids):
    out = bytearray()
    for i in ids:
        b = _uint(i)
        out.append(len(b))
        out += b
    return bytes(out)


def _load(name):
    with open(os.path.join(_DATA_DIR, name), "rb") as fh:
        return fh.read()


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _newell(points):
    nx = ny = nz = 0.0
    count = len(points)
    for i in range(count):
        x1, y1, z1 = points[i]
        x2, y2, z2 = points[(i + 1) % count]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    return nx, ny, nz


class SkpDefinition:
    """Geometry of one component definition (coordinates in metres)."""

    def __init__(self, name):
        self.name = name
        self.verts = []        # [(x, y, z)]
        self.faces = []        # [(vertex index tuple, material index or -1, group id)]
        self.soft_edges = set()  # {(i, j)} with i < j, original vertex indices
        self.loose_edges = []  # [(i, j)]


class SkpInstance:
    def __init__(self, definition_index, matrix, name=""):
        self.definition = definition_index
        self.matrix = matrix  # 4x4 row-major nested tuples, metres
        self.name = name


class SkpMaterial:
    def __init__(self, name, color=(0.8, 0.8, 0.8, 1.0), metallic=0.0, roughness=0.5):
        self.name = name
        self.color = color
        self.metallic = metallic
        self.roughness = roughness


# ---------------------------------------------------------------------------
# writer
# ---------------------------------------------------------------------------

class SkpWriter:
    """Collects definitions / instances / materials and writes a .skp file."""

    LENGTH_UNITS = {"IN": 0, "FT": 1, "MM": 2, "CM": 3, "M": 4}

    def __init__(self, model_name="Model", length_unit="MM", weld_tolerance=0.001):
        self.model_name = model_name
        self.length_unit = length_unit
        self.weld_tol = weld_tolerance  # inches
        self.materials = []
        self.definitions = []
        self.instances = []
        self._next_id = FIRST_FREE_ID
        self.stats = {"faces": 0, "edges": 0, "vertices": 0, "skipped_faces": 0}

    # -- public API ----------------------------------------------------------

    def add_material(self, material):
        self.materials.append(material)
        return len(self.materials) - 1

    def add_definition(self, definition):
        self.definitions.append(definition)
        return len(self.definitions) - 1

    def add_instance(self, instance):
        self.instances.append(instance)

    def write(self, filepath):
        data = self.build()
        with open(filepath, "wb") as fh:
            fh.write(data)

    # -- internals -------------------------------------------------------------

    def _new_id(self):
        value = self._next_id
        self._next_id += 1
        return value

    @staticmethod
    def _folder_name(name):
        bad = '<>:"/\\|?*'
        clean = "".join("_" if (c in bad or ord(c) < 32) else c for c in name).strip()
        return clean or "Material"

    def _unique_material_names(self):
        used = set()
        names = []
        for mat in self.materials:
            base = self._folder_name(mat.name)
            name = base
            n = 1
            while name.lower() in used or name.lower() == "layer_layer0":
                n += 1
                name = "%s_%d" % (base, n)
            used.add(name.lower())
            names.append(name)
        return names

    def _material_xml(self, display_name, mat):
        r, g, b, a = mat.color
        to8 = lambda v: max(0, min(255, int(round(v * 255))))
        esc = (display_name.replace("&", "&amp;").replace('"', "&quot;")
               .replace("<", "&lt;").replace(">", "&gt;"))
        use_trans = 1 if a < 0.999 else 0
        return (
            '<?xml version="1.0" encoding="UTF-8" standalone="no" ?>\n'
            '<materialDocument xmlns="http://sketchup.google.com/schemas/sketchup/1.0/material" '
            'xmlns:mat="http://sketchup.google.com/schemas/sketchup/1.0/material">\n'
            '  <mat:material name="%s" type="0" workflow="1" colorRed="%d" colorGreen="%d" '
            'colorBlue="%d" colorizeType="0" trans="%s" useTrans="%d" pbrPromoState="0" '
            'hasTexture="0">\n'
            '    <mat:pbrMR xScale="1" yScale="1">\n'
            '      <mat:enable_metalness>1</mat:enable_metalness>\n'
            '      <mat:enable_roughness>1</mat:enable_roughness>\n'
            '      <mat:enable_normal>0</mat:enable_normal>\n'
            '      <mat:enable_occlusion>0</mat:enable_occlusion>\n'
            '      <mat:metallicFactor>%s</mat:metallicFactor>\n'
            '      <mat:roughnessFactor>%s</mat:roughnessFactor>\n'
            '      <mat:normalMapStyle>1</mat:normalMapStyle>\n'
            '      <mat:normalScale>1</mat:normalScale>\n'
            '      <mat:occlusionStrength>1</mat:occlusionStrength>\n'
            '      <mat:baseColorFactor>4294967295</mat:baseColorFactor>\n'
            '    </mat:pbrMR>\n'
            '  </mat:material>\n'
            '</materialDocument>\n'
        ) % (esc, to8(r), to8(g), to8(b), _fmt(a if use_trans else 0.5), use_trans,
             _fmt(mat.metallic), _fmt(mat.roughness))

    # .....................................................................

    def _build_definition(self, definition, material_ids, timestamp):
        """Return (record bytes, entities id, bbox) for one definition."""
        scale = 1.0 / INCH
        tol = self.weld_tol

        # weld vertices (SketchUp merges coincident vertices anyway)
        remap = []
        positions = []
        grid = {}
        for v in definition.verts:
            p = (v[0] * scale, v[1] * scale, v[2] * scale)
            key = (round(p[0] / tol), round(p[1] / tol), round(p[2] / tol))
            idx = grid.get(key)
            if idx is None:
                idx = len(positions)
                grid[key] = idx
                positions.append(p)
            remap.append(idx)

        entities_id = self._new_id()

        vertex_ids = {}
        vertex_records = []

        def vertex_id(i):
            vid = vertex_ids.get(i)
            if vid is None:
                vid = self._new_id()
                vertex_ids[i] = vid
                idb = _uint(vid)
                k = len(idb)
                vertex_records.append(
                    _HI.pack(0x09C4, 42 + k) + _HIHI.pack(0x05DC, 6 + k, 0x05DE, k) + idb
                    + _POINT.pack(0x09C5, 24, *positions[i]))
            return vid

        edges = {}       # (a, b) sorted -> [edge id, start, end, soft]
        edge_order = []
        edge_groups = {}  # edge key -> set of face groups
        face_records = []
        face_ids = []

        soft_pairs = set()
        for a, b in definition.soft_edges:
            ra, rb = remap[a], remap[b]
            if ra != rb:
                soft_pairs.add((min(ra, rb), max(ra, rb)))

        def edge_for(a, b):
            key = (a, b) if a < b else (b, a)
            e = edges.get(key)
            if e is None:
                e = [None, a, b, False]
                edges[key] = e
                edge_order.append(key)
            return key, e

        seen_faces = set()
        for indices, mat_index, group in definition.faces:
            loop = []
            for i in indices:
                r = remap[i]
                if not loop or loop[-1] != r:
                    loop.append(r)
            while len(loop) > 1 and loop[0] == loop[-1]:
                loop.pop()
            if len(loop) < 3 or len(set(loop)) != len(loop):
                self.stats["skipped_faces"] += 1
                continue
            pts = [positions[i] for i in loop]
            nx, ny, nz = _newell(pts)
            length = (nx * nx + ny * ny + nz * nz) ** 0.5
            # area = length / 2 ; drop degenerate faces
            if length < 2e-6:
                self.stats["skipped_faces"] += 1
                continue
            sig = frozenset(loop)
            if sig in seen_faces:
                self.stats["skipped_faces"] += 1
                continue
            seen_faces.add(sig)
            n = (nx / length, ny / length, nz / length)
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            cz = sum(p[2] for p in pts) / len(pts)
            d = -(n[0] * cx + n[1] * cy + n[2] * cz)

            uses = []
            for k in range(len(loop)):
                a = loop[k]
                b = loop[(k + 1) % len(loop)]
                key, e = edge_for(a, b)
                edge_groups.setdefault(key, []).append(group)
                uses.append((key, 0 if e[1] == a else 1))
            face_records.append((uses, mat_index, n, d))

        for a, b in definition.loose_edges:
            ra, rb = remap[a], remap[b]
            if ra != rb:
                edge_for(ra, rb)

        # soft edges: diagonals inside one source polygon, or smooth edges
        for key in edge_order:
            groups = edge_groups.get(key, [])
            if len(groups) == 2 and (groups[0] == groups[1] or key in soft_pairs):
                edges[key][3] = True

        # assign ids: edge, then its vertices (same pattern as SketchUp)
        edge_records = []
        edge_ids = []
        edge_uses = {}  # key -> (bytes for forward use, bytes for reversed use)
        for key in edge_order:
            e = edges[key]
            eid = self._new_id()
            e[0] = eid
            v1 = _uint(vertex_id(e[1]))
            v2 = _uint(vertex_id(e[2]))
            edge_ids.append(eid)
            base = _base_bytes(eid, FLAG_SOFT if e[3] else FLAG_NORMAL)
            edge_records.append(
                _HI.pack(0x0BB8, len(base) + 12 + len(v1) + len(v2)) + base
                + _HI.pack(0x0BB9, len(v1)) + v1 + _HI.pack(0x0BBA, len(v2)) + v2)
            idb = _uint(eid)
            head = _HIHI.pack(0x0FA0, 13 + len(idb), 0x0FA1, len(idb)) + idb
            edge_uses[key] = (head + _REV0, head + _REV1)

        face_data, face_records = face_records, []
        n_mats = len(material_ids)
        for uses, mat_index, n, d in face_data:
            fid = self._new_id()
            face_ids.append(fid)
            mat_id = material_ids[mat_index] if 0 <= mat_index < n_mats else None
            loop = b"".join([edge_uses[key][rev] for key, rev in uses])
            loops = _HIHI.pack(0x0DAE, len(loop) + 12, 0x1194, len(loop) + 6)                 + _HI.pack(0x1195, len(loop)) + loop
            base = _base_bytes(fid, FLAG_NORMAL, mat_id)
            if mat_id is not None:
                mb = _uint(mat_id)
                back = _HI.pack(0x0DAF, len(mb)) + mb
            else:
                back = b""
            body = base + back + _PLANE.pack(0x0DAD, 32, n[0], n[1], n[2], d) + loops
            face_records.append(_HI.pack(0x0DAC, len(body)) + body)

        used = [positions[i] for i in vertex_ids]
        if used:
            bbox = (min(p[0] for p in used), min(p[1] for p in used), min(p[2] for p in used),
                    max(p[0] for p in used), max(p[1] for p in used), max(p[2] for p in used))
        else:
            bbox = (0.0,) * 6

        self.stats["faces"] += len(face_ids)
        self.stats["edges"] += len(edge_ids)
        self.stats["vertices"] += len(vertex_records)

        ent_parts = [_node(0x07D0, _node(0x05DC, _id_rec(0x05DE, entities_id)),
                           _u8(0x07D3, FLAG_NORMAL))]
        if vertex_records:
            ent_parts.append(_node(0x1389, *vertex_records))
        if edge_records:
            ent_parts.append(_node(0x138A, *edge_records))
        if face_records:
            ent_parts.append(_node(0x138B, *face_records))
        ent_parts.append(_rec(0x139E, b""))
        ent_parts.append(_rec(0x138E, _id_list(edge_ids + face_ids)))
        ent_parts.append(_node(0x139B, _node(0x639F, _rec(0x63A0, b""))))
        ent_parts.append(_u8(0x139F, 1))
        ent_parts.append(_dbl(0x13A0, *bbox))

        record = _node(
            0x157C,
            _node(0x1388, *ent_parts),
            _rec(0x157D, uuid.uuid4().bytes),
            _str(0x157E, definition.name),
            _rec(0x157F, b""),
            _rec(0x1580, b""),
            _u8(0x1582, 0),
            _u32(0x1583, 0),
            _u32(0x1581, timestamp),
            _rec(0x1B58, self._blob_1b58),
            _rec(0x1585, self._blob_1585),
        )
        return record, entities_id, bbox

    def _instance_record(self, inst, def_entities_id, def_name):
        # 0x1965 holds the instance name (SketchUp "Entity Info > Instance")
        m = inst.matrix
        s = 1.0 / INCH
        # 3x3 linear part row by row, translation (inches), homogeneous w.
        # Verified against a SketchUp 2025 file: x' = v0*x + v1*y + v2*z + v9
        values = (
            m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2],
            m[0][3] * s, m[1][3] * s, m[2][3] * s,
            1.0,
        )
        iid = self._new_id()
        return iid, _node(
            0x1964,
            _node(0x07D0, _node(0x05DC, _id_rec(0x05DE, iid)), _u8(0x07D3, FLAG_NORMAL)),
            _str(0x1965, inst.name or def_name),
            _rec(0x1966, struct.pack("<13d", *values)),
            _id_rec(0x1967, def_entities_id),
            _rec(0x1968, uuid.uuid4().bytes),
        )

    # .....................................................................

    def build(self):
        self._blob_1b58 = _load("skp_def_1b58.bin")
        self._blob_1585 = _load("skp_def_1585.bin")
        timestamp = int(time.time())
        self._next_id = FIRST_FREE_ID

        # materials
        folder_names = self._unique_material_names()
        material_ids = []
        material_recs = []
        for name in folder_names:
            mid = self._new_id()
            material_ids.append(mid)
            material_recs.append(_node(
                0x32C8, _node(0x05DC, _id_rec(0x05DE, mid)),
                _str(0x32CC, name), _u8(0x32CA, 0), _rec(0x32CB, b"")))

        # definitions -- make names unique (instances refer to them by name too)
        def_records = []
        def_ids = []
        def_names = []
        def_bboxes = []
        used_names = set()
        for definition in self.definitions:
            base = definition.name or "Component"
            name = base
            n = 1
            while name in used_names:
                n += 1
                name = "%s#%d" % (base, n)
            used_names.add(name)
            definition.name = name
            rec, ent_id, bbox = self._build_definition(definition, material_ids, timestamp)
            def_records.append(rec)
            def_ids.append(ent_id)
            def_names.append(name)
            def_bboxes.append(bbox)

        # root instances
        inst_records = []
        inst_ids = []
        world_pts = []
        for inst in self.instances:
            iid, rec = self._instance_record(inst, def_ids[inst.definition],
                                             def_names[inst.definition])
            inst_ids.append(iid)
            inst_records.append(rec)
            bb = def_bboxes[inst.definition]
            m = inst.matrix
            for cx in (bb[0], bb[3]):
                for cy in (bb[1], bb[4]):
                    for cz in (bb[2], bb[5]):
                        world_pts.append(tuple(
                            m[r][0] * cx + m[r][1] * cy + m[r][2] * cz + m[r][3] / INCH
                            for r in range(3)))

        root_parts = [_node(0x07D0, _node(0x05DC), _u8(0x07D3, FLAG_NORMAL))]
        if inst_records:
            root_parts.append(_node(0x138C, *inst_records))
        root_parts.append(_rec(0x139E, b""))
        root_parts.append(_rec(0x138E, _id_list(inst_ids)))
        root_parts.append(_node(0x139B, _node(0x639F, _rec(0x63A0, b""))))
        root_parts.append(_u8(0x139F, 0))
        root = _node(0x1388, *root_parts)

        next_id = self._next_id

        sections = {
            0x01F7: _node(0x30D4, _node(0x30D5, *material_recs)),
            0x01F9: _id_wrap(def_records),
            0x01F6: root,
            0x01F5: _node(0x03E8, _id_rec(0x03E9, next_id)),
        }

        template = _parse(_load("skp_template.bin"))
        records = []
        for tag, payload in template:
            if tag in sections:
                payload = sections[tag]
            elif tag == 0x0209:
                payload = _patch_model_name(payload, self.model_name)
            elif tag == 0x0200:
                payload = _patch_length_unit(payload, self.LENGTH_UNITS.get(self.length_unit, 2))
            elif tag == 0x01FA:
                payload = _patch_camera(payload, world_pts)
            records.append([tag, payload])
        model_dat = _rec(0x01F4, _serialize(records))

        # zip container
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
            zf.writestr(_zinfo("meta/meta.dat"), _build_meta(self.model_name, timestamp))
            for name, mat in zip(folder_names, self.materials):
                zf.writestr(_zinfo("materials/%s/material.xml" % name),
                            self._material_xml(mat.name, mat).encode("utf-8"))
            zf.writestr(_zinfo("materials/Layer_Layer0/material.xml"),
                        _load("materials__Layer_Layer0__material.xml"))
            zf.writestr(_zinfo("styles/Style/style.xml"), _load("styles__Style__style.xml"))
            zf.writestr(_zinfo("styles/Style_1/style.xml"), _load("styles__Style_1__style.xml"))
            zf.writestr(_zinfo("model.dat"), model_dat)
        return _load("skp_header.bin") + buf.getvalue()


# ---------------------------------------------------------------------------
# template patching
# ---------------------------------------------------------------------------

def _fmt(value):
    return ("%.6f" % value).rstrip("0").rstrip(".") or "0"


def _zinfo(name):
    info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
    info.compress_type = zipfile.ZIP_DEFLATED
    return info


def _id_wrap(def_records):
    zero = _rec(0x0063, b"\x00\x00\x00\x00")
    return zero + _node(0x1770, zero, _node(0x1771, zero, *def_records))


def _patch_dictionary_value(buf, key, new_value_record, key_tag):
    """Replace the value following <key_tag>=key inside a nested dictionary."""

    def walk(data):
        try:
            recs = _parse(data)
        except (ValueError, struct.error):
            return data, False
        changed = False
        for i, (tag, payload) in enumerate(recs):
            if tag == key_tag and payload == key and i + 1 < len(recs):
                recs[i + 1][1] = new_value_record
                changed = True
            elif len(payload) >= 6:
                sub, sub_changed = walk(payload)
                if sub_changed:
                    recs[i][1] = sub
                    changed = True
        return (_serialize(recs) if changed else data), changed

    out, _ = walk(buf)
    return out


def _patch_model_name(payload, name):
    return _patch_dictionary_value(payload, b"Name", _str(0x38AD, name), 0x36B6)


def _patch_length_unit(payload, unit):
    return _patch_dictionary_value(payload, b"LengthUnit", _i32(0x38A7, unit), 0x61AD)


def _patch_camera(payload, points):
    """Point the default camera at the model."""
    if not points:
        return payload
    mn = [min(p[i] for p in points) for i in range(3)]
    mx = [max(p[i] for p in points) for i in range(3)]
    center = [(mn[i] + mx[i]) * 0.5 for i in range(3)]
    diag = max(1.0, sum((mx[i] - mn[i]) ** 2 for i in range(3)) ** 0.5)
    eye = (center[0] - diag * 0.9, center[1] - diag * 1.2, center[2] + diag * 0.8)
    try:
        cam = _parse(payload)
        inner = _parse(cam[0][1])
    except (ValueError, struct.error, IndexError):
        return payload
    for rec in inner:
        if rec[0] == 0x34BD:
            rec[1] = struct.pack("<3d", *eye)
        elif rec[0] == 0x34BE:
            rec[1] = struct.pack("<3d", *center)
        elif rec[0] == 0x34BF:
            rec[1] = struct.pack("<3d", 0.0, 0.0, 1.0)
        elif rec[0] == 0x34C3:
            rec[1] = struct.pack("<d", diag)
    cam[0][1] = _serialize(inner)
    return _serialize(cam)


def _build_meta(name, timestamp):
    """meta/meta.dat: rebuild the sample with our name, a new GUID and time."""
    recs = _parse(_load("meta__meta.dat"))
    inner = _parse(recs[0][1])
    for rec in inner:
        if rec[0] == 0x006A:
            rec[1] = name.encode("utf-8")
        elif rec[0] == 0x0066:
            rec[1] = uuid.uuid4().bytes
        elif rec[0] == 0x0078:
            rec[1] = _patch_meta_time(rec[1], timestamp)
    recs[0][1] = _serialize(inner)
    return _serialize(recs)


def _patch_meta_time(payload, timestamp):
    try:
        recs = _parse(payload)
    except (ValueError, struct.error):
        return payload
    for rec in recs:
        if rec[0] == 0x00CC and len(rec[1]) == 4:
            rec[1] = struct.pack("<I", timestamp & 0xFFFFFFFF)
        elif len(rec[1]) >= 6 and rec[0] in (0x00C8, 0x00C9, 0x00CA):
            rec[1] = _patch_meta_time(rec[1], timestamp)
    return _serialize(recs)
