"""Export Blender scenes to SketchUp (.skp) and AutoCAD (.dwg) files.

* .skp - native SketchUp 2021+ file (components, materials, soft edges)
* .dwg - AutoCAD R2000 drawing: 2D plan (top view with hidden lines removed,
         safety zones, code label) or 3D faces
"""

bl_info = {
    "name": "SketchUp SKP / DWG Exporter",
    "author": "Blendwgskp",
    "version": (1, 0, 0),
    "blender": (4, 2, 0),
    "location": "File > Export > SketchUp (.skp) / AutoCAD (.dwg)",
    "description": "Export to SketchUp .skp and AutoCAD .dwg (2D plan or 3D)",
    "category": "Import-Export",
}

import math
import os
import time

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty, StringProperty
from bpy_extras.io_utils import ExportHelper

from . import collect as _collect
from .dwg_writer import DwgDocument
from .skp_writer import SkpDefinition, SkpInstance, SkpMaterial, SkpWriter

UNIT_ITEMS = [
    ("MM", "Миллиметры", ""),
    ("CM", "Сантиметры", ""),
    ("M", "Метры", ""),
    ("IN", "Дюймы", ""),
]
UNIT_FACTOR = {"MM": 1000.0, "CM": 100.0, "M": 1.0, "IN": 1.0 / 0.0254}

DEFAULT_LAYERS = {
    "layer_main": "Юмагс_МАФ", "layer_main_color": 18,
    "layer_safety": "Юмагс_ЗОНЫ БЕЗОПАСНОСТИ маф", "layer_safety_color": 18,
    "layer_code": "Юмагс_КОД МАФ", "layer_code_color": 6,
}

ROLE_ITEMS = [
    ("AUTO", "Авто", "Определить по имени объекта/коллекции (\"зона\", \"safety\")"),
    ("MAIN", "Модель", "Основная геометрия (слой МАФ)"),
    ("SAFETY", "Зона безопасности", "Контур зоны безопасности (только в DWG-плане)"),
    ("EXCLUDE", "Не экспортировать", "Объект пропускается"),
]


# ---------------------------------------------------------------------------
# preferences
# ---------------------------------------------------------------------------


class SKPDWG_Preferences(bpy.types.AddonPreferences):
    bl_idname = __package__

    layer_main: StringProperty(name="Слой модели", default=DEFAULT_LAYERS["layer_main"])
    layer_main_color: IntProperty(name="Цвет", default=DEFAULT_LAYERS["layer_main_color"],
                                  min=1, max=255)
    layer_safety: StringProperty(name="Слой зон безопасности",
                                 default=DEFAULT_LAYERS["layer_safety"])
    layer_safety_color: IntProperty(name="Цвет", default=DEFAULT_LAYERS["layer_safety_color"],
                                    min=1, max=255)
    layer_code: StringProperty(name="Слой кода", default=DEFAULT_LAYERS["layer_code"])
    layer_code_color: IntProperty(name="Цвет", default=DEFAULT_LAYERS["layer_code_color"],
                                  min=1, max=255)

    def draw(self, context):
        col = self.layout.column()
        col.label(text="Слои DWG (цвета — индексы AutoCAD ACI):")
        for name, color in (("layer_main", "layer_main_color"),
                            ("layer_safety", "layer_safety_color"),
                            ("layer_code", "layer_code_color")):
            row = col.row()
            row.prop(self, name)
            row.prop(self, color)


def _pref(context, key):
    addon = context.preferences.addons.get(__package__)
    prefs = addon.preferences if addon else None
    return getattr(prefs, key, DEFAULT_LAYERS[key])


# ---------------------------------------------------------------------------
# object role (per object settings)
# ---------------------------------------------------------------------------


def _object_role(obj):
    role = getattr(obj, "skpdwg_role", "AUTO")
    if role != "AUTO":
        return role
    return _collect.object_role(obj)


def _object_layer(obj):
    layer = getattr(obj, "skpdwg_layer", "")
    return layer.strip() or _collect.object_layer(obj)



class OBJECT_PT_skpdwg(bpy.types.Panel):
    bl_label = "Экспорт SKP / DWG"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "object"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        obj = context.object
        if obj is None:
            return
        col = self.layout.column()
        col.prop(obj, "skpdwg_role")
        col.prop(obj, "skpdwg_layer")


# ---------------------------------------------------------------------------
# shared export logic
# ---------------------------------------------------------------------------


class _Progress:
    def __init__(self, context, steps):
        self.wm = context.window_manager
        self.steps = steps
        self.step = 0
        self.wm.progress_begin(0, 1000)

    def stage(self):
        base = self.step
        self.step += 1

        def update(frac):
            self.wm.progress_update(int(1000 * (base + frac) / self.steps))
        return update

    def end(self):
        self.wm.progress_end()


def _gather(context, op, progress, smooth_angle=0.0):
    collection = None
    if getattr(op, "collection", ""):
        collection = bpy.data.collections.get(op.collection)
        if collection is None:
            raise RuntimeError("Коллекция не найдена: %s" % op.collection)
    return _collect.collect(
        context,
        collection=collection,
        selection_only=op.use_selection,
        visible_only=op.use_visible,
        apply_modifiers=op.apply_modifiers,
        smooth_angle=smooth_angle,
        progress=progress,
        role_fn=_object_role,
        layer_fn=_object_layer,
    )


def export_skp(context, filepath, op, progress=None, data=None):
    if data is None:
        data = _gather(context, op, progress, math.radians(op.smooth_angle))
    model_name = op.model_name.strip() or os.path.splitext(os.path.basename(filepath))[0]
    writer = SkpWriter(model_name, op.skp_units)

    for mat in data.materials:
        color, metallic, roughness = _collect.material_color(mat)
        writer.add_material(SkpMaterial(mat.name, color, metallic, roughness))

    def_index = {}
    for inst in data.instances:
        if inst.role == _collect.ROLE_SAFETY and op.skip_safety_in_skp:
            continue
        idx = def_index.get(inst.mesh)
        if idx is None:
            geom = data.meshes[inst.mesh]
            d = SkpDefinition(geom.name)
            d.verts = geom.verts
            d.faces = list(zip(geom.faces, geom.face_mats, geom.face_groups))
            d.soft_edges = geom.soft_edges
            d.loose_edges = geom.loose_edges
            idx = writer.add_definition(d)
            def_index[inst.mesh] = idx
        writer.add_instance(SkpInstance(idx, inst.matrix, inst.name))
    if not writer.instances:
        raise RuntimeError("Нет геометрии для экспорта")
    writer.write(filepath)
    return writer.stats


# --- AutoCAD colour index ---------------------------------------------------

def _aci_palette():
    pal = {1: (255, 0, 0), 2: (255, 255, 0), 3: (0, 255, 0), 4: (0, 255, 255),
           5: (0, 0, 255), 6: (255, 0, 255), 7: (255, 255, 255), 8: (128, 128, 128),
           9: (192, 192, 192)}
    levels = (255, 204, 153, 127, 76)
    import colorsys
    for i in range(10, 250):
        hue = ((i - 10) // 10) * 15.0 / 360.0
        k = (i - 10) % 10
        value = levels[k // 2] / 255.0
        sat = 1.0 if k % 2 == 0 else 0.5
        r, g, b = colorsys.hsv_to_rgb(hue, sat, value)
        pal[i] = (int(r * 255), int(g * 255), int(b * 255))
    for i, v in zip(range(250, 256), (51, 91, 132, 173, 214, 255)):
        pal[i] = (v, v, v)
    return pal


_ACI = None


def nearest_aci(rgb):
    global _ACI
    if _ACI is None:
        _ACI = _aci_palette()
    r, g, b = (int(c * 255) for c in rgb[:3])
    best = 7
    best_d = 1e9
    for idx, (pr, pg, pb) in _ACI.items():
        d = (pr - r) ** 2 + (pg - g) ** 2 + (pb - b) ** 2
        if d < best_d:
            best_d = d
            best = idx
    return best


def export_dwg(context, filepath, op, progress_collect=None, progress_plan=None, data=None):
    from . import plan as _plan

    if data is None:
        data = _gather(context, op, progress_collect)
    lname_main = _pref(context, "layer_main")
    lname_safety = _pref(context, "layer_safety")
    lname_code = _pref(context, "layer_code")

    factor = UNIT_FACTOR[op.dwg_units]
    doc = DwgDocument(op.dwg_units)
    layer_main = doc.add_layer(lname_main, _pref(context, "layer_main_color"))
    layer_safety = None
    custom_layers = {}

    def layer_for(role, override):
        nonlocal layer_safety
        if override:
            lay = custom_layers.get(override)
            if lay is None:
                lay = custom_layers[override] = doc.add_layer(override, 7)
            return lay
        if role == _collect.ROLE_SAFETY:
            if layer_safety is None:
                layer_safety = doc.add_layer(lname_safety,
                                             _pref(context, "layer_safety_color"))
            return layer_safety
        return layer_main

    base_name = os.path.splitext(os.path.basename(filepath))[0]
    block_name = op.block_name.strip() or base_name
    target = doc.add_block(block_name) if op.use_block else doc.modelspace

    count = 0
    if op.dwg_mode == "PLAN":
        segments, (mn, mx) = _plan.build_plan(
            data, factor,
            feature_angle_deg=op.feature_angle,
            hidden_lines=op.hidden_lines,
            precision=op.hlr_precision,
            merge_tol=op.merge_tolerance,
            progress=progress_plan,
        )
        rot = math.radians(op.plan_rotation)
        cr, sr = math.cos(rot), math.sin(rot)
        xs = []
        ys = []
        for (role, override), segs in segments.items():
            layer = layer_for(role, override)
            for a, b in segs:
                if rot:
                    a = (a[0] * cr - a[1] * sr, a[0] * sr + a[1] * cr)
                    b = (b[0] * cr - b[1] * sr, b[0] * sr + b[1] * cr)
                target.add_line((a[0], a[1], 0.0), (b[0], b[1], 0.0), layer)
                xs += (a[0], b[0])
                ys += (a[1], b[1])
                count += 1
        if xs:
            ext = [(min(xs), min(ys)), (max(xs), max(ys))]
        else:
            ext = [(mn[0] * factor, mn[1] * factor), (mx[0] * factor, mx[1] * factor)]
    else:
        count, ext = _add_3dfaces(data, target, layer_for, factor, progress_plan)

    code = op.code_text.strip() if op.code_mode == "CUSTOM" else (
        block_name if op.code_mode == "NAME" else "")
    if code:
        layer_code = doc.add_layer(lname_code, _pref(context, "layer_code_color"))
        width = max(ext[1][0] - ext[0][0], ext[1][1] - ext[0][1], 1.0)
        height = op.text_height * factor / 1000.0 if op.text_height > 0 else width / 40.0
        target.add_text((ext[0][0], ext[1][1] + height * 0.5, 0.0), height, code, layer_code)

    if op.use_block:
        doc.modelspace.add_insert(target, (0.0, 0.0, 0.0))
    doc.set_extents((ext[0][0], ext[0][1], 0.0), (ext[1][0], ext[1][1], 0.0))
    if count == 0:
        raise RuntimeError("Нет геометрии для экспорта")
    doc.save(filepath)
    return count


def _add_3dfaces(data, target, layer_for, factor, progress):
    count = 0
    mn = [1e30, 1e30]
    mx = [-1e30, -1e30]
    mat_aci = [nearest_aci(_collect.material_color(m)[0]) for m in data.materials]
    total = max(1, len(data.instances))
    for n, inst in enumerate(data.instances):
        if progress:
            progress(n / total)
        geom = data.meshes[inst.mesh]
        layer = layer_for(inst.role, inst.layer)
        world = [tuple(c * factor for c in _collect.transform(inst.matrix, v)) for v in geom.verts]
        for p in world:
            mn[0] = min(mn[0], p[0])
            mn[1] = min(mn[1], p[1])
            mx[0] = max(mx[0], p[0])
            mx[1] = max(mx[1], p[1])
        tris_of = {}
        for tri, fi in zip(geom.tris, geom.tri_faces):
            tris_of.setdefault(fi, []).append(tri)
        for fi, face in enumerate(geom.faces):
            mi = geom.face_mats[fi]
            color = mat_aci[mi] if mi >= 0 else 256
            if len(face) == 3 or (len(face) == 4 and len(tris_of.get(fi, ())) == 2
                                  and _convex_quad([world[i] for i in face])):
                target.add_3dface([world[i] for i in face], layer, 0, color)
                count += 1
                continue
            boundary = {(min(face[k], face[(k + 1) % len(face)]),
                         max(face[k], face[(k + 1) % len(face)])) for k in range(len(face))}
            for a, b, c in tris_of.get(fi, ()):
                inv = 4  # degenerate 3rd edge (corner3 == corner4)
                if (min(a, b), max(a, b)) not in boundary:
                    inv |= 1
                if (min(b, c), max(b, c)) not in boundary:
                    inv |= 2
                if (min(c, a), max(c, a)) not in boundary:
                    inv |= 8
                target.add_3dface([world[a], world[b], world[c]], layer, inv, color)
                count += 1
        for a, b in geom.loose_edges:
            target.add_line(world[a], world[b], layer)
            count += 1
    if mx[0] < mn[0]:
        mn = [0.0, 0.0]
        mx = [0.0, 0.0]
    return count, [tuple(mn), tuple(mx)]


def _convex_quad(p):
    for i in range(4):
        a, b, c = p[i], p[(i + 1) % 4], p[(i + 2) % 4]
        u = [b[k] - a[k] for k in range(3)]
        v = [c[k] - b[k] for k in range(3)]
        cr = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        if i == 0:
            ref = cr
            if math.sqrt(sum(x * x for x in ref)) < 1e-12:
                return False
            continue
        d = sum(cr[k] * ref[k] for k in range(3))
        if d <= 0:
            return False
    return True


# ---------------------------------------------------------------------------
# operators
# ---------------------------------------------------------------------------


class _CommonProps:
    # set by Blender's "Collection Exporters" (Collection properties > Exporters)
    collection: StringProperty(name="Коллекция", default="", options={"HIDDEN"})
    use_selection: BoolProperty(name="Только выделенное", default=False)
    use_visible: BoolProperty(name="Только видимое", default=True)
    apply_modifiers: BoolProperty(name="Применить модификаторы", default=True)


class _SkpProps:
    model_name: StringProperty(name="Имя модели", default="",
                               description="Пусто — имя файла")
    skp_units: EnumProperty(name="Единицы в SketchUp", items=UNIT_ITEMS,
                            default="MM", description="Единицы отображения размеров в SketchUp")
    smooth_angle: FloatProperty(name="Смягчать рёбра до, °", default=20.0, min=0.0, max=90.0,
                                description="Рёбра между гладкими гранями с меньшим углом "
                                            "становятся мягкими (как в SketchUp)")
    skip_safety_in_skp: BoolProperty(name="Без зон безопасности", default=True,
                                     description="Не выгружать объекты-зоны в .skp")


class _DwgProps:
    dwg_mode: EnumProperty(
        name="Содержимое DWG",
        items=[("PLAN", "2D план (вид сверху)", "Видимые контуры сверху, как в образце"),
               ("MODEL3D", "3D модель (3DFACE)", "Трёхмерные грани")],
        default="PLAN")
    dwg_units: EnumProperty(name="Единицы чертежа", items=UNIT_ITEMS, default="MM")
    use_block: BoolProperty(name="Поместить в блок", default=True,
                            description="Вся графика в блоке с именем модели (как в образце)")
    block_name: StringProperty(name="Имя блока", default="", description="Пусто — имя файла")
    plan_rotation: FloatProperty(name="Поворот плана, °", default=0.0, min=-360.0, max=360.0,
                                 description="Поворот 2D-плана вокруг начала координат")
    hidden_lines: BoolProperty(name="Удалять невидимые линии", default=True)
    feature_angle: FloatProperty(name="Угол излома, °", default=30.0, min=1.0, max=90.0,
                                 description="Ребро рисуется, если угол между гранями больше")
    hlr_precision: IntProperty(name="Точность", default=1000, min=100, max=20000,
                               description="Число шагов трассировки на габарит плана")
    merge_tolerance: FloatProperty(name="Допуск слияния", default=0.5, min=0.0, max=100.0,
                                   description="В единицах чертежа")
    code_mode: EnumProperty(
        name="Подпись кода",
        items=[("NAME", "Имя блока", ""), ("CUSTOM", "Свой текст", ""), ("NONE", "Нет", "")],
        default="NAME")
    code_text: StringProperty(name="Текст", default="")
    text_height: FloatProperty(name="Высота текста, мм", default=0.0, min=0.0,
                               description="0 — автоматически")


def _draw_common(layout, op):
    box = layout.box()
    box.label(text="Что экспортировать")
    box.prop(op, "use_selection")
    box.prop(op, "use_visible")
    box.prop(op, "apply_modifiers")


def _draw_skp(layout, op):
    box = layout.box()
    box.label(text="SketchUp (.skp)")
    box.prop(op, "model_name")
    box.prop(op, "skp_units")
    box.prop(op, "smooth_angle")
    box.prop(op, "skip_safety_in_skp")


def _draw_dwg(layout, op):
    box = layout.box()
    box.label(text="AutoCAD (.dwg)")
    box.prop(op, "dwg_mode")
    box.prop(op, "dwg_units")
    box.prop(op, "use_block")
    if op.use_block:
        box.prop(op, "block_name")
    if op.dwg_mode == "PLAN":
        box.prop(op, "plan_rotation")
        box.prop(op, "hidden_lines")
        box.prop(op, "feature_angle")
        if op.hidden_lines:
            box.prop(op, "hlr_precision")
        box.prop(op, "merge_tolerance")
    box.prop(op, "code_mode")
    if op.code_mode == "CUSTOM":
        box.prop(op, "code_text")
    if op.code_mode != "NONE":
        box.prop(op, "text_height")


class EXPORT_OT_skp(bpy.types.Operator, ExportHelper, _CommonProps, _SkpProps):
    """Экспорт в SketchUp (.skp)"""
    bl_idname = "export_scene.skp_native"
    bl_label = "Экспорт SKP"
    bl_options = {"PRESET"}

    filename_ext = ".skp"
    filter_glob: StringProperty(default="*.skp", options={"HIDDEN"})

    def draw(self, context):
        _draw_common(self.layout, self)
        _draw_skp(self.layout, self)

    def execute(self, context):
        t0 = time.time()
        prog = _Progress(context, 1)
        try:
            stats = export_skp(context, self.filepath, self, prog.stage())
        except RuntimeError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            prog.end()
        self.report({"INFO"}, "SKP: %d граней, %d рёбер за %.1f с → %s" % (
            stats["faces"], stats["edges"], time.time() - t0, self.filepath))
        return {"FINISHED"}


class EXPORT_OT_dwg(bpy.types.Operator, ExportHelper, _CommonProps, _DwgProps):
    """Экспорт в AutoCAD DWG (R2000)"""
    bl_idname = "export_scene.dwg_native"
    bl_label = "Экспорт DWG"
    bl_options = {"PRESET"}

    filename_ext = ".dwg"
    filter_glob: StringProperty(default="*.dwg", options={"HIDDEN"})

    def draw(self, context):
        _draw_common(self.layout, self)
        _draw_dwg(self.layout, self)

    def execute(self, context):
        t0 = time.time()
        prog = _Progress(context, 2)
        try:
            count = export_dwg(context, self.filepath, self, prog.stage(), prog.stage())
        except RuntimeError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            prog.end()
        self.report({"INFO"}, "DWG: %d объектов за %.1f с → %s" % (
            count, time.time() - t0, self.filepath))
        return {"FINISHED"}


class EXPORT_OT_skp_dwg(bpy.types.Operator, ExportHelper, _CommonProps, _SkpProps, _DwgProps):
    """Экспорт в SketchUp (.skp) и AutoCAD (.dwg) с одинаковым именем"""
    bl_idname = "export_scene.skp_dwg_pair"
    bl_label = "Экспорт SKP + DWG"
    bl_options = {"PRESET"}

    filename_ext = ".skp"
    filter_glob: StringProperty(default="*.skp;*.dwg", options={"HIDDEN"})

    def draw(self, context):
        _draw_common(self.layout, self)
        _draw_skp(self.layout, self)
        _draw_dwg(self.layout, self)

    def execute(self, context):
        t0 = time.time()
        base = os.path.splitext(self.filepath)[0]
        prog = _Progress(context, 2)
        try:
            data = _gather(context, self, prog.stage(), math.radians(self.smooth_angle))
            stats = export_skp(context, base + ".skp", self, data=data)
            count = export_dwg(context, base + ".dwg", self, progress_plan=prog.stage(),
                               data=data)
        except RuntimeError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        finally:
            prog.end()
        self.report({"INFO"}, "SKP: %d граней, DWG: %d объектов за %.1f с" % (
            stats["faces"], count, time.time() - t0))
        return {"FINISHED"}


class SKPDWG_FH_skp(bpy.types.FileHandler):
    bl_idname = "SKPDWG_FH_skp"
    bl_label = "SketchUp (.skp)"
    bl_export_operator = "export_scene.skp_native"
    bl_file_extensions = ".skp"


class SKPDWG_FH_dwg(bpy.types.FileHandler):
    bl_idname = "SKPDWG_FH_dwg"
    bl_label = "AutoCAD (.dwg)"
    bl_export_operator = "export_scene.dwg_native"
    bl_file_extensions = ".dwg"


def menu_export(self, context):
    self.layout.operator(EXPORT_OT_skp.bl_idname, text="SketchUp (.skp)")
    self.layout.operator(EXPORT_OT_dwg.bl_idname, text="AutoCAD (.dwg)")
    self.layout.operator(EXPORT_OT_skp_dwg.bl_idname, text="SketchUp + AutoCAD (.skp + .dwg)")


classes = (SKPDWG_Preferences, OBJECT_PT_skpdwg, EXPORT_OT_skp, EXPORT_OT_dwg, EXPORT_OT_skp_dwg,
           SKPDWG_FH_skp, SKPDWG_FH_dwg)


def register():
    bpy.types.Object.skpdwg_role = EnumProperty(name="Роль", items=ROLE_ITEMS, default="AUTO")
    bpy.types.Object.skpdwg_layer = StringProperty(
        name="Слой DWG", default="", description="Свой слой для объекта (пусто — по роли)")
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.TOPBAR_MT_file_export.append(menu_export)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(menu_export)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Object.skpdwg_role
    del bpy.types.Object.skpdwg_layer
