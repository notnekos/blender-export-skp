"""Minimal native writer for AutoCAD DWG R2000 (AC1015) files.

Implements the subset of the format needed to store 2D/3D linework:
tables (layers, line types, text styles, dimension style, block records,
viewport), dictionaries/layouts, and the entities LINE, 3DFACE, TEXT,
BLOCK/ENDBLK and INSERT.

Format references: Open Design Alliance "Open Design Specification for .dwg
files" and the LibreDWG specification files.
"""

import math
import struct
import time
import uuid

# ---------------------------------------------------------------------------
# bit stream
# ---------------------------------------------------------------------------


class BitWriter:
    """MSB-first bit stream kept in a Python integer (fast for small objects)."""

    __slots__ = ("acc", "n")

    def __init__(self):
        self.acc = 0
        self.n = 0  # number of bits written

    def tell(self):
        return self.n

    def bits(self, value, count):
        self.acc = (self.acc << count) | (value & ((1 << count) - 1))
        self.n += count

    def B(self, v):
        self.acc = (self.acc << 1) | (1 if v else 0)
        self.n += 1

    def BB(self, v):
        self.acc = (self.acc << 2) | (v & 3)
        self.n += 2

    def RC(self, v):
        self.acc = (self.acc << 8) | (v & 0xFF)
        self.n += 8

    def raw(self, data):
        if data:
            self.acc = (self.acc << (8 * len(data))) | int.from_bytes(data, "big")
            self.n += 8 * len(data)

    def append(self, other):
        self.acc = (self.acc << other.n) | other.acc
        self.n += other.n

    def RS(self, v):
        self.raw(struct.pack("<H", v & 0xFFFF))

    def RL(self, v):
        self.raw(struct.pack("<I", v & 0xFFFFFFFF))

    def RD(self, v):
        self.raw(struct.pack("<d", float(v)))

    def BS(self, v):
        v &= 0xFFFF
        if v == 0:
            self.BB(2)
        elif v == 256:
            self.BB(3)
        elif v < 256:
            self.BB(1)
            self.RC(v)
        else:
            self.BB(0)
            self.RS(v)

    def BL(self, v):
        v &= 0xFFFFFFFF
        if v == 0:
            self.BB(2)
        elif v < 256:
            self.BB(1)
            self.RC(v)
        else:
            self.BB(0)
            self.RL(v)

    def BD(self, v):
        v = float(v)
        if v == 0.0:
            self.BB(2)
        elif v == 1.0:
            self.BB(1)
        else:
            self.BB(0)
            self.RD(v)

    def DD(self, v, default):
        v = float(v)
        if v == float(default):
            self.BB(0)
        else:
            self.BB(3)
            self.RD(v)

    def BE(self, x, y, z):
        if x == 0.0 and y == 0.0 and z == 1.0:
            self.B(1)
        else:
            self.B(0)
            self.BD(x)
            self.BD(y)
            self.BD(z)

    def BT(self, v):
        if v == 0.0:
            self.B(1)
        else:
            self.B(0)
            self.BD(v)

    def RD2(self, p):
        self.RD(p[0])
        self.RD(p[1])

    def BD2(self, p):
        self.BD(p[0])
        self.BD(p[1])

    def BD3(self, p):
        self.BD(p[0])
        self.BD(p[1])
        self.BD(p[2])

    def CMC(self, index):
        self.BS(index)

    def TV(self, text):
        data = encode_text(text)
        self.BS(len(data))
        self.raw(data)

    def H(self, code, value):
        if not value:
            self.RC((code & 0xF) << 4)
            return
        data = value.to_bytes((value.bit_length() + 7) // 8, "big")
        self.RC(((code & 0xF) << 4) | len(data))
        self.raw(data)

    def TIMEBLL(self, t):
        days, ms = julian(t)
        self.BL(days)
        self.BL(ms)

    def getvalue(self):
        pad = (-self.n) % 8
        return (self.acc << pad).to_bytes((self.n + pad) // 8, "big")


def encode_text(text):
    """R2000 strings are 8 bit; non ASCII characters use \\U+XXXX escapes."""
    out = []
    for ch in text:
        code = ord(ch)
        if code < 128:
            out.append(ch)
        elif code <= 0xFFFF:
            out.append("\\U+%04X" % code)
        else:
            out.append("?")
    return "".join(out).encode("ascii")


def julian(t):
    jd = t / 86400.0 + 2440587.5
    days = int(jd)
    ms = int(round((jd - days) * 86400000.0))
    return days, ms


_CRC_TABLE = []
for _i in range(256):
    _c = _i
    _r = 0
    # table of the CRC-16/ARC polynomial 0xA001 used by AutoCAD
    for _ in range(8):
        if (_c ^ _r) & 1:
            _r = (_r >> 1) ^ 0xA001
        else:
            _r >>= 1
        _c >>= 1
    _CRC_TABLE.append(_r)


def crc16(data, seed=0xC0C1):
    crc = seed
    for b in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def modular_char(value):
    """Signed modular char (MC) used by the object map."""
    neg = value < 0
    value = abs(value)
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value == 0 and not (byte & 0x40):
            if neg:
                byte |= 0x40
            out.append(byte)
            return bytes(out)
        out.append(byte | 0x80)


def modular_short(value):
    if value > 0x7FFF:
        return struct.pack("<HH", (value & 0x7FFF) | 0x8000, value >> 15)
    return struct.pack("<H", value)


SENTINEL_HEADER_END = bytes((0x95, 0xA0, 0x4E, 0x28, 0x99, 0x82, 0x1A, 0xE5,
                             0x5E, 0x41, 0xE0, 0x5F, 0x9D, 0x3A, 0x4D, 0x00))
SENTINEL_THUMB_BEGIN = bytes((0x1F, 0x25, 0x6D, 0x07, 0xD4, 0x36, 0x28, 0x28,
                              0x9D, 0x57, 0xCA, 0x3F, 0x9D, 0x44, 0x10, 0x2B))
SENTINEL_THUMB_END = bytes((0xE0, 0xDA, 0x92, 0xF8, 0x2B, 0xC9, 0xD7, 0xD7,
                            0x62, 0xA8, 0x35, 0xC0, 0x62, 0xBB, 0xEF, 0xD4))
SENTINEL_VARS_BEGIN = bytes((0xCF, 0x7B, 0x1F, 0x23, 0xFD, 0xDE, 0x38, 0xA9,
                             0x5F, 0x7C, 0x68, 0xB8, 0x4E, 0x6D, 0x33, 0x5F))
SENTINEL_VARS_END = bytes((0x30, 0x84, 0xE0, 0xDC, 0x02, 0x21, 0xC7, 0x56,
                           0xA0, 0x83, 0x97, 0x47, 0xB1, 0x92, 0xCC, 0xA0))
SENTINEL_CLASS_BEGIN = bytes((0x8D, 0xA1, 0xC4, 0xB8, 0xC4, 0xA9, 0xF8, 0xC5,
                              0xC0, 0xDC, 0xF4, 0x5F, 0xE7, 0xCF, 0xB6, 0x8A))
SENTINEL_CLASS_END = bytes((0x72, 0x5E, 0x3B, 0x47, 0x3B, 0x56, 0x07, 0x3A,
                            0x3F, 0x23, 0x0B, 0xA0, 0x18, 0x30, 0x49, 0x75))

# fixed object types
T_TEXT, T_BLOCK, T_ENDBLK, T_INSERT, T_LINE, T_3DFACE = 1, 4, 5, 7, 19, 28
T_DICTIONARY = 42
T_BLOCK_CONTROL, T_BLOCK_HEADER = 48, 49
T_LAYER_CONTROL, T_LAYER = 50, 51
T_STYLE_CONTROL, T_STYLE = 52, 53
T_LTYPE_CONTROL, T_LTYPE = 56, 57
T_VIEW_CONTROL, T_UCS_CONTROL = 60, 62
T_VPORT_CONTROL, T_VPORT = 64, 65
T_APPID_CONTROL, T_APPID = 66, 67
T_DIMSTYLE_CONTROL, T_DIMSTYLE = 68, 69
T_VX_CONTROL = 70
T_MLINESTYLE = 73

# classes (variable types)
CLASSES = [
    # number, proxy flags, app name, c++ name, dxf name, is entity
    (500, 0, "ObjectDBX Classes", "AcDbDictionaryWithDefault", "ACDBDICTIONARYWDFLT", False),
    (501, 0, "ObjectDBX Classes", "AcDbPlaceHolder", "ACDBPLACEHOLDER", False),
    (502, 0, "ObjectDBX Classes", "AcDbLayout", "LAYOUT", False),
]
T_DICTIONARYWDFLT, T_PLACEHOLDER, T_LAYOUT = 500, 501, 502

LW_BYLAYER, LW_BYBLOCK, LW_DEFAULT = 29, 30, 31

# INSUNITS codes
UNITS = {"IN": 1, "FT": 2, "MM": 4, "CM": 5, "M": 6}


# ---------------------------------------------------------------------------
# object model
# ---------------------------------------------------------------------------


class _Obj:
    """A serialisable DWG object (table, dictionary, entity, ...)."""

    def __init__(self, doc, type_):
        self.doc = doc
        self.type = type_
        self.handle = doc._new_handle()
        self.owner = 0
        self.reactors = []
        doc._objects.append(self)

    is_entity = False

    def data(self, w):  # object specific data
        pass

    def handles(self, w):  # object specific handles
        pass

    def serialize(self):
        head = BitWriter()
        head.BS(self.type)
        w = BitWriter()  # everything after the bitsize field
        w.H(0, self.handle)
        w.BS(0)  # no extended data
        if self.is_entity:
            self._entity_common(w)
        else:
            w.BL(len(self.reactors))
        self.data(w)
        bitsize = head.tell() + 32 + w.tell()  # bits up to the handle stream
        if self.is_entity:
            self._entity_handles(w)
        else:
            w.H(4, self.owner)
            for r in self.reactors:
                w.H(4, r)
            w.H(3, 0)  # no extension dictionary
        self.handles(w)
        head.RL(bitsize)
        head.append(w)
        body = head.getvalue()
        out = modular_short(len(body)) + body
        return out + struct.pack("<H", crc16(out))


class _Table(_Obj):
    def __init__(self, doc, type_, name, control):
        _Obj.__init__(self, doc, type_)
        self.name = name
        self.owner = control.handle
        control.entries.append(self)

    def table_common(self, w):
        w.TV(self.name)
        w.B(0)   # 64 flag
        w.BS(0)  # xref index + 1
        w.B(0)   # xref dependent

    def xref_handle(self, w):
        w.H(5, 0)


class _Control(_Obj):
    def __init__(self, doc, type_):
        _Obj.__init__(self, doc, type_)
        self.entries = []

    def data(self, w):
        w.BS(len(self.entries))

    def handles(self, w):
        for e in self.entries:
            w.H(2, e.handle)


class _BlockControl(_Control):
    def __init__(self, doc):
        _Control.__init__(self, doc, T_BLOCK_CONTROL)
        self.model_space = None
        self.paper_space = None

    def handles(self, w):
        for e in self.entries:
            if e is not self.model_space and e is not self.paper_space:
                w.H(2, e.handle)
        w.H(3, self.model_space.handle)
        w.H(3, self.paper_space.handle)

    def data(self, w):
        w.BL(len([e for e in self.entries
                  if e is not self.model_space and e is not self.paper_space]))


class _LtypeControl(_Control):
    def __init__(self, doc):
        _Control.__init__(self, doc, T_LTYPE_CONTROL)
        self.byblock = None
        self.bylayer = None

    def _named(self):
        return [e for e in self.entries if e is not self.byblock and e is not self.bylayer]

    def data(self, w):
        w.BS(len(self._named()))

    def handles(self, w):
        for e in self._named():
            w.H(2, e.handle)
        w.H(3, self.byblock.handle)
        w.H(3, self.bylayer.handle)


class _DimstyleControl(_Control):
    def data(self, w):
        w.BS(len(self.entries))
        w.RC(0)  # no additional handles


class Layer(_Table):
    def __init__(self, doc, name, color=7, linetype=None, plot=True):
        _Table.__init__(self, doc, T_LAYER, name, doc._layer_control)
        self.color = color
        self.linetype = linetype
        self.plot = plot

    def data(self, w):
        self.table_common(w)
        flags = (16 if self.plot else 0) | (LW_DEFAULT << 5)
        w.BS(flags)
        w.CMC(self.color)

    def handles(self, w):
        self.xref_handle(w)
        w.H(5, self.doc._placeholder_normal.handle)
        w.H(5, (self.linetype or self.doc._lt_continuous).handle)


class _Ltype(_Table):
    def __init__(self, doc, name, description=""):
        _Table.__init__(self, doc, T_LTYPE, name, doc._ltype_control)
        self.description = description

    def data(self, w):
        self.table_common(w)
        w.TV(self.description)
        w.BD(0.0)   # pattern length
        w.RC(65)    # alignment 'A'
        w.RC(0)     # number of dashes
        w.raw(bytes(256))  # strings area

    def handles(self, w):
        self.xref_handle(w)


class _Style(_Table):
    def __init__(self, doc, name, font="arial.ttf"):
        _Table.__init__(self, doc, T_STYLE, name, doc._style_control)
        self.font = font

    def data(self, w):
        self.table_common(w)
        w.B(0)      # shape file
        w.B(0)      # vertical
        w.BD(0.0)   # fixed height
        w.BD(1.0)   # width factor
        w.BD(0.0)   # oblique
        w.RC(0)     # generation
        w.BD(2.5)   # last height
        w.TV(self.font)
        w.TV("")

    def handles(self, w):
        self.xref_handle(w)


class _Appid(_Table):
    def __init__(self, doc, name):
        _Table.__init__(self, doc, T_APPID, name, doc._appid_control)

    def data(self, w):
        self.table_common(w)
        w.RC(0)

    def handles(self, w):
        self.xref_handle(w)


class _Vport(_Table):
    def __init__(self, doc, name):
        _Table.__init__(self, doc, T_VPORT, name, doc._vport_control)
        self.center = (0.0, 0.0)
        self.height = 1000.0
        self.aspect = 1.6

    def data(self, w):
        self.table_common(w)
        w.BD(self.height)                 # view height
        w.BD(self.height * self.aspect)   # view width
        w.RD2(self.center)                # view center
        w.BD3((0.0, 0.0, 0.0))            # target
        w.BD3((0.0, 0.0, 1.0))            # view direction
        w.BD(0.0)                         # twist
        w.BD(50.0)                        # lens length
        w.BD(0.0)                         # front clip
        w.BD(0.0)                         # back clip
        w.bits(0, 4)                      # view mode
        w.RC(0)                           # render mode
        w.RD2((0.0, 0.0))                 # lower left
        w.RD2((1.0, 1.0))                 # upper right
        w.B(0)                            # UCSFOLLOW
        w.BS(1000)                        # circle zoom
        w.B(1)                            # fast zoom
        w.BB(3)                           # UCSICON
        w.B(0)                            # grid on
        w.RD2((10.0, 10.0))               # grid spacing
        w.B(0)                            # snap on
        w.B(0)                            # snap style
        w.BS(0)                           # snap isopair
        w.BD(0.0)                         # snap rotation
        w.RD2((0.0, 0.0))                 # snap base
        w.RD2((10.0, 10.0))               # snap spacing
        w.B(0)                            # UCS at origin
        w.B(1)                            # UCS per viewport
        w.BD3((0.0, 0.0, 0.0))            # UCS origin
        w.BD3((1.0, 0.0, 0.0))            # UCS x axis
        w.BD3((0.0, 1.0, 0.0))            # UCS y axis
        w.BD(0.0)                         # UCS elevation
        w.BS(0)                           # UCS ortho view

    def handles(self, w):
        self.xref_handle(w)
        w.H(5, 0)  # named UCS
        w.H(5, 0)  # base UCS


DIM_DEFAULTS = dict(
    DIMSCALE=1.0, DIMASZ=2.5, DIMEXO=0.625, DIMDLI=3.75, DIMEXE=1.25, DIMRND=0.0,
    DIMDLE=0.0, DIMTP=0.0, DIMTM=0.0, DIMTOL=0, DIMLIM=0, DIMTIH=0, DIMTOH=0,
    DIMSE1=0, DIMSE2=0, DIMTAD=1, DIMZIN=8, DIMAZIN=0, DIMTXT=2.5, DIMCEN=2.5,
    DIMTSZ=0.0, DIMALTF=0.03937007874, DIMLFAC=1.0, DIMTVP=0.0, DIMTFAC=1.0,
    DIMGAP=0.625, DIMALTRND=0.0, DIMALT=0, DIMALTD=4, DIMTOFL=1, DIMSAH=0,
    DIMTIX=0, DIMSOXD=0, DIMCLRD=0, DIMCLRE=0, DIMCLRT=0, DIMADEC=0, DIMDEC=2,
    DIMTDEC=2, DIMALTU=2, DIMALTTD=2, DIMAUNIT=0, DIMFRAC=0, DIMLUNIT=2,
    DIMDSEP=44, DIMTMOVE=0, DIMJUST=0, DIMSD1=0, DIMSD2=0, DIMTOLJ=0, DIMTZIN=8,
    DIMALTZ=0, DIMALTTZ=0, DIMUPT=0, DIMATFIT=3, DIMLWD=-2, DIMLWE=-2,
)


def write_dim_vars(w, d, header=False):
    """Dimension variables, shared by the header and DIMSTYLE (R2000 order)."""
    if not header:
        w.TV("")  # DIMPOST
        w.TV("")  # DIMAPOST
    for k in ("DIMSCALE", "DIMASZ", "DIMEXO", "DIMDLI", "DIMEXE", "DIMRND",
              "DIMDLE", "DIMTP", "DIMTM"):
        w.BD(d[k])
    for k in ("DIMTOL", "DIMLIM", "DIMTIH", "DIMTOH", "DIMSE1", "DIMSE2"):
        w.B(d[k])
    for k in ("DIMTAD", "DIMZIN", "DIMAZIN"):
        w.BS(d[k])
    for k in ("DIMTXT", "DIMCEN", "DIMTSZ", "DIMALTF", "DIMLFAC", "DIMTVP",
              "DIMTFAC", "DIMGAP", "DIMALTRND"):
        w.BD(d[k])
    w.B(d["DIMALT"])
    w.BS(d["DIMALTD"])
    for k in ("DIMTOFL", "DIMSAH", "DIMTIX", "DIMSOXD"):
        w.B(d[k])
    for k in ("DIMCLRD", "DIMCLRE", "DIMCLRT"):
        w.CMC(d[k])
    for k in ("DIMADEC", "DIMDEC", "DIMTDEC", "DIMALTU", "DIMALTTD", "DIMAUNIT",
              "DIMFRAC", "DIMLUNIT", "DIMDSEP", "DIMTMOVE", "DIMJUST"):
        w.BS(d[k])
    w.B(d["DIMSD1"])
    w.B(d["DIMSD2"])
    for k in ("DIMTOLJ", "DIMTZIN", "DIMALTZ", "DIMALTTZ"):
        w.BS(d[k])
    w.B(d["DIMUPT"])
    w.BS(d["DIMATFIT"])


class _Dimstyle(_Table):
    def __init__(self, doc, name):
        _Table.__init__(self, doc, T_DIMSTYLE, name, doc._dimstyle_control)

    def data(self, w):
        self.table_common(w)
        write_dim_vars(w, DIM_DEFAULTS)
        w.BS(DIM_DEFAULTS["DIMLWD"])
        w.BS(DIM_DEFAULTS["DIMLWE"])
        w.B(0)  # flag bit 0 of group 70

    def handles(self, w):
        self.xref_handle(w)
        w.H(5, self.doc._style_standard.handle)  # DIMTXSTY
        for _ in range(4):  # DIMLDRBLK, DIMBLK, DIMBLK1, DIMBLK2
            w.H(5, 0)


class _Dictionary(_Obj):
    def __init__(self, doc, owner=0, cloning=1):
        _Obj.__init__(self, doc, T_DICTIONARY)
        self.items = []  # (name, object)
        self.owner = owner.handle if isinstance(owner, _Obj) else owner
        if self.owner:
            self.reactors = [self.owner]
        self.cloning = cloning

    def add(self, name, obj):
        self.items.append((name, obj))
        obj.owner = self.handle
        obj.reactors = [self.handle]
        return obj

    def data(self, w):
        w.BL(len(self.items))
        w.BS(self.cloning)
        w.RC(0)  # hard owner flag
        for name, _ in self.items:
            w.TV(name)

    def handles(self, w):
        for _, obj in self.items:
            w.H(2, obj.handle)


class _DictionaryWithDefault(_Dictionary):
    def __init__(self, doc, owner=0):
        _Dictionary.__init__(self, doc, owner)
        self.type = T_DICTIONARYWDFLT
        self.default = None

    def handles(self, w):
        _Dictionary.handles(self, w)
        w.H(5, self.default.handle if self.default else 0)


class _Placeholder(_Obj):
    def __init__(self, doc):
        _Obj.__init__(self, doc, T_PLACEHOLDER)


class _MlineStyle(_Obj):
    def __init__(self, doc):
        _Obj.__init__(self, doc, T_MLINESTYLE)

    def data(self, w):
        w.TV("Standard")
        w.TV("")
        w.BS(0)
        w.CMC(256)
        w.BD(math.pi / 2)
        w.BD(math.pi / 2)
        w.RC(2)
        for offset in (0.5, -0.5):
            w.BD(offset)
            w.CMC(256)
            w.BS(0x7FFF)  # BYLAYER line type


class _Layout(_Obj):
    def __init__(self, doc, name, tab_order, block_record, paper):
        _Obj.__init__(self, doc, T_LAYOUT)
        self.name = name
        self.tab_order = tab_order
        self.block_record = block_record
        self.paper = paper
        block_record.layout = self

    def data(self, w):
        doc = self.doc
        w.TV("")                 # printer / plotter config
        w.TV("ISO_A3_(420.00_x_297.00_MM)" if self.paper else "")
        w.BS(688 if self.paper else 1712)  # plot flags
        for _ in range(4):       # margins
            w.BD(0.0)
        w.BD(420.0 if self.paper else 0.0)
        w.BD(297.0 if self.paper else 0.0)
        w.TV("")                 # canonical media name
        w.BD2((0.0, 0.0))        # plot origin
        w.BS(1)                  # paper units: millimetres
        w.BS(0)                  # rotation
        w.BS(5 if self.paper else 0)  # plot type: layout / display
        w.BD2((0.0, 0.0))
        w.BD2((0.0, 0.0))
        w.TV("")                 # plot view name
        w.BD(1.0)                # paper units
        w.BD(1.0)                # drawing units
        w.TV("")                 # style sheet
        w.BS(0)                  # standard scale type
        w.BD(1.0)                # standard scale factor
        w.BD2((0.0, 0.0))        # paper image origin
        w.TV(self.name)
        w.BS(self.tab_order)
        w.BS(1)                  # layout flags (PSLTSCALE)
        w.BD3((0.0, 0.0, 0.0))   # INSBASE
        w.RD2((0.0, 0.0))        # LIMMIN
        w.RD2((420.0, 297.0))    # LIMMAX
        w.BD3((0.0, 0.0, 0.0))   # UCS origin
        w.BD3((1.0, 0.0, 0.0))
        w.BD3((0.0, 1.0, 0.0))
        w.BD(0.0)                # UCS elevation
        w.BS(0)                  # UCS ortho view
        ext_min, ext_max = doc._extents() if not self.paper else ((0.0,) * 3, (420.0, 297.0, 0.0))
        w.BD3(ext_min)
        w.BD3(ext_max)

    def handles(self, w):
        w.H(4, self.block_record.handle)
        w.H(4, 0)  # active viewport
        w.H(5, 0)  # base UCS
        w.H(5, 0)  # named UCS


# ---------------------------------------------------------------------------
# entities
# ---------------------------------------------------------------------------


class _Entity(_Obj):
    is_entity = True

    def __init__(self, doc, type_, block, layer, color=256):
        _Obj.__init__(self, doc, type_)
        self.block = block
        self.layer = layer
        self.color = color
        self.prev = 0
        self.next = 0
        self.entmode = block.entmode if block is not None else 2

    def _entity_common(self, w):
        w.B(0)               # no preview / proxy graphics
        w.BB(self.entmode)   # 0 owner given, 1 paper space, 2 model space
        w.BL(0)              # reactors
        w.B(0)               # nolinks = 0 -> explicit prev/next handles
        w.CMC(self.color)
        w.BD(1.0)            # linetype scale
        w.BB(0)              # linetype BYLAYER
        w.BB(0)              # plot style BYLAYER
        w.BS(0)              # visible
        w.RC(LW_BYLAYER)

    def _entity_handles(self, w):
        if self.entmode == 0:
            w.H(4, self.block.handle)
        w.H(3, 0)            # extension dictionary
        w.H(4, self.prev)
        w.H(4, self.next)
        w.H(5, self.layer.handle)


class _Line(_Entity):
    def __init__(self, doc, block, layer, p1, p2, color=256):
        _Entity.__init__(self, doc, T_LINE, block, layer, color)
        self.p1 = p1
        self.p2 = p2

    def data(self, w):
        p1, p2 = self.p1, self.p2
        z0 = p1[2] == 0.0 and p2[2] == 0.0
        w.B(z0)
        w.RD(p1[0])
        w.DD(p2[0], p1[0])
        w.RD(p1[1])
        w.DD(p2[1], p1[1])
        if not z0:
            w.RD(p1[2])
            w.DD(p2[2], p1[2])
        w.BT(0.0)
        w.BE(0.0, 0.0, 1.0)


class _Face3d(_Entity):
    def __init__(self, doc, block, layer, pts, invisible=0, color=256):
        _Entity.__init__(self, doc, T_3DFACE, block, layer, color)
        if len(pts) == 3:
            pts = (pts[0], pts[1], pts[2], pts[2])
        self.pts = [tuple(float(c) for c in p) for p in pts]
        self.invisible = invisible

    def data(self, w):
        c = self.pts
        w.B(self.invisible == 0)  # has no flags
        z0 = c[0][2] == 0.0
        w.B(z0)
        w.RD(c[0][0])
        w.RD(c[0][1])
        if not z0:
            w.RD(c[0][2])
        for i in (1, 2, 3):
            for k in range(3):
                w.DD(c[i][k], c[i - 1][k])
        if self.invisible:
            w.BS(self.invisible)


class _Text(_Entity):
    def __init__(self, doc, block, layer, pos, height, text, color=256):
        _Entity.__init__(self, doc, T_TEXT, block, layer, color)
        self.pos = pos
        self.height = height
        self.text = text

    def data(self, w):
        w.RC(0xFF & ~0x01 if self.pos[2] else 0xFF)
        if self.pos[2]:
            w.RD(self.pos[2])
        w.RD2(self.pos)
        w.BE(0.0, 0.0, 1.0)
        w.BT(0.0)
        w.RD(self.height)
        w.TV(self.text)

    def handles(self, w):
        w.H(5, self.doc._style_standard.handle)


class _Insert(_Entity):
    def __init__(self, doc, block, layer, block_record, pos, scale=1.0, rotation=0.0):
        _Entity.__init__(self, doc, T_INSERT, block, layer)
        self.block_record = block_record
        self.pos = pos
        self.scale = scale
        self.rotation = rotation
        block_record.inserts.append(self)

    def data(self, w):
        w.BD3(self.pos)
        if self.scale == 1.0:
            w.BB(3)
        else:
            w.BB(2)
            w.RD(self.scale)
        w.BD(self.rotation)
        w.BD3((0.0, 0.0, 1.0))
        w.B(0)  # no attributes

    def handles(self, w):
        w.H(5, self.block_record.handle)


class _BlockBegin(_Entity):
    def __init__(self, doc, record):
        _Entity.__init__(self, doc, T_BLOCK, record, doc._layer0)
        self.record = record

    def data(self, w):
        w.TV(self.record.name)


class _BlockEnd(_Entity):
    def __init__(self, doc, record):
        _Entity.__init__(self, doc, T_ENDBLK, record, doc._layer0)


class BlockRecord(_Table):
    """Block table record together with the entities it owns."""

    def __init__(self, doc, name, entmode=0):
        self.layout = None
        self.entities = []
        self.inserts = []
        self.base = (0.0, 0.0, 0.0)
        self.entmode = entmode
        _Table.__init__(self, doc, T_BLOCK_HEADER, name, doc._block_control)
        self.begin = _BlockBegin(doc, self)
        self.end = _BlockEnd(doc, self)

    # -- entity factories --
    def add_line(self, p1, p2, layer, color=256):
        e = _Line(self.doc, self, layer, p1, p2, color)
        self.entities.append(e)
        return e

    def add_3dface(self, pts, layer, invisible=0, color=256):
        e = _Face3d(self.doc, self, layer, pts, invisible, color)
        self.entities.append(e)
        return e

    def add_text(self, pos, height, text, layer, color=256):
        e = _Text(self.doc, self, layer, pos, height, text, color)
        self.entities.append(e)
        return e

    def add_insert(self, block_record, pos=(0.0, 0.0, 0.0), layer=None, scale=1.0, rotation=0.0):
        e = _Insert(self.doc, self, layer or self.doc._layer0, block_record, pos, scale, rotation)
        self.entities.append(e)
        return e

    def link(self):
        ents = self.entities
        for i, e in enumerate(ents):
            e.prev = ents[i - 1].handle if i > 0 else 0
            e.next = ents[i + 1].handle if i + 1 < len(ents) else 0

    def data(self, w):
        self.table_common(w)
        w.B(1 if self.name.startswith("*") and self.entmode == 0 else 0)  # anonymous
        w.B(0)  # has attributes
        w.B(0)  # is xref
        w.B(0)  # xref overlaid
        w.B(0)  # xref loaded
        w.BD3(self.base)
        w.TV("")  # xref path
        for _ in self.inserts:
            w.RC(1)
        w.RC(0)
        w.TV("")  # description
        w.BL(0)   # preview size

    def handles(self, w):
        self.xref_handle(w)
        w.H(3, self.begin.handle)
        w.H(4, self.entities[0].handle if self.entities else 0)
        w.H(4, self.entities[-1].handle if self.entities else 0)
        w.H(3, self.end.handle)
        for ins in self.inserts:
            w.H(4, ins.handle)
        w.H(5, self.layout.handle if self.layout else 0)


# ---------------------------------------------------------------------------
# document
# ---------------------------------------------------------------------------


class DwgDocument:
    def __init__(self, units="MM"):
        self.units = units
        self._handle_seed = 1
        self._objects = []

        # control objects
        self._block_control = _BlockControl(self)
        self._layer_control = _Control(self, T_LAYER_CONTROL)
        self._style_control = _Control(self, T_STYLE_CONTROL)
        self._ltype_control = _LtypeControl(self)
        self._view_control = _Control(self, T_VIEW_CONTROL)
        self._ucs_control = _Control(self, T_UCS_CONTROL)
        self._vport_control = _Control(self, T_VPORT_CONTROL)
        self._appid_control = _Control(self, T_APPID_CONTROL)
        self._dimstyle_control = _DimstyleControl(self, T_DIMSTYLE_CONTROL)
        self._vx_control = _Control(self, T_VX_CONTROL)

        # named object dictionary + plot style placeholder (layers refer to it)
        self._nod = _Dictionary(self, 0)
        self._plotstyles = _DictionaryWithDefault(self, self._nod)
        self._placeholder_normal = self._plotstyles.add("Normal", _Placeholder(self))
        self._plotstyles.default = self._placeholder_normal

        # line types
        self._lt_byblock = _Ltype(self, "ByBlock")
        self._lt_bylayer = _Ltype(self, "ByLayer")
        self._lt_continuous = _Ltype(self, "Continuous", "Solid line")
        self._ltype_control.byblock = self._lt_byblock
        self._ltype_control.bylayer = self._lt_bylayer

        self._layer0 = Layer(self, "0", 7)
        self._style_standard = _Style(self, "Standard")
        _Appid(self, "ACAD")
        self._dimstyle_standard = _Dimstyle(self, "Standard")
        self._vport = _Vport(self, "*Active")

        # blocks
        self.modelspace = BlockRecord(self, "*Model_Space", entmode=2)
        self.paperspace = BlockRecord(self, "*Paper_Space", entmode=1)
        self._block_control.model_space = self.modelspace
        self._block_control.paper_space = self.paperspace

        # remaining dictionaries
        self._groups = _Dictionary(self, self._nod)
        self._mlinestyles = _Dictionary(self, self._nod)
        self._mlinestyle = self._mlinestyles.add("Standard", _MlineStyle(self))
        self._layouts = _Dictionary(self, self._nod)
        self._layouts.add("Model", _Layout(self, "Model", 0, self.modelspace, False))
        self._layouts.add("Layout1", _Layout(self, "Layout1", 1, self.paperspace, True))
        self._plotsettings = _Dictionary(self, self._nod)
        for name, d in (("ACAD_GROUP", self._groups), ("ACAD_LAYOUT", self._layouts),
                        ("ACAD_MLINESTYLE", self._mlinestyles),
                        ("ACAD_PLOTSETTINGS", self._plotsettings),
                        ("ACAD_PLOTSTYLENAME", self._plotstyles)):
            self._nod.items.append((name, d))

        self.layers = {"0": self._layer0}
        self.blocks = {}

    # -- public API ----------------------------------------------------------

    def add_layer(self, name, color=7):
        layer = self.layers.get(name)
        if layer is None:
            layer = Layer(self, name, color)
            self.layers[name] = layer
        return layer

    def add_block(self, name):
        base = name
        n = 1
        while name in self.blocks or name.startswith("*"):
            n += 1
            name = "%s_%d" % (base.lstrip("*"), n)
        block = BlockRecord(self, name, entmode=0)
        self.blocks[name] = block
        return block

    def save(self, path):
        data = self.build()
        with open(path, "wb") as fh:
            fh.write(data)

    # -- internals -------------------------------------------------------------

    def _new_handle(self):
        h = self._handle_seed
        self._handle_seed += 1
        return h

    def set_extents(self, ext_min, ext_max):
        """Drawing extents (EXTMIN/EXTMAX); computed from the entities if not set."""
        self._ext_given = (tuple(ext_min), tuple(ext_max))

    def _extents(self):
        given = getattr(self, "_ext_given", None)
        if given:
            return given
        cache = getattr(self, "_ext_cache", None)
        if cache:
            return cache
        mn = [1e20] * 3
        mx = [-1e20] * 3
        found = False

        def grow(p):
            for i in range(3):
                if p[i] < mn[i]:
                    mn[i] = p[i]
                if p[i] > mx[i]:
                    mx[i] = p[i]

        def visit(block, offset, scale, depth):
            nonlocal found
            if depth > 8:
                return
            for e in block.entities:
                pts = ()
                if isinstance(e, _Line):
                    pts = (e.p1, e.p2)
                elif isinstance(e, _Face3d):
                    pts = e.pts
                elif isinstance(e, _Text):
                    pts = (e.pos,)
                elif isinstance(e, _Insert):
                    visit(e.block_record,
                          tuple(offset[i] + e.pos[i] * scale for i in range(3)),
                          scale * e.scale, depth + 1)
                for p in pts:
                    found = True
                    grow(tuple(offset[i] + p[i] * scale for i in range(3)))

        visit(self.modelspace, (0.0, 0.0, 0.0), 1.0, 0)
        if not found:
            mn, mx = [0.0] * 3, [0.0] * 3
        self._ext_cache = (tuple(mn), tuple(mx))
        return self._ext_cache

    def _header_vars(self):
        w = BitWriter()
        now = time.time()
        ext_min, ext_max = self._extents()
        insunits = UNITS.get(self.units, 4)

        # unit conversion ratios and names as written by AutoCAD R13-R2004
        w.BD(412148564080.0)
        w.BD(6.162483e-14)
        w.BD(1.62263e+13)
        w.BD(2.63294e+26)
        w.TV("meter")
        w.TV("inch")
        w.TV("inch")
        w.TV("sq inch")
        w.BL(24)
        w.BL(0)
        w.H(5, 0)          # current VX table record
        for v in (1, 1, 0, 0, 1, 1, 0, 1, 0):
            # DIMASO DIMSHO PLINEGEN ORTHOMODE REGENMODE FILLMODE QTEXTMODE PSLTSCALE LIMCHECK
            w.B(v)
        for v in (1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0):
            # USRTIMER SKPOLY ANGDIR SPLFRAME MIRRTEXT WORLDVIEW TILEMODE PLIMCHECK
            # VISRETAIN DISPSILH PELLIPSE
            w.B(v)
        w.BS(1)            # PROXYGRAPHICS
        w.BS(3020)         # TREEDEPTH
        w.BS(2)            # LUNITS decimal
        w.BS(4)            # LUPREC
        w.BS(0)            # AUNITS
        w.BS(0)            # AUPREC
        w.BS(1)            # ATTMODE
        w.BS(0)            # PDMODE
        for _ in range(5):
            w.BS(0)        # USERI1-5
        for v in (8, 6, 6, 6, 6, 6, 6, 3, 70, 0, 64, 4, 0, 50):
            # SPLINESEGS SURFU SURFV SURFTYPE SURFTAB1 SURFTAB2 SPLINETYPE SHADEDGE
            # SHADEDIF UNITMODE MAXACTVP ISOLINES CMLJUST TEXTQLTY
            w.BS(v)
        for v in (1.0, 2.5, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0):
            # LTSCALE TEXTSIZE TRACEWID SKETCHINC FILLETRAD THICKNESS ANGBASE PDSIZE PLINEWID
            w.BD(v)
        for _ in range(5):
            w.BD(0.0)      # USERR1-5
        for _ in range(4):
            w.BD(0.0)      # CHAMFERA-D
        w.BD(0.5)          # FACETRES
        w.BD(20.0)         # CMLSCALE
        w.BD(1.0)          # CELTSCALE
        w.TV("acad")       # MENU
        w.TIMEBLL(now)     # TDCREATE
        w.TIMEBLL(now)     # TDUPDATE
        w.BL(0)
        w.BL(0)            # TDINDWG
        w.BL(0)
        w.BL(0)            # TDUSRTIMER
        w.CMC(256)         # CECOLOR
        w.H(0, self._handle_seed)  # HANDSEED
        w.H(5, self._layer0.handle)
        w.H(5, self._style_standard.handle)
        w.H(5, self._lt_bylayer.handle)
        w.H(5, self._dimstyle_standard.handle)
        w.H(5, self._mlinestyle.handle)
        w.BD(0.0)          # PSVPSCALE
        # paper space
        w.BD3((0.0, 0.0, 0.0))
        w.BD3((1e20, 1e20, 1e20))
        w.BD3((-1e20, -1e20, -1e20))
        w.RD2((0.0, 0.0))
        w.RD2((420.0, 297.0))
        w.BD(0.0)
        w.BD3((0.0, 0.0, 0.0))
        w.BD3((1.0, 0.0, 0.0))
        w.BD3((0.0, 1.0, 0.0))
        w.H(5, 0)          # PUCSNAME
        w.H(5, 0)          # PUCSORTHOREF
        w.BS(0)            # PUCSORTHOVIEW
        w.H(5, 0)          # PUCSBASE
        for _ in range(6):
            w.BD3((0.0, 0.0, 0.0))
        # model space
        w.BD3((0.0, 0.0, 0.0))   # INSBASE
        w.BD3(ext_min)
        w.BD3(ext_max)
        w.RD2((0.0, 0.0))
        w.RD2((420.0, 297.0))
        w.BD(0.0)                 # ELEVATION
        w.BD3((0.0, 0.0, 0.0))
        w.BD3((1.0, 0.0, 0.0))
        w.BD3((0.0, 1.0, 0.0))
        w.H(5, 0)                 # UCSNAME
        w.H(5, 0)                 # UCSORTHOREF
        w.BS(0)                   # UCSORTHOVIEW
        w.H(5, 0)                 # UCSBASE
        for _ in range(6):
            w.BD3((0.0, 0.0, 0.0))
        w.TV("")                  # DIMPOST
        w.TV("")                  # DIMAPOST
        write_dim_vars(w, DIM_DEFAULTS, header=True)
        w.H(5, self._style_standard.handle)  # DIMTXSTY
        for _ in range(4):
            w.H(5, 0)             # DIMLDRBLK DIMBLK DIMBLK1 DIMBLK2
        w.BS(DIM_DEFAULTS["DIMLWD"])
        w.BS(DIM_DEFAULTS["DIMLWE"])
        for ctl in (self._block_control, self._layer_control, self._style_control,
                    self._ltype_control, self._view_control, self._ucs_control,
                    self._vport_control):
            w.H(3, ctl.handle)
        w.H(5, self._appid_control.handle)
        w.H(3, self._dimstyle_control.handle)
        w.H(3, self._vx_control.handle)
        w.H(5, self._groups.handle)
        w.H(5, self._mlinestyles.handle)
        w.H(5, self._nod.handle)
        w.BS(1)                   # TSTACKALIGN
        w.BS(70)                  # TSTACKSIZE
        w.TV("")                  # HYPERLINKBASE
        w.TV("")                  # STYLESHEET
        w.H(5, self._layouts.handle)
        w.H(5, self._plotsettings.handle)
        w.H(5, self._plotstyles.handle)
        w.BL(LW_BYLAYER | 0x200 | 0x800 | 0x2000)  # CELWEIGHT, LWDISPLAY off, EXTNAMES, PSTYLEMODE
        w.BS(insunits)
        w.BS(0)                   # CEPSNTYPE
        w.TV("{%s}" % str(uuid.uuid4()).upper())   # FINGERPRINTGUID
        w.TV("{%s}" % str(uuid.uuid4()).upper())   # VERSIONGUID
        w.H(5, self.paperspace.handle)
        w.H(5, self.modelspace.handle)
        w.H(5, self._lt_bylayer.handle)
        w.H(5, self._lt_byblock.handle)
        w.H(5, self._lt_continuous.handle)
        for _ in range(4):
            w.BS(0)
        return w.getvalue()

    def _fit_view(self):
        ext_min, ext_max = self._extents()
        cx = (ext_min[0] + ext_max[0]) * 0.5
        cy = (ext_min[1] + ext_max[1]) * 0.5
        h = max(ext_max[1] - ext_min[1], (ext_max[0] - ext_min[0]) / self._vport.aspect, 1.0)
        self._vport.center = (cx, cy)
        self._vport.height = h * 1.1

    def build(self):
        # block bookkeeping: link entity chains
        for block in [self.modelspace, self.paperspace] + list(self.blocks.values()):
            block.link()
        self._ext_cache = None
        self._fit_view()

        out = bytearray()
        # --- file header (section locators patched later) -----------------
        n_sections = 5
        out += b"AC1015" + bytes(5)
        out += bytes((0, 1))            # maintenance release, unknown (1)
        out += struct.pack("<I", 0)     # thumbnail address (patched)
        out += bytes((0x17, 0))         # writer version, maintenance version
        out += struct.pack("<H", 30)    # code page ANSI_1252
        out += struct.pack("<I", n_sections)
        locator_pos = len(out)
        out += bytes(9 * n_sections)
        crc_pos = len(out)
        out += bytes(2)
        out += SENTINEL_HEADER_END

        sections = {}

        # --- header variables ---------------------------------------------
        start = len(out)
        out += SENTINEL_VARS_BEGIN
        body = self._header_vars()
        chunk = struct.pack("<I", len(body)) + body
        out += chunk + struct.pack("<H", crc16(chunk))
        out += SENTINEL_VARS_END
        sections[0] = (start, len(out) - start)

        # --- classes ------------------------------------------------------
        start = len(out)
        out += SENTINEL_CLASS_BEGIN
        w = BitWriter()
        for number, proxy, app, cpp, dxf, is_entity in CLASSES:
            w.BS(number)
            w.BS(proxy)
            w.TV(app)
            w.TV(cpp)
            w.TV(dxf)
            w.B(0)
            w.BS(0x1F3 if is_entity else 0x1F2)
        body = w.getvalue()
        chunk = struct.pack("<I", len(body)) + body
        out += chunk + struct.pack("<H", crc16(chunk))
        out += SENTINEL_CLASS_END
        sections[1] = (start, len(out) - start)
        out += struct.pack("<I", 0x0DCA)

        # --- objects --------------------------------------------------------
        objects_start = len(out)
        addresses = []
        for obj in sorted(self._objects, key=lambda o: o.handle):
            addresses.append((obj.handle, len(out)))
            out += obj.serialize()

        # --- object map -----------------------------------------------------
        start = len(out)
        last_handle = 0
        last_addr = 0
        page = bytearray()

        def flush(page_bytes):
            size = len(page_bytes) + 2
            blob = struct.pack(">H", size) + bytes(page_bytes)
            return blob + struct.pack(">H", crc16(blob))

        for handle, addr in addresses:
            entry = modular_char(handle - last_handle) + modular_char(addr - last_addr)
            if len(page) + len(entry) > 2030:
                out += flush(page)
                page = bytearray()
                last_handle = 0
                last_addr = 0
                entry = modular_char(handle) + modular_char(addr)
            page += entry
            last_handle = handle
            last_addr = addr
        if page:
            out += flush(page)
        out += flush(b"")
        sections[2] = (start, len(out) - start)

        # --- object free space (section 3) -----------------------------------
        start = len(out)
        days, ms = julian(time.time())
        out += struct.pack("<IIIII", 0, len(self._objects), days, ms, objects_start)
        out += bytes((4,))
        out += struct.pack("<8I", 0x32, 0, 0x64, 0, 0x200, 0, 0xFFFFFFFF, 0)
        sections[3] = (start, len(out) - start)

        # --- template (section 4): description + MEASUREMENT ---------------
        start = len(out)
        out += struct.pack("<HH", 0, 0 if self.units in ("IN", "FT") else 1)
        sections[4] = (start, len(out) - start)

        # --- empty preview image ----------------------------------------------
        thumb = len(out)
        out += SENTINEL_THUMB_BEGIN + struct.pack("<I", 5) + bytes((0,)) + SENTINEL_THUMB_END

        # --- patch header ---------------------------------------------------
        struct.pack_into("<I", out, 0x0D, thumb)
        for i in range(n_sections):
            addr, size = sections[i]
            struct.pack_into("<BII", out, locator_pos + 9 * i, i, addr, size)
        struct.pack_into("<H", out, crc_pos, crc16(out[:crc_pos]))
        return bytes(out)
