"""Build the Korean figure fonts used by figs.py and overview.py (fonts/VizSansKR*.ttf).

usage: python3 tools/build_fonts.py <Noto CJK dir> <out dir>
       e.g. python3 tools/build_fonts.py /usr/share/fonts/opentype/noto fonts   (Ubuntu: apt install fonts-noto-cjk)

Each font is Noto Sans CJK KR (face 1 of the .ttc) subset to the characters the figures use (ASCII, a few symbols,
all Hangul syllables) and converted from CFF to TrueType outlines, so matplotlib embeds it as a real font in PDF
(pdf.fonttype 42) and Chromium does not fall back to Type 3 glyphs. Metrics are unchanged. The family is renamed to
VizSansKR because the OFL does not allow modified fonts to keep the original name. The .ttf files are not committed.
"""
import os
import sys

from fontTools import subset
from fontTools.pens.cu2quPen import Cu2QuPen
from fontTools.pens.ttGlyphPen import TTGlyphPen
from fontTools.ttLib import TTFont, newTable

# (source .ttc, output file, family name, weight class)
FACES = [
    ("NotoSansCJK-Regular.ttc", "VizSansKR.ttf", "VizSansKR", 400),
    ("NotoSansCJK-Medium.ttc", "VizSansKRMedium.ttf", "VizSansKR Medium", 500),
    ("NotoSansCJK-Bold.ttc", "VizSansKRBold.ttf", "VizSansKR Bold", 700),
]
KR_FACE = 1   # index of the Korean face inside the Noto CJK collections

RANGES = [(0x20, 0x7E), (0xA0, 0xA0), (0xB0, 0xB1), (0xB5, 0xB5), (0xB7, 0xB7), (0xD7, 0xD7), (0x394, 0x394),
          (0x3A3, 0x3A3), (0x3B1, 0x3B1), (0x3B8, 0x3B8), (0x3C0, 0x3C0), (0x2013, 0x2014), (0x2018, 0x2019),
          (0x201C, 0x201D), (0x2026, 0x2026), (0x2032, 0x2033), (0x2160, 0x2162), (0x2190, 0x2190), (0x2192, 0x2192),
          (0x2194, 0x2194), (0x2212, 0x2212), (0x221A, 0x221A), (0x221E, 0x221E), (0x2264, 0x2265), (0x2460, 0x2464),
          (0x25CB, 0x25CB), (0x25CE, 0x25CF), (0x2713, 0x2713), (0x300A, 0x300F), (0xAC00, 0xD7A3)]
UNICODES = [c for a, b in RANGES for c in range(a, b + 1)]


def to_truetype(f):
    """CFF outlines -> quadratic glyf (same steps as fontTools' otf2ttf snippet)"""
    order = f.getGlyphOrder()
    gs = f.getGlyphSet()
    glyphs = {}
    for g in order:
        pen = TTGlyphPen(gs)
        gs[g].draw(Cu2QuPen(pen, 1.0, reverse_direction=True))
        glyphs[g] = pen.glyph()
    f["loca"] = newTable("loca")
    f["glyf"] = glyf = newTable("glyf")
    glyf.glyphOrder = order
    glyf.glyphs = glyphs
    del f["CFF "]
    glyf.compile(f)
    hmtx = f["hmtx"]
    for g, gl in glyf.glyphs.items():
        if hasattr(gl, "xMin"):
            hmtx[g] = (hmtx[g][0], gl.xMin)
    f["maxp"] = maxp = newTable("maxp")
    maxp.tableVersion = 0x00010000
    maxp.maxZones, maxp.maxTwilightPoints, maxp.maxStorage = 1, 0, 0
    maxp.maxFunctionDefs = maxp.maxInstructionDefs = maxp.maxStackElements = maxp.maxSizeOfInstructions = 0
    maxp.maxComponentElements = max(len(getattr(g, "components", [])) for g in glyphs.values())
    maxp.compile(f)
    f["post"].formatType = 3.0
    f.sfntVersion = "\x00\x01\x00\x00"


def build(src, out, family, weight):
    f = TTFont(src, fontNumber=KR_FACE)
    opts = subset.Options()
    opts.layout_features = []
    opts.drop_tables += ["GSUB", "GPOS", "GDEF", "BASE", "VORG", "vhea", "vmtx"]
    opts.hinting = False
    opts.desubroutinize = True
    opts.notdef_outline = True
    opts.name_IDs = ["*"]
    opts.name_languages = ["*"]
    sub = subset.Subsetter(opts)
    sub.populate(unicodes=UNICODES)
    sub.subset(f)
    to_truetype(f)
    for rec in f["name"].names:
        if rec.nameID in (1, 4, 16):
            rec.string = family
        elif rec.nameID == 6:
            rec.string = family.replace(" ", "")
    f["OS/2"].usWeightClass = weight
    f.save(out)
    print("written", out, len(f.getBestCmap()), "chars")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    for src, out, family, weight in FACES:
        build(os.path.join(src_dir, src), os.path.join(out_dir, out), family, weight)
