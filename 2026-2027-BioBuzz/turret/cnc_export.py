"""One-piece CNC versions of laminated plates, for a router CNC (e.g. MPCNC) cutting 1/4 in plywood.

The laminated plates are two 1/8 in layers so a laser can cut them. A plate whose two layers have the same outline and
only through-holes can instead be milled from one 1/4 in (6 mm) sheet. This takes layer 1's DXF and writes
<name>_cnc.dxf with:
  OUTSIDE  the outline (cut on the outside of the line, last, with tabs)
  INSIDE   holes and slots big enough to mill with a 1/8 in (3.175 mm) bit (cut on the inside of the line)
  DRILL    holes too small to mill with that bit: a circle at size + a point at the centre (spot them, drill by hand)
The 1/8 in dowel holes (only there to line up the glued layers) are left out.

    python cnc_export.py            (after turret.py; deploy_site.sh runs it)
"""
import ezdxf
from turret import OUT, DOWEL_D

BIT = 3.175          # 1/8 in end mill
MILL_MIN = 4.0       # holes below this go to DRILL (too close to the bit size to mill cleanly)
PLATES = [] if getattr(__import__("turret"), "GB_TURRET", False) else ["base_plate"]   # v8.1GB: no base plate


def export(name):
    src = ezdxf.readfile(OUT / "dxf" / f"{name}_L1.dxf")
    doc = ezdxf.new(units=4)                                   # millimetres
    for layer, color in (("OUTSIDE", 1), ("INSIDE", 5), ("DRILL", 3)):
        doc.layers.add(layer, color=color)
    msp = doc.modelspace()
    shapes = []
    for e in src.modelspace().query("LWPOLYLINE"):
        pts = [(p[0], p[1]) for p in e.get_points()]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        shapes.append((max(xs) - min(xs), max(ys) - min(ys), pts, (sum(xs) / len(xs), sum(ys) / len(ys))))
    shapes.sort(key=lambda s: -s[0] * s[1])
    n = {"OUTSIDE": 0, "INSIDE": 0, "DRILL": 0, "dowel (left out)": 0}
    for i, (w, h, pts, c) in enumerate(shapes):
        size = min(w, h)
        if i == 0:
            layer = "OUTSIDE"
        elif abs(w - DOWEL_D) < 0.05 and abs(h - DOWEL_D) < 0.05:
            n["dowel (left out)"] += 1
            continue
        elif size < MILL_MIN:
            layer = "DRILL"
            msp.add_point(c, dxfattribs={"layer": "DRILL"})
        else:
            layer = "INSIDE"
        msp.add_lwpolyline(pts, close=True, dxfattribs={"layer": layer})
        n[layer] += 1
    doc.saveas(OUT / "dxf" / f"{name}_cnc.dxf")
    print(f"{name}_cnc.dxf: {n}")


if __name__ == "__main__":
    for p in PLATES:
        export(p)
