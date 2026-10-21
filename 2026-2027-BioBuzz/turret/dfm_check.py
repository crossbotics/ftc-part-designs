"""Can a cutting service actually make these plates? Checks every CUT drawing in out/dxf against SendCutSend's published
6061 limits (checked 2026-09-26 on sendcutsend.com/materials/6061-aluminum/):

    1/4" (0.250"): min bridge 0.095" (2.41 mm), min hole 0.057" (1.45 mm), countersink hole centre >= 0.338" (8.59 mm)
                   from any edge, tapped hole centre >= tap hole/2 + 0.075" from an edge
    1/8" (0.125"): min bridge 0.044" (1.12 mm)  (the turret ring, cut as two stacked layers)

A "thin spot" is material narrower than the min bridge: it disappears when the part is eroded by half that width.

    python dfm_check.py
"""
import math
import sys
from pathlib import Path

import ezdxf
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

OUT = Path(__file__).parent / "out" / "dxf"
EIGHTH = {"turret_ring": 1.12, "motor_adapter": 1.12}   # parts cut from 1/8" stock
MIN_BRIDGE, MIN_HOLE, CSK_EDGE, TAP_EDGE = 2.41, 1.45, 8.59, 3.3 / 2 + 1.9
SKIP = {"front_plate"}                  # 3 mm wood / print, not aluminum
GEARS = {"turret_ring"}
REVIEW = []


def check(path):
    doc = ezdxf.readfile(path)
    rings = [Polygon([(p[0], p[1]) for p in e.get_points()]) for e in doc.modelspace().query("LWPOLYLINE[layer=='CUT']")]
    if not rings:
        if path.stem.endswith("_cnc"):   # cnc_export.py's OUTSIDE/INSIDE/DRILL format: same outline as the laser layers
            print(f"skip {path.stem:18s} CNC-service format (same geometry as its laser layers, checked by laser_check.py)")
            return 0
        raise SystemExit(f"{path.name}: no CUT outline; unknown DXF format")
    outer = max(rings, key=lambda r: r.area)
    holes = [r for r in rings if r is not outer]
    part = outer.difference(unary_union(holes))
    mb = EIGHTH.get(path.stem, MIN_BRIDGE)
    opened = part.buffer(-mb / 2 * 0.98, join_style=1).buffer(mb / 2 * 0.98, join_style=1)   # round: true minimum width (mitred joins rebuilt pointed gear tips and hid them)
    lost = part.difference(opened)
    pieces = list(lost.geoms if hasattr(lost, "geoms") else [lost])
    thin = [g for g in pieces if g.area > 0.5]
    if path.stem in GEARS:            # gear tooth tips taper below any min-bridge rule: report them for supplier review
        tips = [g for g in pieces if 0.02 < g.area <= 0.5]
        if tips:
            REVIEW.append(f"{path.stem}: {len(tips)} tooth tips narrower than {mb} mm (tip land ~0.6 mm). "
                          "Ask the cutter to approve this DXF (tips may come out rounded), or 3D-print / buy the part.")
    small = [h for h in holes if 2 * math.sqrt(h.area / math.pi) < MIN_HOLE]
    def edge(p):
        pt = Point(p)
        return min([outer.exterior.distance(pt)] + [h.exterior.distance(pt) for h in holes if not h.buffer(0.5).contains(pt)])
    csk = [(e.dxf.center.x, e.dxf.center.y) for e in doc.modelspace().query("CIRCLE[layer=='CSK_M4']")]
    tap = [(e.dxf.center.x, e.dxf.center.y) for e in doc.modelspace().query("CIRCLE[layer=='TAP_M4']")]
    csk_bad = [round(edge(p), 1) for p in csk if edge(p) < CSK_EDGE]
    tap_bad = [round(edge(p), 1) for p in tap if edge(p) < TAP_EDGE]
    problems = len(thin) + len(small) + len(csk_bad) + len(tap_bad)
    where = ", ".join(f"({g.centroid.x:.0f},{g.centroid.y:.0f})" for g in thin[:4])
    print(f"{'OK  ' if not problems else 'FAIL'} {path.stem:18s} {'1/8' if mb < 2 else '1/4'}in  thin spots {len(thin)}"
          + (f" at {where}" if thin else "") + f" | holes < 1.45 mm: {len(small)} | countersinks {len(csk)} (too close: {csk_bad})"
          + f" | tapped {len(tap)} (too close: {tap_bad})")
    return problems


def main():
    bad = sum(check(p) for p in sorted(OUT.glob("*.dxf")) if p.stem not in SKIP)
    for r in REVIEW:
        print("REVIEW", r)
    print(f"{bad} problem(s), {len(REVIEW)} item(s) needing supplier review")
    return bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
