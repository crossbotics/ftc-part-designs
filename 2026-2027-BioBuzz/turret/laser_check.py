"""v7: can the team's diode laser cut these parts? Checks every CUT drawing in out/dxf that's plywood (the laminated
*_L1/_L2 layers and the front plate) for 1/8" (3 mm) Baltic birch on the Elegoo Phecda 20 W:

    min web 2.5 mm (thinner strips char through or snap), min hole 2.0 mm, part fits the 400 x 400 mm bed and a
    12 x 20 in (305 x 508 mm) sheet with a 5 mm margin. Then a rough nesting count of 12 x 20 in sheets.

    python laser_check.py
"""
import math
import sys
from pathlib import Path

import ezdxf
from shapely.geometry import Polygon
from shapely.ops import unary_union

OUT = Path(__file__).parent / "out" / "dxf"
MIN_WEB, MIN_HOLE, BED, SHEET, MARGIN = 2.5, 2.0, (400.0, 400.0), (304.8, 508.0), 5.0
PLY = lambda stem: stem.endswith(("_L1", "_L2")) or stem in ("front_plate", "fit_test") or stem.startswith("belt_test")


def part_of(path):
    doc = ezdxf.readfile(path)
    rings = [Polygon([(p[0], p[1]) for p in e.get_points()]) for e in doc.modelspace().query("LWPOLYLINE[layer=='CUT']")]
    outer = max(rings, key=lambda r: r.area)
    holes = [r for r in rings if r is not outer]
    return outer.difference(unary_union(holes)) if holes else outer, outer, holes


def fits(w, h, W, H):
    return (w <= W - 2 * MARGIN and h <= H - 2 * MARGIN) or (h <= W - 2 * MARGIN and w <= H - 2 * MARGIN)


def check(path):
    part, outer, holes = part_of(path)
    opened = part.buffer(-MIN_WEB / 2 * 0.98, join_style=1).buffer(MIN_WEB / 2 * 0.98, join_style=1)
    lost = part.difference(opened)
    thin = [g for g in (lost.geoms if hasattr(lost, "geoms") else [lost]) if g.area > 0.5]
    small = [h for h in holes if 2 * math.sqrt(h.area / math.pi) < MIN_HOLE]
    x0, y0, x1, y1 = outer.bounds
    w, h = x1 - x0, y1 - y0
    ok_bed, ok_sheet = fits(w, h, *BED), fits(w, h, *SHEET)
    problems = len(thin) + len(small) + (not ok_bed) + (not ok_sheet)
    where = ", ".join(f"({g.centroid.x:.0f},{g.centroid.y:.0f})" for g in thin[:4])
    print(f"{'OK  ' if not problems else 'FAIL'} {path.stem:24s} {w:5.0f} x {h:5.0f} mm | thin spots {len(thin)}"
          + (f" at {where}" if thin else "") + f" | holes < {MIN_HOLE} mm: {len(small)} | bed {'ok' if ok_bed else 'TOO BIG'}"
          + f" | 12x20 sheet {'ok' if ok_sheet else 'TOO BIG'}")
    return problems, (w, h)


def sheets_needed(sizes):
    """Shelf packing of bounding boxes (longest side along the sheet's 508 mm), 5 mm gaps. Rough, conservative."""
    W, H = SHEET[1] - 2 * MARGIN, SHEET[0] - 2 * MARGIN
    boxes = sorted([(max(s), min(s)) for s in sizes], key=lambda b: -b[1])
    sheets = []
    for bw, bh in boxes:
        placed = False
        for sh in sheets:
            for row in sh:
                if row[1] >= bh and row[0] + bw <= W:
                    row[0] += bw + 5; placed = True; break
            if not placed and sum(r[1] + 5 for r in sh) + bh <= H:
                sh.append([bw + 5, bh]); placed = True
            if placed:
                break
        if not placed:
            sheets.append([[bw + 5, bh]])
    return len(sheets)


def main():
    res = [check(p) for p in sorted(OUT.glob("*.dxf")) if PLY(p.stem)]
    bad = sum(r[0] for r in res)
    print(f"{bad} problem(s)")
    return bad, res


if __name__ == "__main__":
    bad, res = main()
    print(f"about {sheets_needed([r[1] for r in res])} sheets of 12 x 20 in, 1/8 in Baltic birch (rough shelf packing, one set)")
    sys.exit(1 if bad else 0)
