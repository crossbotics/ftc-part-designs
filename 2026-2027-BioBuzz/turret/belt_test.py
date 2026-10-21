"""Belt test plates: cut these FIRST, from the same 6.35 mm stock on the same cutter as the real plates.

The jackshaft belt runs on a fixed centre distance (no slots, no tensioner); the motor belt is on the adjustable
mount, so its plate is optional. The theory says 315 mm belt -> 73.06 mm and 410 mm belt -> 145.00 mm, but real belts and a real cutter can be a few tenths off. Each plate has one fixed
bearing hole and five engraved positions at -0.4, -0.2, 0, +0.2, +0.4 mm. Try the real pulleys and belt at each;
enter the best label as MOTOR_C_TRIM / JACK_C_TRIM in turret.py and rebuild before cutting the real plates.

    python belt_test.py      -> out/dxf/belt_test_motor.dxf, out/dxf/belt_test_jack.dxf (+ inline SVG in viewer.html)
"""
import math
import re
from pathlib import Path

import ezdxf
from shapely.geometry import Point
from shapely.ops import unary_union

import turret as T

OFFSETS = [-0.4, -0.2, 0.0, 0.2, 0.4]
HERE = Path(__file__).parent


def motor_plate():
    """Main-shaft bearing at the origin; five motor mounts fanned around it at C + offset."""
    c0 = T.MOTOR_C - T.MOTOR_C_TRIM                        # theoretical centre distance
    angs = [-60, -30, 0, 30, 60]
    feats, holes, csk, labels = [Point(0, 0).buffer(20)], [T.circ(0, 0, T.BRG_1611)], [], []
    for off, a in zip(OFFSETS, angs):
        r = c0 + off; x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        feats.append(Point(x, y).buffer(22))
        holes.append(T.motor_holes(x, y))                  # 14 mm centre hole + 4x M4 on the 16 mm square
        if T.FW_MOTOR_CSK:
            csk += [(x + dx, y + dy) for dx, dy in T.MOTOR_BOLTS]
        lr = r - 30                                        # engraved inside the plate, on the bearing side
        labels.append((f"{off:+.1f}" if off else "0", lr * math.cos(math.radians(a)), lr * math.sin(math.radians(a))))
    outline = unary_union(feats).convex_hull.buffer(4)
    title = f"MOTOR BELT {T.MOTOR_BELT} - C {c0:.2f}"
    return outline.difference(unary_union(holes)), csk, labels, title, (12, -24)


def jack_plate():
    """One bearing at the origin; five bearing holes at C + offset, 10 degrees apart."""
    c0 = T.JACK_C - T.JACK_C_TRIM
    angs = [-20, -10, 0, 10, 20]
    feats, holes, labels = [Point(0, 0).buffer(16)], [T.circ(0, 0, T.BRG_1611)], []
    for off, a in zip(OFFSETS, angs):
        r = c0 + off; x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        feats.append(Point(x, y).buffer(14))
        holes.append(T.circ(x, y, T.BRG_1611))
        lr = r - 20
        labels.append((f"{off:+.1f}" if off else "0", lr * math.cos(math.radians(a)), lr * math.sin(math.radians(a))))
    outline = unary_union(feats).convex_hull.buffer(4)
    title = f"JACK BELT {T.JACK_BELT} - C {c0:.2f}"
    return outline.difference(unary_union(holes)), [], labels, title, (20, -5)


def write(name, plate, csk, labels, title, tpos, notes):
    doc = ezdxf.new("R2010"); doc.units = ezdxf.units.MM
    for lay, col in (("CUT", 7), ("CSK_M4", 1), ("ENGRAVE", 5), ("NOTES", 3)):
        doc.layers.add(lay, color=col)
    msp = doc.modelspace()
    for g in (plate.geoms if hasattr(plate, "geoms") else [plate]):
        for ring in [g.exterior, *g.interiors]:
            msp.add_lwpolyline(list(ring.coords), close=True, dxfattribs={"layer": "CUT"})
    for x, y in csk:
        msp.add_circle((x, y), T.CSK_D / 2, dxfattribs={"layer": "CSK_M4"})
    for text, x, y in labels:
        msp.add_text(text, height=6, dxfattribs={"layer": "ENGRAVE"}).set_placement((x, y), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    msp.add_text(title, height=3, dxfattribs={"layer": "ENGRAVE"}).set_placement(tpos)
    x0, y0, x1, y1 = plate.bounds
    for i, line in enumerate(notes):
        msp.add_text(line, height=3, dxfattribs={"layer": "NOTES"}).set_placement((x0, y0 - 8 - 5 * i))
    doc.saveas(HERE / "out" / "dxf" / f"{name}.dxf")


def svg(plates):
    """Inline SVG preview of both plates for the web page (strokes follow the text colour)."""
    pad, gap, scale = 10, 30, 1.0
    boxes = [p[0].bounds for p in plates]
    widths = [b[2] - b[0] for b in boxes]; H = max(b[3] - b[1] for b in boxes) + 2 * pad + 18
    W = sum(widths) + gap + 2 * pad
    out = [f'<svg viewBox="0 0 {W:.0f} {H:.0f}" role="img" aria-label="The two belt test plates" style="width:100%;height:auto;max-width:720px">']
    xoff = pad
    for (plate, csk, labels, title, _), b in zip(plates, boxes):
        tx = lambda x: xoff + (x - b[0]); ty = lambda y: pad + (b[3] - y)
        d = ""
        for g in (plate.geoms if hasattr(plate, "geoms") else [plate]):
            for ring in [g.exterior, *g.interiors]:
                pts = list(ring.coords)
                d += "M" + " L".join(f"{tx(x):.1f},{ty(y):.1f}" for x, y in pts) + " Z "
        out.append(f'<path d="{d}" fill="currentColor" fill-opacity="0.08" fill-rule="evenodd" stroke="currentColor" stroke-width="0.8"/>')
        for text, x, y in labels:
            out.append(f'<text x="{tx(x):.1f}" y="{ty(y) + 2.5:.1f}" font-size="7" text-anchor="middle" fill="currentColor">{text}</text>')
        out.append(f'<text x="{tx(b[0]):.1f}" y="{H - 6:.1f}" font-size="7.5" font-weight="700" fill="currentColor">{title.replace(" - ", " · ")} mm</text>')
        xoff += (b[2] - b[0]) + gap
    out.append("</svg>")
    return "\n".join(out)


def main():
    m = motor_plate(); j = jack_plate()
    common = ["Cut from the SAME 6.35 mm stock on the SAME cutter as the real plates (hole size matters).",
              "Engraved numbers = the trim to enter in turret.py (MOTOR_C_TRIM / JACK_C_TRIM)."]
    write("belt_test_motor", *m, notes=common + ["Main-shaft 1611 bearing in the centre hole; move the motor between the five mounts.",
                                                 "OPTIONAL: the real motor mount slides +/-2 mm (+/-1.4 mm of belt center distance) (adapter + jack screw). Motor screws: M4 x 12 button heads."])
    write("belt_test_jack", *j, notes=common + ["Fixed 1611 bearing in the left hole; move the second shaft between the five holes."])
    v = HERE / "viewer.html"; s = v.read_text()
    block = "<!--BELT_TEST_SVG-->" + svg([m, j]) + "<!--/BELT_TEST_SVG-->"
    s2 = re.sub(r"<!--BELT_TEST_SVG-->.*?<!--/BELT_TEST_SVG-->", lambda _: block, s, flags=re.S)
    if s2 != s:
        v.write_text(s2)
    print(f"motor plate {m[0].bounds[2]-m[0].bounds[0]:.0f} x {m[0].bounds[3]-m[0].bounds[1]:.0f} mm, "
          f"jack plate {j[0].bounds[2]-j[0].bounds[0]:.0f} x {j[0].bounds[3]-j[0].bounds[1]:.0f} mm; SVG {'updated' if s2 != s else ('unchanged' if '<!--BELT_TEST_SVG-->' in s else 'marker not found')}")


if __name__ == "__main__":
    main()
