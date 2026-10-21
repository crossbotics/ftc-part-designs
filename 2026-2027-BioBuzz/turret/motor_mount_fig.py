"""v6: labelled drawing of the adjustable flywheel-motor mount, seen from outside the motor-side plate (front of the
turret on the right). Written into viewer.html between <!--MOTOR_MOUNT_SVG--> markers.

    python motor_mount_fig.py
"""
import math
import re
from pathlib import Path

from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

import turret as T

HERE = Path(__file__).parent
M = T.FW_MOTOR
X0, X1, Z0, Z1 = -50.0, 46.0, -40.0, 40.0            # drawing window: display x (front = +), z, both from the motor axis
LABEL_W = 82.0                                     # label column on the right


def dx(y):
    return -(y - M[0])                                 # seen from outside the motor side: forward (-Y) is to the right


def path(poly):
    d = ""
    for g in (poly.geoms if hasattr(poly, "geoms") else [poly]):
        for ring in [g.exterior, *g.interiors]:
            d += "M" + " L".join(f"{dx(y) - X0:.1f},{Z1 - (z - M[1]):.1f}" for y, z in ring.coords) + " Z "
    return d


def belt_outline():
    """Pitch-line belt loop around the 16T and 48T pulleys (external tangents + arcs)."""
    c1, c2 = Point(*M), Point(T.FW_Y, T.FW_Z)
    r1, r2 = T.htd_pd(T.MOTOR_PULLEY_T) / 2, T.htd_pd(T.MAIN_MOTOR_PULLEY_T) / 2
    return unary_union([c1.buffer(r1, quad_segs=32), c2.buffer(r2, quad_segs=48)]).convex_hull


def main():
    view = box(M[0] - X1, M[1] + Z0, M[0] - X0, M[1] + Z1)
    side = T.side_plate_2d(False).intersection(view)
    jz = T.motor_rel((0.0, T.JACK_DZ))[1]
    lug = box(*T.motor_rel((T.LUG_Y[0], T.JACK_DZ - T.LUG_W)), *T.motor_rel((T.LUG_Y[1], T.JACK_DZ + T.LUG_W)))
    ya, yb = T.motor_rel((T.YOKE_BRIDGE[0], 0))[0], T.motor_rel((T.YOKE_BRIDGE[1], 0))[0]
    yarm = T.motor_rel((T.YOKE_ARM_Y, 0))[0]
    yoke = unary_union([box(ya, jz - T.YOKE_ARM[1], yb, jz + T.YOKE_ARM[1]),
                        box(yb - 0.01, jz + T.YOKE_ARM[0], yarm, jz + T.YOKE_ARM[1]),
                        box(yb - 0.01, jz - T.YOKE_ARM[1], yarm, jz - T.YOKE_ARM[0])])
    screw = LineString([(ya - 4.0, jz), (ya + T.JACK_SCREW_L, jz)]).buffer(2.0, cap_style=2)
    head = box(ya - 4.0, jz - 3.5, ya, jz + 3.5)
    belt = belt_outline().difference(belt_outline().buffer(-4.0)).intersection(view)
    W, H = X1 - X0 + LABEL_W, Z1 - Z0
    s = [f'<svg viewBox="0 0 {W:.0f} {H:.0f}" role="img" aria-label="The adjustable flywheel motor mount, seen from outside" '
         f'style="width:100%;height:auto;max-width:720px;font-family:var(--font-ui,system-ui)">']
    s.append(f'<path d="{path(side)}" fill="currentColor" fill-opacity="0.07" fill-rule="evenodd" stroke="currentColor" stroke-width="0.5"/>')
    s.append(f'<path d="{path(T.motor_adapter_2d(True))}" fill="none" stroke="currentColor" stroke-width="0.5" stroke-dasharray="2 1.5" fill-rule="evenodd"/>')
    s.append(f'<path d="{path(belt)}" fill="currentColor" fill-opacity="0.35" stroke="none"/>')
    s.append(f'<path d="{path(Point(*M).buffer(T.htd_pd(16) / 2 + 2))}" fill="none" stroke="currentColor" stroke-width="0.6"/>')
    for p in T.MOTOR_BOLTS:
        s.append(f'<path d="{path(Point(*T.motor_rel(p)).buffer(3.8))}" fill="currentColor" fill-opacity="0.25" stroke="currentColor" stroke-width="0.3"/>')
    s.append(f'<path d="{path(yoke)}" fill="currentColor" fill-opacity="0.3" stroke="currentColor" stroke-width="0.5"/>')
    s.append(f'<path d="{path(lug)}" style="fill:var(--accent,#c8321f)" fill-opacity="0.55" stroke="currentColor" stroke-width="0.5"/>')
    s.append(f'<path d="{path(screw)} {path(head)}" fill="currentColor" stroke="none"/>')
    for p in T.CLAMP_BOLTS + T.LUG_BOLTS:
        s.append(f'<path d="{path(Point(*T.motor_rel(p)).buffer(3.8))}" style="fill:var(--accent,#c8321f)" stroke="currentColor" stroke-width="0.4"/>')
        c = T.motor_rel(p); cx, cz = dx(c[0]) - X0, Z1 - (c[1] - M[1])
        s.append(f'<path d="M{cx - 1.2:.1f},{cz:.1f} h2.4 M{cx:.1f},{cz - 1.2:.1f} v2.4" stroke="#fff" stroke-width="0.6"/>')
    for p in T.YOKE_SCREWS:
        s.append(f'<path d="{path(Point(*T.motor_rel(p)).buffer(3.5))}" fill="currentColor" fill-opacity="0.6"/>')

    def pt(y_off, z_off):                               # motor-relative (Y, Z) -> svg
        return -y_off - X0, Z1 - z_off
    def label(anchor, text_xy, lines, bold=None, end="start"):
        (ax, az), (tx, tz) = pt(*anchor), text_xy
        s.append(f'<path d="M{ax:.1f},{az:.1f} L{tx - (1 if end == "start" else -1):.1f},{tz - 1.2:.1f}" stroke="currentColor" stroke-width="0.3" fill="none"/>')
        for i, ln in enumerate(lines):
            w = "700" if i == 0 else "400"
            s.append(f'<text x="{tx:.1f}" y="{tz + 4.3 * i:.1f}" font-size="3.6" font-weight="{w}" text-anchor="{end}" fill="currentColor">{ln}</text>')
    # one label column on the right, top to bottom in the same order as the parts they point to
    col = X1 - X0 + 6
    from shapely.ops import nearest_points
    e = nearest_points(T.motor_adapter_2d(True).exterior, Point(*T.motor_rel((-21.0, -17.0))))[0]
    adapter_edge = (e.x - M[0], e.y - M[1])
    rows = [(T.CLAMP_BOLTS[0], ["Clamp bolt", "2.5 mm key"]),
            ((T.YOKE_SCREWS[1][0], T.YOKE_SCREWS[1][1]), ["Jack yoke (printed)", "bolted to the side plate"]),
            (T.LUG_BOLTS[1], ["2 clamp bolts through the lug", "2.5 mm key"]),
            ((T.YOKE_BRIDGE[0] - 4, T.JACK_DZ), ["Jack screw, 2.5 mm key", "clockwise = belt tighter"]),
            ((T.LUG_Y[1] - 1.5, T.JACK_DZ - T.LUG_W + 1.5), ["Tension lug (printed)", "clamped to the adapter plate"]),
            ((0.0, -12.0), ["Window: 4 motor screws", "(reach them with the pulley off)"]),
            (adapter_edge, ["Adapter plate (dashed)", "behind the side plate"]),
            (T.CLAMP_BOLTS[1], ["Clamp bolt", "2.5 mm key"])]
    for i, (anchor, lines) in enumerate(rows):
        label(anchor, (col, 5.5 + i * 9.6), lines)
    for i, ln in enumerate(["\u2190 belt to the 48T", "on the main shaft"]):
        s.append(f'<text x="3" y="{55 + 4.2 * i:.1f}" font-size="3.4" fill="currentColor">{ln}</text>')
    # slide arrow under the motor
    ax, az = pt(0, -30.5)
    s.append(f'<path d="M{ax - 9:.1f},{az:.1f} H{ax + 9:.1f} M{ax - 9:.1f},{az:.1f} l2,-1.3 M{ax - 9:.1f},{az:.1f} l2,1.3 '
             f'M{ax + 9:.1f},{az:.1f} l-2,-1.3 M{ax + 9:.1f},{az:.1f} l-2,1.3" stroke="currentColor" stroke-width="0.5" fill="none"/>')
    s.append(f'<text x="{ax:.1f}" y="{az + 6.5:.1f}" font-size="3.4" text-anchor="middle" fill="currentColor">motor slides ±2 mm</text>')
    s.append(f'<text x="{X1 - X0 - 2:.1f}" y="{H - 2:.1f}" font-size="3.2" text-anchor="end" fill="currentColor" fill-opacity="0.7">front of turret →</text>')
    s.append("</svg>")
    svg = "\n".join(s)
    v = HERE / "viewer.html"; src = v.read_text()
    block = "<!--MOTOR_MOUNT_SVG-->" + svg + "<!--/MOTOR_MOUNT_SVG-->"
    new = re.sub(r"<!--MOTOR_MOUNT_SVG-->.*?<!--/MOTOR_MOUNT_SVG-->", lambda _: block, src, flags=re.S)
    if new == src and "<!--MOTOR_MOUNT_SVG-->" not in src:
        print("marker not found in viewer.html")
    else:
        v.write_text(new); print("motor mount figure written")
    (HERE / "out" / "motor_mount.svg").write_text(svg)


if __name__ == "__main__":
    main()
