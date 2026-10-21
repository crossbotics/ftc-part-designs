"""v8.1: spin radius of every turning part (plan distance from the turret axis), over hood, arm and camera poses.

    python reach.py [limit_mm]      -> parts beyond the limit; out/reach.json; exit 1 if any part is beyond it

The lifter and the start cube's front are both 203.2 mm from the axis (axis 1 in forward of the robot centre). The
default limit, 201.2, keeps 2 mm for build error, frame flex, plugs and ties. Nominal modelled parts only: no cables,
belt slack or motor-slide travel (Astra v8.1 review).
"""
import json
import math
import sys

import numpy as np

import turret as T
import check as C

BOUNDARY = 203.2
LIMIT = float(sys.argv[1]) if len(sys.argv) > 1 else BOUNDARY - 2.0
HOODS = (0.0, 12.5, 25.0)
BALLS = ("pollen", "nectar", C.swing_deg("nectar") + C.ARM_OVERSWING)
TILTS = (T.LL_TILT_RANGE[0], T.LL_TILT, T.LL_TILT_RANGE[1])

def verts(shape):
    return np.array([(p.X, p.Y, p.Z) for p in shape.tessellate(0.2, 0.2)[0]])

def rot_x(v, axis_yz, deg):
    """Rotate points about an X-parallel axis through (Y, Z) — the same moves check.pose makes."""
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    y, z = v[:, 1] - axis_yz[0], v[:, 2] - axis_yz[1]
    return np.column_stack([v[:, 0], axis_yz[0] + c * y - s * z, axis_yz[1] + s * y + c * z])

def plan_r(v):
    return float(np.hypot(v[:, 0], v[:, 1]).max()) if len(v) else 0.0

def reach_table(parts):
    M, Jw = (T.FW_Y, T.FW_Z), T.world(T.J)
    reach = {}
    for n, (s, g, *_) in parts.items():
        if g == "fixed":
            continue
        v = verts(s)
        for a in HOODS:
            for ball in BALLS:
                for tilt in TILTS:
                    w = v
                    if g == "arm":
                        w = rot_x(w, Jw, C.swing_deg(ball))
                    if g in ("carrier", "arm"):
                        w = rot_x(w, M, a)
                    if g == "idler" and ball == "nectar" and T.IDLER_SLOTS:
                        w = w + (0, T.IDLER_SLOTS[0][1] - T.IDLER_SLOTS[0][0], 0)
                    if g == "ll":
                        w = rot_x(w, T.LL_PIVOT, T.LL_TILT - tilt)
                    reach[n] = max(reach.get(n, 0.0), plan_r(w))
                    if g not in ("carrier", "arm", "ll", "idler"):
                        break
                if g not in ("carrier", "arm", "idler"):
                    break
            if g not in ("carrier", "arm"):
                break
    for a in HOODS:
        for ball in BALLS:
            for n, s in C.springs(a, ball).items():
                reach[n] = max(reach.get(n, 0.0), plan_r(verts(s)))
    return dict(sorted(reach.items(), key=lambda kv: -kv[1]))

def main():
    parts = T.build()[0]
    reach = reach_table(parts)
    n0, r0 = next(iter(reach.items()))
    print(f"spin radius {r0:.1f} mm ({n0}); limit {LIMIT:.1f}; margin to the lifter / start cube {BOUNDARY - r0:.1f} mm")
    over = {n: r for n, r in reach.items() if r > LIMIT}
    for n, r in over.items():
        print(f"  {r:6.1f}  {n}")
    T.OUT.mkdir(exist_ok=True)
    (T.OUT / "reach.json").write_text(json.dumps({"limit": LIMIT, "reach": reach}, indent=1))
    raise SystemExit(1 if over else 0)

if __name__ == "__main__":
    main()
