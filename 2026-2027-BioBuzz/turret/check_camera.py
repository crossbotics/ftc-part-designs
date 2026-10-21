"""Camera check: is the Limelight's view cone clear of the turret, at each tilt and hood angle?"""
import math
import turret as T
from check import pose, overlap
from build123d import Solid, Vector, Wire

def frustum(tilt, near=3.0, far=450.0):
    v, up = T.ll_dirs(tilt)
    lens = Vector(0, T.LL_PIVOT[0] - 30 * up[0] + T.LL_DIMS[2] * v[0], T.LL_PIVOT[1] - 30 * up[1] + T.LL_DIMS[2] * v[1])
    V, U, X = Vector(0, *v), Vector(0, *up), Vector(1, 0, 0)
    h, w = math.tan(math.radians(T.LL_FOV[0] / 2)), math.tan(math.radians(T.LL_FOV[1] / 2))
    def rect(d):
        return Wire.make_polygon([lens + (V + X * sx * h + U * sy * w) * d for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))], close=True)
    return Solid.make_loft([rect(near), rect(far)]), lens

if __name__ == "__main__":
    parts, _ = T.build()
    bad = 0
    for alpha in (0.0, T.HOOD_ALPHA_MAX):
        for tilt in range(T.LL_TILT_RANGE[0], T.LL_TILT_RANGE[1] + 1, 5):
            sh = pose(parts, alpha, "nectar", tilt)
            fr, lens = frustum(tilt)
            hits = {n: round(overlap(fr, s)) for n, s in sh.items() if not n.startswith(("ll_", "limelight"))}
            hits = {k: v for k, v in hits.items() if v > 1}
            bad += bool(hits)
            print(f"hood {alpha:4.1f}  tilt {tilt}: lens {lens.Z:.0f} mm above base; view blocked by {hits or 'nothing'}")
    print("BLOCKED:", bad)
    raise SystemExit(1 if bad else 0)
