"""v8: which camera tilt is best? For every tilt in the Limelight mount's range (1 deg steps) and a few heights of the
base plate on the robot, the share of scoring spots (every spot in the shooting map where all four shots score) from
which the camera reads our up-CELL's tags (camera_coverage.sees_target: incidence, field of view, tag size in pixels).
The lens height is the base-plate height plus the lens's height on the turret at that tilt (from the model), and the lens
sits ~180 mm ahead of the turret axis, toward the target (Astra review: the first version put it on the axis). These are
geometric scores on a sampled map (incidence and tag-size rules; no ray test against the HIVE frame or other robots),
not measured detection rates: test with the real camera.

    python camera_tilt_study.py      -> table + out/camera_tilt.json
"""
import json
import math

import approach_check as A
import camera_coverage as C
import turret as T

BASE_HEIGHTS = (180.0, 200.0, 220.0, 242.0, 258.0)   # base-plate top above the tiles, mm (258 = the v8 start limit)


def lens_z(tilt):
    """Lens centre above the base-plate top (mm), as check_camera.frustum places it."""
    v, up = T.ll_dirs(tilt)
    return T.LL_PIVOT[1] - 30 * up[1] + T.LL_DIMS[2] * v[1]


def lens_fwd(tilt):
    """Lens centre's distance ahead of the turret axis (mm; the turret's front is -y)."""
    v, up = T.ll_dirs(tilt)
    return -(T.LL_PIVOT[0] - 30 * up[0] + T.LL_DIMS[2] * v[0])


def scoring_rows():
    sm = json.loads((T.OUT / "approach" / "shoot_map.json").read_text())
    K = ("Pollen-down", "Pollen-up", "Nectar-down", "Nectar-up")
    return [r for r in sm["rows"] if not r.get("frame") and all(r[k] for k in K)
            and abs(r["x"] - A.NEIGH_X / 2) <= 72 * A.IN - C.ROBOT_HALF and abs(r["u"]) <= 72 * A.IN - C.ROBOT_HALF]


def main():
    rows = scoring_rows()
    tilts = list(range(T.LL_TILT_RANGE[0], T.LL_TILT_RANGE[1] + 1))
    table = {}
    for hb in BASE_HEIGHTS:
        table[hb] = {}
        for tl in tilts:
            h = (hb + lens_z(tl)) / 1000
            table[hb][tl] = 100 * sum(C.sees_target(r["x"], r["u"], h, tl, lens_fwd(tl) / 1000) for r in rows) / len(rows)
    print(f"{len(rows)} scoring spots. Share (%) where the camera reads our target's tags:")
    print("base plate top ->  " + "  ".join(f"{hb:5.0f}" for hb in BASE_HEIGHTS) + "   (mm above the tiles)")
    for tl in tilts:
        print(f"tilt {tl:2d} deg (lens {lens_z(tl):5.1f} up, {lens_fwd(tl):5.1f} fwd) " + "  ".join(f"{table[hb][tl]:5.1f}" for hb in BASE_HEIGHTS))
    best = {}
    for hb in BASE_HEIGHTS:
        top = max(table[hb].values())
        good = [tl for tl in tilts if table[hb][tl] >= top - 1.0]        # within 1 point of the best
        best[hb] = {"best_pct": round(top, 1), "tilts_within_1pt": [min(good), max(good)]}
        print(f"base {hb:.0f}: best {top:.1f}% ; tilts within 1 point: {min(good)}-{max(good)} deg")
    out = {"spots": len(rows), "lens_z": {tl: round(lens_z(tl), 1) for tl in tilts},
           "pct": {str(int(hb)): {str(tl): round(v, 1) for tl, v in table[hb].items()} for hb in BASE_HEIGHTS}, "best": {str(int(k)): v for k, v in best.items()}}
    (T.OUT / "camera_tilt.json").write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
