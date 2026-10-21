"""Where on the field can the Limelight see a HIVE AprilTag cluster?

APPROXIMATE geometry (from Competition Manual V1 section 9.6 - verify against the official field CAD):
  * HIVE frame centered on the field; two HIVEs (red, blue) 25.5 in apart (along field Y), each tips about a
    horizontal pivot 43.95 in up; its two CELLs sit ~15.4 in either side of the pivot along field X.
  * The HIVE tips 30 deg. Tag clusters are on each CELL's underside, facing down. Tipping rotates BOTH undersides,
    so both face down-and-toward the side the up-cell is on (normal = (+-sin30, 0, -cos30)).
  * Estimated tag-cluster centres: up-cell ~1.20 m high, ~0.30 m out from the pivot;
    down-cell ~0.70 m high, ~0.45 m out on the other side.
Camera: Limelight 3A, 640x480, 54.5 x 42 deg FOV. The turret points the camera at the HIVE centre (yaw).
Detection rule of thumb: tag seen < 65 deg off its normal, and >= 12 px across (3.25 in tag, foreshortened).

    python camera_coverage.py
"""
import math
import numpy as np

FIELD = 3.58                                  # inside of the perimeter, m (12 ft field)
TAG = 0.0825                                  # 3.25 in
HFOV, VFOV, HPIX = math.radians(54.5), math.radians(42.0), 640
MAX_INC, MIN_PX = math.radians(65), 12.0
FRAME = (1.256 / 2, 0.989 / 2)                # HIVE frame base half-size (x, y): robots can't be inside
ROBOT_HALF = 0.229

def tags(tip):
    """tip=+1: up-cells at +X. Returns list of (pos, normal) for the 4 clusters (2 HIVEs x 2 CELLs)."""
    s, c = math.sin(math.radians(30)), math.cos(math.radians(30))
    n = np.array([tip * s, 0.0, -c])
    out = []
    for hy in (-0.324, 0.324):                # red / blue HIVE, 25.5 in apart
        # field CAD (field_cad/cell_tag_cad.py): cluster centres, relative to each pivot; both face down and outward
        out.append((np.array([tip * 0.3005, hy, 1.2460]), n))    # up cell (was 0.30 / 1.20)
        out.append((np.array([-tip * 0.2626, hy, 0.9209]), n))   # down cell (was 0.45 / 0.70)
    return out

def visible(cam, tilt_deg, tip):
    n0 = np.linalg.norm(cam[:2])
    if n0 < 1e-9:                                             # right at the field centre, under the frame: no aim direction
        return 0
    yaw_dir = -cam[:2] / n0                                   # turret aims at the HIVE centre
    fwd = np.array([yaw_dir[0] * math.cos(math.radians(tilt_deg)), yaw_dir[1] * math.cos(math.radians(tilt_deg)),
                    math.sin(math.radians(tilt_deg))])
    right = np.array([yaw_dir[1], -yaw_dir[0], 0.0]); up = np.cross(right, fwd)
    best = 0
    for p, n in tags(tip):
        v = p - cam; d = np.linalg.norm(v); u = v / d
        inc = math.acos(max(-1, min(1, float(-u @ n))))       # angle between line of sight and the tag's face normal
        if inc > MAX_INC:
            continue
        x, y, z = u @ right, u @ up, u @ fwd
        if z <= 0 or abs(math.atan2(x, z)) > HFOV / 2 or abs(math.atan2(y, z)) > VFOV / 2:
            continue
        px = TAG * math.cos(inc) / d * (HPIX / 2) / math.tan(HFOV / 2)   # v5: pinhole focal length in pixels
        if px >= MIN_PX:
            best += 1
    return best

def coverage(cam_h, tilt):
    xs = np.linspace(-FIELD / 2 + ROBOT_HALF, FIELD / 2 - ROBOT_HALF, 41)
    tot = ok = 0
    grid = []
    for y in xs[::-1]:
        row = ""
        for x in xs:
            if abs(x) < FRAME[0] + ROBOT_HALF and abs(y) < FRAME[1] + ROBOT_HALF:
                row += "#"; continue
            tot += 1
            n = min(visible(np.array([x, y, cam_h]), tilt, +1), visible(np.array([x, y, cam_h]), tilt, -1))
            ok += n > 0
            row += "." if n == 0 else ("o" if n == 1 else "@")
        grid.append(row)
    return ok / tot, grid

# ---- v6.1: only OUR target counts --------------------------------------------------------------------------
# The team cares about one tag cluster: the one under the up-CELL we're shooting at. And the turret points at that
# opening, so the camera yaws toward it (not toward the HIVE's centre). HIVE coordinates as approach_check: x along
# the pivots (our HIVE at 0, the other at +25.5 in), u out from our pivot toward our opening, z up.

@__import__("functools").lru_cache(None)
def target_tag():
    """Centre and face normal of the tag cluster under our up-CELL (middle of the CELL's underside)."""
    import approach_check as A
    p = A.PIV + 0.3251 * A.AX - 0.0379 * A.NUP                  # field CAD: (0, 0.3005, 1.2460) m
    return p, -A.NUP                                            # faces down and toward our opening's side


def sees_target(x, u, cam_h=0.40, tilt_deg=35.0, lens_fwd=0.0):
    """True if the Limelight on a robot centred at (x, u), yawed at our opening, can read our up-CELL's tags.
    v8 (Astra): lens_fwd (m) puts the lens that far ahead of the turret axis, toward the target (the real camera sits
    ~0.18 m in front of the axis; 0 keeps the old axis-centred estimate, which the v7 page and the simulator used)."""
    import entry_window as E
    uc = (E.B[0] + E.P[0]) / 2
    aim = np.array([0.0 - x, uc - u])
    if np.linalg.norm(aim) < 1e-6:
        return False
    yaw = aim / np.linalg.norm(aim)
    cam = np.array([x + yaw[0] * lens_fwd, u + yaw[1] * lens_fwd, cam_h])
    t = math.radians(tilt_deg)
    fwd = np.array([yaw[0] * math.cos(t), yaw[1] * math.cos(t), math.sin(t)])
    right = np.array([yaw[1], -yaw[0], 0.0]); up = np.cross(right, fwd)
    p, n = target_tag()
    v = p - cam; d = np.linalg.norm(v); w = v / d
    inc = math.acos(max(-1, min(1, float(-w @ n))))
    if inc > MAX_INC:
        return False
    a, b, c = w @ right, w @ up, w @ fwd
    if c <= 0 or abs(math.atan2(a, c)) > HFOV / 2 or abs(math.atan2(b, c)) > VFOV / 2:
        return False
    return TAG * math.cos(inc) / d * (HPIX / 2) / math.tan(HFOV / 2) >= MIN_PX


if __name__ == "__main__":
    import json, pathlib
    sm = pathlib.Path(__file__).parent / "out" / "approach" / "shoot_map.json"
    if sm.exists():   # of the spots that score in every case (figure 6), how many see our target's tags?
        rows = [r for r in json.loads(sm.read_text())["rows"] if not r.get("frame")
                and all(r[k] for k in ("Pollen-down", "Pollen-up", "Nectar-down", "Nectar-up"))
                and abs(r["x"] - 0.324) <= 1.829 - ROBOT_HALF and abs(r["u"]) <= 1.829 - ROBOT_HALF]
        for tl in (20, 35, 50):
            seen = sum(sees_target(r["x"], r["u"], 0.40, tl) for r in rows)
            print(f"our up-CELL tags, tilt {tl}: seen from {seen}/{len(rows)} scoring spots on the field ({100 * seen / len(rows):.0f}%)")
        print()
    print("coverage = share of reachable field positions that see >=1 tag cluster in BOTH HIVE tip states\n")
    for h in (0.33, 0.40):
        print(f"lens {h:.2f} m above the floor: " + "  ".join(f"tilt {t}: {100*coverage(h, t)[0]:3.0f}%" for t in (20, 30, 35, 40, 50)))
    for t in (35,):
        cov, grid = coverage(0.40, t)
        print(f"\nmap: lens 0.40 m, tilt {t} deg (# = HIVE frame, . = no tag, o = 1 cluster, @ = 2+ clusters; worst of both tip states)")
        print("\n".join(grid[::2]))
