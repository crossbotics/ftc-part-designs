"""Shot tolerance against the REAL up-cell opening (not a point target).

Geometry (Competition Manual V1 9.6; since 2026-09-29 measured from the official field CAD, see the constants below):
  * CELL = 12 in deep box whose pentagon opening (20.0 in wide, 13.9 in tall: 7.8 in straight sides, a roof with a
    flat 1.1 in peak, bottom corners cut 1 in at 45 deg) is on its OUTER end. HIVE pivot 43.95 in up; the cell's inner
    end sits 9.42 in from the pivot.
  * Tipped 30 deg: the opening faces outward and 30 deg UP (normal (cos30, sin30)). In side view it's a segment
    from the bottom rim B = (0.489 m out, 1.358 m up) to the roof peak P = (0.313 m out, 1.663 m up).
  * Inside, the cell floor slopes 30 deg DOWN toward the pivot, so a ball that gets past the rim rolls in.
We shoot from the side the HIVE is tipped toward, square-on, aimed at the opening's centre line.
A shot is IN if the ball's centre crosses the opening line (moving inward) with at least one ball radius of
clearance from the bottom rim and from the roof (which narrows toward the peak).

    python entry_window.py
"""
import math
import numpy as np
import turret as T
from shot_tolerance import RHO, CD, G

IN = 0.0254
PIVOT_Z = 43.95 * IN
S30, C30 = 0.5, math.sqrt(3) / 2
# Field CAD (2026-09-29): the opening is measured from the official field CAD (field_cad/compare_cad.py; blue scoring
# CELL, sectioned 3 mm inside its outer end). It matches the manual's figures within ~3 mm: pivot 1116.3 mm (43.95 in),
# tilt 30.00 deg, 20.03 in wide, bottom rim 53.45 in up (manual ~53.5), roof peak 65.47 in (manual ~65.6).
TILT = 30.0                      # deg the HIVE tips; the up-cell's axis points out and 30 deg up
CELL_OUT = 0.5445                # pivot -> the opening plane (where the rim's inner edge is), along the cell axis
CELL_S0 = -0.03567               # the bottom rim's offset from the pivot line, up the opening plane
HALF_W = 0.25445                 # opening half-width (CAD 508.9 mm)
CHAMFER = 0.0259                 # the two bottom corners are cut at 45 deg, 25.9 mm each way
SIDE = 0.1984                    # straight sides: bottom rim -> where the roof starts, along the plane
OPEN_H = 0.3525                  # bottom rim -> roof peak, along the plane (CAD 13.87 in; manual "approximately 14 in")
PEAK_HALF = 0.0139               # the peak is flat, 27.8 mm wide (a tube)
def _opening():
    """Returns B (bottom rim centre) and P (roof peak), in (u out from the pivot, z up)."""
    t = math.radians(TILT)
    ax = np.array([math.cos(t), math.sin(t)]); along = np.array([-math.sin(t), math.cos(t)])   # up the opening plane
    b = np.array([0.0, PIVOT_Z]) + CELL_OUT * ax + CELL_S0 * along
    return b, b + OPEN_H * along
B, P = _opening()                                # B ~ (0.4894, 1.3577) m, P ~ (0.3131, 1.6630) m (CAD 0.4894/1.3577, 0.3132/1.6628)
RIM_Z = (float(B[1]), float(P[1]))
L = np.linalg.norm(P - B)                        # 0.3525 m along the opening plane
TDIR = (P - B) / L
NORM = np.array([math.cos(math.radians(TILT)), math.sin(math.radians(TILT))])   # opening faces out (+u) and up (v6.1: from TILT)
LAUNCH_Z = 0.33                                  # turret base plate ~0.22 m + pinch ~0.11 m
BALLS = {"Pollen": (0.025, T.POLLEN_D / 2000), "Nectar": (0.041, T.NECTAR_D / 2000)}

def roof_top(r):
    """Height of the roof (along the plane, from B) a ball's width off the centre line, where the ball's edge meets it.
    Field CAD: the peak is flat (PEAK_HALF each side), then the roof slopes down to the sides at SIDE."""
    w = r + 0.005
    if w <= PEAK_HALF:
        return L
    return L - (L - SIDE) * min(1.0, (w - PEAK_HALF) / (HALF_W - PEAK_HALF))

def crossing(D, elev, v, m, r, dt=5e-4):
    """Launch from u = D (horizontal distance turret -> HIVE pivot) toward the HIVE. Returns (s along the opening
    from B, crossing angle to the plane (deg), descent angle (deg), entry speed) or None."""
    k = 0.5 * RHO * CD * math.pi * r * r / m
    u, z = D, LAUNCH_Z
    vu, vz = -v * math.cos(math.radians(elev)), v * math.sin(math.radians(elev))
    f = lambda uu, zz: (np.array([uu, zz]) - B) @ NORM          # >0 outside (shooter side) the opening plane
    prev = f(u, z)
    for _ in range(20000):
        s_ = math.hypot(vu, vz)
        vu -= k * s_ * vu * dt; vz -= (G + k * s_ * vz) * dt
        u, z = u + vu * dt, z + vz * dt
        cur = f(u, z)
        if prev > 0 >= cur:                                     # crossed the plane moving inward
            pt = np.array([u, z]); s = (pt - B) @ TDIR
            vel = np.array([vu, vz]) / math.hypot(vu, vz)
            ang = math.degrees(math.asin(min(1, abs(vel @ NORM))))
            return s, ang, math.degrees(math.atan2(-vz, -vu)), math.hypot(vu, vz)
        prev = cur
        if z < 0 or u < -1:
            return None
    return None

def window(D, elev, m, r):
    """Range of exit speeds that go in; returns (v_mid, +/- %, descent deg, crossing angle deg, entry speed).
    The crossing point s rises monotonically with exit speed, so bisect for the two limits."""
    def s_at(v):
        c = crossing(D, elev, v, m, r)
        return (c[0], c) if c else (-1.0, None)
    # find a speed that crosses the plane at all (too slow never reaches it)
    vs = np.arange(3.0, 14.0, 0.25)
    samples = [(v, *s_at(v)) for v in vs]
    good = [(v, s, c) for v, s, c in samples if c is not None]
    if not good:
        return None
    # v5 (Codex review): the ball-radius clearance depends on each trajectory's own crossing angle, so it is
    # evaluated per trajectory (it was taken from one sample and reused at both limits).
    marg = lambda c: r / max(math.sin(math.radians(c[1])), 0.2)
    top = roof_top(r)
    def solve(g):
        """Lowest speed where g(s, crossing) >= 0; g rises with speed."""
        prev = None
        for v, s, c in samples:
            if c is not None and g(s, c) >= 0:
                if prev is None:
                    return None
                a_, b_ = prev, v
                for _ in range(30):
                    mid = (a_ + b_) / 2
                    sm, cm = s_at(mid)
                    if cm is None or g(sm, cm) < 0: a_ = mid
                    else: b_ = mid
                return b_
            prev = v
        return None
    v_lo = solve(lambda s, c: s - marg(c))                  # clears the bottom rim by a ball radius
    v_hi = solve(lambda s, c: s + marg(c) - top)            # first speed that would touch the roof
    if v_lo is None or v_hi is None or v_hi <= v_lo:
        return None
    # entries while still RISING head for the roof/back wall (bounce-out risk): trim them off
    def desc(v):
        c = crossing(D, elev, v, m, r)
        return c[2] if c else -90
    MIN_DESC = 10.0
    if desc(v_lo) < MIN_DESC:
        return None
    if desc(v_hi) < MIN_DESC:
        a_, b_ = v_lo, v_hi
        for _ in range(30):
            mid = (a_ + b_) / 2
            if desc(mid) >= MIN_DESC: a_ = mid
            else: b_ = mid
        v_hi = a_
    mid = (v_lo + v_hi) / 2
    c = crossing(D, elev, mid, m, r)
    return mid, 100 * (v_hi - v_lo) / 2 / mid, c[2], c[1], c[3]

def best_angle(d, m, r, angles=range(50, 86, 1)):
    """Launch angle with the widest speed window at distance d (turret -> opening centre)."""
    D = d + (B[0] + P[0]) / 2
    best = None
    for a in angles:
        w = window(D, a, m, r)
        if w and (best is None or w[1] > best[1][1]):
            best = (a, w)
    return best

if __name__ == "__main__":
    print(f"opening: {L/IN:.1f} in along the plane ({(B[0]-P[0])/IN:.1f} in deep horizontally, {(P[1]-B[1])/IN:.1f} in tall);"
          f" launch {LAUNCH_Z} m up; distances are turret -> opening centre")
    for ball, (m, r) in BALLS.items():
        print(f"\n== {ball} ==   cells: exit m/s  ±allowed%  (descent°)")
        angles = (50, 55, 60, 65, 70, 75, 80, 85)
        print("dist  | " + " | ".join(f"{a:>2d}°" + " " * 15 for a in angles))
        for d in (0.75, 1.0, 1.25, 1.5, 2.0, 2.5):
            D = d + (B[0] + P[0]) / 2
            cells = []
            for a in angles:
                w = window(D, a, m, r)
                cells.append(f"{w[0]:4.1f} ±{w[1]:4.1f}% ({w[2]:2.0f}°)" if w else "        —         ")
            print(f"{d:4.2f}m | " + " | ".join(cells))
        print("best angle per distance:", ", ".join(
            f"{d} m: {b[0]}° (±{b[1][1]:.1f}%)" for d in (0.75, 1.0, 1.25, 1.5, 2.0, 2.5) for b in [best_angle(d, m, r, range(50, 86, 2))] if b))
