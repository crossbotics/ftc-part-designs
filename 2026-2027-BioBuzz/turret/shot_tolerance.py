"""How much exit-speed error can a HIVE shot tolerate? Point-mass trajectories with quadratic drag.

Target: centre of the up-facing CELL opening, ~1.51 m above the tiles (manual 9.6: opening 53.5-65.6 in).
Launch: ~0.30 m above the tiles (robot deck ~0.13 m + turret pinch ~0.17 m) - adjust LAUNCH_Z.
Tolerance = speed error that moves the ball's crossing of the target height by +/- WINDOW (horizontal).
"""
import math
import numpy as np
import turret as T

RHO, CD, G = 1.2, 0.5, 9.81
BALLS = {"Pollen": (0.025, T.POLLEN_D / 2000), "Nectar": (0.041, T.NECTAR_D / 2000)}
LAUNCH_Z, TARGET_Z = 0.33, 1.51   # v5: launch height as entry_window.py
WINDOWS = (0.05, 0.075, 0.10)          # +/- m of landing error the opening can absorb (assumed)
ELEV = np.arange(35, 86, 1.0)          # search range (deg)
HOOD = (60, 85)   # v4 hood range (v3 was 41-69)

def x_at_target(v, elev, m, r, dt=5e-4):
    """Horizontal distance where the ball comes DOWN through TARGET_Z (None if it never gets there)."""
    k = 0.5 * RHO * CD * math.pi * r * r / m
    x, z = 0.0, LAUNCH_Z
    vx, vz = v * math.cos(math.radians(elev)), v * math.sin(math.radians(elev))
    while True:
        s = math.hypot(vx, vz)
        vx -= k * s * vx * dt; vz -= (G + k * s * vz) * dt
        xn, zn = x + vx * dt, z + vz * dt
        if vz < 0 and zn <= TARGET_Z <= z:
            return x + (xn - x) * (z - TARGET_Z) / (z - zn), math.degrees(math.atan2(-vz, vx))
        if zn < 0 or xn > 6:
            return None, None
        x, z = xn, zn

def solve_v(d, elev, m, r):
    """Smallest exit speed whose descending crossing of TARGET_Z is at distance d."""
    prev = None
    for v in np.arange(3.0, 20.0, 0.25):
        x, _ = x_at_target(v, elev, m, r)
        if x is not None and x >= d:
            if prev is None:
                return None              # already past d at the lowest speed that reaches the height: too close
            lo, hi = prev, v
            for _ in range(30):
                mid = (lo + hi) / 2
                xm, _ = x_at_target(mid, elev, m, r)
                if xm is None or xm < d: lo = mid
                else: hi = mid
            return hi
        prev = v if x is not None else None
    return None

def tol_at(d, e, m, r, window=0.075):
    v = solve_v(d, e, m, r)
    if v is None:
        return None
    xp, desc = x_at_target(v * 1.01, e, m, r); xm, _ = x_at_target(v * 0.99, e, m, r)
    if xp is None or xm is None:
        return None
    return v, window / abs((xp - xm) / (0.02 * v)) / v * 100, desc

if __name__ == "__main__":
    m, r = BALLS["Pollen"]   # Pollen and Nectar have the same drag-per-mass here, so their paths match
    print(f"launch {LAUNCH_Z} m -> target {TARGET_Z} m, drag Cd {CD}. Cells: exit speed m/s / allowed speed error for +/-7.5 cm landing")
    angles = (45, 50, 55, 60, 65, 69)
    print("dist  | " + " | ".join(f"{a:>2d} deg      " for a in angles))
    for d in (0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0):
        cells = []
        for e in angles:
            t = tol_at(d, e, m, r)
            cells.append(f"{t[0]:4.1f} / {t[1]:4.1f}%" if t else "    —       ")
        print(f"{d:4.2f}m | " + " | ".join(cells))
    print("— = can't come DOWN into the opening at that angle (too close) or out of reach")
