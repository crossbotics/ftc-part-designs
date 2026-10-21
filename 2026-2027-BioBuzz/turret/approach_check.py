"""v6.1: angled approaches and HIVE-structure obstructions, in 3D.

entry_window.py judges shots square-on to the up-CELL, in 2D. This script puts the robot at a bearing to the CELL
and checks every candidate shot against the whole HIVE Structure:
  * the target opening as a 3D pentagon rim (20 in wide, 7.61 in straight sides, 14 in to the peak; Figure 9-11).
    The ball must pass every rim edge by one ball radius and cross the opening plane inside the pentagon;
  * the other alliance's HIVE, whose centre is 25.5 in along the pivot axis (Figure 9-10), in either state.
    "down": tipped opposite to ours, so its DOWN-cell hangs on our side, 30.6 in off the tiles at its lowest
    (Team Update 01). "up": tipped the same way, so its UP-cell sits beside our target;
  * each HIVE's connecting bar, both A-frame legs of each frame triangle, and the crossbar at the pivots.
Structure solids are inflated by about 1 in for the frame tubes, and the ball must stay one radius clear of them.
The robot can't be inside the frame's base (38.95 in deep, so 19.5 in each side of the pivot), plus half an
18 in robot.

Coordinates (metres): x along the pivot axis (+x toward the other HIVE); u horizontal, out from the pivot on the
side our up-CELL faces; z up. Bearing psi: 0 = square-on; + = coming from the other HIVE's side.

    python approach_check.py
"""
import math

import numpy as np
import shapely
from shapely.geometry import Point, Polygon

import entry_window as E
import turret as T
from shot_tolerance import CD, G, RHO

IN = 0.0254
C30, S30 = math.cos(math.radians(30)), 0.5
PIV = np.array([0.0, 0.0, E.PIVOT_Z])
AX = np.array([0.0, C30, S30])            # our HIVE's axis, toward our up-CELL
NUP = np.array([0.0, -S30, C30])          # "up" across the CELLs (= along the opening plane, B -> P)
EX = np.array([1.0, 0.0, 0.0])
NORM = np.array([0.0, C30, S30])          # opening normal (outward)
B3 = np.array([0.0, E.B[0], E.B[1]])
A_IN, A_OUT = 9.42 * IN, E.CELL_OUT       # CELL inner end (CAD back skin 239.2 mm) and the opening plane
S0 = float((B3 - PIV) @ NUP)              # CELL floor offset from the pivot line (-35.7 mm)
HW, SIDE, TOP, CH, PH = E.HALF_W, E.SIDE, E.OPEN_H, E.CHAMFER, E.PEAK_HALF
# opening (x, s from the bottom rim), inner edges, counter-clockwise; from the field CAD (entry_window constants)
PENT = [(-HW + CH, 0), (HW - CH, 0), (HW, CH), (HW, SIDE), (PH, TOP), (-PH, TOP), (-HW, SIDE), (-HW, CH)]
PENT_POLY = Polygon(PENT)
NEIGH_X = 25.5 * IN                        # HIVE centres (CAD: +/-323.85 mm)
FRAME_TUBE = 1.0 * IN                      # radius allowance for frame tubes / CELL outlines
FOOT_U = 0.4946                            # floor bars run +/-494.6 mm along the depth (CAD; manual 38.95 in)
FOOT_X_OUT = NEIGH_X / 2 + 0.2791          # floor bar: middle of its 577.8-628.1 mm footprint (602.95 mm; Astra review: the
                                           # vertex-weighted 609.4 left the inner edge 6 mm uncovered); FRAME_TUBE covers +/-25.4
# frame tubes from the field CAD (field_cad/frame_cad.py), in HIVE-frame offsets from each HIVE centre:
LEG_FOOT = (-0.27895, 0.4682, 0.0208)      # outboard of the HIVE centre, +/-u, height (tube centre-line)
LEG_APEX = (0.00295, 0.0150, 1.0387)       # the two legs meet just under the pivot, not at it
CROSSBAR = (0.02845, 0.62215, 1.0528)      # x from, x to (our HIVE at 0), height
PIVOT_BLOCKS = [((-0.02265, 0.0, 1.1044), (0.00785, 0.0, 1.0282), 0.040),    # A-frame top corner (82 x 78 mm block)
                ((0.0364, 0.0, 1.0805), (0.0364, 0.0, 1.1055), 0.025)]       # axle holder (41 x 33 x 65 mm)
MIN_DESC = 10.0                            # deg, as entry_window.window()

# opening rim: 5 edges in 3D
RIM = [(B3 + x0 * EX + s0 * NUP, B3 + x1 * EX + s1 * NUP) for (x0, s0), (x1, s1) in zip(PENT, PENT[1:] + PENT[:1])]


def cell_prism(axis, up, xc):
    """(axis, up, lateral centre, cross-section polygon inflated for the frame, a-range): a CELL as a prism."""
    sec = Polygon([(x, S0 + s) for x, s in PENT]).buffer(FRAME_TUBE, join_style=2)
    return (np.asarray(axis), np.asarray(up), xc, sec, (A_IN - 0.5 * IN, A_OUT + 0.5 * IN))


def neighbour(state):
    """Solids of the other HIVE (centre x = NEIGH_X)."""
    if state == "down":   # tipped opposite: its axis toward us is (0, cos30, -sin30); its "up" is (0, sin30, cos30)
        ax, up = np.array([0.0, C30, -S30]), np.array([0.0, S30, C30])
    else:                 # tipped like ours
        ax, up = AX, NUP
    prisms = [cell_prism(ax, up, NEIGH_X), cell_prism(-ax, up, NEIGH_X)]
    bar = (PIV + NEIGH_X * EX - A_OUT * ax + (S0 - 1 * IN) * up, PIV + NEIGH_X * EX + A_OUT * ax + (S0 - 1 * IN) * up)
    return prisms, [bar]


def frame_rods():
    """Our HIVE's bar and our own down-CELL, the two frame triangles, the crossbar and the blocks under each pivot.
    Field CAD: tube centre-lines, legs meeting 1039 mm up, crossbar 1053 mm up. Returns ([(a, b, radius)], prisms)."""
    rods = [(PIV - A_OUT * AX + (S0 - 1 * IN) * NUP, PIV + (A_OUT - FRAME_TUBE) * AX + (S0 - 1 * IN) * NUP, FRAME_TUBE)]
    for xc, sg in ((0.0, 1), (NEIGH_X, -1)):        # sg: +1 our HIVE's triangle (outboard = -x), -1 the other one's
        for su in (1, -1):
            foot = np.array([xc + sg * LEG_FOOT[0], su * LEG_FOOT[1], LEG_FOOT[2]])
            apex = np.array([xc + sg * LEG_APEX[0], su * LEG_APEX[1], LEG_APEX[2]])
            rods.append((apex, foot, FRAME_TUBE))
        for a, b, rad in PIVOT_BLOCKS:
            rods.append((np.array([xc + sg * a[0], a[1], a[2]]), np.array([xc + sg * b[0], b[1], b[2]]), rad))
    rods.append((np.array([CROSSBAR[0], 0, CROSSBAR[2]]), np.array([CROSSBAR[1], 0, CROSSBAR[2]]), FRAME_TUBE))
    return rods, [cell_prism(-AX, NUP, 0.0)]


UP_SEC = Polygon([(x, S0 + s) for x, s in PENT]).buffer(FRAME_TUBE, join_style=2)


def in_up_body(p, r):
    """True where the ball overlaps our own up-CELL's walls or is inside it, anywhere behind the opening plane.
    (A ball only gets there legally through the opening, which shots() tracks as `entered`.)"""
    q = p - PIV
    a, s, x = q @ AX, q @ NUP, q[..., 0]
    grown = UP_SEC.buffer(r)
    minx, mins, maxx, maxs = grown.bounds
    cand = (a > A_IN - 0.5 * IN - r) & (a < A_OUT) & (x > minx) & (x < maxx) & (s > mins) & (s < maxs)
    out = np.zeros(p.shape[:-1], bool)
    if cand.any():
        out[cand] = shapely.contains_xy(grown, x[cand], s[cand])
    return out


def seg_dist(p, a, b):
    """Distance from points p (..., 3) to segment ab."""
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0, 1)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1)


def in_prism(p, prism, r):
    """True where the ball (centre p, radius r) overlaps the prism (conservative on the corners)."""
    ax, up, xc, sec, (a0, a1) = prism
    q = p - PIV
    a, s, x = q @ ax, q @ up, q[..., 0] - xc
    grown = sec.buffer(r)
    minx, mins, maxx, maxs = grown.bounds
    cand = (a > a0 - r) & (a < a1 + r) & (x > minx) & (x < maxx) & (s > mins) & (s < maxs)
    out = np.zeros(p.shape[:-1], bool)
    if cand.any():
        out[cand] = shapely.contains_xy(grown, x[cand], s[cand])
    return out


def shots(robot_xu, elevs, speeds, r, m, state, dt=1e-3, tmax=2.0):
    """Every (elevation, speed) from robot_xu aimed (in plan) at the opening centre. Returns (ok, why) arrays:
    why 0 = in, 1 = misses the opening, 2 = touches the rim, 3/4 = WOULD have gone in but first hits the other HIVE /
    the frame or our own HIVE's bar or down-CELL."""
    uc = (E.B[0] + E.P[0]) / 2
    dplan = np.array([0.0 - robot_xu[0], uc - robot_xu[1]]); dplan /= np.linalg.norm(dplan)
    E_, V_ = np.meshgrid(np.radians(elevs), speeds, indexing="ij")
    n = E_.size
    p = np.tile([robot_xu[0], robot_xu[1], E.LAUNCH_Z], (n, 1)).astype(float)
    vh, vz = (V_ * np.cos(E_)).ravel(), (V_ * np.sin(E_)).ravel()
    v = np.stack([vh * dplan[0], vh * dplan[1], vz], axis=1)
    k = 0.5 * RHO * CD * math.pi * r * r / m
    why = np.full(n, -1)
    rim_min = np.full(n, 9.0)
    nprism, nbars = neighbour(state)
    rods, own = frame_rods()
    prev = (p - B3) @ NORM
    live = np.ones(n, bool)
    blocked = np.zeros(n, int)
    entered = np.zeros(n, bool)
    for _ in range(int(tmax / dt)):
        sp = np.linalg.norm(v, axis=1)
        v[:, :2] -= (k * sp)[:, None] * v[:, :2] * dt
        v[:, 2] -= (G + k * sp * v[:, 2]) * dt
        p = p + v * dt
        f = (p - B3) @ NORM
        L = live
        # rim clearance, until the centre is a full radius inside the plane
        for a, b in RIM:
            rim_min[L] = np.minimum(rim_min[L], seg_dist(p[L], a, b))
        hitN = np.zeros(n, bool); hitF = np.zeros(n, bool)
        for pr in nprism:
            hitN[L] |= in_prism(p[L], pr, r)
        for a, b in nbars:
            hitN[L] |= seg_dist(p[L], a, b) < r + FRAME_TUBE
        for a, b, rad in rods:
            hitF[L] |= seg_dist(p[L], a, b) < r + rad
        for pr in own:
            hitF[L] |= in_prism(p[L], pr, r)
        # remember the first thing hit, but keep flying: an obstruction only counts if the shot would have scored
        out = L & ~entered                      # once through the opening the ball is in the CELL (floor, walls)
        blocked[out & hitN & (blocked == 0)] = 3
        blocked[out & hitF & (blocked == 0)] = 4
        cross = L & (prev > 0) & (f <= 0)
        q = p - B3
        inside = np.zeros(n, bool)
        if cross.any():
            inside[cross] = shapely.contains_xy(PENT_POLY, q[cross, 0], q[cross] @ NUP)
        why[cross & ~inside & (why < 0)] = 1
        # same rule as entry_window: entries still rising or flatter than 10 deg head for the roof/back wall
        desc = np.degrees(np.arctan2(-v[:, 2], np.hypot(v[:, 0], v[:, 1])))
        why[cross & inside & (desc < MIN_DESC) & (why < 0)] = 1
        entered |= cross & inside & (desc >= MIN_DESC)
        # field CAD round: our own up-CELL is solid from outside. A ball can't reach the opening plane from behind by
        # passing through the CELL (the old model let a ball fly in through the back and drop back in the front).
        body = L & ~entered
        if body.any():
            body[body] = in_up_body(p[body], r)
        blocked[body & (blocked == 0)] = 4
        # a shot only counts once it has crossed the opening from outside (the robot can start below the
        # opening's extended plane at close range)
        done_in = L & entered & (f < -r) & (why < 0)
        why[done_in & (rim_min < r)] = 2
        why[done_in & (rim_min >= r)] = 0
        live = why < 0
        live &= ~((v[:, 2] < 0) & (p[:, 2] < E.B[1] - 0.3)) & (np.abs(p[:, 0]) < 6) & (np.abs(p[:, 1]) < 6)
        prev = f
        if not live.any():
            break
    why[why < 0] = 1
    why = np.where((why == 0) & (blocked > 0), blocked, why)
    return why.reshape(E_.shape)


def window(why_row, speeds):
    """Widest contiguous run of good speeds -> (mid, +/-%), or None."""
    good = why_row == 0
    best, cur = None, None
    for i, g in enumerate(good):
        if g:
            cur = (cur[0], i) if cur else (i, i)
            if best is None or cur[1] - cur[0] > best[1] - best[0]:
                best = cur
        else:
            cur = None
    if not best or best[1] == best[0]:
        return None
    lo, hi = speeds[best[0]], speeds[best[1]]
    return (lo + hi) / 2, 100 * (hi - lo) / (hi + lo)


LAUNCH = {"Pollen": (55.0, 84.0), "Nectar": (58.0, 85.0)}   # real launch elevations (sim_feed: hood range + offsets)
MIN_U = FOOT_U + 9 * IN                                      # square-on, the robot centre can't be closer than this
# Where the robot can drive (manual Figures 9-7, 9-8, rule G409): the frame is two triangles, one at each end of the
# 49.46 in width, joined only by the crossbar at the top. On the floor there is just each triangle's bottom bar
# (along the 38.95 in depth, at the width ends); robots can drive between them, under the HIVEs. The legs lean in
# from the feet to the apexes, so they also reach in over a robot, up to the robot's height. Our robot is at most
# ROBOT_TOP tall in a match (base plate <= 241.2 mm + turret 244.4 mm with the hood flat; height_budget.py). The
# lowest HIVE part, a down-CELL at 30.6 in (TU01), clears it. The robot counts as a 9 in-radius circle (it can turn
# a flat side to the leg) plus FRAME_TUBE for the tube and its brackets.
ROBOT_TOP = 0.486
ROBOT_R = 9 * IN


def frame_floor_segments():
    """Plan-view segments (x, u) of the frame parts at or below ROBOT_TOP: two floor bars and four leg stubs
    (field CAD centre-lines; the floor bars are 50 mm wide, so FRAME_TUBE covers them)."""
    segs = []
    k = (ROBOT_TOP - LEG_FOOT[2]) / (LEG_APEX[2] - LEG_FOOT[2])
    for xc, sg in ((0.0, 1), (NEIGH_X, -1)):
        xb = NEIGH_X / 2 - sg * FOOT_X_OUT
        segs.append(((xb, -FOOT_U), (xb, FOOT_U)))
        for su in (1, -1):
            f = (xc + sg * LEG_FOOT[0], su * LEG_FOOT[1]); t = (xc + sg * LEG_APEX[0], su * LEG_APEX[1])
            segs.append((f, (f[0] + k * (t[0] - f[0]), f[1] + k * (t[1] - f[1]))))
    return segs


def robot_fits(rx, ru):
    """True if the robot's centre can be at (rx, ru) without overlapping the frame."""
    p = np.array([rx, ru])
    for a, b in frame_floor_segments():
        a, b = np.array(a), np.array(b)
        t = np.clip((p - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
        if np.linalg.norm(p - (a + t * (b - a))) < ROBOT_R + FRAME_TUBE:
            return False
    return True


def main(balls=("Pollen", "Nectar"), states=("down", "up")):
    uc = (E.B[0] + E.P[0]) / 2
    speeds = np.arange(4.5, 12.0, 0.02)
    print(f"target CELL: rim {E.RIM_Z[0]/IN:.1f}-{E.RIM_Z[1]/IN:.1f} in; other HIVE at x = {NEIGH_X/IN:.1f} in")
    nd = neighbour("down")[0][0]
    ax, up, xc, sec, (a0, a1) = nd
    zs = [(PIV + a * ax + s * up)[2] for a in (a0, a1) for s in (sec.bounds[1], sec.bounds[3])]
    print(f"model check: other HIVE's down-CELL lowest point {min(zs)/IN:.1f} in (Team Update 01: 30.6 in)")
    print(f"closest robot centre to the pivot line (frame base + half robot): {MIN_U:.3f} m "
          f"= {MIN_U - uc:.2f} m from the opening centre, square-on")
    labels = {1: "miss", 2: "rim", 3: "other HIVE", 4: "frame"}
    for ball in balls:
        m, r = E.BALLS[ball]
        elevs = np.arange(LAUNCH[ball][0], LAUNCH[ball][1] + 0.1, 2.0)
        for state in states:
            print(f"\n== {ball}, other HIVE {state} ==  best launch angle and speed room (+/-%); \"-x.x what\" = speed room the "
                  f"structure takes away")
            print("dist  | " + " | ".join(f"{p:+4d}°".ljust(22) for p in (-60, -45, -30, -15, 0, 15, 30, 45, 60)))
            for d in (0.5, 0.75, 1.0, 1.5, 2.0, 2.5):
                cells = []
                for psi in (-60, -45, -30, -15, 0, 15, 30, 45, 60):
                    ps = math.radians(psi)
                    rx, ru = d * math.sin(ps), uc + d * math.cos(ps)
                    if not robot_fits(rx, ru):
                        cells.append("(hits the frame)"); continue
                    why = shots((rx, ru), elevs, speeds, r, m, state)
                    best = None
                    for i, el in enumerate(elevs):
                        w = window(why[i], speeds)
                        if w and (best is None or w[1] > best[1]):
                            best = (el, w[1], i)
                    if best is None:
                        blk = [labels[c] for c in (3, 4) if (why == c).any()]
                        cells.append("none" + (f" ({','.join(blk)})" if blk else "")); continue
                    el, pct, i = best
                    # what the structure costs: best window if blocked shots had been allowed through
                    free = max((window(np.where(why[j] >= 3, 0, why[j]), speeds) or (0, 0))[1] for j in range(len(elevs)))
                    blk = sorted({labels[c] for c in (3, 4) if (why == c).any()}) if free - pct > 0.2 else []
                    cells.append(f"{el:3.0f}° ±{pct:3.1f}%" + (f" -{free - pct:.1f} {'/'.join(blk)}" if blk else ""))
                print(f"{d:4.2f}m | " + " | ".join(f"{c:<22}" for c in cells))


if __name__ == "__main__":
    import sys
    a = sys.argv[1:]
    main(*(([a[0]], [a[1]]) if len(a) == 2 else ()))
