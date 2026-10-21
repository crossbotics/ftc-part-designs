"""Height budget: how tall is the turret, and what does that leave for the rest of the robot?

Competition Manual (TU01) R102: the STARTING CONFIGURATION must fit an 18 in cube (457.2 mm). R105.A: after the
match starts the robot may expand, but must stay inside 18 x 24 in and 29 in (736.6 mm) tall, limited mechanically.
So the 18 in limit only applies to the start pose: the hood starts parked at its steep end (alpha 0, held by the
servo at init, which R103 allows). The flat-hood pose only has to fit 29 in. Heights are from the robot surface the turret bolts to (its underside).
    python height_budget.py [robot_surface_above_floor_mm]
"""
import sys
import turret as T
from check import pose

LIMIT = 457.2                     # R102 starting configuration
MATCH_LIMIT = 736.6               # R105.A in-match height (29 in)
MARGIN = 6.0                      # keep a few mm for bolt heads, tolerance, a flexing plate
ROBOT_Z = 0.0 if getattr(T, "GB_TURRET", False) else -T.T   # the robot surface under the turret, in turret z

def extremes(alphas=None, parts=None):
    parts = parts or T.build()[0]
    top, top_name, bot, bot_name = -1e9, "", 1e9, ""
    for alpha in (alphas or (0.0, T.HOOD_ALPHA_MAX)):
        for ball in ("pollen", "nectar"):
            for tilt in T.LL_TILT_RANGE:
                for n, s in pose(parts, alpha, ball, tilt).items():
                    bb = s.bounding_box()
                    if bb.max.Z > top: top, top_name = bb.max.Z, f"{n} (hood {alpha:.0f}, {ball}, camera {tilt})"
                    if bb.min.Z < bot: bot, bot_name = bb.min.Z, n
    # 2026-10-04 (user: "the intent is to show how much height above the body is allowed"): heights are measured from
    # the robot surface the turret bolts to (its underside): the base plate's bottom, or the goBILDA bearing's bottom
    return top - ROBOT_Z, top_name, bot - ROBOT_Z, bot_name

if __name__ == "__main__":
    parts = T.build()[0]
    top, top_name, bot, bot_name = extremes((0.0,), parts)
    mtop, mtop_name, _, _ = extremes(None, parts)
    print(f"start config top: {top:6.1f} mm above the robot surface   <- {top_name}  (hood parked steep)")
    print(f"in-match top    : {mtop:6.1f} mm                            <- {mtop_name}")
    print(f"lowest part     : {bot:6.1f} mm (below the robot surface) <- {bot_name}")
    room = LIMIT - MARGIN - top
    print(f"=> the robot surface under the turret can be at most {room:.0f} mm ({room/25.4:.2f} in) above the floor (18 in start cube)")
    print(f"   in the match that makes the robot at most {room + mtop:.0f} mm ({(room + mtop)/25.4:.1f} in) tall; limit 29 in")
    print(f"   everything under it (drivetrain, intake, indexer, the turret motor hanging {-bot:.0f} mm down) must fit in that")
    if len(sys.argv) > 1:
        h = float(sys.argv[1])
        print(f"with the robot surface at {h:.0f} mm: start {h + top:.0f} mm, in-match {h + mtop:.0f} mm "
              f"({'OK' if h + top <= LIMIT - MARGIN and h + mtop <= MATCH_LIMIT - MARGIN else 'TOO TALL'})")
