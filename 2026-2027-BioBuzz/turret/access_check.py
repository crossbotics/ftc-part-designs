"""Tool access: can a hex key / nut driver actually reach every bolt head and nut?

For each fastener recorded by turret.add_fasteners (FASTENER_ACCESS), a tool-sized cylinder is pushed straight out
from the head along the bolt axis. We report how far it gets before touching another part (the "free length") and
what blocks it. A hex key needs about 25 mm of straight room (ball-end drivers can tilt ~25 degrees, so a little
less can work); a nut driver needs about 20 mm.

    python access_check.py            # default pose (hood steep, Pollen, camera at its default tilt)
"""
import math
import sys
from collections import defaultdict

from build123d import Plane, Solid, Vector

import turret as T
from check import pose, overlap

NEED_KEY, NEED_NUT = 25.0, 20.0
# v6: in-season ADJUSTMENTS must be reachable on the fully assembled turret (turning the turret is allowed)
ADJUST = {"csk_carrier_s_3": "stop cam, servo side (worn-ball squeeze)", "csk_carrier_m_3": "stop cam, motor side (worn-ball squeeze)",
          "fastener_m8_nut_0": "turret roller preload (slotted roller)", "fastener_m8_nut_3": "turret roller preload (slotted roller)",
          "fastener_ll_lock_s": "camera tilt lock", "fastener_ll_lock_m": "camera tilt lock",
          # v6 adjustable motor mount: loosen the 4 clamps, turn the jack screw, retighten
          "fastener_motor_clamp_0": "motor belt clamp bolt", "fastener_motor_clamp_1": "motor belt clamp bolt",
          "fastener_motor_clamp_lug_0": "motor belt clamp bolt (lug)", "fastener_motor_clamp_lug_1": "motor belt clamp bolt (lug)",
          "fastener_motor_jack": "motor belt jack screw",
          # v6 (Codex review): turret gear mesh / turret motor swap, through the deck access holes (turret turned)
          "fastener_turret_motor_0": "turret gear mesh (motor screw)", "fastener_turret_motor_1": "turret gear mesh (motor screw)",
          "fastener_turret_motor_2": "turret gear mesh (motor screw)"}
MAX_L = 60.0


def tool_for(name, dia):
    """(radius, needed length, description) of the tool for this fastener."""
    if "nut" in name:
        r = dia / 2 + 2.0                               # nut driver / socket wall
        return r, NEED_NUT, f"nut driver Ø{2 * r:.0f}"
    if name.startswith("csk_") or "stop_cam" in name or "robot_mount" in name or "motor_clamp" in name or "fw_motor" in name or "fastener_hub_" in name or "spring_end" in name or name.startswith("csk_arm"):
        return 1.7, NEED_KEY, "2.5 mm hex key"         # flat / button heads
    if "m8" in name:
        return 3.7, NEED_KEY, "6 mm hex key"
    if "switch" in name:
        return 1.6, NEED_KEY, "2.5 mm hex key"         # M3 socket head
    return 2.0, NEED_KEY, "3 mm hex key"               # M4 socket head


def inside_tool(own, other):
    """v8: a bolt's own end stands up inside the hollow nut driver that turns its nut, so it doesn't block it."""
    return own.startswith("fastener_m8_nut_") and other == own.replace("fastener_m8_nut_", "fastener_m8_tip_")


def free_length(shapes, own, start, direction, r):
    d = Vector(*direction); p0 = Vector(*start) + d * 0.3
    def blockers(L):
        tool = Solid.make_cylinder(r, L, Plane(origin=p0, z_dir=d))
        return [n for n, s in shapes.items() if n != own and not n.startswith("cable_loop") and not inside_tool(own, n)
                and overlap(tool, s) > 0.05]
    if not blockers(MAX_L):
        return MAX_L, []
    lo, hi = 0.0, MAX_L
    for _ in range(12):
        mid = (lo + hi) / 2
        if blockers(mid): hi = mid
        else: lo = mid
    return lo, blockers(min(MAX_L, hi + 2.0))


def main():
    parts, _ = T.build()
    shapes = pose(parts, 0.0, "pollen", T.LL_TILT)
    moving = {n for n, v in parts.items() if v[1] != "fixed"}
    rows, bad = [], 0
    for name, group, start, direction, dia in T.FASTENER_ACCESS:
        r, need, tool = tool_for(name, dia)
        L, who = free_length(shapes, name, start, direction, r)
        ok = L >= need
        bad += not ok
        rows.append((ok, name, tool, L, need, who))
    groups = defaultdict(list)
    for ok, name, tool, L, need, who in rows:
        key = name.rstrip("0123456789_-").replace("_m_", "_").replace("_s_", "_")
        groups[(ok, key, tool, tuple(sorted(set(w.rstrip("0123456789_") for w in who))))].append(L)
    for (ok, key, tool, who), Ls in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        print(f"{'OK     ' if ok else 'BLOCKED'} {key:32s} x{len(Ls):2d}  {tool:18s} free {min(Ls):5.1f}-{max(Ls):5.1f} mm"
              + (f"   blocked by: {', '.join(who)}" if who else ""))
    print(f"\n{len(rows)} fasteners checked, {bad} without straight tool access (structural ones are fine if the build order covers them)")
    # adjustments: must pass fully assembled; fixed-base ones may use any turret heading
    from build123d import Axis
    print("\nIN-SEASON ADJUSTMENTS (must be reachable fully assembled):")
    adj_bad = 0
    for name, group, start, direction, dia in T.FASTENER_ACCESS:
        if name not in ADJUST:
            continue
        r, need, tool = tool_for(name, dia)
        best, at = free_length(shapes, name, start, direction, r)[0], 0
        if group == "fixed" and best < need:
            for phi in range(15, 360, 15):
                rot = {n: (s_.rotate(Axis.Z, phi) if n in moving else s_) for n, s_ in shapes.items()}
                L, _ = free_length(rot, name, start, direction, r)
                if L > best:
                    best, at = L, phi
                if best >= need:
                    break
        ok = best >= need
        adj_bad += not ok
        print(f"  {'OK     ' if ok else 'BLOCKED'} {ADJUST[name]:42s} {tool:16s} free {best:5.1f} mm" + (f" (turret turned {at} deg)" if at else ""))
    print(f"{adj_bad} adjustment(s) blocked")
    return adj_bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
