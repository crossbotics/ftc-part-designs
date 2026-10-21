"""v2 checks: part-vs-part interference and ball-path sweeps across hood angle, ball size and camera tilt.

    python check.py
"""
import itertools
import math
import sys

import turret as T
from build123d import Axis, Plane, Pos, Sphere, Solid, Vector

X_AXIS = (1, 0, 0)

def pose(parts, alpha, ball, tilt):
    """Move carrier/arm/camera groups: hood alpha (deg), ball ('pollen'/'nectar', or the arm's swing off its stop in
    degrees), camera tilt (deg)."""
    M = (0, T.FW_Y, T.FW_Z)
    Jw = T.world(T.J)
    darm = swing_deg(ball)
    out = {}
    for n, (s, g, c, k, spin) in parts.items():
        if g == "arm":
            s = s.rotate(Axis((0, *Jw), X_AXIS), darm)
        if g in ("carrier", "arm"):
            s = s.rotate(Axis(M, X_AXIS), alpha)
        if g == "idler" and ball == "nectar" and T.IDLER_SLOTS:
            s = Pos(0, T.IDLER_SLOTS[0][1] - T.IDLER_SLOTS[0][0], 0) * s
        if g == "ll":
            s = s.rotate(Axis((0, *T.LL_PIVOT), X_AXIS), T.LL_TILT - tilt)
        out[n] = s
    out.update(springs(alpha, ball))
    return out

def swing_deg(ball):
    if isinstance(ball, (int, float)):
        return float(ball)
    return math.degrees(T.TH_N - T.TH_P) if ball == "nectar" else 0.0

ARM_OVERSWING = T.OPEN_STOP_EXTRA   # v6.1: the arm bumper stops the arm this far past the design Nectar position, so the
                                    # sweep runs to it. (Without it the geared-arm sim throws a Nectar's arm to 75-93 deg; the
                                    # upper-shaft spacers would reach the carrier slot ends ~7.7 deg past Nectar.)

def strip(p0, p1, w, t):
    """Flat spring strip from p0 to p1 (both at the same X): w wide along X, t thick. v6: the constant-force springs were
    rods 12.7 mm across, which is right along X but 12x too thick sideways (the real strip is ~0.3 mm; t = 1 is margin)."""
    p0, p1 = Vector(*p0), Vector(*p1)
    d = p1 - p0; L = d.length; z = d / L; x = Vector(1, 0, 0); y = z.cross(x)
    return Solid.make_box(w, t, L, Plane(origin=p0 - x * (w / 2) - y * (t / 2), x_dir=x, z_dir=z))

def rod(p0, p1, r):
    p0, p1 = Vector(*p0), Vector(*p1)
    return Solid.make_cylinder(r, (p1 - p0).length, Plane(origin=p0, z_dir=p1 - p0))

def springs(alpha, ball):
    """Springs as solid rods so they take part in the interference check.
    Arm: AndyMark constant-force strips (0.5in wide) from the coil drum to the arm post, both sides.
    Idlers: goBILDA 2915-0001-0003 (6.5mm OD) from the axle tab hook to the anchor, both sides."""
    darm = math.radians(swing_deg(ball))           # signed, like TH_N - TH_P
    J = T.J
    a = T.SPRING_A
    c, s_ = math.cos(darm), math.sin(darm)
    a = (J[0] + c * (a[0] - J[0]) - s_ * (a[1] - J[1]), J[1] + s_ * (a[0] - J[0]) + c * (a[1] - J[1]))
    out = {}
    for sx in (-1, 1):
        xs = sx * (T.X_DRV + 8.0)
        out[f"spring_arm_{sx}_ref"] = strip((xs, *T.world(a, alpha)), (xs, *T.world(T.SPRING_B[0], alpha)), 12.7, 1.0)
    dy = (T.IDLER_SLOTS[0][1] - T.IDLER_SLOTS[0][0]) if ball == "nectar" and T.IDLER_SLOTS else 0.0
    for i, (y_p, _, z) in enumerate(T.IDLER_SLOTS):
        ay, az = T.IDLER_ANCHORS[i]
        ang = math.atan2(az - z, ay - (y_p + dy))
        hook = (y_p + dy + 14 * math.cos(ang), z + 14 * math.sin(ang))
        for sx in (-1, 1):
            x = sx * (T.SIDE_X_OUT + 4.5)
            out[f"spring_idler_{i}_{sx}_ref"] = rod((x, *hook), (x, ay, az), 3.25)
    return out

# pairs that touch or pass through by design (shafts in bearings, belts on pulleys, bolted faces)
ALLOWED = [
    ("shaft_main_ref", "side_plate"), ("shaft_main_ref", "hood_carrier"), ("shaft_main_ref", "pulley_main"),
    ("shaft_main_ref", "flywheel_main"), ("shaft_jack_ref", "hood_carrier"), ("shaft_jack_ref", "swing_arm"),
    ("shaft_jack_ref", "pulley_jack"), ("shaft_jack_ref", "gear_jack"), ("shaft_upper_ref", "swing_arm"),
    ("shaft_upper_ref", "flywheel_upper"), ("shaft_upper_ref", "gear_upper"), ("belt_", "pulley_"),
    ("hood_servo_ref", "side_plate_servo"), ("limelight_ref", "ll_mount"),
    ("gear_jack_60t", "gear_upper_48t"), ("servo_pinion_ref", "hood_carrier_servo"),
    ("turret_pinion_ref", "turret_ring"), ("gb_gear64_ref", "gb_ring_ref"), ("cf_post", "swing_arm"), ("spring_arm", "cf_"),
    ("idler_axle", "idler_roller"), ("idler_axle", "idler_spool"), ("idler_axle", "side_plate"), ("idler_axle", "idler_spacer"),
    ("encoder_sleeve", "shaft_main"), ("encoder_sleeve", "encoder_v2"), ("encoder_sleeve", "encoder_bracket"),
    ("encoder_bracket", "encoder_v2"), ("encoder_bracket", "side_plate_motor"), ("encoder_sleeve", "collar_"),
    ("bearing_", "side_plate"), ("bearing_", "hood_carrier"), ("bearing_", "swing_arm"), ("bearing_", "shaft_"),
    ("bearing_608", "tire_"), ("bearing_idler", "idler_"), ("spacer_", "shaft_"), ("collar_", "shaft_"),
    ("bearing_", "spacer_"), ("bearing_", "collar_"), ("spacer_", "spacer_"), ("pulley_", "spacer_"), ("pulley_", "collar_"),
    ("gear_", "spacer_"), ("flywheel_", "spacer_"), ("bearing_", "pulley_"), ("bearing_", "gear_"),
    ("shaft_jack_ref", "belt_jack"), ("pulley_", "belt_"),
    ("spring_idler", "idler_spool"), ("spring_idler", "idler_anchor"),
    ("spring_idler", "idler_axle"),
    ("home_magnet", "cable_drum"), ("home_magnet", "home_flag"), ("home_flag", "deck"), ("home_switch", "cable_tray"), ("cable_drum", "deck"),
    ("cf_axle", "hood_carrier"), ("cf_axle", "cf_drum"),
    ("servo_pinion_ref", "hood_gear"), ("hood_gear", "hood_carrier_servo"), ("front_plate", "crossbar"), ("swing_arm", "arm_bumper"), ("arm_bumper_core", "arm_bumper_ring"), ("cf_coil", "cf_drum"),
    ("spring_arm", "fastener_spring_end"), ("spring_arm", "cf_clamp"), ("spring_arm", "heatset_cf_"),   # the strips' simple stand-in passes through the cf_ parts (allowed above) and so through their inserts   # v6.1: the strips are clamped under the screw + clamp plate and bend over its rounded edge
    ("hub_", "gear_"), ("hub_", "pulley_"), ("hub_", "flywheel_"), ("hub_", "shaft_"), ("hub_", "spacer_"), ("hub_", "collar_"),
]
def allowed(a, b):
    return any((a.startswith(x) and b.startswith(y)) or (b.startswith(x) and a.startswith(y)) for x, y in ALLOWED)

def overlap(a, b):
    A, B = a.bounding_box(), b.bounding_box()
    if (A.min.X > B.max.X or B.min.X > A.max.X or A.min.Y > B.max.Y or B.min.Y > A.max.Y
            or A.min.Z > B.max.Z or B.min.Z > A.max.Z):
        return 0.0
    r = a & b
    return r.volume if r else 0.0

def ball_path(alpha, ball):
    d = T.POLLEN_D if ball == "pollen" else T.NECTAR_D
    s = T.S_P if ball == "pollen" else T.S_N
    psi = math.radians(T.psi_exit(s, alpha))
    rc = T.R1 + d / 2 - T.COMPRESSION / 2
    y0 = T.FW_Y + T.R1 + d / 2
    pinch = (T.FW_Y + rc * math.cos(psi), T.FW_Z + rc * math.sin(psi))
    # v8.1GB: the indexer delivers the ball on the turret axis (y 0); it only moves over to ride the main wheel once it
    # touches it (a centred Nectar first touches at z ~59). The old straight column at y0 from z -60 put a Nectar 8.7 mm
    # off-centre through the bearing's 105 mm bore, which the real ball never does.
    rr = T.R1 + d / 2
    def y_at(z):
        dz = T.FW_Z - z
        return max(0.0, T.FW_Y + math.sqrt(rr * rr - dz * dz)) if abs(dz) < rr else 0.0
    pts = [(y_at(z) if y0 > 0 else y0 * (z >= T.FW_Z - 10), z) for z in range(-60, int(T.FW_Z), 10)]
    pts += [(y0 + (pinch[0] - y0) * k / 8, T.FW_Z + (pinch[1] - T.FW_Z) * k / 8) for k in range(1, 9)]
    ex = (-math.sin(psi), math.cos(psi))
    pts += [(pinch[0] + ex[0] * s_, pinch[1] + ex[1] * s_) for s_ in range(10, 300, 15)]
    return d, pts

def main():
    import datetime, hashlib
    src = hashlib.sha1(open(T.__file__, 'rb').read()).hexdigest()[:10]
    print(f'check.py run {datetime.datetime.now():%Y-%m-%d %H:%M}, turret.py sha1 {src}, springs {T.CF_SPRINGS_PER_SIDE}/side, bumper at +{T.OPEN_STOP_EXTRA} deg')
    parts, _ = T.build()
    problems = 0
    for alpha in (0.0, T.HOOD_ALPHA_MAX):
        for ball in ("pollen", "nectar"):
            for tilt in (T.LL_TILT_RANGE[0], T.LL_TILT_RANGE[1]):
                shapes = pose(parts, alpha, ball, tilt)
                for a, b in itertools.combinations(shapes, 2):
                    if allowed(a, b):
                        continue
                    v = overlap(shapes[a], shapes[b])
                    if v > 1.0:
                        problems += 1
                        print(f"alpha={alpha:4.1f} {ball:6s} tilt={tilt}: {a} x {b}: {v:.0f} mm3")
                if tilt != T.LL_TILT_RANGE[0]:
                    continue
                d, pts = ball_path(alpha, ball)
                hits = {}
                for (y, z) in pts:
                    sph = Pos(0, y, z) * Sphere(d / 2 - 0.5)
                    for n, s in shapes.items():
                        if n.startswith(("flywheel_main", "flywheel_upper", "idler_roller")):
                            continue
                        v = overlap(sph, s)
                        if v > 1.0:
                            hits[n] = max(hits.get(n, 0), round(v))
                if hits:
                    problems += 1
                print(f"ball path alpha={alpha:4.1f} {ball:6s}: {hits or 'clear'}")
    # v6 (open-item review): the ends of each motion aren't enough. Sweep the hood through the middle and the arm from its
    # stop to ARM_OVERSWING past the Nectar position (a hard-hit Nectar knocks it past, per the swing-arm feed sim).
    moving_groups = ("carrier", "arm")
    dn = math.degrees(abs(T.TH_N - T.TH_P))
    sgn = 1 if T.TH_N > T.TH_P else -1
    sweep_hits = {}
    for alpha in (0.0, T.HOOD_ALPHA_MAX / 2, T.HOOD_ALPHA_MAX):
        for sw in (0.0, dn / 3, 2 * dn / 3, dn, dn + ARM_OVERSWING / 2, dn + ARM_OVERSWING):
            if alpha in (0.0, T.HOOD_ALPHA_MAX) and sw in (0.0, dn):
                continue                                             # already checked above
            shapes = pose(parts, alpha, sgn * sw, T.LL_TILT)
            mov = [n for n in shapes if (n in parts and parts[n][1] in moving_groups) or n.startswith("spring_arm")]
            for a in mov:
                for b in shapes:
                    if b == a or (b in mov and b < a) or allowed(a, b):
                        continue
                    v = overlap(shapes[a], shapes[b])
                    if v > 1.0:
                        sweep_hits[(a, b)] = max(sweep_hits.get((a, b), (0, 0, 0)), (round(v), alpha, round(sw, 1)))
    print(f"hood 0/{T.HOOD_ALPHA_MAX / 2:.1f}/{T.HOOD_ALPHA_MAX:.0f} x arm swing 0..{dn + ARM_OVERSWING:.1f} deg (Nectar {dn:.1f} + {ARM_OVERSWING:.0f} overswing):",
          {k: f"{v[0]} mm3 at hood {v[1]}, swing {v[2]}" for k, v in sweep_hits.items()} or "clear")
    problems += len(sweep_hits)
    # v6.1: the bumper must actually catch the arm: 0.5 deg past the stop the arm overlaps it, and nothing else yet
    sh = pose(parts, 0.0, sgn * (dn + ARM_OVERSWING + 0.5), T.LL_TILT)
    arm_b = max(overlap(sh[f"swing_arm_{s_}"], sh[f"arm_bumper_ring_{b_}"]) for s_, b_ in (("servo", "s"), ("motor", "m")))
    others = {k: round(overlap(sh[k], sh[m_]), 1) for k in sh if k.startswith(("spacer_upper", "shaft_upper", "flywheel_upper"))
              for m_ in ("hood_carrier_servo", "hood_carrier_motor") if overlap(sh[k], sh[m_]) > 0.05}
    ok_b = arm_b > 0.05 and not others
    print(f"arm bumper: stops the arm at {dn + ARM_OVERSWING:.1f} deg (arm overlaps it by {arm_b:.1f} mm3 0.5 deg further); "
          f"upper shaft parts vs carriers there: {others or 'clear'} -> {'OK' if ok_b else 'PROBLEM'}")
    problems += 0 if ok_b else 1
    # meshing gears: the upper gear must find a clean phase at both arm positions
    ok = True
    for ball in ("pollen", "nectar"):
        sh = pose(parts, 0.0, ball, T.LL_TILT)
        Sw = T.world(T.S_P if ball == "pollen" else T.S_N)
        pitch = 360 / T.SYNC_TEETH_S
        best = min(overlap(sh["gear_jack_60t"], sh["gear_upper_48t"].rotate(Axis((0, *Sw), X_AXIS), ph))
                   for ph in [i * pitch / 48 for i in range(48)])
        ok &= best < 0.5
    print("sync gears mesh cleanly:", ok)
    # retention: with the ball centred on each idler's line, it must touch the main wheel and the idler
    for ball in ("pollen", "nectar"):
        sh = pose(parts, 0.0, ball, T.LL_TILT)
        d = T.POLLEN_D if ball == "pollen" else T.NECTAR_D
        for i, psi in enumerate(T.IDLER_PSI if T.IDLERS else []):
            a = math.radians(psi)
            rc = T.R1 + d / 2 - 2.0
            sph = Pos(0, T.FW_Y + rc * math.cos(a), T.FW_Z + rc * math.sin(a)) * Sphere(d / 2)
            w, r = overlap(sph, sh["flywheel_main_ref"]), overlap(sph, sh[f"idler_roller_{i}"])
            held = w > 1 and r > 1
            ok &= held
            print(f"retention {ball:6s} idler {i} ({psi:+.0f} deg): wheel {w:.0f} mm3, roller {r:.0f} mm3 -> {'held' if held else 'NOT HELD'}")
    problems += 0 if ok else 1
    # turret rotation: everything that turns vs everything fixed, across the soft-limit travel
    # v8: always run (it was inside the cable-tray branch, so a build without the tray skipped it). The cut-off height is
    # now the top of the tallest fixed part above the base plate, so it fits any cable method.
    fixed = {n: v[0] for n, v in parts.items() if v[1] == "fixed" and not n.startswith("cable_loop")}
    moving = {n: v[0] for n, v in parts.items() if v[1] != "fixed"}
    z_cut = max(sh.bounding_box().max.Z for sh in fixed.values()) + 2
    hits = {}
    for phi in range(-int(T.CABLE_TRAVEL), int(T.CABLE_TRAVEL) + 1, 10):
        for n, sh in moving.items():
            bb = sh.bounding_box()
            if bb.min.Z > z_cut:                     # above every fixed part: can't reach them
                continue
            r = sh.rotate(Axis.Z, phi)
            for m, fs in fixed.items():
                if allowed(n, m):
                    continue
                v = overlap(r, fs)
                if v > 1.0:
                    hits[(n, m)] = max(hits.get((n, m), 0), round(v))
    print(f"turret rotation +/-{T.CABLE_TRAVEL:.0f} deg vs fixed parts (below z {z_cut:.0f}): {hits or 'clear'}")
    problems += len(hits)
    if getattr(T, "CABLES", False):
        # v6 (open-item review): the cable ribbon moves as the turret turns; check it at every heading against the parts
        # that turn (rotated) and the fixed ones. It runs 0.5 mm off the drum and tray walls by design.
        low_moving = {n: s for n, s in moving.items() if s.bounding_box().min.Z <= T.TRAY_H + 2}
        rhits = {}
        for phi in range(-int(T.CABLE_TRAVEL), int(T.CABLE_TRAVEL) + 1, 10):
            rib = T.cable_ribbon(phi)
            for n, s in list(low_moving.items()) + list(fixed.items()):
                s2 = s.rotate(Axis.Z, phi) if n in low_moving else s
                v = overlap(rib, s2)
                if v > 1.0:
                    rhits[n] = max(rhits.get(n, (0, 0)), (round(v), phi))
        print(f"cable ribbon at every heading +/-{T.CABLE_TRAVEL:.0f} deg: " +
              (str({k: f"{v[0]} mm3 at {v[1]} deg" for k, v in rhits.items()}) if rhits else "clear"))
        problems += len(rhits)
        # the home magnet must sit within the switch's 5 mm side range at home, and nowhere near it at +/-20 deg
    if "home_magnet_ref" in parts and "home_switch_ref" in parts:
        import math as _m
        mag = parts["home_magnet_ref"][0].bounding_box().center()
        sw = parts["home_switch_ref"][0].bounding_box().center()
        sense_r = T.SW_IN_R + T.SW_DIMS[1] if getattr(T, "HOME_FLAG", False) else T.SW_OUT_R
        gap = _m.hypot(mag.X, mag.Y) - sense_r - (T.MAG_DIMS[1] - 0.2) / 2
        print(f"home magnet to switch's sensing side, radial gap {gap:.1f} mm (side trigger range 5 mm), "
              f"magnet {_m.degrees(_m.atan2(mag.Y, mag.X)):.1f} deg, switch {_m.degrees(_m.atan2(sw.Y, sw.X)):.1f} deg")
        if not 0 < gap < 5:
            print("  -> PROBLEM: the magnet is outside the switch's range"); problems += 1
        da = abs(((_m.degrees(_m.atan2(mag.Y, mag.X)) - _m.degrees(_m.atan2(sw.Y, sw.X))) + 180) % 360 - 180)
        dz = abs(mag.Z - (T.switch_base_z() + 9.0)) if hasattr(T, "switch_base_z") else 0.0   # switch's sensor ~9 mm up
        if da > 2.0 or dz > 4.0:
            print(f"  -> PROBLEM: magnet off the switch's sensor at home (angle {da:.1f} deg, height {dz:.1f} mm)"); problems += 1
        mag_s = parts["home_magnet_ref"][0]
        far = min(mag_s.rotate(Axis.Z, ph).distance_to(parts["home_switch_ref"][0]) for ph in (-20, 20))
        print(f"  magnet to switch at +/-20 deg: {far:.1f} mm (must be well outside the 5 mm range)")
        if far < 10.0:
            print("  -> PROBLEM: the magnet is still near the switch 20 deg from home"); problems += 1
    print("PROBLEMS:", problems)
    return problems

if __name__ == "__main__":
    sys.exit(1 if main() else 0)
