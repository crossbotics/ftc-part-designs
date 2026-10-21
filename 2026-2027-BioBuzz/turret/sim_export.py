"""Field simulator data: the HIVE geometry and shot physics constants, straight from the checked Python models.

sim/sim_core.js runs the same shot check as approach_check.shots() in the browser. It reads every number from
out/sim/sim_geom.json, written here, so the simulator can't drift from the models the page's figures come from.
The shooting map (shoot_map.json) is copied in too, for the floor overlay and for sim/validate.mjs.

    python sim_export.py
"""
import json
import shutil

import approach_check as A
import camera_coverage as C
import entry_window as E
import turret as T
from shot_tolerance import CD, G, RHO

IN = A.IN
MOTOR_FREE_RPM = 6000.0                    # goBILDA 5203-2402-0001 Yellow Jacket, 1:1


def burst_table():
    """4-ball bursts, 0.25 s apart (sim_rapid.py): each shot's exit speed relative to the first, and when it fires, with
    and without the firing gate (the code waits until the flywheel is back within +/-1% before releasing a ball)."""
    import sim_rapid as SR
    out = {}
    for name, gate in (("gate", 0.01), ("nogate", None)):
        out[name] = {}
        for ball, seq in (("Pollen", "PPPP"), ("Nectar", "NNNN")):
            ev, _, _ = SR.run(0.25, [seq], gate=gate)
            e = [float(x) for x in ev[0]]
            tf = getattr(SR.run, "t_fire", None)
            times = [round(float(x), 3) for x in tf[0]] if (gate is not None and tf is not None) else [0.25 * i for i in range(4)]
            out[name][ball] = {"drop_pct": [round(100 * (x / e[0] - 1), 2) for x in e], "t": times}
    return out


def turret_table():
    """What the v7 turret actually launches (sim_feed.py), for the simulator's "from the turret" ball behaviour:
    for each ball and wear, on a grid of wheel surface speed x hood angle: exit speed, launch angle and backspin.
    Popped at 3 m/s, no idlers. Worn Pollen's exit speed stops rising near 7.9 m/s: above ~8 m/s of wheel speed the
    soft ball slips for its whole time in the wheels, so friction (not wheel speed) sets its speed."""
    import math
    import sim_feed as SF
    # Astra review: the first 0.5 m/s x 5 deg grid interpolated up to 1.4 deg / 1.2% off; this grid is 2x finer each way
    # and starts lower, so slow shots aren't extrapolated
    surf = [round(3.5 + 0.25 * i, 2) for i in range(27)]          # 3.5 .. 10.0 m/s (the motor tops out at 10.05)
    hoods = [2.5 * i for i in range(11)]                           # 0 .. 25 deg
    # Wear index 0 (new) .. 1 (medium) .. 2 (worn) = ball stiffness, straight between sim_feed's BALL_K values
    # (8000 / 4000 / 1500 N/m per contact). Astra round 3: blending just those three endpoints misstated in-between balls
    # (Nectar hit rate ~65% vs ~40% direct), so the table has 13 wear levels: a quarter apart from new to medium, an
    # eighth from medium to worn, where soft balls start to slip. Checked by sim/checks/wear_check.py + .mjs.
    widx = [0.25 * i for i in range(4)] + [1 + 0.125 * i for i in range(9)]    # finer from medium to worn, where soft balls start to slip
    stiff = lambda x: SF.BALL_K["new"] + (SF.BALL_K["mid"] - SF.BALL_K["new"]) * x if x <= 1 else SF.BALL_K["mid"] + (SF.BALL_K["worn"] - SF.BALL_K["mid"]) * (x - 1)
    names = {0.0: "new", 1.0: "mid", 2.0: "worn"}
    wears = [names.get(x, f"w{x:.3f}") for x in widx]
    for x, n in zip(widx, wears):
        SF.BALL_K.setdefault(n, stiff(x))
    keep = SF.WHEEL_SPEED
    out = {"surface": surf, "hood": hoods, "wear": wears, "wear_idx": widx, "wear_k": [SF.BALL_K[n] for n in wears], "balls": {}}
    try:
        for b in SF.BALLS:
            out["balls"][b.capitalize()] = {w: {"exit": [], "launch": [], "spin": [], "ok": []} for w in wears}
        for vs in surf:
            SF.WHEEL_SPEED = vs
            cases = [(b, 3.0, h, "none", w) for b in SF.BALLS for w in wears for h in hoods]
            _, st, ev, _ = SF.run(cases)
            spin = -SF.run.exit_spin * 60 / (2 * math.pi)             # + = backspin
            for i, c in enumerate(cases):
                t = out["balls"][c[0].capitalize()][c[4]]
                if c[2] == hoods[0]:
                    for k in t: t[k].append([])
                t["exit"][-1].append(round(float(ev[i]), 3)); t["launch"][-1].append(round(float(SF.run.exit_el[i]), 2))
                t["spin"][-1].append(round(float(spin[i]), 1)); t["ok"][-1].append(int(st[i] == 1))
    finally:
        SF.WHEEL_SPEED = keep
    return out


def exit_fit():
    """Ball exit speed vs flywheel surface speed, from the feed simulation (sim_feed.py): medium-worn balls, popped at
    3 m/s, hood 0/15/25 deg, surface 5-10 m/s. A straight line fits Pollen within ~0.02 m/s and Nectar within ~0.13 m/s
    (max_err is exported). Used only in the Simple model; the turret model uses turret_table()."""
    import numpy as np
    import sim_feed as SF
    keep = SF.WHEEL_SPEED
    fit = {}
    try:
        pts = {b: [] for b in SF.BALLS}
        for vs in (5.0, 6.0, 7.0, 8.5, 10.0):
            SF.WHEEL_SPEED = vs
            cases = [(b, 3.0, h, "none", "mid") for b in SF.BALLS for h in (0.0, 15.0, 25.0)]
            ev = SF.run(cases)[2]
            for c, e in zip(cases, ev):
                pts[c[0]].append((vs, float(e)))
        for b, pp in pts.items():
            X = np.array([[v, 1.0] for v, _ in pp]); y = np.array([e for _, e in pp])
            k, c = np.linalg.lstsq(X, y, rcond=None)[0]
            fit[b.capitalize()] = {"k": float(k), "c": float(c), "max_err": float(np.abs(X @ [k, c] - y).max())}
    finally:
        SF.WHEEL_SPEED = keep
    return fit


def main():
    out = T.OUT / "sim"
    out.mkdir(parents=True, exist_ok=True)
    geom = {
        "note": "metres; HIVE frame: x along the pivot axis (+x toward the other HIVE), u out toward our up-CELL, z up",
        "rho": RHO, "cd": CD, "g": G,
        "balls": {k: {"m": m, "r": r} for k, (m, r) in E.BALLS.items()},
        "launch_z": E.LAUNCH_Z,
        "launch": A.LAUNCH,
        "pivot_z": E.PIVOT_Z,
        "B": [0.0, float(E.B[0]), float(E.B[1])],
        "P": [0.0, float(E.P[0]), float(E.P[1])],
        "uc": float((E.B[0] + E.P[0]) / 2),            # opening centre, as shoot_map.UC: the turret aims here in plan
        "s0": A.S0, "a_in": A.A_IN, "a_out": A.A_OUT,
        "pent": A.PENT,
        "neigh_x": A.NEIGH_X, "frame_tube": A.FRAME_TUBE,
        "foot_u": A.FOOT_U, "foot_x_out": A.FOOT_X_OUT,
        "min_desc": A.MIN_DESC,
        "robot_top": A.ROBOT_TOP,
        # frame tubes and pivot blocks (a, b, radius) and the other HIVE's bar, straight from the model, so the page
        # can't drift from it
        "rods": [[list(map(float, a)), list(map(float, b)), float(rad)] for a, b, rad in A.frame_rods()[0]],
        "neigh_bars": {st: [[list(map(float, a)), list(map(float, b))] for a, b in A.neighbour(st)[1]] for st in ("down", "up")},
        "frame_floor_segs": [[list(map(float, a)), list(map(float, b))] for a, b in A.frame_floor_segments()],
        # flywheel drive, for the exit speed -> motor rpm readout
        "wheel_d": T.FW1_D / 1000, "belt_ratio": T.MAIN_MOTOR_PULLEY_T / T.MOTOR_PULLEY_T, "motor_free_rpm": MOTOR_FREE_RPM,
        "exit_fit": exit_fit(),
        "turret_shots": turret_table(),
        "burst": burst_table(),
        # camera (camera_coverage.sees_target): Limelight on the turret, yawed with it; our up-CELL's tag cluster
        "camera": {"h": 0.40, "tilt": float(T.LL_TILT), "tilt_range": list(T.LL_TILT_RANGE), "hfov": C.HFOV, "vfov": C.VFOV,
                   "hpix": C.HPIX, "max_inc": C.MAX_INC, "min_px": C.MIN_PX, "tag": C.TAG,
                   "target_p": list(map(float, C.target_tag()[0])), "target_n": list(map(float, C.target_tag()[1]))},
        "field_half": 72 * IN, "tile": 24 * IN,
        "rim_z": list(E.RIM_Z),
        "speeds": [4.5, 12.0, 0.03],
        "elev_step": 2.0,
    }
    # match-start balls from the field CAD (../field_cad/staging.py); robot preloads are left out
    st_f = out / "staging.json"
    if st_f.exists():
        geom["staging"] = [b for b in json.loads(st_f.read_text()) if b["where"] != "preload"]
    else:
        print("WARNING: out/sim/staging.json is missing (run ../field_cad/staging.py): 'Match start' will be empty")
    (out / "sim_geom.json").write_text(json.dumps(geom, indent=1))
    sm = T.OUT / "approach" / "shoot_map.json"
    if sm.exists():
        shutil.copy(sm, out / "shoot_map.json")
    print("wrote", out / "sim_geom.json")


if __name__ == "__main__":
    main()
