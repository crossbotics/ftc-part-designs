"""Data for the design-decision diagrams in viewer.html (written to out/viewer/diagrams.json;
pack_meshes.py merges it into turret_model.json).

  1. shots:     side-view trajectories into the up-facing HIVE cell (why the hood had to get steeper)
  2. tolerance: allowed exit-speed error vs hood angle at three distances (why steep is forgiving)
  3. height:    the 18 in start stack (hood parked steep) plus the in-match extra when the hood flattens (29 in limit)
  4. mechanism: key axis positions for the side-view schematic (from turret.py)

    python diagrams.py
"""
import json
import math
from pathlib import Path

import turret as T
from shot_tolerance import BALLS, TARGET_Z, solve_v, tol_at, RHO, CD, G
import entry_window as EW
LAUNCH_Z = EW.LAUNCH_Z

m, r = BALLS["Pollen"]   # Pollen and Nectar fly the same here (same drag per unit mass)

def path(v, elev, dt=4e-3, x_max=3.2):
    k = 0.5 * RHO * CD * math.pi * r * r / m
    x, z = 0.0, LAUNCH_Z
    vx, vz = v * math.cos(math.radians(elev)), v * math.sin(math.radians(elev))
    pts = [(0.0, LAUNCH_Z)]
    for _ in range(4000):
        s = math.hypot(vx, vz)
        vx -= k * s * vx * dt; vz -= (G + k * s * vz) * dt
        x += vx * dt; z += vz * dt
        pts.append((round(x, 3), round(z, 3)))
        if z < TARGET_Z - 0.25 and vz < 0 or x > x_max or z < 0:
            break
    return pts

POLLEN_HOOD = range(55, 81)   # v5: the Pollen hood's reach (55-80 deg); unreachable optima aren't shown

def main():
    out = {}
    # 1. shots at the REAL opening: best launch angle per distance (widest speed window, descending entry)
    shots = []
    mid_u = (EW.B[0] + EW.P[0]) / 2
    for d in (0.75, 1.5, 2.5):
        best = EW.best_angle(d, m, r, POLLEN_HOOD)
        e, w = best
        D = d + mid_u
        opening = [[round(D - EW.B[0], 3), round(EW.B[1], 3)], [round(D - EW.P[0], 3), round(EW.P[1], 3)]]
        shots.append({"label": f"{d} m · {e}°", "d": d, "elev": e, "v": round(w[0], 2), "tol": round(w[1], 1),
                      "desc": round(w[2]), "pts": path(w[0], e), "opening": opening})
    # 45 deg aimed at a cell 1.0 m away: no exit speed drops it in descending
    D = 1.0 + mid_u
    fail_v = next((v / 10 for v in range(40, 200) if max(z for _, z in path(v / 10, 45)) >= EW.P[1] + 0.02), 6.0)
    shots.append({"label": "45° from 1.0 m", "d": 1.0, "elev": 45, "v": fail_v, "pts": path(fail_v, 45), "fail": True,
                  "opening": [[round(D - EW.B[0], 3), round(EW.B[1], 3)], [round(D - EW.P[0], 3), round(EW.P[1], 3)]]})
    out["shots"] = {"launch_z": LAUNCH_Z, "target_z": TARGET_Z, "shots": shots}
    # 2. tolerance curves against the real opening (descending entries only)
    curves = []
    for d in (0.75, 1.25, 2.0):
        pts = []
        for e in range(50, 87, 2):
            w = EW.window(d + mid_u, e, m, r)
            if w:
                pts.append([e, round(w[1], 2), round(w[0], 2)])
        curves.append({"d": d, "pts": pts})
    out["tolerance"] = {"curves": curves, "hood": [55, 80], "window_cm": None}
    # 3. height: start configuration (18 in cube, hood parked steep) and in-match (29 in), from height_budget.py
    from height_budget import extremes, LIMIT, MATCH_LIMIT, MARGIN
    hp = T.build()[0]
    top, top_name, bot, bot_name = extremes((0.0,), hp)
    mtop, mtop_name, _, _ = extremes(None, hp)
    out["height"] = {"limit": LIMIT, "match_limit": MATCH_LIMIT, "margin": MARGIN, "turret_top": round(top, 1),
                     "match_top": round(mtop, 1), "turret_bottom": round(bot, 1),
                     "base_max": round(LIMIT - MARGIN - top, 1), "top_part": top_name, "match_top_part": mtop_name}
    # 5. camera coverage: which field spots see a HIVE tag cluster (HIVE tipped toward +X; camera_coverage.py)
    import numpy as np
    import camera_coverage as C
    import approach_check as A
    xs = np.linspace(-C.FIELD / 2 + C.ROBOT_HALF, C.FIELD / 2 - C.ROBOT_HALF, 33)
    grid = []
    for y in xs[::-1]:
        row = []
        for x in xs:
            # robots can drive under the HIVEs; only the frame's floor bars and legs keep the robot out
            # (approach_check.robot_fits). Here the pivots run along y, HIVEs at y = -/+0.324: its x = y + 0.324.
            if not A.robot_fits(y + A.NEIGH_X / 2, x):
                row.append(-1)
            else:
                row.append(1 if C.visible(np.array([x, y, 0.40]), 35, +1) > 0 else 0)
        grid.append(row)
    fav = [v for row in grid for x, v in zip(xs, row) if v >= 0 and x > 0]
    out["coverage"] = {"grid": grid, "extent": float(C.FIELD / 2 - C.ROBOT_HALF), "field": C.FIELD,
                       "favorable_pct": round(100 * sum(fav) / len(fav)), "lens_m": 0.40, "tilt": 35}
    # 5b. (v6.1) the camera on the field: can it read OUR up-CELL's tags? HIVE coordinates as the shoot map
    # (x toward the other HIVE, u toward our opening). One bit-string per camera tilt; '#' = robot can't be there.
    from camera_tilt_study import lens_fwd as _lf
    _lens_fwd = lambda tl: _lf(tl) / 1000          # v8 (Astra): the lens sits ~0.16 m ahead (v8.1) of the turret axis
    step, xs_, us_ = 0.08, None, None
    xs_ = [round(-1.32 + step * i, 3) for i in range(int((1.96 + 1.32) / step) + 1)]
    us_ = [round(-1.64 + step * i, 3) for i in range(int(3.28 / step) + 1)]
    tilts = {}
    for tl in range(20, 51, 5):
        tilts[str(tl)] = ["".join("#" if not A.robot_fits(x, u) else ("1" if C.sees_target(x, u, 0.40, tl, _lens_fwd(tl)) else "0")
                                  for x in xs_) for u in us_]
    sm = T.OUT / "approach" / "shoot_map.json"
    pct = {}
    if sm.exists():
        K = ("Pollen-down", "Pollen-up", "Nectar-down", "Nectar-up")
        rows = [r for r in json.loads(sm.read_text())["rows"] if not r.get("frame") and all(r[k] for k in K)
                and abs(r["x"] - A.NEIGH_X / 2) <= 72 * A.IN - C.ROBOT_HALF and abs(r["u"]) <= 72 * A.IN - C.ROBOT_HALF]
        for tl in range(20, 51, 5):
            pct[str(tl)] = round(100 * sum(C.sees_target(r["x"], r["u"], 0.40, tl, _lens_fwd(tl)) for r in rows) / len(rows))
    out["camera"] = {"xs": xs_, "us": us_, "step": step, "tilts": tilts, "scoring_pct": pct, "lens_m": 0.40, "default_tilt": int(T.LL_TILT)}
    # 8. (v6.1) the 3D viewer's shot animation plays the feed simulation's real path: mid-wear balls popped at 3.0 m/s,
    # hood 0..25 deg in 5 deg steps. Ball centre (mm, turret frame), arm swing off its stop (rad), until it's shot.
    import sim_feed as SF
    fcases = [(b_, 3.0, h, "none", "mid") for b_ in SF.BALLS for h in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0)]
    _, fst, fev, ftd = SF.run(fcases, record=50)
    feed = {}
    for i, c in enumerate(fcases):
        pts = [(t, p_[i], ph[i], st[i], vv[i]) for t, p_, ph, st, vv in SF.run.rec]
        k = next((j for j, q in enumerate(pts) if q[3] != 0), len(pts) - 1)
        pts = pts[:k + 1]
        feed[f"{c[0]}-{int(c[2])}"] = {
            "shot": bool(fst[i] == 1), "t": [round(q[0], 5) for q in pts],
            "y": [round(q[1][0] * 1000, 1) for q in pts], "z": [round(q[1][1] * 1000, 1) for q in pts],
            "arm": [round(q[2] - T.TH_P, 4) for q in pts],
            "v_exit": [round(float(pts[-1][4][0]), 2), round(float(pts[-1][4][1]), 2)], "speed": round(float(fev[i]), 2)}
    out["feed"] = feed
    # 6. the 45-degree rule, generalised: minimum-speed angle = 45 + half the angle up to the target
    dz = TARGET_Z - LAUNCH_Z
    rule = [[round(d, 2), round(45 + math.degrees(math.atan2(dz, d)) / 2, 2)] for d in [0.6 + 0.05 * i for i in range(41)]]
    drag = []
    for d in (0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5):
        vs = [(solve_v(d, e, m, r), e) for e in range(40, 88)]
        vs = [t for t in vs if t[0]]
        if vs:
            v, e = min(vs)
            drag.append([d, e, round(v, 2)])
    best_pts = []
    for d in (0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5):
        b_ = EW.best_angle(d, m, r, POLLEN_HOOD)
        if b_:
            best_pts.append([d, b_[0], round(b_[1][1], 1)])
    out["angles"] = {"rule": rule, "drag": drag, "best": best_pts, "hood": [55, 80], "dz": round(dz, 2)}
    # engineering-math worked examples (eng_math.py)
    import eng_math
    out["math"] = eng_math.compute()
    # 4. mechanism schematic positions (turret frame, mm)
    W = T.world
    out["mechanism"] = {
        "M": [T.FW_Y, T.FW_Z], "R1": T.R1, "R2": T.R2,
        "S_p": list(W(T.S_P)), "S_n": list(W(T.S_N)), "J": list(W(T.J)),
        "S_p_max": list(W(T.S_P, T.HOOD_ALPHA_MAX)), "J_max": list(W(T.J, T.HOOD_ALPHA_MAX)),
        "servo": list(T.SERVO_AXIS), "sector_r": T.SECTOR_RP, "motor1": list(T.FW_MOTOR),
        "motor2": list(T.FW_MOTOR2) if T.SECOND_MOTOR else None, "ll": list(T.LL_PIVOT),
        "cf_a": list(W(T.SPRING_A)), "cf_b": list(W(T.SPRING_B[0])),
        "deck_top": T.DECK_TOP, "pollen": T.POLLEN_D, "nectar": T.NECTAR_D, "compression": T.COMPRESSION,
        "front_y": T.FRONT_Y, "rear_y": T.REAR_Y, "exit_elev": [round(90 - T.psi_exit(T.S_P), 1),
                                                              round(90 - T.psi_exit(T.S_P) - T.HOOD_ALPHA_MAX, 1)],
    }
    # 7. where the robot can shoot from, relative to the HIVE (shoot_map.py is slow, so it's run on its own)
    sm = T.OUT / "approach" / "shoot_map.json"
    if sm.exists():
        out["shootmap"] = json.loads(sm.read_text())
    else:
        print("note: out/approach/shoot_map.json missing; run shoot_map.py for figure 7")
    (T.OUT / "viewer").mkdir(parents=True, exist_ok=True)
    (T.OUT / "viewer" / "diagrams.json").write_text(json.dumps(out))
    print("diagrams.json written:", {k: len(json.dumps(v)) for k, v in out.items()})

if __name__ == "__main__":
    main()
