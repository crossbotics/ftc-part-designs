"""v6.1: where on the field the robot can shoot from, relative to the HIVE (plan view).

Runs approach_check.shots() on a polar grid around our up-CELL's opening: distance from the opening centre (in plan)
and bearing (0 = square-on, + = the other HIVE's side). Each point gets, per ball and per state of the other HIVE,
the best launch angle and its speed room (the widest run of exit speeds that scores). The map shows the WORST of
the four (Pollen/Nectar x other HIVE down/up), so a green spot works for every ball and every HIVE state.
Writes out/approach/shoot_map.json, which diagrams.py adds to the page (figure 7).

    python shoot_map.py          # reuses out/approach/shoot_map.json points if the model's fingerprint matches;
                                 # --fresh recomputes all (~18 min, 11 cores); --adopt accepts an old file with no fingerprint
"""
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import approach_check as A
import entry_window as E
import turret as T

UC = (E.B[0] + E.P[0]) / 2                       # opening centre, horizontal distance from the pivot line
DISTS = [0.15, 0.2, 0.25, 0.33, 0.4] + [round(0.5 + 0.125 * i, 3) for i in range(17)]     # 0.15 .. 2.5 m
BEARS = list(range(-85, 86, 5))
SPEEDS = np.arange(4.5, 12.0, 0.03)


def inside_frame(rx, ru):
    return not A.robot_fits(rx, ru)


def point(args):
    d, psi = args
    ps = math.radians(psi)
    rx, ru = d * math.sin(ps), UC + d * math.cos(ps)
    if inside_frame(rx, ru):
        return {"d": d, "b": psi, "x": rx, "u": ru, "frame": True}
    res = {"d": d, "b": psi, "x": round(rx, 3), "u": round(ru, 3)}
    for ball in ("Pollen", "Nectar"):
        m, r = E.BALLS[ball]
        els = np.arange(A.LAUNCH[ball][0], A.LAUNCH[ball][1] + 0.1, 2.0)
        for state in ("down", "up"):
            why = A.shots((rx, ru), els, SPEEDS, r, m, state)
            best, free = None, 0.0
            for i, el in enumerate(els):
                w = A.window(why[i], SPEEDS)
                if w and (best is None or w[1] > best[1]):
                    best = (float(el), w[1])
                fw = A.window(np.where(why[i] >= 3, 0, why[i]), SPEEDS)
                free = max(free, fw[1] if fw else 0.0)
            res[f"{ball}-{state}"] = [best[0], round(best[1], 2)] if best else None
            res[f"{ball}-{state}-free"] = round(free, 2)
            if best is None:            # would a launch steeper than the hood allows have scored?
                steep = np.arange(A.LAUNCH[ball][1] + 1, 89.1, 1.0)
                w2 = A.shots((rx, ru), steep, SPEEDS, r, m, state)
                res[f"{ball}-{state}-steep"] = any(A.window(w2[i], SPEEDS) for i in range(len(steep)))
    return res


def fingerprint():
    """A hash of everything a shot result depends on: the full source of the model modules (so every constant, helper and
    derived geometry is covered: drag, gravity, frame, placement, ...) plus the ball sizes. The cache is reused only if
    it matches (Astra round 2: hashing selected functions and values missed CD, RHO, G, the floor bars and helpers)."""
    import hashlib
    from pathlib import Path
    here = Path(__file__).parent
    src = "".join((here / f).read_text() for f in ("approach_check.py", "entry_window.py", "shot_tolerance.py", "shoot_map.py"))
    return hashlib.sha1((src + repr((T.POLLEN_D, T.NECTAR_D))).encode()).hexdigest()[:16]

def main():
    pts = [(d, b) for d in DISTS for b in BEARS]
    old = {}
    f = T.OUT / "approach" / "shoot_map.json"
    fp = fingerprint()
    if f.exists() and "--fresh" not in sys.argv:        # same sim, same grid points: only compute what's new
        prev = json.loads(f.read_text())
        if prev.get("fingerprint", fp if "--adopt" in sys.argv else None) == fp:
            old = {(r["d"], r["b"]): r for r in prev["rows"] if not r.get("frame")}
        else:
            print("model changed since the cached map (fingerprint differs): recomputing every point")
    todo = [p for p in pts if p not in old]
    with ProcessPoolExecutor(max_workers=max(1, (os.cpu_count() or 4) - 3)) as ex:
        new = dict(zip(todo, ex.map(point, todo, chunksize=2)))
    rows = []
    for p in pts:
        r = old.get(p) or new[p]
        ps = math.radians(r["b"])                       # placement at the exact spot, as point() uses (rows store rounded x, u)
        if not r.get("frame") and inside_frame(r["d"] * math.sin(ps), UC + r["d"] * math.cos(ps)):
            r = {"d": r["d"], "b": r["b"], "x": r["x"], "u": r["u"], "frame": True}
        rows.append(r)
    # plan geometry for the drawing (metres; x along the pivot axis, u out toward our opening's side)
    hw = A.HW
    geom = {
        "opening": [[-hw, UC], [hw, UC]],            # where the pentagon's plane is, roughly (plan)
        "rim_u": [float(E.B[0]), float(E.P[0])],      # bottom / top rim horizontal distance from the pivot
        "pivot_x": [0.0, float(A.NEIGH_X)],
        "foot": [float(A.NEIGH_X / 2 - A.FOOT_X_OUT), float(A.NEIGH_X / 2 + A.FOOT_X_OUT), float(A.FOOT_U)],
        "frame_segs": [[list(map(float, a)), list(map(float, b))] for a, b in A.frame_floor_segments()],
        "robot_r": float(A.ROBOT_R + A.FRAME_TUBE),
        "cell_a": [float(A.A_IN), float(A.A_OUT)], "cell_hw": float(hw), "uc": float(UC),
    }
    out = {"dists": DISTS, "bears": BEARS, "rows": rows, "geom": geom, "fingerprint": fp}
    (T.OUT / "approach").mkdir(parents=True, exist_ok=True)
    (T.OUT / "approach" / "shoot_map.json").write_text(json.dumps(out))
    worst = []
    for r in rows:
        if r.get("frame"):
            continue
        vals = [r[k][1] if r[k] else 0.0 for k in ("Pollen-down", "Pollen-up", "Nectar-down", "Nectar-up")]
        worst.append((min(vals), r["d"], r["b"]))
    print("points:", len(rows), "| no score:", sum(1 for w in worst if w[0] == 0),
          "| blocked by structure:", sum(1 for r in rows if not r.get("frame") and any(
              (r[k][1] if r[k] else 0) < r[k + "-free"] - 0.2 for k in ("Pollen-down", "Pollen-up", "Nectar-down", "Nectar-up"))))
    for w in sorted(worst)[:12]:
        print(f"  worst: ±{w[0]:.1f}% at {w[1]} m, {w[2]:+d}°")


if __name__ == "__main__":
    main()
