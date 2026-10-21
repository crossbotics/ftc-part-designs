"""v6.2: feed check across the hood range, with the robot accelerating, with and without the rear guard.

hood 0..25 deg x pop 1.6..4.4 m/s x Pollen/Nectar x new/mid/worn balls x robot acceleration along the turret's Y.
Where each ball ends up: "shot"; "down the hole" (fell back down the feed column: harmless, the indexer pops it again);
"behind" (fell behind the feed column, y > 55 mm: into the deck opening / off the turret); "front"; a ball that only
drifts back after falling below the base plate counts as "down the hole" (it's back in the indexer's space).

    python feed_sweep.py            # with the rear guard as modelled in turret.py
    python feed_sweep.py --no-guard
"""
import itertools
import sys

import numpy as np

import sim_feed as S
import turret as T

HOODS = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0]
POPS = [round(1.6 + 0.2 * i, 1) for i in range(15)]
ACCELS = [-8.0, -4.0, 0.0, 4.0, 8.0]      # m/s^2 (FTC robots brake and accelerate at roughly 0.5-1 g)


def classify(status, rec):
    """'behind' = the ball got behind the feed column (y > 55 mm) while above the base plate (z > 0): it can land in the
    deck opening or leave the turret. A ball that drifts back only after falling below the base plate is back in the
    indexer's space ('down the hole')."""
    P = np.stack([q[1] for q in rec]) * 1000                     # steps x cases x (y, z)
    behind = ((P[:, :, 0] > 55) & (P[:, :, 1] > 0)).any(0)
    front = ((P[:, :, 0] < -55) & (P[:, :, 1] > 0)).any(0)
    return np.where(status == 1, "shot", np.where(behind, "behind", np.where(front, "front", "down the hole")))


def main():
    if "--no-guard" in sys.argv:
        T.REAR_GUARD_SEGS = []
    cases = list(itertools.product(S.BALLS, POPS, HOODS, ["none"], S.BALL_K))
    print(f"rear guard: {T.REAR_GUARD_SEGS or 'none'}")
    tot = {}
    fails = 0
    for ay in ACCELS:
        S.ROBOT_AY = ay
        _, status, _, _ = S.run(cases, record=100)
        kind = classify(status, S.run.rec)
        for k in set(kind):
            tot[(ay, k)] = int((kind == k).sum())
        worst = {}
        for ball in S.BALLS:
            for h in HOODS:
                sel = [i for i, c in enumerate(cases) if c[0] == ball and c[2] == h]
                first = next((p for p in POPS if all(kind[i] == "shot" for i in sel if cases[i][1] >= p)), None)
                worst[(ball, h)] = first
        beh = [(c[0], c[2], c[1]) for c, k in zip(cases, kind) if k == "behind"]
        at3 = [c for c, k in zip(cases, kind) if c[1] >= 3.0 and k != "shot"]
        print(f"\nrobot accel {ay:+.0f} m/s^2: behind {len(beh)}, misses at pops >= 3.0: {len(at3)} {at3[:4]}")
        for ball in S.BALLS:
            print(f"  {ball:6s} every pop >= ... shoots, by hood: " + "  ".join(f"{h:.0f}°:{worst[(ball, h)]}" for h in HOODS))
        if beh:
            print("  behind:", beh[:12])
        fails += len(beh) + len(at3)
    S.ROBOT_AY = 0.0
    print("FAILURES (balls behind the feed + misses at pops >= 3.0 m/s):", fails)
    return fails


if __name__ == "__main__":
    raise SystemExit(1 if main() else 0)
