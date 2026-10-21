"""Continuous-wear check of sim_core.turretForward(): random ball / wear index (between table levels) / hood / wheel
speed; run sim_feed directly at that ball stiffness (the same straight-line stiffness mapping as sim_export) and save.
Then:  node sim/checks/wear_check.mjs  compares the lookup and fails if it's off by more than the limits."""
import json, math, random, sys
from pathlib import Path
HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent.parent))
import sim_feed as SF
K = SF.BALL_K
stiff = lambda x: K["new"] + (K["mid"] - K["new"]) * x if x <= 1 else K["mid"] + (K["worn"] - K["mid"]) * (x - 1)
random.seed(11)
out = []; keep = SF.WHEEL_SPEED
for n in range(120):
    b = random.choice(list(SF.BALLS)); x = round(random.uniform(0.02, 1.98), 3)
    h = round(random.uniform(0.2, 24.8), 3); vs = round(random.uniform(4.2, 9.9), 3)
    key = f"chk{n}"; K[key] = stiff(x); SF.WHEEL_SPEED = vs
    _, st, ev, _ = SF.run([(b, 3.0, h, "none", key)])
    if st[0] == 1:
        out.append({"ball": b.capitalize(), "wear": x, "hood": h, "surface": vs, "exit": float(ev[0]),
                    "launch": float(SF.run.exit_el[0]), "spin": float(-SF.run.exit_spin[0] * 60 / (2 * math.pi))})
SF.WHEEL_SPEED = keep
(HERE / "wear_check.json").write_text(json.dumps(out))
print(len(out), "direct feed runs at in-between wear")
