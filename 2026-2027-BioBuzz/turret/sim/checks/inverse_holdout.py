"""Hold-out check of sim_core.turretShot(): run sim_feed at random (hood, wheel speed) settings that are NOT on the
exported table, then (in node) invert their launch angle and exit speed and compare. Writes holdout.json."""
import json, math, random, sys
from pathlib import Path
HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent.parent))
import sim_feed as SF
random.seed(7)
cases = []
for _ in range(60):                                 # anywhere inside the table
    b = random.choice(list(SF.BALLS)); w = random.choice(["new", "mid", "worn"])
    cases.append((b, w, round(random.uniform(0.3, 24.7), 3), round(random.uniform(4.6, 9.9), 3)))
for _ in range(20):                                 # slow shots and the hood range's ends (Astra round 2)
    b = random.choice(list(SF.BALLS)); w = random.choice(["new", "mid", "worn"])
    cases.append((b, w, random.choice([0.0, 25.0, round(random.uniform(0, 25), 3)]), round(random.uniform(3.6, 4.8), 3)))
out = []
keep = SF.WHEEL_SPEED
for b, w, h, vs in cases:
    SF.WHEEL_SPEED = vs
    _, st, ev, _ = SF.run([(b, 3.0, h, "none", w)])
    if st[0] != 1: continue
    out.append({"ball": b.capitalize(), "wear": w, "hood": h, "surface": vs, "exit": float(ev[0]),
                "launch": float(SF.run.exit_el[0]), "spin": float(-SF.run.exit_spin[0] * 60 / (2 * math.pi))})
SF.WHEEL_SPEED = keep
(HERE / "holdout.json").write_text(json.dumps(out, indent=0))
print(len(out), "hold-out shots (", len(cases) - len(out), "the feed sim did not launch )")
