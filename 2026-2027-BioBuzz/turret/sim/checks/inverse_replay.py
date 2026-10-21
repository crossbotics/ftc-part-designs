"""Forward replay: run the settings turretShot() chose (holdout_inverse.json) through sim_feed again and compare what
comes out with what was asked for. Exits 1 if any shot is off by more than the limits.
    python sim/checks/inverse_replay.py"""
import json, math, sys
from pathlib import Path
HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent.parent))
import sim_feed as SF
LIM = {"launch_deg": 0.6, "exit_pct": 0.8, "spin_rpm": 12.0}       # what the simulator's notes promise
rows = json.loads((HERE / "holdout_inverse.json").read_text())
keep = SF.WHEEL_SPEED; worst = {k: 0.0 for k in LIM}; bad = []
for r in rows:
    SF.WHEEL_SPEED = r["pick_surface"]
    _, st, ev, _ = SF.run([(r["ball"].lower(), 3.0, r["pick_hood"], "none", r["wear"])])
    if st[0] != 1: bad.append((r, "not launched")); continue
    got = {"launch_deg": abs(float(SF.run.exit_el[0]) - r["launch"]), "exit_pct": 100 * abs(float(ev[0]) / r["exit"] - 1),
           "spin_rpm": abs(float(-SF.run.exit_spin[0] * 60 / (2 * math.pi)) - r["pick_spin"])}
    for k in LIM:
        worst[k] = max(worst[k], got[k])
        if got[k] > LIM[k]: bad.append((r, k, round(got[k], 3)))
SF.WHEEL_SPEED = keep
print(f"{len(rows)} replayed; worst launch {worst['launch_deg']:.2f} deg, exit {worst['exit_pct']:.2f} %, spin {worst['spin_rpm']:.1f} rpm"
      f" (limits {LIM['launch_deg']} deg, {LIM['exit_pct']} %, {LIM['spin_rpm']} rpm)")
for b in bad[:10]: print("  FAIL", b)
sys.exit(1 if bad or len(rows) < 60 else 0)                       # too few replayed rows fails too
