# v6.1 rounds 2-3 (Astra): judge EVERY long-Nectar shot against the scoring window at ITS OWN simulated launch angle
# (sim_rapid run.exit_el), count unlaunched balls as failures (must be 4/4), and calibrate the wheel speed so shot 1
# leaves mid-window at its own angle.
import math, numpy as np, sim_rapid as R, entry_window as E
m, r = E.BALLS["Nectar"]
D = 2.5 + (E.B[0] + E.P[0]) / 2
def win(el):
    vm, pct = E.window(D, el, m, r)[:2]
    return vm * (1 - pct / 100), vm, vm * (1 + pct / 100)
a, b = 7.0, 8.0
for _ in range(14):
    tv = (a + b) / 2
    ev, _, _ = R.run(0.5, ["N"], alpha_deg=25.0, target_v=tv)
    a, b = (tv, b) if ev[0][0] < win(R.run.exit_el[0][0])[1] else (a, tv)
tv = round((a + b) / 2, 3); free = R.MOTOR_FREE / R.GEAR * R.R1
print(f"long Nectar 2.5 m, hood 25 deg: calibrated wheel {tv} m/s ({100*tv/free:.0f}% of free)")
def show(label, ev, els, extra=""):
    ok, rows, margins = 0, [], []
    for v, el in zip(ev, els):
        if not (v > 0 and math.isfinite(el)):
            rows.append("FAILED"); continue
        lo, vm, hi = win(el); inside = lo <= v <= hi; ok += inside
        margins.append(min(v / lo - 1, 1 - v / hi) * 100)
        rows.append(f"{v:.3f}@{el:.2f}deg [{lo:.3f}-{hi:.3f}]")
    good = [v for v in ev if v > 0]
    sp = 100 * (max(good) - min(good)) / max(good) if good else float("nan")
    print(f"  {label}: in window {ok}/{len(ev)}, worst edge margin {min(margins):.2f}%, spread {sp:.1f}% {extra}\n     " + "; ".join(rows))
for iv in (0.25, 0.33):
    ev, _, _ = R.run(iv, ["NNNN"], alpha_deg=25.0, target_v=tv); show(f"fixed {1/iv:.0f}/s", ev[0], R.run.exit_el[0])
for g in (0.01, 0.02):
    ev, _, _ = R.run(0.25, ["NNNN"], alpha_deg=25.0, target_v=tv, gate=g); tf = R.run.t_fire[0]
    show(f"gate +/-{g*100:.0f}% (min 0.25 s)", ev[0], R.run.exit_el[0], f"avg {(len(tf)-1)/(tf[-1]-tf[0]):.2f}/s")
for iv in (0.25, 0.33):
    ev, _, _ = R.run(iv, ["NNNN"], alpha_deg=0.0, target_v=8.5)
    e = [x for x in ev[0] if x > 0]; print(f"  close Nectar 8.5, fixed {1/iv:.0f}/s: {len(e)}/4 launched, spread {100*(max(e)-min(e))/max(e):.1f}%")
ev, _, _ = R.run(0.25, ["NNNN"], alpha_deg=0.0, target_v=8.5, gate=0.01); tf = R.run.t_fire[0]
print(f"  close Nectar gate 1%: avg {(len(tf)-1)/(tf[-1]-tf[0]):.2f}/s")
