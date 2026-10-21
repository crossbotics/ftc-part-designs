"""The "Shooting at the HIVE from anywhere" table on the page, generated from out/approach/shoot_map.json.

Each cell: best launch angle and speed room at that spot, the WORST of the other HIVE's two states, on the side away
from the other HIVE (negative bearings; the other HIVE's side is described in the text). Pollen on top, Nectar grey.
Writes between <!--HIVE_TABLE--> markers in viewer.html and prints the numbers the section's bullets quote.

    python hive_table.py
"""
import json
import re
from pathlib import Path

import turret as T

DISTS = (0.5, 0.75, 1.0, 1.5, 2.0, 2.5)
SIDES = (0, 30, 60)


def worst(r, ball):
    a, b = r[f"{ball}-down"], r[f"{ball}-up"]
    if not a or not b:
        return None
    return a if a[1] <= b[1] else b


def main():
    M = json.loads((T.OUT / "approach" / "shoot_map.json").read_text())
    R = {(r["d"], r["b"]): r for r in M["rows"]}
    h = ['<table class="adj"><tr><th>From the opening</th><th>Square-on</th><th>30° to the side</th><th>60° to the side</th></tr>']
    for d in DISTS:
        cells = []
        for s in SIDES:
            r = R[(d, -s)]
            if r.get("frame"):
                cells.append("robot can't be here (frame base)"); continue
            txt = []
            for ball in ("Pollen", "Nectar"):
                w = worst(r, ball)
                txt.append(f"{w[0]:.0f}°, ±{w[1]:.1f}%" if w else "can't score")
            cells.append(f'{txt[0]}<br><span class="mut">{txt[1]}</span>')
        h.append(f"<tr><td>{d:.2f}&nbsp;m</td>" + "".join(f"<td>{c}</td>" for c in cells) + "</tr>")
    h.append("</table>")
    v = Path(__file__).parent / "viewer.html"
    src = v.read_text()
    new = re.sub(r"<!--HIVE_TABLE-->.*?<!--/HIVE_TABLE-->", lambda _: "<!--HIVE_TABLE-->" + "\n".join(h) + "<!--/HIVE_TABLE-->", src, flags=re.S)
    if new != src:
        v.write_text(new)
    # numbers the bullets quote
    rows = [r for r in M["rows"] if not r.get("frame")]
    def room(r, ball):
        w = worst(r, ball); return w[1] if w else 0.0
    print("worst anywhere 0.5-2.5 m, |bearing| <= 60, away side:")
    for ball in ("Pollen", "Nectar"):
        rs = [(room(r, ball), r["d"], r["b"]) for r in rows if 0.5 <= r["d"] <= 2.5 and -60 <= r["b"] <= 0]
        print(f"  {ball}: {min(rs)}")
    for d in (0.33, 0.5):
        for ball in ("Pollen", "Nectar"):
            vals = [(r["b"], worst(r, ball)) for r in rows if r["d"] == d and abs(r["b"]) <= 60 and r["b"] <= 0]
            els = [w[0] for _, w in vals if w]; rms = [w[1] for _, w in vals if w]
            print(f"  {d} m {ball}: launch {min(els) if els else '-'}-{max(els) if els else '-'} deg, room {min(rms) if rms else 0:.1f}-{max(rms) if rms else 0:.1f}%"
                  f" ({sum(1 for _, w in vals if not w)} spots can't score)")
    # the other HIVE's side: where does it take away room?
    lost = [(r["d"], r["b"]) for r in rows if r["b"] > 0 and any(
        (r[k][1] if r[k] else 0) < r[k + "-free"] - 0.2 for k in ("Pollen-down", "Pollen-up", "Nectar-down", "Nectar-up"))]
    print("  other HIVE takes room at (d, bearing):", sorted(lost)[:40], "..." if len(lost) > 40 else "")
    print("  max d where blocked:", max((d for d, b in lost), default=None), " min bearing:", min((b for d, b in lost), default=None))


if __name__ == "__main__":
    main()
