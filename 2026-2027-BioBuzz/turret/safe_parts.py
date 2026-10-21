"""v8.1GB: which parts are the same in turret v8.1 and v8.1GB, so the team can make or buy them before picking.

    python safe_parts.py     -> out/safe_parts.md (+ printed summary)

Compares the files the team builds from, geometry only (not file dates): every DXF layer (cut paths, rounded to
0.01 mm), every final STL (triangles, rounded to 0.001 mm), and the bought-parts list (out/bom.csv: part number and
quantity). Run turret.py in both folders first.
"""
import csv
import hashlib
import struct
from pathlib import Path

import ezdxf

HERE = Path(__file__).parent
A, B = HERE.parent / "v8.1" / "out", HERE / "out"


def dxf_key(p):
    doc = ezdxf.readfile(p)
    items = []
    for e in doc.modelspace():
        if e.dxftype() == "LWPOLYLINE":
            items.append((e.dxf.layer, tuple((round(x, 2), round(y, 2)) for x, y, *_ in e.get_points())))
        elif e.dxftype() == "CIRCLE":
            items.append((e.dxf.layer, (round(e.dxf.center.x, 2), round(e.dxf.center.y, 2), round(e.dxf.radius, 3))))
        elif e.dxftype() in ("TEXT", "MTEXT"):
            continue                                   # notes may differ in wording only
    return hashlib.sha1(repr(sorted(items)).encode()).hexdigest()


def stl_key(p):
    data = p.read_bytes()
    if data[:5].lower() == b"solid":                 # ASCII
        tris = sorted(tuple(round(float(v), 3) for v in line.split()[1:]) for line in data.decode(errors="ignore").splitlines()
                      if line.strip().startswith("vertex"))
    else:
        n = struct.unpack("<I", data[80:84])[0]
        tris = sorted(tuple(round(v, 3) for v in struct.unpack("<12f", data[84 + 50 * i: 84 + 50 * i + 48])[3:]) for i in range(n))
    return hashlib.sha1(repr(tris).encode()).hexdigest()


def compare(sub, pattern, key):
    a = {p.name: p for p in (A / sub).glob(pattern)}
    b = {p.name: p for p in (B / sub).glob(pattern)}
    same = sorted(n for n in a.keys() & b.keys() if key(a[n]) == key(b[n]))
    diff = sorted(n for n in a.keys() & b.keys() if n not in same)
    return same, diff, sorted(a.keys() - b.keys()), sorted(b.keys() - a.keys())


def bom(path):
    out = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            out[(r["sku_or_file"], r["part"])] = r["qty"]
    return out


def main():
    lines = ["# Turret parts: the same in v8.1 and v8.1GB", "",
             "Safe to make or buy now, whichever version is picked (compared file by file, geometry only).", ""]
    for title, sub, pat, key in (("Laser-cut plywood layers (DXF)", "dxf", "*.dxf", dxf_key), ("3D prints (STL)", "stl", "*.stl", stl_key)):
        same, diff, only_a, only_b = compare(sub, pat, key)
        lines += [f"## {title}", "", f"**Same in both ({len(same)}):** " + ", ".join(f"`{n}`" for n in same), ""]
        if diff:
            lines += [f"**Different, wait ({len(diff)}):** " + ", ".join(f"`{n}`" for n in diff), ""]
        if only_a:
            lines += [f"**Only in v8.1 ({len(only_a)}):** " + ", ".join(f"`{n}`" for n in only_a), ""]
        if only_b:
            lines += [f"**Only in v8.1GB ({len(only_b)}):** " + ", ".join(f"`{n}`" for n in only_b), ""]
        print(f"{title}: same {len(same)}, different {len(diff)}, only v8.1 {len(only_a)}, only v8.1GB {len(only_b)}")
    ba, bb = bom(A / "bom.csv"), bom(B / "bom.csv")
    same = sorted(k for k in ba.keys() & bb.keys() if ba[k] == bb[k])
    qty = sorted(k for k in ba.keys() & bb.keys() if ba[k] != bb[k])
    lines += ["## Parts list rows (out/bom.csv)", "", f"**Same part and quantity in both ({len(same)}).**", ""]
    lines += [f"- {k[1]} ({k[0]}), qty {ba[k]}" for k in same] + [""]
    if qty:
        lines += ["**Same part, different quantity (buy the smaller number now):**", ""] + [f"- {k[1]} ({k[0]}): v8.1 {ba[k]}, v8.1GB {bb[k]}" for k in qty] + [""]
    lines += ["**Only in v8.1:**", ""] + [f"- {k[1]} ({k[0]}), qty {ba[k]}" for k in sorted(ba.keys() - bb.keys())] + [""]
    lines += ["**Only in v8.1GB:**", ""] + [f"- {k[1]} ({k[0]}), qty {bb[k]}" for k in sorted(bb.keys() - ba.keys())] + [""]
    print(f"BOM rows: same {len(same)}, qty differs {len(qty)}, only v8.1 {len(ba.keys() - bb.keys())}, only v8.1GB {len(bb.keys() - ba.keys())}")
    (B / "safe_parts.md").write_text("\n".join(lines))


if __name__ == "__main__":
    main()
