"""v6 printed PROTOTYPE: every part as a 3D print, so the team can see how the turret goes together before buying parts.

Short-term, show-and-fit only (not for shooting). Plates print at full size (needs a 300 mm+ bed). Bought parts are
printed stand-ins: goBILDA's own STEP models where we have them (gear/pulley teeth, hub patterns, wheels, motors,
servo, camera), plus simple printable versions of shafts (a little undersized so printed parts slide on), bearings
(plain bushings that spin), spacers, the encoder and the home switch. Each stand-in keeps its real part number as the
backup to buy.

    python proto_print.py      -> out/proto/<step>/PETG_<part>__x<qty>.stl, out/proto/print_list.csv
"""
import csv
import math
import shutil
from pathlib import Path

from build123d import Compound, Pos, Box, export_stl, import_step
from shapely.geometry import Point
from shapely.ops import unary_union

import turret as T

HERE = Path(__file__).parent
OUT = HERE / "out" / "proto"
VCAD = HERE / "vendor_cad"
SHAFT_CLEAR = 0.25            # printed REX shafts this much under size so printed hubs/gears slide on
BUSH_BORE = 8.4               # printed bearings: round bore that spins on a printed shaft / M8 bolt

STEPS = [  # (folder, title, what's in it) in build order = print order
    ("01_base", "Base", "base plate, turret rollers, " + ("cable tray, " if T.CABLES else "") + "turret motor + pinion, home switch"),
    ("02_ring_deck", "Ring and deck", "ring gear, deck, deck spacers, " + ("cable drum" if T.CABLES else "home magnet holder")),
    ("03_frame", "Frame", "side plates, L-brackets, crossbars"),
    ("04_motor_mount", "Flywheel motor mount", "adapter, lug, yoke, flywheel motor"),
    ("05_hood_shafts", "Hood and shafts", "hood carriers, swing arms, gears, pulleys, hubs, wheels, shafts, bushings, spacers, collars, servo, springs"),
    ("06_last", "Last", "encoder, camera mount, front plate"),
]

rows = []


def save(step, name, shape, qty, backup="", note="", settings="standard"):
    d = OUT / step
    d.mkdir(parents=True, exist_ok=True)
    fn = f"PETG_{name}__x{qty}.stl"            # the whole prototype prints in PETG; the name says so
    bb = shape.bounding_box()
    # v6.1: vendor-CAD stand-ins (Gecko treads, motors) tessellated at 0.02 mm came out 67-342 MB; dummies and wheels
    # only need to look right, so they get a coarser mesh (well under a 0.2 mm print layer)
    tol, ang = (0.1, 0.35) if settings in ("dummy", "wheel") else (0.02, 0.1)
    export_stl(Pos(-bb.min.X, -bb.min.Y, -bb.min.Z) * shape, str(d / fn), tolerance=tol, angular_tolerance=ang)
    rows.append((step, fn, qty, backup, note, settings, f"{bb.max.X - bb.min.X:.0f} x {bb.max.Y - bb.min.Y:.0f} x {bb.max.Z - bb.min.Z:.0f}"))


def copy_stl(step, src, qty, backup="", note="", settings="standard"):
    d = OUT / step
    d.mkdir(parents=True, exist_ok=True)
    fn = f"PETG_{Path(src).stem}__x{qty}.stl"
    shutil.copy(HERE / "out" / "stl" / T.stl_file(Path(src).stem), d / fn)   # production STLs carry their filament first
    rows.append((step, fn, qty, backup, note, settings, ""))


def vendor(step, sku, pattern, qty, name, note="", settings="standard", biggest_only=False):
    f = next(p for p in sorted(VCAD.glob(pattern)) if p.suffix.lower() in (".step", ".stp"))
    s = import_step(str(f))
    sol = s.solids()
    shape = max(sol, key=lambda q: q.volume) if biggest_only else Compound(sol)
    save(step, name, shape, qty, sku, note, settings)


SETTINGS = {  # key -> (layer, walls / infill, orientation and notes)
    "plate": ("0.20 mm", "4 walls, 30% gyroid", "flat on the bed"),
    "gear": ("0.12 mm", "6 walls, 100% infill", "flat, teeth vertical; no supports"),
    "shaft": ("0.16 mm", "100% infill", "lying on a flat side of the hex, with a brim"),
    "wheel": ("0.20 mm", "3 walls, 15% infill", "flat"),
    "dummy": ("0.28 mm", "2 walls, 10% infill", "any; supports where the slicer asks"),
    "standard": ("0.20 mm", "3 walls, 25% infill", "the flattest face down"),
}


USED_FOR = {  # key -> which parts get these settings (plain words for the page)
    "plate": "base plate, deck, side plates, hood carriers, swing arms",
    "gear": "gears, pulleys, the turret ring, hubs, collars, servo shaft",
    "shaft": "hex shafts",
    "wheel": "the 96 mm and 72 mm Gecko wheel stand-ins",
    "dummy": "motors, servo, encoder, camera, home switch (look only)",
    "standard": "everything else",
}

FILAMENT_MD = [
    "## Filament and printers", "",
    "- **Filament: PETG** (PLA also works). Don't use ASA for this: it shrinks about 0.5% as it cools, so the big plates curl "
    "at the corners and the bolt holes end up about 1.4 mm out over 280 mm. It also gives off fumes.",
    "- **Dry the PETG first** if the roll is old or was left open: 65 °C for 4–6 hours. Wet PETG strings and prints weak.",
    "- **Start from the printer's own PETG profile.** Then change only the layer height, walls and infill from the table below.",
    "- **Bed:** a textured PEI plate. On a smooth plate, put down glue stick first, or PETG can stick too hard and tear the surface.",
    "- **Test print first:** one gear and one shaft. The gear should slide on and turn freely. If it's tight, raise the "
    "slicer's X-Y hole compensation by about 0.1 mm and test again. Then print the rest.",
]


def bbox_mm(p):
    import struct
    import numpy as np
    b = Path(p).read_bytes()
    if b[:5] == b"solid" and b"facet" in b[:400]:
        v = np.array([list(map(float, l.split()[1:4])) for l in b.decode().splitlines() if l.strip().startswith("vertex")])
    else:
        n = struct.unpack("<I", b[80:84])[0]
        v = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"].reshape(-1, 3)
    return sorted((v.max(0) - v.min(0)).tolist(), reverse=True)


def big_parts(limit=250):
    """Prototype parts longer than `limit` mm, longest first: these go on the 350 mm printer."""
    out = []
    for f in OUT.glob("*/*.stl"):
        d = bbox_mm(f)
        if d[0] > limit:
            out.append((d[0], f.name.split("__")[0].removeprefix("PETG_").replace("_", " ")))
    return sorted(out, reverse=True)


def stl_cm3(p):
    import struct
    import numpy as np
    b = Path(p).read_bytes()
    if b[:5] == b"solid" and b"facet" in b[:400]:
        v = [list(map(float, l.split()[1:4])) for l in b.decode().splitlines() if l.strip().startswith("vertex")]
        t = np.array(v).reshape(-1, 3, 3)
    else:
        n = struct.unpack("<I", b[80:84])[0]
        a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        t = a["v"].reshape(-1, 3, 3).astype(float)
    return abs(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6) / 1000


def write_guide(grams):
    """PROTOTYPE.md in out/proto, and the same table as HTML into viewer.html between <!--PROTO_TABLE--> markers."""
    import re
    md = ["# Printed prototype: print order and assembly", "",
          "A full-size model of the turret printed on a 300 mm+ bed, to see how it goes together before buying parts. "
          "It's for fitting and showing, not shooting: the motors, servo, encoder and camera are dummies.", "",
          f"About {sum(grams.values()) / 1000:.1f} kg of PETG (or PLA) in total. Print one folder at a time, in order; "
          "each folder is one build step, so you can assemble as the prints come off.", "",
          ]
    big = big_parts()
    md += FILAMENT_MD
    md += [f"- **Printers:** print the big parts on the Creality K2 Plus (350 mm bed): "
           + ", ".join(f"{n} ({d:.0f} mm)" for d, n in big)
           + ". Print gears, shafts and small parts on the Bambu H2C, which gives finer teeth. Run both at once.", ""]
    md += ["## Print settings", "", "| Type | Used for | Layer | Walls / infill | How to place it |", "|---|---|---|---|---|"]
    md += [f"| {k} | {USED_FOR[k]} | {a} | {b} | {c} |" for k, (a, b, c) in SETTINGS.items()]
    md += ["", "## Print order", ""]
    for s_, title, what in STEPS:
        md += [f"### {s_[:2]} · {title} (~{grams[s_]:.0f} g)", "", what, "", "| File | Qty | Settings | Real part to buy later |", "|---|---|---|---|"]
        md += [f"| {r[1]} | {r[2]} | {r[5]} | {r[3]} |" for r in rows if r[0] == s_]
        md.append("")
    md += ["## Fitting tips", "",
           "- **Shafts** are printed 0.25 mm under size so printed gears, pulleys and hubs slide on. If one is tight, "
           "ease the hex bore with a file; don't force it.",
           "- **Bearings** are plain bushings that spin on the printed shafts (and on the M8 roller bolts).",
           ("- **Square nuts / counterbores:** the real plywood plates use M4 square nuts pressed into pockets and "
            "button heads in counterbores. The printed prototype plates are one solid piece instead: where a square nut "
            "would go there's a 3.3 mm hole, and M4 screws cut their own thread. Don't overtighten. Heat-set inserts go "
            "where the real design uses them (M3 in the servo insert, M4 elsewhere)." if T.LAMINATE else
            "- **Tapped holes** in the metal version are 3.3 mm holes in the printed plates: M4 screws cut their own "
            "thread. Don't overtighten. Heat-set inserts go where the real design uses them."),
           "- **Still needed:** the M4 and M3 screws and nuts from the parts list (they carry over to the real build), "
           "M4 and M3 heat-set inserts, and the 2 belts if you want to see the drive turn (or skip them). Rubber bands can "
           "stand in for the constant-force springs.",
           "- **Assembly:** follow the page's \"Assembly order and adjustments\" with the printed parts in place of the real ones."]
    (OUT / "PROTOTYPE.md").write_text("\n".join(md) + "\n")
    # page table
    h = ['<table class="proto">', "<tr><th>Step</th><th>File</th><th>Qty</th><th>Print as</th><th>Real part (buy later)</th></tr>"]
    for s_, title, what in STEPS:
        rs = [r for r in rows if r[0] == s_]
        for i, r in enumerate(rs):
            cell = f'<td rowspan="{len(rs)}"><b>{s_[:2]} · {title}</b><br><span class="mut">~{grams[s_]:.0f} g</span></td>' if i == 0 else ""
            h.append(f'<tr>{cell}<td><a data-file="proto/{s_}/{r[1]}" download>{r[1].split("__")[0]}</a></td><td>{r[2]}</td>'
                     f'<td>{r[5]}</td><td>{r[3]}</td></tr>')
    h.append("</table>")
    import html as H
    def md2html(line):
        return re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", H.escape(line[2:], quote=False))
    g = ['<div class="psettings"><h3>Filament and printers</h3><ul>']
    g += [f"<li>{md2html(l)}</li>" for l in FILAMENT_MD if l.startswith("- ")]
    g.append("<li><b>Printers:</b> print the big parts on the Creality K2 Plus (350&nbsp;mm bed): "
             + ", ".join(f"{H.escape(n)} ({d:.0f}&nbsp;mm)" for d, n in big)
             + ". Print gears, shafts and small parts on the Bambu H2C, which gives finer teeth. Run both at once.</li></ul>")
    g.append('<h3>Settings by part type</h3><p class="mut">Each file in the table further down says which type it is.</p>')
    g.append('<table class="proto"><tr><th>Type</th><th>Used for</th><th>Layer</th><th>Walls / infill</th><th>How to place it</th></tr>')
    g += [f"<tr><td><b>{k}</b></td><td>{USED_FOR[k]}</td><td>{a}</td><td>{b}</td><td>{c}</td></tr>" for k, (a, b, c) in SETTINGS.items()]
    g.append("</table></div>")
    v = HERE / "viewer.html"; src = v.read_text()
    src2 = re.sub(r"<!--PROTO_SETTINGS-->.*?<!--/PROTO_SETTINGS-->", lambda _: "<!--PROTO_SETTINGS-->" + "\n".join(g) + "<!--/PROTO_SETTINGS-->", src, flags=re.S)
    new = re.sub(r"<!--PROTO_TABLE-->.*?<!--/PROTO_TABLE-->", lambda _: "<!--PROTO_TABLE-->" + "\n".join(h) + "<!--/PROTO_TABLE-->", src, flags=re.S)
    new = re.sub(r"<!--PROTO_TABLE-->.*?<!--/PROTO_TABLE-->", lambda _: "<!--PROTO_TABLE-->" + "\n".join(h) + "<!--/PROTO_TABLE-->", src2, flags=re.S)
    new = re.sub(r'<span id="protoKg">[^<]*</span>', f'<span id="protoKg">{sum(grams.values()) / 1000:.1f}</span>', new)
    if new != src:
        v.write_text(new)


def tube(od, idia, length):
    return T.plate(Point(0, 0).buffer(od / 2, quad_segs=48).difference(Point(0, 0).buffer(idia / 2, quad_segs=48)), length)


def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    parts, plates2d = T.build()
    stacks = T.shaft_stacks()

    # ---- 1 base ----
    s = "01_base"
    GB = getattr(T, "GB_TURRET", False)       # v8.1GB: the goBILDA turret bearing is bought (no stand-in)
    if not GB:
        save(s, "base_plate", T.plate(plates2d["base_plate"]), 1, "base_plate_L1.dxf + base_plate_L2.dxf (plywood)" if T.LAMINATE else "base_plate.dxf (aluminum later)", "", "plate")
        for f, q in (("roller_tire.stl", 6), ("roller_spacer.stl", 6), ("roller_cap.stl", 6), ("roller_mid_spacer.stl", 6)):
            copy_stl(s, f, q)
        save(s, "bearing_608_standin", tube(22.0, BUSH_BORE, 7.0), 12, "1600-0722-0008 (2-pack) x6", "plain bushing; spins on the M8 bolt")
    else:
        copy_stl(s, "switch_arm.stl", 1, "the real one is this print (PA-CF)", "screws to the turret bearing's housing; the home switch sits on it")
    if T.CABLES:
        for k in range(T.TRAY_N):
            copy_stl(s, f"cable_tray_{k}.stl", 1)
    if not GB:
        vendor(s, "5203-2402-0005", "5203-2402-0005*", 1, "turret_motor_standin", "motor body + shaft, one piece", "dummy")
        vendor(s, "2303-4008-0020", "2303-4008-0020*", 1, "turret_pinion_20T", "", "gear")
    save(s, "home_switch_standin", T.home_switch(), 1, "REV-31-1462", "", "dummy")

    # ---- 2 ring and deck ----
    s = "02_ring_deck"
    if not GB:
        copy_stl(s, "turret_ring_print.stl", 1, "the real one is this print, in PA-CF" if T.LAMINATE else "turret_ring.dxf (2x 1/8in aluminum later)", "full thickness; holes sized for M4 heat-sets", "gear")
    save(s, "deck", T.plate(plates2d["deck"]), 1, "deck_L1.dxf + deck_L2.dxf (plywood)" if T.LAMINATE else "deck.dxf", "", "plate")
    if GB:
        copy_stl(s, "deck_shim.stl", 1, "the real one is this print", "goes between the bearing's ring and the deck")
    else:
        copy_stl(s, "deck_spacer.stl", 8)
    if T.CABLES:
        for k in range(T.DRUM_N):
            copy_stl(s, f"cable_drum_{k}.stl", 1)
    if getattr(T, "HOME_FLAG", False):
        copy_stl(s, "home_flag.stl", 1, "the real one is this print", "2 M4 heat-sets; the switch's magnet sits in its pocket")

    # ---- 3 frame ----
    s = "03_frame"
    sps = T.plate(plates2d["side_plate_servo"])
    if T.SERVO_INSERT:   # v7 (Astra round 3): the printed insert's lip needs its recess in the inner face here too
        sps = sps - T.plate(T.servo_insert_2d()[1].buffer(0.15), T.PLY_LAYER + 0.01).moved(T.Location((0, 0, -0.01)))
    save(s, "side_plate_servo", sps, 1, "side_plate_servo_L1.dxf + side_plate_servo_L2.dxf (plywood)" if T.LAMINATE else "side_plate_servo.dxf", "", "plate")
    save(s, "side_plate_motor", T.plate(plates2d["side_plate_motor"]), 1, "side_plate_motor_L1.dxf + side_plate_motor_L2.dxf (plywood)" if T.LAMINATE else "side_plate_motor.dxf", "", "plate")
    if T.SERVO_INSERT:   # v7 (Astra round 2): the prototype's printed side plate has the insert's opening, so print the insert
        copy_stl(s, "servo_insert.stl", 1, "the real one is this print, in PA-CF", "4 M3 heat-sets for the servo")
    copy_stl(s, "l_bracket.stl", 4)
    copy_stl(s, "crossbar.stl", 3 - (1 if T.REAR_GUARD else 0))
    if T.REAR_GUARD:
        copy_stl(s, "rear_guard.stl", 1)

    # ---- 4 flywheel motor mount ----
    s = "04_motor_mount"
    if T.LAMINATE:   # v7 (Astra round 2): the real printed part, with its square-nut pockets
        copy_stl(s, "motor_adapter.stl", 1, "the real one is this print, in PA-CF", "4 M4 square nuts in the pockets")
    else:
        save(s, "motor_adapter", T.plate(T.motor_adapter_2d(), T.ADAPT_T), 1, "motor_adapter.dxf (1/8in aluminum later)", "", "plate")
    copy_stl(s, "motor_lug.stl", 1)
    copy_stl(s, "motor_yoke.stl", 1)
    vendor(s, "5203-2402-0001", "5203-2402-0001*", 1, "flywheel_motor_standin", "motor body + shaft, one piece", "dummy")

    # ---- 5 hood and shafts ----
    s = "05_hood_shafts"
    save(s, "hood_carrier_servo_side", T.countersunk(T.plate(plates2d["hood_carrier"]), T.CARRIER_CSK + T.HOOD_GEAR_CSK, 0.0), 1,
         "hood_carrier_servo_L1.dxf + hood_carrier_servo_L2.dxf (plywood)" if T.LAMINATE else "hood_carrier.dxf", "countersinks printed in", "plate")
    save(s, "hood_carrier_motor_side", T.countersunk(T.plate(plates2d["hood_carrier"]), T.CARRIER_CSK, T.T), 1,
         "hood_carrier_motor_L1.dxf + hood_carrier_motor_L2.dxf (plywood)" if T.LAMINATE else "hood_carrier.dxf", "countersinks printed in (other face)", "plate")
    save(s, "swing_arm_servo_side", T.countersunk(T.plate(plates2d["swing_arm"]), T.ARM_CSK, 0.0), 1, "PA-CF_swing_arm_servo.stl / PA-CF_swing_arm_motor.stl (the real ones are prints)" if T.LAMINATE else "swing_arm.dxf",
         "anchor countersink printed in", "plate")
    save(s, "swing_arm_motor_side", T.countersunk(T.plate(plates2d["swing_arm"]), T.ARM_CSK, T.T), 1, "PA-CF_swing_arm_servo.stl / PA-CF_swing_arm_motor.stl (the real ones are prints)" if T.LAMINATE else "swing_arm.dxf",
         "anchor countersink printed in (other face)", "plate")
    vendor(s, "2302-0032-0108", "*0108.STEP", 1, "hood_gear_108T", "", "gear")
    vendor(s, "2303-4008-0020", "2303-4008-0020*", 1, "servo_pinion_20T", "", "gear")
    vendor(s, "1922-0025-0036", "1922-0025-0036*", 1, "servo_shaft", "", "gear")
    vendor(s, "2000-0025-0002", "2000-0025-0002*", 1, "hood_servo_standin", "", "dummy")
    vendor(s, "2312-0414-0060", "2312-0414-0060*", 1, "gear_60T", "", "gear")
    vendor(s, "2312-0414-0048", "2312-0414-0048*", 1, "gear_48T", "", "gear")
    vendor(s, "3417-4008-0024", "3417-4008-0024*", 2, "pulley_24T", "", "gear")
    vendor(s, "3417-4008-0016", "3417-4008-0016*", 1, "pulley_16T", "", "gear")
    vendor(s, "3415-0014-0048", "3415-0014-0048*", 1, "pulley_48T", "", "gear")
    vendor(s, "1309-0016-4008", "1309-0016-4008*", 5, "sonic_hub", "", "gear")
    vendor(s, "3613-0014-0096", "3613-0014-0096*", 3, "gecko_96", "main wheel stack: 3 per stack", "wheel")
    vendor(s, "3613-0014-0072", "3613-0014-0072*", 3, "gecko_72", "upper wheel stack: 3 per stack", "wheel")
    vendor(s, "2910-0920-4008", "2910-0920-4008*", 3, "collar", "", "gear")
    # shafts at their cut lengths, slightly undersized REX
    rex = T.rex_bore(-SHAFT_CLEAR)
    main_L = next(int(r[4].split("CUT TO ")[1].split("mm")[0]) for r in T.bom() if "main flywheel shaft" in str(r[4]))
    jack_L = next(int(r[4].split("CUT TO ")[1].split("mm")[0]) for r in T.bom() if "jackshaft + upper shaft" in str(r[4]))
    save(s, f"shaft_main_{main_L}mm", T.plate(rex, main_L), 1, "2106-4008-2400", "print lying flat", "shaft")
    save(s, f"shaft_jack_upper_{jack_L}mm", T.plate(rex, jack_L), 2, "2106-4008-1680", "jackshaft + upper shaft; print lying flat", "shaft")
    # bearings as flanged bushings, spacers as tubes (counts from the shaft stacks)
    nb = sum(1 for st in (stacks["Main flywheel shaft"], stacks["Jackshaft"], stacks["Upper flywheel shaft"])
             for it in st["items"] if it[4] == "bearing")
    bush = T.plate(Point(0, 0).buffer(7.5, quad_segs=48).difference(Point(0, 0).buffer(BUSH_BORE / 2, quad_segs=48)), 1.0) + \
        Pos(0, 0, 1.0) * tube(14.0, BUSH_BORE, 4.0)
    save(s, "bearing_1611_standin", bush, nb, "1611-0514-4008 (2-pack)", "flanged bushing: body in the plate hole, flange outside")
    for sku, n in sorted(T.spacer_counts().items()):
        L = int(sku[-3:]) / 10 if sku.startswith("1522") else 1.0
        save(s, f"spacer_{L:g}mm", tube(10.0, 8.4, L), n, sku)
    for f, q in (("stop_cam.stl", 2), ("cf_spring_drum.stl", 2), ("cf_axle.stl", 2), ("guide_spacer.stl", 4), ("cf_anchor.stl", 2), ("cf_clamp.stl", 2), ("arm_bumper_core.stl", 2), ("arm_bumper_ring.stl", 2)):
        copy_stl(s, f, q)

    # ---- 6 last ----
    s = "06_last"
    for f, q in (("encoder_bracket.stl", 1), ("encoder_bracket_leg.stl", 2), ("encoder_sleeve.stl", 1), ("limelight_mount.stl", 1),
                 ("front_plate.stl", 1)):
        copy_stl(s, f, q)
    save(s, "encoder_standin", T.plate(T.encoder_2d(), T.ENC_T), 1, "REV-11-3174", "full 15.3 mm thick incl. ears (REV gives no ear thickness): the M4 x 30 ear screws fit it and the real encoder", "dummy")
    vendor(s, "3122-0002-0001", "3122-0002-0001*", 1, "limelight_standin", "", "dummy")

    OUT.mkdir(parents=True, exist_ok=True)
    grams = {s_: 0.0 for s_, _, _ in STEPS}
    for r in rows:
        grams[r[0]] += stl_cm3(OUT / r[0] / r[1]) * r[2] * 1.25 * 0.6        # PLA/PETG, walls + ~30% infill
    write_guide(grams)
    with open(OUT / "print_list.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "file", "qty", "real part / backup to buy", "note", "settings", "size mm"])
        w.writerows(rows)
    print(f"{len(rows)} files, {sum(r[2] for r in rows)} prints -> {OUT}")


if __name__ == "__main__":
    main()
