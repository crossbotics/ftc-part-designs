"""Print settings for the real (competition) printed parts, on the turret page between <!--PRINT_SETTINGS--> markers.

The filament of every part comes from turret.py's FILAMENT_STL table (the same table that names the STL files and
colours the 3D model), and the part list comes from the parts list itself, so this section can't drift from them.
Filament numbers are Bambu Lab's own (bambulab.com/en-us/filament/pa6-cf, /petg-hf, /tpu-95a-hf, read 2026-10-02).

    python print_settings.py          (deploy_site.sh runs it)
"""
import re
import struct
from pathlib import Path

import numpy as np

import turret as T

HERE = Path(__file__).parent
PAGE = HERE / "viewer.html"

# part type -> (layer, walls / infill, placement, why); the first matching STL prefix sets a part's type
TYPES = {
    "gear":  ("0.12 mm", "6 walls, 100% infill (top/bottom don't matter when solid)", "flat, teeth standing up; no supports",
              "fine layers give clean teeth; solid so the teeth don't flex"),
    "solid": ("0.16 mm", "100% infill (top/bottom don't matter when solid)", "flat end down",
              "clamped under bolts or setting a stop: solid plastic doesn't squash over a season"),
    "strong": ("0.20 mm", "5 walls, 7 top + 6 bottom layers (~1.4 / 1.2 mm), 50% gyroid", "the largest flat face down; supports only where the slicer insists",
               "takes load and holds heat-set inserts (5 walls = solid plastic around each insert)"),
    "tire":  ("0.16 mm", "5 walls, 7 top + 7 bottom layers (~1.1 mm), 40% gyroid", "upright; tree supports under the groove",
              "the groove the ring gear rides in must come out smooth"),
    "light": ("0.20 mm", "3 walls, 4 top + 3 bottom layers (~0.8 / 0.6 mm), 20% gyroid", "flat on the bed",
              "big and lightly loaded: fewer walls save hours and filament"),
    "soft":  ("0.20 mm", "100% infill", "flat",
              "TPU stays soft at any infill; solid gives the bumper an even feel"),
}
TYPE_OF = [("turret_ring_print", "gear"), ("pulley_", "gear"),
           ("spacer_", "solid"), ("roller_spacer", "solid"), ("roller_cap", "solid"), ("roller_mid_spacer", "solid"),
           ("deck_spacer", "solid"), ("guide_spacer", "solid"), ("cf_axle", "solid"), ("stop_cam", "solid"),
           ("arm_bumper_core", "solid"), ("encoder_sleeve", "solid"), ("encoder_bracket_leg", "solid"),
           ("roller_tire", "tire"),
           ("cable_tray", "light"), ("cable_drum", "light"), ("rear_guard", "light"), ("front_plate", "light"),
           ("arm_bumper_ring", "soft")]          # everything else: strong

FIL = {   # filament -> rows of the material table (Bambu's ranges; our pick first)
    "PA-CF": {"name": "PA-CF (Bambu Lab PA6-CF)", "nozzle": "hardened steel, 0.4 mm", "temp": "280 °C (260–300)",
              "bed": "110 °C (100–120), textured PEI + a thin coat of glue stick", "chamber": "door closed; chamber heat on, 50–60 °C",
              "speed": "under 100 mm/s", "fan": "0–40%", "dry": "80 °C for 8–12 h, then print from a dry box"},
    "PETG":  {"name": "PETG (Bambu Lab PETG HF)", "nozzle": "any 0.4 mm", "temp": "245 °C (230–260)",
              "bed": "70 °C (60–80), textured PEI", "chamber": "door closed; chamber heat off",
              "speed": "the profile's speed", "fan": "the profile's setting (0–80%)", "dry": "65 °C for 8 h if the roll was left open"},
    "TPU":   {"name": "TPU 95A (Bambu Lab TPU 95A HF)", "nozzle": "any 0.4 mm", "temp": "230 °C (220–240)",
              "bed": "35 °C (30–45), with a brim", "chamber": "door closed; chamber heat off",
              "speed": "under 80 mm/s", "fan": "50–100%", "dry": "70 °C for 8 h"},
}
ROWS = [("nozzle", "Nozzle"), ("temp", "Nozzle temp"), ("bed", "Bed"), ("chamber", "Chamber"),
        ("speed", "Speed"), ("fan", "Part fan"), ("dry", "Dry first")]


def longest_mm(f):
    b = f.read_bytes(); n = struct.unpack_from("<I", b, 80)[0]
    v = np.frombuffer(b, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)["v"]
    v = v.reshape(-1, 3)
    return float((v.max(0) - v.min(0)).max())


def parts():
    """[(file, qty, label, filament, type)] for every printed part in the parts list."""
    out = []
    for sec, qty, label, files, _ in T.bom():
        if not sec.startswith("3D print"):
            continue
        for mo in re.finditer(r"([A-Za-z0-9_-]+?)(?:_(\d)\.\.(\d))?\.stl", files):
            stem = re.sub(r"^(PA-CF|PETG|TPU)_", "", mo.group(1))
            names = [f"{mo.group(1)}_{i}.stl" for i in range(int(mo.group(2)), int(mo.group(3)) + 1)] if mo.group(2) else [mo.group(0)]
            typ = next((t for pre, t in TYPE_OF if stem.startswith(pre)), "strong")
            q = 1 if mo.group(2) else (qty // len(re.findall(r"\.stl", files)) or 1)
            for n in names:
                out.append((n, q, label, T.filament_of_stl(stem), typ))
    return out


def html():
    P = parts()
    big = sorted({(round(longest_mm(T.OUT / "stl" / n)), n) for n, *_ in P if longest_mm(T.OUT / "stl" / n) > 300}, reverse=True)
    h = ['<div class="psettings">',
         '<p>These are for the real competition parts. The <a href="#prototype">printed prototype</a> further down prints '
         'everything in PETG with lighter settings. Each STL file name starts with its filament, so you can sort the folder '
         'into print batches.</p>',
         '<h3>Same settings on both printers</h3><ul>',
         '<li><b>Yes, use the same numbers on the Bambu H2C and the Creality K2 Plus.</b> The settings belong to the '
         'filament, not the printer. Start from each slicer\'s own profile for that filament (Bambu Studio: "Bambu PA6-CF", '
         '"Bambu PETG HF", "Bambu TPU 95A HF"; Creality Print: its PA-CF, PETG and TPU profiles), then check the numbers '
         'below and set the walls, infill and layer height from the part-type table.</li>',
         '<li><b>Nozzle for PA-CF:</b> carbon fiber wears out brass in a few hours. The H2C\'s nozzles are hardened steel. '
         'On the K2 Plus, check that the fitted nozzle is the hardened-steel one before printing PA-CF.</li>',
         '<li><b>TPU:</b> feed it from the external spool holder on either printer, not through the AMS or the CFS: soft '
         'filament jams in the feeders.</li>',
         '<li><b>Dry PA-CF every time.</b> Nylon soaks up water from the air within hours, and wet PA-CF prints weak and '
         'brittle with a rough surface. Dry it, then print straight from a dry box.</li>',
         '<li><b>No PLA on this turret.</b> It softens near 55–60 °C (a closed car, a warm motor) and slowly squashes under bolts.</li>']
    if big:
        h.append('<li><b>Over 300 mm long, so print on the K2 Plus (350 mm bed); on the H2C they leave almost no room '
                 'at the bed edges:</b> ' +
                 ", ".join(f"{n} ({mm} mm)" for mm, n in big) + '.</li>')
    h.append('</ul><h3>Filament settings</h3><div class="tablewrap"><table class="proto"><tr><th></th>' +
             "".join(f"<th>{FIL[m]['name']}</th>" for m in FIL) + "</tr>")
    for k, lab in ROWS:
        h.append(f"<tr><td><b>{lab}</b></td>" + "".join(f"<td>{FIL[m][k]}</td>" for m in FIL) + "</tr>")
    h.append('</table></div><p class="mut">Ranges are Bambu Lab\'s published ones for each filament; the first number is '
             'where to start.</p>')
    h.append('<h3>Settings by part type</h3><p class="mut">Walls are the outlines around the sides; top and bottom layers '
             'are the solid skins on the top and bottom faces. They are separate settings in the slicer. Make the skins about '
             'as thick as the walls (a 0.4 mm nozzle makes each wall about 0.42 mm wide).</p>')
    h.append('<div class="tablewrap"><table class="proto"><tr><th>Type</th><th>Parts</th>'
             '<th>Layer</th><th>Walls, top/bottom, infill</th><th>How to place it</th><th>Why</th></tr>')
    for typ, (layer, walls, place, why) in TYPES.items():
        names = []
        for n, q, lab, fil, t in P:
            if t == typ and n not in [x[0] for x in names]:
                names.append((n, q))
        if not names:
            continue
        h.append(f"<tr><td><b>{typ}</b></td><td>" + ", ".join(f"{n} ×{q}" for n, q in names) +
                 f"</td><td>{layer}</td><td>{walls}</td><td>{place}</td><td>{why}</td></tr>")
    h.append("</table></div></div>")
    return "\n".join(h)


def main():
    s = PAGE.read_text()
    a, b = "<!--PRINT_SETTINGS-->", "<!--/PRINT_SETTINGS-->"
    i, j = s.index(a) + len(a), s.index(b)
    PAGE.write_text(s[:i] + html() + s[j:])
    print("print settings written:", len(parts()), "files")


if __name__ == "__main__":
    main()
