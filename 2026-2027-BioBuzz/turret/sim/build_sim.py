"""Builds the field simulator page: field_sim.html with sim_core.js, sim_geom.json and shoot_map.json inlined.
Writes ../out/sim/index.html (field.glb sits next to it, from ../../field_cad/export_field.py).

    python sim/build_sim.py        (from v7/, after sim_export.py)
"""
from pathlib import Path
HERE = Path(__file__).parent
OUT = HERE.parent / "out" / "sim"
html = (HERE / "field_sim.html").read_text()
core = (HERE / "sim_core.js").read_text()
geom = (OUT / "sim_geom.json").read_text()
smap = (OUT / "shoot_map.json").read_text()
for k, v in (("/*@SIMCORE@*/", core), ("/*@GEOM@*/", geom), ("/*@MAP@*/", smap)):
    assert html.count(k) == 1, k
    assert "</script" not in v, k
    html = html.replace(k, v)
(OUT / "index.html").write_text(html)
print("wrote", OUT / "index.html", f"{len(html) / 1e3:.0f} kB")
