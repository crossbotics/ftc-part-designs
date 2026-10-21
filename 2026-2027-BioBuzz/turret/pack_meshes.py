"""Pack viewer STLs into one JSON (int16-quantized triangle soups, base64) for the web viewer."""
import base64, json, struct
from pathlib import Path
import numpy as np

src = Path(__file__).parent / "out" / "viewer"
meta = json.loads((src / "viewer_meta.json").read_text())
meshes = {}
for f in sorted(src.glob("*.stl")):
    b = f.read_bytes(); n = struct.unpack_from("<I", b, 80)[0]
    tri = np.frombuffer(b, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
    v = tri["v"].reshape(-1, 3).astype(np.float64)
    lo, hi = v.min(0), v.max(0); scale = np.maximum(hi - lo, 1e-6) / 65535
    q = np.round((v - lo) / scale).astype(np.uint16)
    meshes[f.stem] = {"lo": lo.round(4).tolist(), "s": scale.tolist(), "b": base64.b64encode(q.tobytes()).decode()}
meta["meshes"] = meshes
vpath = Path(__file__).parent / "vendor_links.json"
if vpath.exists():
    meta["vendor_links"] = {k: v for k, v in json.loads(vpath.read_text()).items() if v}   # SKU -> product page (vendor_links.py)
dpath = src / "diagrams.json"
if dpath.exists():
    meta["diagrams"] = json.loads(dpath.read_text())   # design-decision figures (diagrams.py)
(src / "turret_model.json").write_text(json.dumps(meta, separators=(",", ":")))
print(f"{len(meshes)} meshes, {(src / 'turret_model.json').stat().st_size/1e6:.2f} MB")
