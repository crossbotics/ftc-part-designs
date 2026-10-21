// Checks sim_core.js against the Python shooting map (shoot_map.json): same best launch angle and speed room.
//   node sim/validate.mjs [n]      (from v7/, after sim_export.py)
import { createRequire } from "module";
import fs from "fs";
const require = createRequire(import.meta.url);
const { make } = require("./sim_core.js");
const G = JSON.parse(fs.readFileSync(new URL("../out/sim/sim_geom.json", import.meta.url)));
const M = JSON.parse(fs.readFileSync(new URL("../out/sim/shoot_map.json", import.meta.url)));
const S = make(G);
const n = +(process.argv[2] || 24);
const rows = M.rows.filter(r => !r.frame);
// spread the sample over distance and bearing, deterministic
const pick = [];
for (let i = 0; i < n; i++) pick.push(rows[Math.floor((i + 0.5) * rows.length / n)]);
let worstRoom = 0, angleMiss = 0, scoreMismatch = 0, cases = 0;
const t0 = Date.now();
for (const r of pick) {
  for (const ball of ["Pollen", "Nectar"]) for (const state of ["down", "up"]) {
    const py = r[`${ball}-${state}`];
    // the map stores x, u rounded to 1 mm; recompute the exact spot from its distance and bearing, as shoot_map does
    const b = r.b * Math.PI / 180, js = S.solve([r.d * Math.sin(b), G.uc + r.d * Math.cos(b)], ball, state);
    cases++;
    if (!py !== !js.elev) { scoreMismatch++; console.log("MISMATCH score/no-score", r.d, r.b, ball, state, py, js); continue; }
    if (!py) continue;
    const dRoom = Math.abs(py[1] - js.room);
    worstRoom = Math.max(worstRoom, dRoom);
    if (py[0] !== js.elev) { angleMiss++; console.log(`angle differs at ${r.d} m ${r.b}°, ${ball}/${state}: py ${py[0]}° ±${py[1]}%, js ${js.elev}° ±${js.room.toFixed(2)}%`); }
  }
}
const ms = (Date.now() - t0) / cases;
console.log(`${cases} cases: score/no-score mismatches ${scoreMismatch}, angle differences ${angleMiss}, worst speed-room difference ${worstRoom.toFixed(3)} %-points; ${ms.toFixed(0)} ms per solve`);
process.exit(scoreMismatch || angleMiss || worstRoom > 0.05 ? 1 : 0);
