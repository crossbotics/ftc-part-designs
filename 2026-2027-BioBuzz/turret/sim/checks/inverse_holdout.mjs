// Regression gate for sim_core.turretShot() (run inverse_holdout.py first, then this, then inverse_replay.py).
//  1. every table entry the feed sim launched must be found from its own launch angle and exit speed;
//  2. every hold-out shot (feed runs off the table grid) must be found; the chosen settings go to holdout_inverse.json
//     so inverse_replay.py can run them through the feed sim again and compare what actually comes out.
// Exits 1 on any miss.       node sim/checks/inverse_holdout.mjs
import { createRequire } from "module"; import fs from "fs";
const require = createRequire(import.meta.url);
const { make } = require("../sim_core.js");
const G = JSON.parse(fs.readFileSync(new URL("../../out/sim/sim_geom.json", import.meta.url)));
const H = JSON.parse(fs.readFileSync(new URL("./holdout.json", import.meta.url)));
const S = make(G), T = G.turret_shots;
let knots = 0, knotMiss = 0;
for (const b in T.balls) for (const w of T.wear) { const R = T.balls[b][w];
  R.exit.forEach((row, i) => row.forEach((e, j) => { if (!R.ok[i][j]) return; knots++; if (!S.turretShot(b, w, R.launch[i][j], e)) knotMiss++; })); }
// a hold-out setting next to a table entry the feed sim didn't launch is outside the valid table: not finding it is right
const validCell = (c) => { const R = T.balls[c.ball][c.wear], Sx = T.surface, Hx = T.hood;
  const i = Sx.findIndex((x, k) => k < Sx.length - 1 && c.surface >= x && c.surface <= Sx[k + 1]);
  const j = Math.min(Hx.length - 2, Hx.findIndex((x, k) => k < Hx.length - 1 && c.hood >= x && c.hood <= Hx[k + 1]));
  return i >= 0 && j >= 0 && R.ok[i][j] && R.ok[i][j + 1] && R.ok[i + 1][j] && R.ok[i + 1][j + 1]; };
let miss = 0, outside = 0; const chosen = [];
for (const c of H) {
  if (!validCell(c)) { outside++; continue; }
  const r = S.turretShot(c.ball, c.wear, c.launch, c.exit);
  if (!r) { miss++; console.log("  not found:", c.ball, c.wear, "hood", c.hood, "wheel", c.surface); continue; }
  chosen.push({ ...c, pick_hood: r.hood, pick_surface: r.surface, pick_spin: r.spin });
}
fs.writeFileSync(new URL("./holdout_inverse.json", import.meta.url), JSON.stringify(chosen));
console.log(`table entries: ${knots}, not found ${knotMiss}; hold-out shots: ${H.length} (${outside} outside the valid table), not found ${miss}`);
// coverage (Astra round 3): an empty or thin test must fail, not pass
process.exit(knotMiss || miss || chosen.length < 60 || outside > 5 ? 1 : 0);
