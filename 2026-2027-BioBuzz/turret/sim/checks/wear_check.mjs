// Compares turretForward() with direct feed runs at in-between wear (wear_check.py). Exits 1 past the limits.
import { createRequire } from "module"; import fs from "fs";
const require = createRequire(import.meta.url);
const { make } = require("../sim_core.js");
const G = JSON.parse(fs.readFileSync(new URL("../../out/sim/sim_geom.json", import.meta.url)));
const C = JSON.parse(fs.readFileSync(new URL("./wear_check.json", import.meta.url)));
// limits: 95% of runs within P95, none beyond LIM (the worst cases are slow launches near where the feed sim stops launching)
const S = make(G), LIM = { launch: 1.5, exit: 2.0, spin: 20 }, P95 = { launch: 0.5, exit: 1.0, spin: 10 };   // deg, %, rpm
const e = { launch: [], exit: [], spin: [] }; let nul = 0; const worst = [];
for (const c of C) {
  const r = S.turretForward(c.ball, c.wear, c.hood, c.surface);
  if (!r) { nul++; continue; }
  const d = { launch: Math.abs(r.launch - c.launch), exit: 100 * Math.abs(r.exit / c.exit - 1), spin: Math.abs(r.spin - c.spin) };
  for (const k in e) e[k].push(d[k]);
  worst.push([Math.max(d.launch / LIM.launch, d.exit / LIM.exit, d.spin / LIM.spin), c, d]);
}
const q = (a, p) => { const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.floor(p * s.length))] ?? 0; };
console.log(`${C.length} direct runs, ${nul} not covered by the table`);
for (const k in e) console.log(`  ${k}: median ${q(e[k], 0.5).toFixed(2)}, 95% ${q(e[k], 0.95).toFixed(2)} (limit ${P95[k]}), worst ${Math.max(...e[k]).toFixed(2)} (limit ${LIM[k]})`);
worst.sort((a, b) => b[0] - a[0]).slice(0, 4).forEach(([s_, c, d]) => console.log("  worst:", c.ball, "wear", c.wear, "hood", c.hood, "wheel", c.surface, JSON.stringify(Object.fromEntries(Object.entries(d).map(([k, v]) => [k, +v.toFixed(2)])))));
const fail = C.length < 100 || nul > 10 || Object.keys(e).some(k => e[k].some(x => x > LIM[k]) || q(e[k], 0.95) > P95[k]);
process.exit(fail ? 1 : 0);
