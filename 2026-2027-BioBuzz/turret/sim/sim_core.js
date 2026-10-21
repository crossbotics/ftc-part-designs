/* Field simulator physics. A line-by-line port of approach_check.shots() / shoot_map.point() (v7), so the
 * browser judges shots with the same rules and step order as the Python models behind the page's figures (known
 * difference: Shapely's rounded buffer corners vs exact distance, under 0.06 mm). All numbers come from
 * sim_geom.json (written by sim_export.py). validate.mjs checks this file against shoot_map.json.
 *
 * HIVE frame (metres): x along the pivot axis (+x toward the other HIVE), u out toward our up-CELL, z up.
 * Our HIVE's pivot is at (0, 0, pivot_z). "state" is the other HIVE: "up" = tipped the same way as ours,
 * "down" = tipped the opposite way (its down-CELL hangs on our side).
 */
(function (root) {
  "use strict";
  const C30 = Math.cos(Math.PI / 6), S30 = 0.5, IN = 0.0254;
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const mul = (a, k) => [a[0] * k, a[1] * k, a[2] * k];

  function segDist(p, a, b) {
    const ab = sub(b, a), t = Math.min(1, Math.max(0, dot(sub(p, a), ab) / dot(ab, ab)));
    const c = add(a, mul(ab, t));
    return Math.hypot(p[0] - c[0], p[1] - c[1], p[2] - c[2]);
  }
  function segClosest(p, a, b) {
    const ab = sub(b, a), t = Math.min(1, Math.max(0, dot(sub(p, a), ab) / dot(ab, ab)));
    return add(a, mul(ab, t));
  }
  // 2D polygon helpers (convex, counter-clockwise)
  function offsetConvex(poly, d) {          // shapely buffer(d, join_style=mitre) of a convex CCW polygon
    const n = poly.length, lines = [];
    for (let i = 0; i < n; i++) {
      const a = poly[i], b = poly[(i + 1) % n], dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy);
      const nx = dy / L, ny = -dx / L;
      lines.push({ p: [a[0] + nx * d, a[1] + ny * d], d: [dx, dy], n: [nx, ny] });
    }
    const out = [];
    for (let i = 0; i < n; i++) {
      const l1 = lines[(i - 1 + n) % n], l2 = lines[i];
      const den = l1.d[0] * l2.d[1] - l1.d[1] * l2.d[0];
      const t = ((l2.p[0] - l1.p[0]) * l2.d[1] - (l2.p[1] - l1.p[1]) * l2.d[0]) / den;
      out.push([l1.p[0] + l1.d[0] * t, l1.p[1] + l1.d[1] * t]);
    }
    return out;
  }
  function inPoly(poly, x, y) {             // strictly inside a convex CCW polygon
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      if ((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) <= 0) return false;
    }
    return true;
  }
  function segDist2(x, y, a, b) {
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const t = Math.min(1, Math.max(0, ((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy)));
    return Math.hypot(x - a[0] - dx * t, y - a[1] - dy * t);
  }
  function polyDist(poly, x, y) {
    let m = Infinity;
    for (let i = 0; i < poly.length; i++) m = Math.min(m, segDist2(x, y, poly[i], poly[(i + 1) % poly.length]));
    return m;
  }

  function make(G) {
    const PZ = G.pivot_z, PIV = [0, 0, PZ], FT = G.frame_tube, NX = G.neigh_x;
    const AX = [0, C30, S30], NUP = [0, -S30, C30], EX = [1, 0, 0], NORM = [0, C30, S30];
    const B3 = G.B, S0 = G.s0, A_IN = G.a_in, A_OUT = G.a_out;
    const PENT = G.pent;
    const RIM = PENT.map((q, i) => {
      const q2 = PENT[(i + 1) % PENT.length];
      return [add(add(B3, mul(EX, q[0])), mul(NUP, q[1])), add(add(B3, mul(EX, q2[0])), mul(NUP, q2[1]))];
    });
    const SEC = offsetConvex(PENT.map(([x, s]) => [x, S0 + s]), FT);      // CELL cross-section + frame allowance
    let bx0 = Infinity, bx1 = -Infinity, bs0 = Infinity, bs1 = -Infinity;
    SEC.forEach(([x, s]) => { bx0 = Math.min(bx0, x); bx1 = Math.max(bx1, x); bs0 = Math.min(bs0, s); bs1 = Math.max(bs1, s); });
    const prism = (ax, up, xc) => ({ ax, up, xc, a0: A_IN - 0.5 * IN, a1: A_OUT + 0.5 * IN });
    function neighbour(state) {
      let ax, up;
      if (state === "down") { ax = [0, C30, -S30]; up = [0, S30, C30]; } else { ax = AX; up = NUP; }
      const prisms = [prism(ax, up, NX), prism(mul(ax, -1), up, NX)];
      return { prisms, bars: G.neigh_bars[state] };              // the other HIVE's bar, exported from the model
    }
    const rods = G.rods;                                          // [a, b, radius]: frame tubes, pivot blocks, our bar
    const own = [prism(mul(AX, -1), NUP, 0)];
    const NEIGH = { down: neighbour("down"), up: neighbour("up") };

    function inUpBody(p, r) {                 // as approach_check.in_up_body()
      const q = sub(p, PIV), a = dot(q, AX), s = dot(q, NUP), x = q[0];
      if (!(a > A_IN - 0.5 * IN - r && a < A_OUT && x > bx0 - r && x < bx1 + r && s > bs0 - r && s < bs1 + r)) return false;
      return inPoly(SEC, x, s) || polyDist(SEC, x, s) < r;
    }
    function inPrism(p, pr, r) {
      const q = sub(p, PIV), a = dot(q, pr.ax), s = dot(q, pr.up), x = q[0] - pr.xc;
      if (!(a > pr.a0 - r && a < pr.a1 + r && x > bx0 - r && x < bx1 + r && s > bs0 - r && s < bs1 + r)) return false;
      return inPoly(SEC, x, s) || polyDist(SEC, x, s) < r;
    }

    // opening centre, horizontal distance from the pivot line: (B.u + P.u) / 2, P = roof peak
    const P3 = G.P, uc = G.uc;                 // roof peak and opening centre, from entry_window (exported)

    /* One shot, same rules and step order as approach_check.shots(). p0, v0 in the HIVE frame.
     * Returns why: 0 in, 1 miss, 2 touches the rim, 3 would score but hits the other HIVE first, 4 ... the frame
     * or our own bar / down-CELL. */
    // Axis-aligned boxes around each group of parts. judge() only runs the exact tests when the ball's centre is
    // inside a box grown by the ball radius plus the frame allowance, so skipping never changes a result.
    const boxOf = (pts) => { const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
      pts.forEach(q => { for (let i = 0; i < 3; i++) { lo[i] = Math.min(lo[i], q[i]); hi[i] = Math.max(hi[i], q[i]); } }); return [lo, hi]; };
    const prismCorners = (pr) => {
      const out = [];
      for (const a of [pr.a0, pr.a1]) for (const [x, s] of SEC) out.push(add(add(PIV, mul(pr.ax, a)), add(mul(EX, x + pr.xc), mul(pr.up, s))));
      return out;
    };
    const near = (p, box, g) => p[0] > box[0][0] - g && p[0] < box[1][0] + g && p[1] > box[0][1] - g && p[1] < box[1][1] + g && p[2] > box[0][2] - g && p[2] < box[1][2] + g;
    const RIM_BOX = boxOf(RIM.flat());
    const ROD_BOXES = rods.map(([a, b]) => boxOf([a, b]));
    const OWN_BOX = boxOf(own.flatMap(prismCorners));
    const NBOX = {}; for (const st of ["down", "up"]) NBOX[st] = { prisms: boxOf(NEIGH[st].prisms.flatMap(prismCorners)), bars: NEIGH[st].bars.map(sg => boxOf(sg)) };

    /* Backspin (Magnus lift), optional: spinRpm about the horizontal axis square to the launch direction, top of the
     * ball moving back. Lift coefficient C_L = r*omega/|v| (the usual rule for a slowly spinning ball), so the force is
     * 0.5*rho*pi*r^3*(omega x v). Spin is taken as constant through the flight. spinRpm = 0 gives the Python model. */
    function spinVec(v0, spinRpm) {
      if (!spinRpm) return null;
      const L = Math.hypot(v0[0], v0[1]) || 1, w = spinRpm * 2 * Math.PI / 60;
      return [v0[1] / L * w, -v0[0] / L * w, 0];
    }
    function judge(p0, v0, ball, state, dt = 1e-3, tmax = 2.0, spinRpm = 0) {
      const { m, r } = G.balls[ball];
      const W = spinVec(v0, spinRpm), cm = 0.5 * G.rho * Math.PI * r * r * r / m;
      const NB = NBOX[state], gR = r + 1e-6, gF = r + FT + 1e-6;
      const k = 0.5 * G.rho * G.cd * Math.PI * r * r / m;
      const N = NEIGH[state];
      let p = p0.slice(), v = v0.slice();
      let why = -1, rimMin = 9, blocked = 0, entered = false;
      let prev = dot(sub(p, B3), NORM);
      const nsteps = Math.trunc(tmax / dt);         // as Python's int(tmax / dt)
      for (let it = 0; it < nsteps; it++) {
        const sp = Math.hypot(v[0], v[1], v[2]);
        // drag + gravity (+ Magnus), all from this step's starting velocity (Astra: one combined forward-Euler step)
        const m0 = W ? cm * (W[1] * v[2] - W[2] * v[1]) : 0, m1 = W ? cm * (W[2] * v[0] - W[0] * v[2]) : 0, m2 = W ? cm * (W[0] * v[1] - W[1] * v[0]) : 0;
        v[0] += (m0 - k * sp * v[0]) * dt; v[1] += (m1 - k * sp * v[1]) * dt;
        v[2] += (m2 - G.g - k * sp * v[2]) * dt;
        p = [p[0] + v[0] * dt, p[1] + v[1] * dt, p[2] + v[2] * dt];
        const f = dot(sub(p, B3), NORM);
        // rim clearance only matters once it's under r; outside the rim's box grown by r it can't be
        if (near(p, RIM_BOX, gR)) for (const [a, b] of RIM) rimMin = Math.min(rimMin, segDist(p, a, b));
        let hitN = false, hitF = false;
        if (near(p, NB.prisms, gF)) for (const pr of N.prisms) if (!hitN && inPrism(p, pr, r)) hitN = true;
        N.bars.forEach(([a, b], i) => { if (!hitN && near(p, NB.bars[i], gF) && segDist(p, a, b) < r + FT) hitN = true; });
        rods.forEach(([a, b, rad], i) => { if (!hitF && near(p, ROD_BOXES[i], r + rad + 1e-6) && segDist(p, a, b) < r + rad) hitF = true; });
        if (!hitF && near(p, OWN_BOX, gF)) for (const pr of own) if (!hitF && inPrism(p, pr, r)) hitF = true;
        if (!entered) {
          if (hitN && blocked === 0) blocked = 3;
          if (hitF && blocked === 0) blocked = 4;
        }
        const cross = prev > 0 && f <= 0;
        let inside = false;
        if (cross) { const q = sub(p, B3); inside = inPoly(PENT, q[0], dot(q, NUP)); }
        if (cross && !inside && why < 0) why = 1;
        const desc = Math.atan2(-v[2], Math.hypot(v[0], v[1])) * 180 / Math.PI;
        if (cross && inside && desc < G.min_desc && why < 0) why = 1;
        if (cross && inside && desc >= G.min_desc) entered = true;
        // our own up-CELL is solid from outside: behind the opening plane without having come through the opening
        if (!entered && blocked === 0 && inUpBody(p, r)) blocked = 4;
        if (entered && f < -r && why < 0) why = rimMin < r ? 2 : 0;
        let live = why < 0;
        live = live && !(v[2] < 0 && p[2] < B3[2] - 0.3) && Math.abs(p[0]) < 6 && Math.abs(p[1]) < 6;
        prev = f;
        if (!live) break;
      }
      if (why < 0) why = 1;
      if (why === 0 && blocked > 0) why = blocked;
      return why;
    }

    /* Can the Limelight read our up-CELL's tags? As camera_coverage.sees_target(), but the camera looks along the
     * turret's actual yaw (dir: unit plan vector in the HIVE frame), so it also works when the turret is aimed by hand. */
    function seesTarget(xu, dir, tiltDeg) {
      const C = G.camera, t = tiltDeg * Math.PI / 180;
      const cam = [xu[0], xu[1], C.h];
      const fwd = [dir[0] * Math.cos(t), dir[1] * Math.cos(t), Math.sin(t)];
      const right = [dir[1], -dir[0], 0];
      const up = [right[1] * fwd[2] - right[2] * fwd[1], right[2] * fwd[0] - right[0] * fwd[2], right[0] * fwd[1] - right[1] * fwd[0]];
      const v = sub(C.target_p, cam), d = Math.hypot(...v), w = mul(v, 1 / d);
      const inc = Math.acos(Math.max(-1, Math.min(1, -dot(w, C.target_n))));
      const res = { seen: false, inc: inc * 180 / Math.PI, dist: d, px: 0, why: "" };
      if (inc > C.max_inc) { res.why = "the tags are turned too far away from the camera"; return res; }
      const a = dot(w, right), b = dot(w, up), c = dot(w, fwd);
      if (c <= 0 || Math.abs(Math.atan2(a, c)) > C.hfov / 2) { res.why = "the tags are outside the camera's view (side to side)"; return res; }
      if (Math.abs(Math.atan2(b, c)) > C.vfov / 2) { res.why = Math.atan2(b, c) > 0 ? "the tags are above the camera's view" : "the tags are below the camera's view"; return res; }
      res.px = C.tag * Math.cos(inc) / d * (C.hpix / 2) / Math.tan(C.hfov / 2);
      if (res.px < C.min_px) { res.why = "the tags are too small (too far or too slanted)"; return res; }
      res.seen = true; return res;
    }
    /* What the v7 turret does for a wanted launch angle and exit speed (feed-simulation table, sim_export.turret_table):
     * the hood angle and wheel surface speed that give them for this ball and wear, and the backspin that comes out.
     * Returns {hood, surface, spin} or null when the turret can't launch that (angle outside the hood's reach for this
     * ball, or faster than the wheels can throw it). */
    function turretShot(ball, wear, elev, v) {
      // Candidates: exact table entries, edges between two valid entries, and table cells (2 hood angles x 2 wheel speeds,
      // all four launched: bilinear launch(u, w) and exit(u, w) = the wanted angle and speed, damped Newton from a few
      // starts). POLICY: when more than one setting throws this angle and speed, use the lowest wheel speed (least motor
      // effort; on worn Pollen's plateau they're the same ball). Other settings can give a different spin (Astra round 2:
      // up to ~70 rpm for medium Nectar near hood 25); no case was found where that changed whether a shot scores.
      const T = G.turret_shots, R = T.balls[ball][wear], S_ = T.surface, H = T.hood, O = R.ok;
      let best = null;
      const take = (hood, surface, spin) => { if (!best || surface < best.surface - 1e-6) best = { hood, surface, spin }; };
      // exact table entries (an isolated valid entry has no valid cell around it)
      for (let i = 0; i < S_.length; i++) for (let j = 0; j < H.length; j++)
        if (O[i][j] && Math.abs(R.launch[i][j] - elev) < 0.05 && Math.abs(R.exit[i][j] - v) < 0.006) take(H[j], S_[i], R.spin[i][j]);
      // edges between two valid neighbouring entries (least squares along the edge, same tolerances)
      const edge = (i0, j0, i1, j1) => {
        const dL = R.launch[i1][j1] - R.launch[i0][j0], dE = 100 * (R.exit[i1][j1] - R.exit[i0][j0]);
        const rl = elev - R.launch[i0][j0], re = 100 * (v - R.exit[i0][j0]), den = dL * dL + dE * dE;
        if (den < 1e-12) return;
        const t = Math.min(1, Math.max(0, (dL * rl + dE * re) / den));
        if (Math.abs(dL * t - rl) < 0.05 && Math.abs(dE * t - re) < 0.6)
          take(H[j0] + (H[j1] - H[j0]) * t, S_[i0] + (S_[i1] - S_[i0]) * t, R.spin[i0][j0] + (R.spin[i1][j1] - R.spin[i0][j0]) * t);
      };
      for (let i = 0; i < S_.length; i++) for (let j = 0; j < H.length; j++) {
        if (!O[i][j]) continue;
        if (i + 1 < S_.length && O[i + 1][j]) edge(i, j, i + 1, j);
        if (j + 1 < H.length && O[i][j + 1]) edge(i, j, i, j + 1);
      }
      const bil = (M, i, j, u, w) => M[i][j] * (1 - u) * (1 - w) + M[i][j + 1] * u * (1 - w) + M[i + 1][j] * (1 - u) * w + M[i + 1][j + 1] * u * w;
      for (let i = 0; i < S_.length - 1; i++) for (let j = 0; j < H.length - 1; j++) {
        if (!(O[i][j] && O[i][j + 1] && O[i + 1][j] && O[i + 1][j + 1])) continue;
        const Ls = [R.launch[i][j], R.launch[i][j + 1], R.launch[i + 1][j], R.launch[i + 1][j + 1]];
        const Es = [R.exit[i][j], R.exit[i][j + 1], R.exit[i + 1][j], R.exit[i + 1][j + 1]];
        // tolerances: 0.05 deg and 6 mm/s (the feed model isn't exactly bilinear inside a cell; replayed errors are larger)
        if (elev < Math.min(...Ls) - 0.05 || elev > Math.max(...Ls) + 0.05 || v < Math.min(...Es) - 0.006 || v > Math.max(...Es) + 0.006) continue;
        for (const [u0, w0] of [[0.5, 0], [0.5, 0.5], [0, 0], [1, 0], [0.5, 1]]) {
          let u = u0, w = w0;
          for (let it = 0; it < 30; it++) {
            const rl = bil(R.launch, i, j, u, w) - elev, re = (bil(R.exit, i, j, u, w) - v) * 100;     // scale: 1 deg ~ 0.01 m/s
            if (Math.abs(rl) < 0.005 && Math.abs(re) < 0.005) break;
            const Lu = (1 - w) * (Ls[1] - Ls[0]) + w * (Ls[3] - Ls[2]), Lw = (1 - u) * (Ls[2] - Ls[0]) + u * (Ls[3] - Ls[1]);
            const Eu = 100 * ((1 - w) * (Es[1] - Es[0]) + w * (Es[3] - Es[2])), Ew = 100 * ((1 - u) * (Es[2] - Es[0]) + u * (Es[3] - Es[1]));
            const lam = 1e-3, a = Lu * Lu + Eu * Eu + lam, b = Lu * Lw + Eu * Ew, c = Lw * Lw + Ew * Ew + lam;   // (J^T J + lam I) d = -J^T r
            const g1 = Lu * rl + Eu * re, g2 = Lw * rl + Ew * re, det = a * c - b * b;
            if (Math.abs(det) < 1e-12) break;
            u = Math.min(1, Math.max(0, u - (c * g1 - b * g2) / det)); w = Math.min(1, Math.max(0, w - (a * g2 - b * g1) / det));
          }
          if (Math.abs(bil(R.launch, i, j, u, w) - elev) < 0.05 && Math.abs(bil(R.exit, i, j, u, w) - v) < 0.006) {
            take(H[j] + u * (H[j + 1] - H[j]), S_[i] + w * (S_[i + 1] - S_[i]), bil(R.spin, i, j, u, w));
            break;
          }
        }
      }
      return best;
    }
    /* Forward: what the turret actually throws for a hood angle and wheel speed, for a ball of any wear index from new (0)
     * through medium (1) to worn (2) (= ball stiffness 8000 -> 4000 -> 1500 N/m, straight in between; the table has 9 wear
     * levels, a quarter apart). Bilinear inside a table cell whose 4 corners launched, linear along a valid edge, exact at
     * a valid entry; linear between the two neighbouring wear levels. null where the feed sim has no launch to go on
     * (outside the table, or next to a setting that didn't launch): the page treats that ball as not launched. */
    function forwardAt(R, surface, hood) {
      const S_ = G.turret_shots.surface, H = G.turret_shots.hood, O = R.ok, eps = 1e-6;
      for (let i = 0; i < S_.length - 1; i++) {
        if (surface < S_[i] - eps || surface > S_[i + 1] + eps) continue;
        const w = Math.min(1, Math.max(0, (surface - S_[i]) / (S_[i + 1] - S_[i])));
        for (let j = 0; j < H.length - 1; j++) {
          if (hood < H[j] - eps || hood > H[j + 1] + eps) continue;
          const u = Math.min(1, Math.max(0, (hood - H[j]) / (H[j + 1] - H[j])));
          const c = [[i, j, (1 - u) * (1 - w)], [i, j + 1, u * (1 - w)], [i + 1, j, (1 - u) * w], [i + 1, j + 1, u * w]].filter(q => q[2] > 1e-9);
          if (c.every(([a, b]) => O[a][b])) {       // every corner that carries weight launched (a knot or an edge needs only its own)
            const f = (M) => c.reduce((acc, [a, b, wt]) => acc + M[a][b] * wt, 0);
            return { launch: f(R.launch), exit: f(R.exit), spin: f(R.spin) };
          }
        }
      }
      return null;
    }
    function turretForward(ball, wearIdx, hood, surface) {
      const T = G.turret_shots, WI = T.wear_idx || [0, 1, 2], x = Math.min(2, Math.max(0, wearIdx));
      let k = WI.findIndex((a, n) => n < WI.length - 1 && x >= a - 1e-9 && x <= WI[n + 1] + 1e-9);
      if (k < 0) return null;
      const f = (x - WI[k]) / (WI[k + 1] - WI[k]);
      const a = f < 1 - 1e-9 ? forwardAt(T.balls[ball][T.wear[k]], surface, hood) : null;
      const b = f > 1e-9 ? forwardAt(T.balls[ball][T.wear[k + 1]], surface, hood) : null;
      if (f <= 1e-9) return a;
      if (f >= 1 - 1e-9) return b;
      if (!a || !b) return null;                  // don't bridge a wear level that didn't launch
      return { launch: a.launch + (b.launch - a.launch) * f, exit: a.exit + (b.exit - a.exit) * f, spin: a.spin + (b.spin - a.spin) * f };
    }
    /* One turret shot with shot-to-shot variation: the turret is set (hood, wheel speed) for the planned launch angle and
     * exit speed at the chosen wear; this ball's wear, the hood and the wheel speed then scatter (V = 1-sigma values:
     * wear index, hood deg, wheel-speed %), and the feed table gives what actually leaves. null = didn't launch.
     * normal() draws a standard normal number. Used by the page's Fire / hit rate and by pickShot(). */
    function variedTurret(ball, wear, elev, v, V, normal) {
      const set = turretShot(ball, wear, elev, v); if (!set) return null;
      const WI = G.turret_shots.wear_idx, wi = WI[G.turret_shots.wear.indexOf(wear)];
      let w = wi + normal() * V.wear;                                            // kept between new and worn by reflecting
      while (w < 0 || w > 2) w = w < 0 ? -w : 4 - w;
      const hood = Math.min(25, Math.max(0, set.hood + normal() * V.hood));     // hood play, inside the mechanism's range
      const surf = Math.min(10.0, set.surface * (1 + normal() * V.spd / 100));  // the motor can't spin the wheels past ~10 m/s
      const act = turretForward(ball, w, hood, surf);
      return act ? { ...act, w } : { failed: true, w };
    }
    // Seeded standard-normal generator (xorshift + Box-Muller): the same "random" balls for every candidate angle, so
    // their hit rates compare fairly and the pick doesn't flicker between solves.
    function seededNormal(seed = 12345) {
      let s = seed >>> 0 || 1;
      const u = () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return (s + 0.5) / 4294967296; };
      return () => Math.sqrt(-2 * Math.log(u())) * Math.cos(2 * Math.PI * u());
    }
    function hitRateAt(xu, ball, state, wear, elev, v, V, N = 300, seed = 12345) {
      const nrm = seededNormal(seed), d = aimDir(xu); let hit = 0;
      for (let n = 0; n < N; n++) {
        const a = variedTurret(ball, wear, elev, v, V, nrm), y = nrm() * V.aim * Math.PI / 180;
        if (!a || a.failed) continue;
        const dir = [d[0] * Math.cos(y) - d[1] * Math.sin(y), d[0] * Math.sin(y) + d[1] * Math.cos(y)];
        const L = launchDir(xu, a.launch, a.exit, dir);
        if (judge(L.p0, L.v0, ball, state, 1e-3, 2.0, a.spin) === 0) hit++;
      }
      return hit / N;
    }
    /* Which of the solver's candidate angles to use. The speed room alone is nearly flat across angles at longer range
     * (e.g. Nectar from a corner: 3.5% at 57 deg, 4.0% at 81 deg), so "widest room" often picked a lob, but a lob is
     * much more sensitive to hood and launch-angle error, needs ~9-10 m/s (the motor's limit) and spends longer in the
     * air. With variation on (turret model): the candidate with the best hit rate under that variation (300 shots,
     * same random balls for each); within 3 points, the lower exit speed. Variation off: the lowest exit speed within
     * 0.5 points of the widest speed room. Returns the chosen candidate (+ p when rated). */
    function pickShot(xu, ball, state, wear, cands, V, N = 300) {
      if (!cands || !cands.length) return null;
      const bestRoom = Math.max(...cands.map(c => c.room));
      const lowV = (list) => list.reduce((a, c) => (c.v < a.v ? c : a));
      if (!V || !(V.wear || V.hood || V.spd || V.aim)) return lowV(cands.filter(c => c.room >= bestRoom - 0.5));
      const rated = cands.filter(c => c.room >= Math.min(2, bestRoom * 0.5)).map(c => ({ ...c, p: hitRateAt(xu, ball, state, wear, c.elev, c.v, V, N) }));
      const bestP = Math.max(...rated.map(c => c.p));
      return lowV(rated.filter(c => c.p >= bestP - 0.03));
    }
    // the launch angles the turret reaches for this ball and wear (turret model), from the table
    function turretLaunchRange(ball, wear) {
      const R = G.turret_shots.balls[ball][wear]; let lo = 90, hi = 0;
      R.launch.forEach((row, i) => row.forEach((l, j) => { if (R.ok[i][j]) { lo = Math.min(lo, l); hi = Math.max(hi, l); } }));
      return [lo, hi];
    }
    // a shot in any plan direction (manual aiming): dir = unit plan vector in the HIVE frame
    function launchDir(xu, elevDeg, speed, dir, launchZ) {
      const e = elevDeg * Math.PI / 180, vh = speed * Math.cos(e);
      return { p0: [xu[0], xu[1], launchZ ?? G.launch_z], v0: [vh * dir[0], vh * dir[1], speed * Math.sin(e)] };
    }
    const aimDir = (xu) => { const d = [0 - xu[0], uc - xu[1]], L = Math.hypot(d[0], d[1]); return [d[0] / L, d[1] / L]; };
    function launch(xu, elevDeg, speed, launchZ) {
      const e = elevDeg * Math.PI / 180, dp = aimDir(xu), vh = speed * Math.cos(e);
      return { p0: [xu[0], xu[1], launchZ ?? G.launch_z], v0: [vh * dp[0], vh * dp[1], speed * Math.sin(e)] };
    }
    function arange(a, b, s) { const o = []; for (let i = 0; a + i * s < b - 1e-9; i++) o.push(a + i * s); return o; }
    const SPEEDS = arange(G.speeds[0], G.speeds[1], G.speeds[2]);
    function window_(whys, speeds, okFn) {       // widest contiguous run of good speeds -> [mid, +/-%] or null
      let best = null, cur = null;
      whys.forEach((w, i) => {
        if (okFn(w)) {
          cur = cur ? [cur[0], i] : [i, i];
          if (!best || cur[1] - cur[0] > best[1] - best[0]) best = cur.slice();
        } else cur = null;
      });
      if (!best || best[1] === best[0]) return null;
      const lo = speeds[best[0]], hi = speeds[best[1]];
      return [(lo + hi) / 2, 100 * (hi - lo) / (hi + lo)];
    }
    /* Best launch angle at a spot, as shoot_map.point(): the angle (2 degree steps over the hood's range) with the
     * widest run of exit speeds that score. Returns {elev, v, room, free, counts} or {elev: null, free}. Works in
     * chunks (one angle per call of step()) so the page stays smooth. */
    function solver(xu, ball, state, launchZ, spinRpm = 0, elevRange = null) {
      // Simple model: the hood range as in the Python map. Turret model: what the turret reaches for this ball and wear
      // (2 degree steps from the lowest whole degree).
      const [e0, e1] = elevRange ? [Math.ceil(elevRange[0]), Math.floor(elevRange[1])] : G.launch[ball];
      const els = arange(e0, e1 + 0.1, G.elev_step);
      let i = 0, best = null, free = 0; const cands = [];
      const tally = { 1: 0, 2: 0, 3: 0, 4: 0, 5: 0 };
      return {
        n: els.length,
        step() {
          if (i >= els.length) return true;
          const el = els[i++];
          const whys = SPEEDS.map(s => {
            const sp = typeof spinRpm === "function" ? spinRpm(el, s) : spinRpm;
            if (sp === null) return 5;                                  // the turret can't launch this
            const L = launch(xu, el, s, launchZ); return judge(L.p0, L.v0, ball, state, 1e-3, 2.0, sp);
          });
          whys.forEach(w => { if (w > 0) tally[w]++; });
          const w = window_(whys, SPEEDS, x => x === 0);
          if (w) cands.push({ elev: el, v: w[0], room: w[1] });
          if (w && (!best || w[1] > best.room)) best = { elev: el, v: w[0], room: w[1] };
          const fw = window_(whys, SPEEDS, x => x === 0 || x === 3 || x === 4);
          free = Math.max(free, fw ? fw[1] : 0);
          return i >= els.length;
        },
        result() { return best ? { ...best, free, tally, cands } : { elev: null, free, tally }; },
      };
    }
    function solve(xu, ball, state, launchZ, spinRpm = 0, elevRange = null) { const s = solver(xu, ball, state, launchZ, spinRpm, elevRange); while (!s.step()); return s.result(); }

    /* Flight for the animation: the same drag model, plus rough bounces off the structure, the floor and the
     * inside of the CELL (the bounces are for show; judge() decides what scores). Samples every `every` s. */
    const UP_CELL = prism(AX, NUP, 0);
    function contactPrism(p, pr, r) {       // rough signed-distance contact with a CELL box -> {depth, n} or null
      const q = sub(p, PIV), a = dot(q, pr.ax), s = dot(q, pr.up), x = q[0] - pr.xc;
      let bestD = -Infinity, n = null;
      for (let i = 0; i < SEC.length; i++) {
        const A = SEC[i], Bp = SEC[(i + 1) % SEC.length], dx = Bp[0] - A[0], dy = Bp[1] - A[1], L = Math.hypot(dx, dy);
        const nx = dy / L, ny = -dx / L, d = (x - A[0]) * nx + (s - A[1]) * ny;
        if (d > bestD) { bestD = d; n = add(mul(EX, nx), mul(pr.up, ny)); }
      }
      const da0 = pr.a0 - a, da1 = a - pr.a1;
      if (da0 > bestD) { bestD = da0; n = mul(pr.ax, -1); }
      if (da1 > bestD) { bestD = da1; n = pr.ax; }
      return bestD < r ? { depth: r - bestD, n } : null;
    }
    function fly(p0, v0, ball, state, opts = {}) {
      const { m, r } = G.balls[ball];
      const k = 0.5 * G.rho * G.cd * Math.PI * r * r / m;
      const N = NEIGH[state], dt = 5e-4, every = opts.every ?? 1 / 120, tmax = opts.tmax ?? 3.5;
      const W = spinVec(v0, opts.spin || 0), cm = 0.5 * G.rho * Math.PI * r * r * r / m;
      const colliders = [...RIM.map(s => [s, r]), ...rods.map(([a, b, rad]) => [[a, b], r + rad]), ...N.bars.map(s => [s, r + FT])];
      const boxes = [...N.prisms, ...own];
      const others = opts.others || [];         // balls resting in our CELL: [x, u, z, r] (they don't move)
      let p = p0.slice(), v = v0.slice(), prev = dot(sub(p, B3), NORM), inCell = !!opts.startInCell, entered = !!opts.startInCell, rest = 0, hitBall = false, settled = false;
      // path samples: [t, x, u, z, nx, nu, nz]; n = the surface the ball is touching (0 0 0 in the air), so the page can
      // spin it in flight and roll it on surfaces
      const path = [[0, p[0], p[1], p[2], 0, 0, 0]];
      let tNext = every, cN = null, touching = false, sampN = null;
      const IMPACT = 0.3, CRR = 0.03;             // m/s: slower than this into a surface = resting/rolling contact; rolling resistance
      // A hit (fast into the surface) loses speed once: restitution e, and mu of the sliding speed. Slow contact (resting,
      // rolling) only stops the ball sinking in; rolling is handled after all contacts, per unit time.
      const bounce = (n, e = 0.45, mu = 0.8) => {
        const vn = dot(v, n); cN = n;
        if (vn < -IMPACT) { const vt = sub(v, mul(n, vn)); v = add(mul(vt, mu), mul(n, -e * vn)); }
        else if (vn < 0) v = sub(v, mul(n, vn));
      };
      for (let t = dt; t <= tmax; t += dt) {
        const sp = Math.hypot(v[0], v[1], v[2]);
        cN = null;
        const mag = W && !inCell && !touching;   // Magnus in free flight only (the spin is gone once the ball is rolling)
        const m0 = mag ? cm * (W[1] * v[2] - W[2] * v[1]) : 0, m1 = mag ? cm * (W[2] * v[0] - W[0] * v[2]) : 0, m2 = mag ? cm * (W[0] * v[1] - W[1] * v[0]) : 0;
        v[0] += (m0 - k * sp * v[0]) * dt; v[1] += (m1 - k * sp * v[1]) * dt; v[2] += (m2 - G.g - k * sp * v[2]) * dt;
        p = [p[0] + v[0] * dt, p[1] + v[1] * dt, p[2] + v[2] * dt];
        const f = dot(sub(p, B3), NORM);
        if (!entered && prev > 0 && f <= 0) {
          const q = sub(p, B3);
          if (inPoly(PENT, q[0], dot(q, NUP))) entered = true;
        }
        if (entered && f < -r) inCell = true;
        for (const o of others) {                  // other balls: static spheres
          const d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]], L = Math.hypot(d[0], d[1], d[2]), rr = r + o[3];
          if (L < rr && L > 1e-9) { const n = mul(d, 1 / L); p = [o[0] + n[0] * rr, o[1] + n[1] * rr, o[2] + n[2] * rr]; bounce(n, 0.3, 0.9); hitBall = true; }
        }
        if (inCell) {                           // inside our up-CELL: all 8 walls of its cross-section, and the back wall
          for (let e = 0; e < PENT.length; e++) {
            const q = sub(p, PIV), x = q[0], s = dot(q, NUP) - S0;
            const A = PENT[e], Bq = PENT[(e + 1) % PENT.length], dx = Bq[0] - A[0], dy = Bq[1] - A[1], L = Math.hypot(dx, dy);
            const nx = -dy / L, ny = dx / L;                                   // inward normal (the polygon is counter-clockwise)
            const dist = (x - A[0]) * nx + (s - A[1]) * ny;
            if (dist < r) { const n = add(mul(EX, nx), mul(NUP, ny)); p = add(p, mul(n, r - dist)); bounce(n, 0.3, 0.9); }
          }
          const a = dot(sub(p, PIV), AX);
          if (a < A_IN + r) { p = add(p, mul(AX, A_IN + r - a)); bounce(AX, 0.3); }
        } else {
          for (const [[a, b], rr] of colliders) {
            const c = segClosest(p, a, b), d = sub(p, c), L = Math.hypot(d[0], d[1], d[2]);
            if (L < rr && L > 1e-9) { const n = mul(d, 1 / L); p = add(c, mul(n, rr)); bounce(n); }
          }
          for (const pr of boxes) {
            const c = contactPrism(p, pr, r);
            if (c) { p = add(p, mul(c.n, c.depth)); bounce(c.n); }
          }
          // our up-CELL is open at its outer end: a ball lined up with the opening passes; anything else meets a wall
          if (!entered) {
            const q = sub(p, PIV), aa = dot(q, AX), ss = dot(q, NUP) - S0;
            if (!(inPoly(PENT, q[0], ss) && aa > UP_CELL.a0)) {
              const c = contactPrism(p, UP_CELL, r);
              if (c) { p = add(p, mul(c.n, c.depth)); bounce(c.n); }
            }
          }
        }
        if (p[2] < r) { p[2] = r; bounce([0, 0, 1], 0.5, 0.985); }
        if (opts.walls) opts.walls(p, v, r, bounce);
        // after all contact corrections: the whole ball back outside the opening plane = it has to come in again
        const fPost = dot(sub(p, B3), NORM);
        if (entered && fPost > r) { entered = false; inCell = false; }
        prev = fPost;
        touching = !!cN;
        if (cN) {                                 // rolling on a surface: a rolling ball speeds up at 5/7 of the slope's pull
          sampN = cN;                             // (solid-sphere rule), and rolling resistance slows it a little
          const gdn = -G.g * cN[2], gt = [-gdn * cN[0], -gdn * cN[1], -G.g - gdn * cN[2]];
          v = add(v, mul(gt, -2 / 7 * dt));
          const vn = dot(v, cN), vt = sub(v, mul(cN, vn)), st = Math.hypot(vt[0], vt[1], vt[2]);
          if (st > 1e-9) v = sub(v, mul(vt, Math.min(st, CRR * G.g * dt) / st));
        }
        if (t >= tNext) { const n = sampN || [0, 0, 0]; path.push([t, p[0], p[1], p[2], n[0], n[1], n[2]]); sampN = null; tNext += every; }
        if (Math.hypot(v[0], v[1], v[2]) < 0.03 && touching) { if ((rest += dt) > 0.3) { settled = true; break; } } else rest = 0;
      }
      path.push([tmax, p[0], p[1], p[2], ...(touching && cN ? cN : [0, 0, 0])]);
      // retained = it came in through the opening and is still inside the cavity when it stops (or time runs out)
      // inCell: inside our up-CELL now; settled: at rest (only settled balls count as held; see the page)
      return { path, inCell, settled, end: p, vEnd: v, hitBall };
    }

    return { G, uc, B3, P3, RIM, SEC, rods, own, NEIGH, UP_CELL, PIV, AX, NUP, judge, launch, launchDir, aimDir, seesTarget, turretShot, turretForward, turretLaunchRange, variedTurret, seededNormal, hitRateAt, pickShot, solver, solve, fly, SPEEDS, window_ };
  }

  const api = { make };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.SimCore = api;
})(typeof self !== "undefined" ? self : this);
