"""2D side-view feed simulation: a ball is popped up the column from below the turret.
Does it get carried into the flywheel pinch, or fall back / roll out?

Model (Y forward-back, Z up, SI units):
  * main flywheel: fixed axis, constant surface speed, friction mu_w
  * upper flywheel: on the real SWING ARM (v6, after the Codex review): it rotates about the jackshaft J (which tilts
    with the hood), pulled toward its hard stop by the constant-force springs; ball forces, including wheel friction,
    act on it through their torque about J. (v5 used a radial slider, which gave a launch direction 5-13 deg off.)
  * idler rollers: slide back in their horizontal slots against springs, free-spinning (tiny friction)
  * ball: rigid body with linear contact stiffness k_ball (its squeeze), damping, spin, gravity
All runs are integrated together (numpy, one row per case).

    python sim_feed.py
"""
import itertools
import math

import numpy as np

import turret as T

# ---- assumptions (not measured) ----
BALLS = {"pollen": (0.025, T.POLLEN_D / 2000), "nectar": (0.041, T.NECTAR_D / 2000)}   # kg, m (team: Pollen 25 g, Nectar 41 g, 2026-09-29; was AndyMark 41.3)
WHEEL_SPEED = T.WHEEL_SURFACE_SPEED   # m/s surface speed of the main wheel
UPPER_RATIO = (T.SYNC_TEETH_J / T.SYNC_TEETH_S) * T.R2 / T.R1   # upper surface / main surface (v5: 0.9375 -> backspin)
MU_WHEEL, MU_IDLER = 0.8, 0.05
UPPER_MASS, UPPER_F0, UPPER_K = 0.25, 41.0, 200.0   # v3: 2x constant-force springs give ~41 -> 47.5 N over the swing
SPRING_F = 2 * T.CF_SPRINGS_PER_SIDE * T.CF_SPRING_N   # v6.1: stacked constant-force springs (both sides), along their real line
I_UPPER = 0.7 * 0.156 * (T.R2 / 1000) ** 2 + 0.2e-4   # upper stack spin inertia: 3x 72 mm Gecko (52 g) + hub/gear/shaft (same as sim_rapid)
OPEN_STOP = T.OPEN_STOP_DEG                        # v6.1: the arm bumper (deg of swing off the Pollen stop); None = no stop
STOP_EXTRA = 0.0            # m: how much closer than the 7mm-squeeze position the hard stop sits
IDLER_MASS = 0.03
IDLER_SPRINGS = {"none": None}   # v4: no idlers (set T.IDLERS = True and re-add v3's spring entry to compare)
if T.IDLER_SLOTS:
    _tr = (T.IDLER_SLOTS[0][1] - T.IDLER_SLOTS[0][0]) / 1000
    IDLER_SPRINGS["idler springs"] = [(f0, (f1 - f0) / _tr) for f0, f1 in T.IDLER_FORCES]
BALL_K = {"new": 8000.0, "mid": 4000.0, "worn": 1500.0}   # N/m PER CONTACT (= 2x whole-ball stiffness: BALL_K = 200 F for F newtons at 10 mm total squeeze); balls soften over a season
POP_SPEEDS = [1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.4]   # m/s upward, measured 30mm below the base plate
ALPHAS = [0.0, T.HOOD_ALPHA_MAX]
DT, T_END = 1e-5, 0.35
# v6.2: the robot is often driving while it fires. ROBOT_AY is its acceleration along the turret's Y (m/s^2, + = toward the
# turret's rear); in the turret frame the ball feels -m * ROBOT_AY. OBSTACLES are fixed parts the ball can hit in the side
# view (turret frame, mm): rounded segments ((y0, z0), (y1, z1), radius). The rear guard is added from turret.py.
ROBOT_AY = 0.0
OBSTACLE_K, OBSTACLE_MU = 20000.0, 0.3
def obstacles():
    obs = [((y, z), (y, z), 7.0) for y, z in T.CROSSBARS]                         # printed crossbars, 14 mm
    obs += [(tuple(a), tuple(b), r) for a, b, r in getattr(T, "REAR_GUARD_SEGS", [])]
    return obs

M = np.array([T.FW_Y, T.FW_Z]) / 1000
R1, R2 = T.R1 / 1000, T.R2 / 1000

def run(cases=None, record=0):
    """record > 0: also keep every record-th step's ball position and arm angle (for the viewer's shot animation)."""
    cases = cases or list(itertools.product(BALLS, POP_SPEEDS, ALPHAS, IDLER_SPRINGS, BALL_K))
    n = len(cases)
    m = np.array([BALLS[c[0]][0] for c in cases]); r = np.array([BALLS[c[0]][1] for c in cases])
    I = 2 / 3 * m * r * r
    kb = np.array([BALL_K[c[4]] for c in cases]); cb = 2 * 0.15 * np.sqrt(kb * m)
    alpha = np.radians([c[2] for c in cases])
    has_idler = np.array([IDLER_SPRINGS[c[3]] is not None for c in cases])
    def spr(c, k):   # (preload, rate) for roller k: one tuple for all rollers, or a list per roller
        v = IDLER_SPRINGS[c[3]] or (0.0, 0.0)
        return v[k] if isinstance(v, list) else v
    F0i = np.array([[spr(c, k)[0] for c in cases] for k in range(2)])
    Ki = np.array([[spr(c, k)[1] for c in cases] for k in range(2)])

    # ball starts centered on the turret axis, below the base, moving up
    p = np.stack([np.zeros(n), np.full(n, -0.03)], 1)
    v = np.stack([np.zeros(n), np.array([c[1] for c in cases])], 1)
    w = np.zeros(n)                                              # ball spin (about X)
    psi_s = np.radians(T.psi_exit(T.S_P)) + alpha
    # upper wheel on its swing arm: pivot J (tilts with the hood), arm angle phi in the carrier frame, hard stop at TH_P
    Jw = np.array([T.world(T.J, math.degrees(a)) for a in alpha]) / 1000
    L = T.ARM_L / 1000
    sgn = 1.0 if T.TH_N > T.TH_P else -1.0          # direction that opens the pinch
    phi = np.full(n, T.TH_P); phid = np.zeros(n)
    # v6 (Astra verification): the upper wheel is GEARED to the jackshaft (60:48 external mesh, q = 1.25), so its spin is
    # w_S = -q w_J + (1 + q) phid, the ball's friction on it acts on the arm through the gear, and its spin inertia adds
    # I_S (1 + q)^2 to the arm's. The jackshaft is driven at constant speed.
    Q = T.SYNC_TEETH_J / T.SYNC_TEETH_S
    I_arm = UPPER_MASS * L * L + I_UPPER * (1 + Q) ** 2
    # constant-force springs along their real attachment line: post on the arm tail -> coil drum on the carrier
    J_mm = np.array(T.J); A0 = np.array(T.SPRING_A) - J_mm; B_mm = np.array(T.SPRING_B[0])
    def spring_torque(ph):
        d = ph - T.TH_P
        post = J_mm[None, :] + np.stack([A0[0] * np.cos(d) - A0[1] * np.sin(d), A0[0] * np.sin(d) + A0[1] * np.cos(d)], 1)
        u = B_mm[None, :] - post; u /= np.linalg.norm(u, axis=1)[:, None]
        rr = (post - J_mm[None, :]) / 1000
        return (rr[:, 0] * u[:, 1] - rr[:, 1] * u[:, 0]) * SPRING_F     # N m, carrier frame (hood tilt doesn't change it)
    # idler sliders
    slots = [(yp / 1000, z / 1000) for yp, _, z in T.IDLER_SLOTS]
    travel = T.IDLER_TRAVEL / 1000
    iy = np.array([[s[0]] * n for s in slots]); iv = np.zeros_like(iy)
    ir = T.IDLER_D / 2000
    status = np.zeros(n, int)            # 0 running, 1 shot, 2 fell back, 3 out the rear, 4 out the front
    exit_v = np.zeros(n); t_done = np.zeros(n); exit_w = np.zeros(n); exit_el = np.zeros(n); exit_phi = np.zeros(n); exit_pmax = np.zeros(n)
    z_pinch = M[1] + (R1 + 0.04) * np.sin(psi_s)
    g = np.array([0, -9.81])

    def contact(c, rc, vc, spin_c):
        """Ball vs circle (center c, radius rc, velocity vc, surface spin spin_c rad/s). Returns force on ball, torque, normal."""
        d = p - c; dist = np.linalg.norm(d, axis=1); nrm = d / dist[:, None]
        delta = r + rc - dist
        on = (delta > 0) & (status == 0)                  # v6.1 (Astra): finished balls no longer push on the wheels
        vrel = v - vc
        fn = np.where(on, kb * delta - cb * np.einsum("ij,ij->i", vrel, nrm), 0.0).clip(min=0)
        t = np.stack([-nrm[:, 1], nrm[:, 0]], 1)
        # surface velocities at the contact point along t
        vb_t = np.einsum("ij,ij->i", v, t) + w * (-r) * 1.0          # ball surface (contact at -r*n)
        vo_t = np.einsum("ij,ij->i", vc, t) + spin_c * rc              # object surface (contact at +rc*n)
        slip = vb_t - vo_t
        return fn, nrm, t, slip, on

    rec = []
    OBS = obstacles()
    for step in range(int(T_END / DT)):
        live = status == 0
        if record and step % record == 0:
            rec.append((step * DT, p.copy(), phi.copy(), status.copy(), v.copy()))
        if not live.any():
            break
        F = np.tile(g, (n, 1)) * m[:, None]; Tq = np.zeros(n)
        F[:, 0] -= m * ROBOT_AY
        for (a0, a1, rad) in OBS:                                     # fixed parts (crossbars, rear guard)
            A_ = np.array(a0) / 1000; B_ = np.array(a1) / 1000; AB = B_ - A_
            tt = np.clip(((p - A_) @ AB) / max(AB @ AB, 1e-12), 0, 1) if AB @ AB > 0 else np.zeros(n)
            q = A_ + tt[:, None] * AB
            fn, nrm, t, slip, on = contact(q, rad / 1000, np.zeros((n, 2)), 0.0)
            fn = np.where(on, OBSTACLE_K / BALL_K["mid"] * fn, 0.0)       # stiffer than a wheel: printed plastic
            ft = -OBSTACLE_MU * fn * np.tanh(slip / 0.05)
            F += fn[:, None] * nrm + ft[:, None] * t; Tq += -r * ft
        # main wheel (spins so its front face moves up)
        mu = MU_WHEEL
        fn, nrm, t, slip, on = contact(np.tile(M, (n, 1)), R1, np.zeros((n, 2)), WHEEL_SPEED / R1)
        ft = -mu * fn * np.tanh(slip / 0.05)
        F += fn[:, None] * nrm + ft[:, None] * t; Tq += -r * ft
        # upper wheel (counter-rotating) on its swing arm about J
        ang = phi + alpha
        ra = np.stack([np.cos(ang), np.sin(ang)], 1) * L            # J -> wheel centre
        cs = Jw + ra
        vcs = np.stack([-ra[:, 1], ra[:, 0]], 1) * phid[:, None]
        w_up = -WHEEL_SPEED * UPPER_RATIO / R2 + (1 + Q) * phid        # geared: the arm's swing changes the wheel's spin
        fn, nrm, t, slip, on = contact(cs, R2, vcs, w_up)
        ft = -mu * fn * np.tanh(slip / 0.05)
        Fb = fn[:, None] * nrm + ft[:, None] * t
        F += Fb; Tq += -r * ft
        # generalised force on the arm angle: the ball's reaction on the wheel centre (moment about J) plus, through the
        # gear, (1 + q) x the friction torque about the wheel's own centre
        tau_ball = -(ra[:, 0] * Fb[:, 1] - ra[:, 1] * Fb[:, 0]) - (1 + Q) * R2 * ft
        tau_grav = ra[:, 0] * (-UPPER_MASS * 9.81)
        tau = tau_ball + tau_grav + spring_torque(phi)
        opened = sgn * (phi - T.TH_P) * L                             # arc the wheel has moved off its stop (m)
        phid += np.where((opened <= 0) & (sgn * tau < 0), 0, tau / I_arm) * DT
        phi += phid * DT
        back = sgn * (phi - T.TH_P) < 0
        if OPEN_STOP is not None:                                     # open-side hard stop (dead stop, no rebound)
            lim = T.TH_P + sgn * math.radians(OPEN_STOP)
            over = sgn * (phi - lim) > 0
            phid = np.where(over, 0.0, phid); phi = np.where(over, lim, phi)
        phid = np.where(back, 0.0, phid); phi = np.where(back, T.TH_P, phi)
        # idlers
        for k, (y0, z0) in enumerate(slots):
            ci = np.stack([iy[k], np.full(n, z0)], 1)
            fn, nrm, t, slip, on = contact(ci, ir, np.stack([iv[k], np.zeros(n)], 1), 0.0)
            fn = np.where(has_idler, fn, 0)
            ft = -MU_IDLER * fn * np.tanh(slip / 0.05)
            F += fn[:, None] * nrm + ft[:, None] * t; Tq += -r * ft
            Fr = -fn * nrm[:, 0] - (F0i[k] + Ki[k] * (iy[k] - y0))
            iv[k] += np.where((iy[k] <= y0) & (Fr < 0), 0, Fr / IDLER_MASS) * DT
            iy[k] += iv[k] * DT
            iv[k] = np.where(iy[k] < y0, np.maximum(iv[k], 0), iv[k]); iy[k] = np.clip(iy[k], y0, y0 + travel)
            iv[k] = np.where(iy[k] >= y0 + travel, np.minimum(iv[k], 0), iv[k])
        a = F / m[:, None]
        v = np.where(live[:, None], v + a * DT, v); p = np.where(live[:, None], p + v * DT, p)
        w = np.where(live, w + Tq / I * DT, w)
        tnow = (step + 1) * DT
        speed = np.linalg.norm(v, axis=1)
        dm = np.linalg.norm(p - M, axis=1)
        shot = live & (p[:, 1] > z_pinch + 0.03) & (dm > R1 + r + 0.005) & (speed > 3.0)
        fell = live & (p[:, 1] < -0.12)
        rear = live & ~shot & (p[:, 0] > 0.16)
        front = live & ~shot & (p[:, 0] < -0.20)
        for code, mask in ((1, shot), (2, fell), (3, rear), (4, front)):
            status[mask] = code; t_done[mask] = tnow
        exit_v[shot] = speed[shot]; exit_w[shot] = w[shot]
        exit_el[shot] = np.degrees(np.arctan2(v[shot, 1], -v[shot, 0]))   # launch elevation (ball flies toward -Y)
        exit_phi[shot] = phi[shot]; exit_pmax = np.maximum(exit_pmax, sgn * (phi - T.TH_P))
    run.exit_spin = exit_w
    run.exit_el = exit_el; run.exit_phi = exit_phi; run.exit_pmax = exit_pmax
    run.rec = rec; run.v_end = v.copy(); run.p_end = p.copy()
    return cases, status, exit_v, t_done

if __name__ == "__main__":
    cases, status, exit_v, t_done = run()
    res = {}
    for c, st, ev in zip(cases, status, exit_v):
        res.setdefault((c[0], c[2], c[3]), {}).setdefault(c[4], []).append((c[1], st, ev))
    print("min pop speed that gets shot (m/s)   |  exit speed at 3.0 m/s pop (m/s)")
    print(f"{'ball':7s} {'hood':>4s} {'idlers':9s}| {'new':>5s} {'mid':>5s} {'worn':>5s} | {'new':>5s} {'mid':>5s} {'worn':>5s}  spread")
    for (ball, alpha, idl), byk in res.items():
        mins, exits = [], []
        for kname in BALL_K:
            runs = sorted(byk[kname])
            ok = [v0 for v0, st, ev in runs if st == 1]
            # first speed from which every faster pop also shoots
            first = next((v0 for v0, st, ev in runs if all(s2 == 1 for v2, s2, e2 in runs if v2 >= v0)), None)
            mins.append(f"{first:5.1f}" if first else "  —  ")
            e3 = [ev for v0, st, ev in runs if abs(v0 - 3.0) < 1e-9 and st == 1]
            exits.append(e3[0] if e3 else None)
        es = [e for e in exits if e]
        spread = f"{100*(max(es)-min(es))/max(es):4.0f}%" if len(es) == 3 else "  n/a"
        print(f"{ball:7s} {alpha:4.0f} {idl:9s}| " + " ".join(mins) + " | " +
              " ".join(f"{e:5.1f}" if e else "  —  " for e in exits) + f"  {spread}")
    sp = -run.exit_spin[status == 1] * 60 / (2 * math.pi)   # ball flies toward -Y: backspin is negative spin about +X
    if len(sp):
        print(f"\nexit spin: {np.median(sp):+.0f} rpm median ({sp.min():+.0f} to {sp.max():+.0f}); + = backspin")
    print("\nlaunch elevation vs the design (90 - psi - alpha), shots from a 3.0 m/s pop:")
    for (ball, alpha, idl), _ in res.items():
        sel = [i for i, c in enumerate(cases) if (c[0], c[2], c[3]) == (ball, alpha, idl) and abs(c[1] - 3.0) < 1e-9 and status[i] == 1]
        design = 90 - T.psi_exit(T.S_P if ball == "pollen" else T.S_N) - alpha
        els = [run.exit_el[i] for i in sel]
        if els:
            print(f"  {ball:7s} hood {alpha:4.0f}: design {design:5.1f}  sim {min(els):5.1f}..{max(els):5.1f}  (off {np.mean(els) - design:+.1f})")
    fails = {}
    for c, st in zip(cases, status):
        if st in (3, 4): fails[(c[0], c[2], c[3])] = fails.get((c[0], c[2], c[3]), 0) + 1
    print("\nescapes out the rear/front:", fails or "none")
