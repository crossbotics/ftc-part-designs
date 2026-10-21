"""Rapid-fire simulation: several balls popped up in a row, Pollen/Nectar mixes, with flywheel spin-down + recovery.

Adds to sim_feed.py's 2D model:
  * flywheel speed is a state: balls take energy out, the motor (5203-2402-0001 via 3:1 belt, velocity PF control)
    puts it back; the two wheels are geared together so they share one speed
  * several balls per run, with ball-ball contact
All assumptions are listed below; wheel inertia is estimated from goBILDA's published wheel weights.

    python sim_rapid.py
"""
import itertools
import math

import numpy as np

import turret as T

# ---- assumptions ----
BALLS = {"P": (0.025, T.POLLEN_D / 2000), "N": (0.041, T.NECTAR_D / 2000)}
BALL_K = 4000.0                      # N/m: a mid-season ball
MU_WHEEL, MU_IDLER = 0.8, 0.05
TARGET_V = T.WHEEL_SURFACE_SPEED     # m/s wheel surface speed setpoint
POP_V = 3.0                          # m/s upward pop from the indexer
UPPER_MASS, UPPER_F0, UPPER_K = 0.25, 41.0, 200.0
# v6 (Astra verification): the upper wheel on its GEARED swing arm, with the constant-force springs along their real line,
# the same model as sim_feed.py (was a radial slider). SPRING_MULT scales the spring force (1 = two am-4666).
SPRING_MULT = T.CF_SPRINGS_PER_SIDE                # v6.1: springs stacked per side
OPEN_STOP = T.OPEN_STOP_DEG                        # v6.1: arm bumper
I_UPPER = 0.7 * 0.156 * (T.R2 / 1000) ** 2 + 0.2e-4   # upper stack spin inertia (the same estimate I_BASE uses)
Q = T.SYNC_TEETH_J / T.SYNC_TEETH_S
IDLER_MASS = 0.03
IDLERS = [(f0, (f1 - f0) / ((T.IDLER_SLOTS[0][1] - T.IDLER_SLOTS[0][0]) / 1000)) for f0, f1 in T.IDLER_FORCES] if T.IDLER_SLOTS else []
# rotating inertia reflected to the main shaft (kg m^2): 3x 96mm Gecko (105 g), 3x 72mm Gecko (52 g) x (R1/R2)^2,
# hubs/pulleys/shafts, motor rotor x 3^2
# v5 (Codex review): the upper shaft turns (60/48)x the main shaft, so its inertia reflects by (60/48)^2 (was (48/36)^2 from v4)
I_BASE = 0.7 * 0.315 * 0.048 ** 2 + 0.6e-4 + (0.7 * 0.156 * 0.036 ** 2 + 0.2e-4) * (T.SYNC_TEETH_J / T.SYNC_TEETH_S) ** 2 + 1.35e-4
MOTOR_STALL, MOTOR_FREE = 0.144, 6000 * 2 * math.pi / 60      # N m, rad/s (goBILDA 5203-2402-0001)
GEAR = T.MAIN_MOTOR_PULLEY_T / T.MOTOR_PULLEY_T
KP = 8.0                             # velocity loop: full power at ~12% speed error
DT = 2e-5

M = np.array([T.FW_Y, T.FW_Z]) / 1000
R1, R2 = T.R1 / 1000, T.R2 / 1000

def run(interval, seqs, extra_inertia=0.0, n_motors=1, alpha_deg=0.0, target_v=None, gate=None):
    """Each scenario = one sequence string like 'PNPN'. Returns per-ball exit speed (0 = not shot) and wheel speed at entry.
    gate: None = fire every `interval` s. A number (e.g. 0.01) = the encoder gate: fire the next ball only when at least
    `interval` s have passed AND the wheel is back within that fraction of its target speed. run.t_fire gives the times."""
    TARGET = TARGET_V if target_v is None else target_v
    S, N = len(seqs), max(len(q) for q in seqs)
    m = np.zeros((S, N)); r = np.zeros((S, N)); active_def = np.zeros((S, N), bool)
    for i, q in enumerate(seqs):
        for j, c in enumerate(q):
            m[i, j], r[i, j] = BALLS[c]; active_def[i, j] = True
    I = 2 / 3 * m * r * r + 1e-12
    cb = 2 * 0.15 * np.sqrt(BALL_K * np.maximum(m, 1e-6))
    Ieff = I_BASE + extra_inertia
    M11 = Ieff                                           # I_BASE already holds the upper stack reflected by q^2
    M12 = -Q * (1 + Q) * I_UPPER
    M22 = UPPER_MASS * (T.ARM_L / 1000) ** 2 + (1 + Q) ** 2 * I_UPPER
    DET = M11 * M22 - M12 ** 2
    w_target = TARGET / R1
    w = np.full(S, w_target)                       # main shaft rad/s
    park = np.array([0.5, -0.5])
    p = np.tile(park, (S, N, 1)).astype(float); v = np.zeros((S, N, 2)); spin = np.zeros((S, N))
    state = np.zeros((S, N), int)                  # 0 waiting, 1 in play, 2 shot, 3 failed
    t_start = np.arange(N) * interval
    nxt = np.zeros(S, int); last_fire = np.full(S, -1e9); t_fire = np.full((S, N), np.nan)
    exit_el = np.full((S, N), np.nan)   # v6.1 r3: per-shot launch elevation, same convention as sim_feed
    exit_v = np.zeros((S, N)); w_entry = np.zeros((S, N)); entered = np.zeros((S, N), bool)
    alpha = math.radians(alpha_deg)
    psi_s = math.radians(T.psi_exit(T.S_P)) + alpha
    # geared swing arm about the jackshaft J (tilts with the hood)
    Jw = np.array(T.world(T.J, alpha_deg)) / 1000
    L = T.ARM_L / 1000
    sgn = 1.0 if T.TH_N > T.TH_P else -1.0
    phi = np.full(S, T.TH_P); phid = np.zeros(S); arm_max = np.zeros(S)
    I_arm = UPPER_MASS * L * L + I_UPPER * (1 + Q) ** 2
    J_mm = np.array(T.J); A0 = np.array(T.SPRING_A) - J_mm; B_mm = np.array(T.SPRING_B[0]); SF = SPRING_MULT * 2 * T.CF_SPRING_N
    def spring_torque(ph):
        d = ph - T.TH_P
        post = J_mm[None, :] + np.stack([A0[0] * np.cos(d) - A0[1] * np.sin(d), A0[0] * np.sin(d) + A0[1] * np.cos(d)], 1)
        u = B_mm[None, :] - post; u /= np.linalg.norm(u, axis=1)[:, None]
        rr = (post - J_mm[None, :]) / 1000
        return (rr[:, 0] * u[:, 1] - rr[:, 1] * u[:, 0]) * SF
    slots = [(yp / 1000, z / 1000) for yp, _, z in T.IDLER_SLOTS]
    travel = T.IDLER_TRAVEL / 1000; ir = T.IDLER_D / 2000
    iy = np.array([[sl[0]] * S for sl in slots]); iv = np.zeros_like(iy)
    z_pinch = M[1] + (R1 + 0.04) * math.sin(psi_s)
    t_end = (N - 1) * interval + 0.6 if gate is None else (N - 1) * max(interval, 0.25) + 2.0
    min_w = w.copy()

    def circ_contact(c, rc, vc, surf):
        d = p - c; dist = np.linalg.norm(d, axis=2) + 1e-12; nrm = d / dist[..., None]
        delta = r + rc - dist
        fn = np.where((delta > 0) & (state == 1), BALL_K * delta - cb * np.einsum("snk,snk->sn", v - vc, nrm), 0).clip(min=0)
        t = np.stack([-nrm[..., 1], nrm[..., 0]], 2)
        slip = np.einsum("snk,snk->sn", v - vc, t) - spin * r - surf
        return fn, nrm, t, slip

    for step in range(int(t_end / DT)):
        tnow = step * DT
        if gate is None:
            start = (state == 0) & active_def & (tnow >= t_start[None, :])
        else:
            ready = (tnow - last_fire >= interval) & (np.abs(w / w_target - 1) <= gate) & (nxt < N)   # v6.1 r2: within +/-gate
            start = np.zeros((S, N), bool)
            idx = np.where(ready & active_def[np.arange(S), np.minimum(nxt, N - 1)])[0]
            start[idx, nxt[idx]] = True
            t_fire[idx, nxt[idx]] = tnow; last_fire[idx] = tnow; nxt[idx] += 1
        p[start] = np.array([0.0, -0.03]); v[start] = np.array([0.0, POP_V]); state[start] = 1
        F = np.zeros((S, N, 2)); F[..., 1] = -9.81 * m; Tq = np.zeros((S, N)); P_out = np.zeros(S)
        vs = w * R1                                           # both wheels share surface speed
        # main wheel
        fn, nrm, t, slip = circ_contact(M, R1, 0.0, vs[:, None])
        ft = -MU_WHEEL * fn * np.tanh(slip / 0.1)
        F += fn[..., None] * nrm + ft[..., None] * t; Tq += -r * ft
        P_out += (-ft * vs[:, None]).sum(1) * -1             # power the wheel gives the ball
        P_main = (ft * vs[:, None]).sum(1)                   # main wheel only (the upper wheel's share is Q_th_up)
        entered |= (fn > 0) & ~entered & (state == 1)
        w_entry = np.where((fn > 0) & (w_entry == 0), w[:, None], w_entry)
        # upper wheel on its geared swing arm
        ang = phi + alpha
        ra = np.stack([np.cos(ang), np.sin(ang)], 1) * L
        cs = Jw[None, :] + ra
        vcs = np.stack([-ra[:, 1], ra[:, 0]], 1) * phid[:, None]
        ur = (T.SYNC_TEETH_J / T.SYNC_TEETH_S) * T.R2 / T.R1        # upper surface / main surface
        surf = -ur * vs + (1 + Q) * phid * R2                         # geared: the arm's swing changes the wheel's spin
        fn, nrm, t, slip = circ_contact(cs[:, None, :], R2, vcs[:, None, :], surf[:, None])
        ft = -MU_WHEEL * fn * np.tanh(slip / 0.1)
        Fb = fn[..., None] * nrm + ft[..., None] * t
        F += Fb; Tq += -r * ft
        P_out += (ft * ur * vs[:, None]).sum(1) * -1
        tau_ball = (-(ra[:, None, 0] * Fb[..., 1] - ra[:, None, 1] * Fb[..., 0]) - (1 + Q) * R2 * ft).sum(1)
        Q_phi = tau_ball + ra[:, 0] * (-UPPER_MASS * 9.81) + spring_torque(phi)
        Q_th_up = Q * R2 * ft.sum(1)                                  # the upper wheel's drag reaches the shaft through the gear
        # idlers
        for k, (y0, z0) in enumerate(slots):
            ci = np.stack([iy[k], np.full(S, z0)], 1)[:, None, :]
            fn, nrm, t, slip = circ_contact(ci, ir, np.stack([iv[k], np.zeros(S)], 1)[:, None, :], 0.0)
            ft = -MU_IDLER * fn * np.tanh(slip / 0.1)
            F += fn[..., None] * nrm + ft[..., None] * t; Tq += -r * ft
            f0, kk = IDLERS[k]
            Fr = (-fn * nrm[..., 0]).sum(1) - (f0 + kk * (iy[k] - y0))
            iv[k] += np.where((iy[k] <= y0) & (Fr < 0), 0, Fr / IDLER_MASS) * DT; iy[k] += iv[k] * DT
            iv[k] = np.where(iy[k] < y0, np.maximum(iv[k], 0), iv[k]); iy[k] = np.clip(iy[k], y0, y0 + travel)
        # ball-ball
        for a, b in itertools.combinations(range(N), 2):
            d = p[:, a] - p[:, b]; dist = np.linalg.norm(d, axis=1) + 1e-12
            ov = r[:, a] + r[:, b] - dist
            on = (ov > 0) & (state[:, a] == 1) & (state[:, b] == 1)
            f = np.where(on, 2000.0 * ov, 0)[:, None] * d / dist[:, None]
            F[:, a] += f; F[:, b] -= f
        # flywheel: motor torque (PF velocity control) minus power taken by the balls
        wm = w * GEAR
        u = np.clip(w_target * GEAR / MOTOR_FREE + KP * (w_target - w) * GEAR / MOTOR_FREE, 0, 1)
        tau_m = n_motors * GEAR * (u * MOTOR_STALL - MOTOR_STALL * wm / MOTOR_FREE)
        # v6.1 (Astra): shaft and arm are ONE coupled system through the gear (w_S = -q w + (1+q) phidot):
        #   [M11 M12; M12 M22] [wdot; phiddot] = [Q_theta; Q_phi]
        Q_th = tau_m - P_main / np.maximum(w, 1) + Q_th_up
        a_th = (M22 * Q_th - M12 * Q_phi) / DET; a_ph = (M11 * Q_phi - M12 * Q_th) / DET
        opened = sgn * (phi - T.TH_P)
        lim_o = math.radians(OPEN_STOP) if OPEN_STOP is not None else 1e9
        locked = ((opened <= 0) & (sgn * a_ph < 0)) | ((opened >= lim_o) & (sgn * a_ph > 0))
        a_th = np.where(locked, Q_th / M11, a_th); a_ph = np.where(locked, 0.0, a_ph)
        w = w + a_th * DT
        phid = phid + a_ph * DT
        phi = phi + phid * DT
        # stops: the arm stops dead and the matching impulse goes into the shaft (M11 dw = M12 phidot)
        hit_c = sgn * (phi - T.TH_P) < 0
        hit_o = sgn * (phi - T.TH_P) > lim_o
        hit = hit_c | hit_o
        w = np.where(hit, w + M12 * phid / M11, w)
        phi = np.where(hit_c, T.TH_P, np.where(hit_o, T.TH_P + sgn * lim_o, phi)); phid = np.where(hit, 0.0, phid)
        arm_max = np.maximum(arm_max, sgn * (phi - T.TH_P))
        min_w = np.minimum(min_w, w)
        # balls
        live = state == 1
        v = np.where(live[..., None], v + F / np.maximum(m, 1e-9)[..., None] * DT, v)
        p = np.where(live[..., None], p + v * DT, p)
        spin = np.where(live, spin + Tq / I * DT, spin)
        spd = np.linalg.norm(v, axis=2); dm = np.linalg.norm(p - M, axis=2)
        shot = live & (p[..., 1] > z_pinch + 0.03) & (dm > R1 + r + 0.005) & (spd > 3.0)
        fail = live & ((p[..., 1] < -0.12) | (p[..., 0] > 0.16) | (p[..., 0] < -0.2) |
                       ((tnow - (t_start[None, :] if gate is None else np.nan_to_num(t_fire, nan=tnow))) > 0.55))
        exit_el[shot] = np.degrees(np.arctan2(v[..., 1], -v[..., 0]))[shot]
        exit_v[shot] = spd[shot]; state[shot] = 2; state[fail & ~shot] = 3
        p[shot | fail] = park; v[shot | fail] = 0
    run.arm_max = np.degrees(arm_max); run.t_fire = t_fire; run.exit_el = exit_el
    return exit_v, w_entry * R1, min_w * R1

if __name__ == "__main__":
    # the robot holds at most 4 balls: 4-ball volleys, one flywheel motor (v5 default)
    seqs = ["PPPP", "NNNN", "PNPN", "NNPP"]
    nm = 2 if T.SECOND_MOTOR else 1
    print(f"{nm} flywheel motor(s); setpoint {TARGET_V} m/s surface, pop {POP_V} m/s, mid-season balls")
    for interval in (0.25, 0.33):
        ev, went, minv = run(interval, seqs, 0.0, nm)
        print(f"-- {1/interval:3.1f} balls/s (every {interval*1000:.0f} ms)")
        for i, q in enumerate(seqs):
            ex = " ".join(f"{c}{e:4.1f}" if e > 0 else f"{c}  x " for c, e in zip(q, ev[i]))
            by = {c: [e for cc, e in zip(q, ev[i]) if cc == c and e > 0] for c in set(q)}
            spread = "  ".join(f"{c}: {100*(max(v)-min(v))/max(v):3.1f}%" for c, v in sorted(by.items()) if len(v) > 1)
            print(f"   {q}: exit m/s {ex} | wheel dips to {minv[i]:4.1f} m/s | spread {spread}")
