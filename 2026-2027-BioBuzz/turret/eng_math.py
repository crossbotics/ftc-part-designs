"""Numbers for the page's "Engineering math" section, computed from the model and the simulations so the worked
examples always match the design. diagrams.py stores the result under "math" in diagrams.json.

    python eng_math.py      # prints every value
"""
import math

import turret as T

G = 9.81
IN = 0.0254


def compute(run_sims=True):
    m = {}
    f = lambda v, n=1: int(round(v)) if n == 0 else round(v, n)
    # ---- 1. flywheel speed from the drive train ----
    gear = T.MAIN_MOTOR_PULLEY_T / T.MOTOR_PULLEY_T
    n_wheel = 6000 / gear
    r1 = T.R1 / 1000
    v_free = 2 * math.pi * n_wheel / 60 * r1
    m.update(motor_rpm=6000, pul_small=T.MOTOR_PULLEY_T, pul_big=T.MAIN_MOTOR_PULLEY_T, gear=f(gear, 0), n_wheel=f(n_wheel, 0),
             r1_mm=f(T.R1, 0), r1_m=f(T.R1 / 1000, 3), v_free=f(v_free, 2), v_target=f(T.WHEEL_SURFACE_SPEED, 1),
             pct_free=f(100 * T.WHEEL_SURFACE_SPEED / v_free, 0), n_target=f(T.WHEEL_SURFACE_SPEED / r1 * 60 / (2 * math.pi), 0))

    # ---- 2. the 45-degree rule, generalised (worked example at 1.0 m) ----
    import entry_window as EW
    y = 1.18                                            # opening centre above the launch point, m (as figure 6)
    x = 1.0
    phi = math.degrees(math.atan2(y, x))
    th = 45 + phi / 2
    vmin = math.sqrt(G * (y + math.hypot(x, y)))
    def v_at(theta):
        t = math.radians(theta)
        den = 2 * math.cos(t) ** 2 * (x * math.tan(t) - y)
        return math.sqrt(G * x * x / den) if den > 0 else float("nan")
    best = EW.best_angle(x, 0.025, T.POLLEN_D / 2000, range(55, 81, 1))
    m.update(ex_x=x, ex_y=y, ex_phi=f(phi, 1), ex_theta=f(th, 1), ex_vmin=f(vmin, 2), ex_best=best[0], ex_tol=f(best[1][1], 1),
             ex_v_p5=f(v_at(th + 5), 2), ex_v_m5=f(v_at(th - 5), 2),
             ex_pct5=f(100 * (max(v_at(th + 5), v_at(th - 5)) / vmin - 1), 1),
             lvl_x=1.5, lvl_v=f(math.sqrt(G * 1.5), 2))

    # ---- 3. speed tolerance: why a few percent matters ----
    m.update(sens_pct=3, sens_range=6, sens_d=1.5, sens_cm=f(0.06 * 150, 0))

    # ---- 4. backspin from the sync gears ----
    ratio = (T.SYNC_TEETH_J / T.SYNC_TEETH_S) * T.R2 / T.R1
    dv = (1 - ratio) * T.WHEEL_SURFACE_SPEED
    w_p = dv / (T.POLLEN_D / 1000)
    w_n = dv / (T.NECTAR_D / 1000)
    m.update(zj=T.SYNC_TEETH_J, zs=T.SYNC_TEETH_S, r2_mm=f(T.R2, 0), ratio=f(ratio, 4), dv=f(dv, 2),
             spin_p=f(w_p * 60 / (2 * math.pi), 0), spin_n=f(w_n * 60 / (2 * math.pi), 0), spin_p_rad=f(w_p, 1))

    # ---- 5. spring pinch force (lever about the jackshaft) ----
    tail = T.ARM_L + T.ARM_TAIL
    f_spr = 2 * T.CF_SPRINGS_PER_SIDE * T.CF_SPRING_N          # v6.1: stacked springs, both sides
    m.update(cf_n=f(T.CF_SPRING_N, 1), cf_total=f(f_spr, 1), arm_l=f(T.ARM_L, 1), arm_tail=f(tail, 1),
             pinch=f(f_spr * tail / T.ARM_L, 0))

    # ---- 6. flywheel energy, dip and recovery (why one motor is enough) ----
    import sim_rapid as SR
    I = SR.I_BASE
    w0 = T.WHEEL_SURFACE_SPEED / r1
    E0 = 0.5 * I * w0 ** 2
    mN = 0.041
    v_exit = 8.3
    ke = 0.5 * mN * v_exit ** 2
    m.update(I=f(I * 1e4, 2), w0=f(w0, 0), E0=f(E0, 1), mN=mN, v_exit=v_exit, ke=f(ke, 2))
    if run_sims:
        ev, went, minv = SR.run(0.33, ["N"])
        v_dip = float(minv[0])
        _, _, minv4 = SR.run(0.25, ["NNNN"])
        ev3, _, _ = SR.run(0.33, ["NNNN"])
        ev4, _, _ = SR.run(0.25, ["NNNN"])
        spread = lambda e: 100 * (max(e) - min(e)) / max(e)
        m.update(spread4=f(spread(list(ev4[0])), 1), spread3=f(spread(list(ev3[0])), 1))
    else:
        v_dip = 7.2
    w1 = v_dip / r1
    E_take = 0.5 * I * (w0 ** 2 - w1 ** 2)
    ts, wf = SR.MOTOR_STALL, SR.MOTOR_FREE
    P = lambda ww: ts * ww * gear * (1 - ww * gear / wf)     # motor power at wheel speed ww
    p_hi, p_lo = P(w0), P(w1)
    t_rec = E_take / ((p_hi + p_lo) / 2)
    m.update(v_dip=f(v_dip, 1), w1=f(w1, 0), E_take=f(E_take, 1), take_x=f(E_take / ke, 1),
             tau_s=ts, w_f=f(wf, 0), frac_hi=f(w0 * gear / wf, 2), frac_lo=f(w1 * gear / wf, 2),
             p_hi=f(p_hi, 1), p_lo=f(p_lo, 1), p_avg=f((p_hi + p_lo) / 2, 1), t_rec=f(t_rec, 2), rate=f(1 / t_rec, 1),
             p_max=f(ts * wf / 4, 1))

    # ---- 7. turret aiming ----
    GB = getattr(T, "GB_TURRET", False)                # v8.1GB: goBILDA turret, 176T ring / 64T, 312 rpm motor
    ratio_t = T.GB_RATIO if GB else T.RING_TEETH / T.PINION_TEETH
    rpm = T.GB_MOTOR_RPM if GB else 1150
    cpr_motor = T.GB_MOTOR_CPR if GB else 145.1         # goBILDA encoder counts per output rev (5203-2402-0019 / -0005)
    counts = cpr_motor * ratio_t
    half_w = (20 * IN - T.POLLEN_D / 1000) / 2
    m.update(ring=176 if GB else T.RING_TEETH, pinion=64 if GB else T.PINION_TEETH, ratio_t=f(ratio_t, 2 if GB else 0), turret_rpm=f(rpm / ratio_t, 1),
             t180=f(0.5 / (rpm / ratio_t / 60), 2), cpr_motor=cpr_motor, counts=f(counts, 0), deg_count=f(360 / counts, 2),
             half_w=f(half_w * 100, 1), half_w_m=f(half_w, 3), aim_deg=f(math.degrees(math.atan2(half_w, 2.5)), 1))

    # ---- 8. belt length (why the motor mount can be fixed holes) ----
    d1, d2 = T.htd_pd(T.MOTOR_PULLEY_T), T.htd_pd(T.MAIN_MOTOR_PULLEY_T)
    C = T.MOTOR_C
    L = 2 * C + math.pi * (d1 + d2) / 2 + (d2 - d1) ** 2 / (4 * C)
    m.update(d1=f(d1, 2), d2=f(d2, 2), C=f(C, 1), L_calc=f(L, 1), belt=T.MOTOR_BELT)

    # ---- 9. cable U-loop ----
    m.update(rin=f(T.C_RIN, 0), rout=f(T.C_ROUT, 0), rb=f(T.C_RB, 1), u_ratio=f(T.C_RIN / (T.C_RIN + T.C_ROUT), 2),
             travel=f(T.CABLE_TRAVEL, 0), u_travel=f(2 * T.CABLE_TRAVEL * T.C_RIN / (T.C_RIN + T.C_ROUT), 0),
             cable_len=f(T.CABLE_LEN, 0))

    # ---- 10. height budget ----
    from height_budget import LIMIT, MATCH_LIMIT, MARGIN
    from height_budget import extremes
    _hp = T.build()[0]
    start_top, match_top = round(extremes((0.0,), _hp)[0], 1), round(extremes(None, _hp)[0], 1)   # v7: from the model (height_budget.py)
    m.update(lim=LIMIT, match_lim=MATCH_LIMIT, margin=MARGIN, start_top=start_top, match_top=match_top,
             base_max=f(LIMIT - MARGIN - start_top, 0), robot_match=f(LIMIT - MARGIN - start_top + match_top, 0),
             robot_match_in=f((LIMIT - MARGIN - start_top + match_top) / 25.4, 1))

    # ---- 11. countersink depth ----
    m.update(csk_d=T.CSK_D, hole_d=4.3, csk_depth=f((T.CSK_D - 4.3) / 2, 2), plate_t=T.T_PLATE,
             left=f(T.T_PLATE - (T.CSK_D - 4.3) / 2, 1))
    return m


if __name__ == "__main__":
    for k, v in compute().items():
        print(f"{k:12s} {v}")
