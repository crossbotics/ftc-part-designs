# v6.1 r2: spring-service pose. Slide the upper shaft's 48T gear + its hub (servo side, +X) and the collar (motor side, -X)
# outward along the shaft, at several hood angles, and report what they hit.
import check as C, turret as T
from build123d import Pos
parts, _ = T.build()
names = [n for n in parts if n.startswith(("gear_upper", "hub_upper_gear", "collar_upper"))]
print("moving:", names)
for alpha in (0.0, 12.5, 25.0, T.HOOD_ALPHA_MAX):
    sh = C.pose(parts, alpha, "pollen", T.LL_TILT_RANGE[0])
    worst = {}
    for dx in (2.5, 5, 7.5, 10, 12.5, 15, 17.5, 20):
        for n in names:
            c = sh[n].center(); sgn = 1 if c.X > 0 else -1
            m = Pos(sgn * dx, 0, 0) * sh[n]
            for o, s in sh.items():
                if o in names or o.startswith("shaft_upper") or "upper" in o and "shaft" in o:
                    continue
                v = C.overlap(m, s)
                if v > 1.0:
                    worst.setdefault((n, o), (dx, v))
    print(f"hood {alpha:5.1f}: " + ("clear to 20 mm" if not worst else "; ".join(f"{n} x {o} from {dx} mm ({v:.0f} mm3)" for (n, o), (dx, v) in worst.items())))
