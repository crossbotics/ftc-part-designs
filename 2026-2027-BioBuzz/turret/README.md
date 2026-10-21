# BIOBUZZ turret v8.1GB

> **Work in progress.** This design is still changing, and parts of it are not yet built and tested on a robot.
> Please try it and [tell us what works and what does not](https://github.com/crossbotics/ftc-part-designs/issues).

3D view, decisions, parts list and downloads: https://crossbotics.com/parts/turret/

**In this repo:** the build123d model and its check scripts. **Not in this repo:**
- goBILDA's own CAD files (`vendor_cad/`): download them from the goBILDA pages listed in `vendor_links.json`
  (only `proto_print.py` needs them).
- The official BIOBUZZ field CAD: download it from FIRST. Scripts that measure shots against the field
  (`approach_check.py`, `shoot_map.py`, `sim_export.py` and the `sim/` page) need it.
- Build output (`out/`): run the scripts below. Ready-made downloads (STEP, STL, DXF, the printed
  prototype kit) are on the [releases page](https://github.com/crossbotics/ftc-part-designs/releases).

Setup (Python 3.12): `python -m venv .venv && .venv/bin/pip install -r requirements.txt`, then run the
scripts below with `.venv/bin/python` (they say `../.venv/bin/python` because of where they first lived).


v8.1GB = v8.1 on a **bought turret bearing**: the goBILDA Gear-Driven Turret (3208-0004-0001, 2.75:1, 105 mm bore) replaces the plywood base plate, printed ring gear, 6 rollers and turret pinion; a 312 rpm Yellow Jacket drives its 64T gear; a printed shim sits between its gear ring and the deck, and a printed arm on its housing holds the home switch. Everything above the deck is exactly v8.1 (same files). `safe_parts.py` lists which parts are the same in both versions (out/safe_parts.md). z = 0 is the bearing housing's bottom (the robot side). v8.1 stays untouched as the other option. See ../DESIGN_LOG.md, "v8.1GB".

v8.1 = v8 with every turning part within **203 mm** of the turret axis (`reach.py`: 201.1, 2.1 mm margin). With the axis 1 in forward of the robot centre, the turret clears the lifter at every heading and fits the 18 in start cube, so the lifter interlock should not be needed (202.9 → 201.1 mm after the Astra review: 2 mm margin to the 203.2 boundary; confirm on the built robot before dropping the software limit). The flywheel motor sits lower on a **315 mm** belt (jack tensioner kept, ±1.4 mm of centre distance); the side plates, front crossbar (now above the motor) and camera pivot moved back; the motor-side plate has a lower front corner under the motor window. The shot is unchanged. Start footprint 267 × 305 mm (arm closed), 193 mm tall above the base plate. See ../DESIGN_LOG.md, "v8.1".

v8 = v7 made compact at the start: no cable tray (cable method chosen later; zip-tie hole pairs at the back of the deck and base plate), turret motor at 135°, home switch on a base-plate pad with the magnet in a printed holder under a deck ear, camera pivot 17 mm lower, rear ball guard raised above the roller bolts' ends. See ../DESIGN_LOG.md, "v8".

(History below.)

v7 = v6.2 made in-house wherever possible: the frame plates are laser-cut 1/8" Baltic birch (two layers glued, square nuts and button-head counterbores instead of threads and countersinks), and the turret ring, swing arms, motor adapter, servo insert, 24T/48T pulleys and shaft spacers are printed in PA-CF. Cut `fit_test.dxf` first. `laser_check.py` checks every plywood layer. See DESIGN_LOG.md, "v7".

v6.2 = v6.1 plus a rear ball guard (a printed wall on the rear crossbar, so balls can't drop behind the feed hole, even while the robot drives) and the camera set to 45° (it sees our up-CELL's tags from 90% of scoring spots). See DESIGN_LOG.md, "v6.2". `feed_sweep.py` checks the feed across hood angle, pop speed, ball wear and robot acceleration.

v6.1 = v6 plus two stacked constant-force springs per side (room for three), an arm bumper stop, spring anchor blocks, and a geared swing-arm model in both simulations. See DESIGN_LOG.md, "v6.1".

v5, made buildable with a 20 W diode laser plus a cutting service, and with every in-season adjustment reachable on the assembled turret. See DESIGN_LOG.md, "v6".

- **Hood gear:** bought (goBILDA 2302-0032-0108, 108T), bolted to the servo-side carrier.
- **Turret ring:** 3D-printed in PA-CF, with 8 M4 heat-sets for the deck spacers.
- **Fasteners:** no tapped holes: M4 square nuts glued into pockets in the plywood (deck, motor-side plate) and the printed motor adapter; heat-sets in printed parts (ring, servo insert with M3, crossbars, cams…); button heads in counterbores on the hood carriers. L-bracket screws are staggered. Stop cams adjust from the ball side.
- **Flywheel motor belt:** adjustable. The motor sits on a printed PA-CF adapter plate that slides ±2 mm; 4 clamp bolts and a jack screw, all reachable from outside. The jackshaft belt stays on a fixed centre (belt test plate).
- **Front flair plate:** optional, 3 mm.

Rebuild (from this folder, with the project venv):

    ../.venv/bin/python turret.py        # model, DXF/STL/STEP, BOM, stacks
    ../.venv/bin/python check.py         # interference, ball path, rotation sweep (~15 min)
    ../.venv/bin/python safe_parts.py    # parts the same in v8.1 and v8.1GB (run turret.py in both first)
    ../.venv/bin/python reach.py         # spin radius of every turning part; exits 1 above 203 mm
    ../.venv/bin/python laser_check.py   # plywood webs >= 2.5 mm, holes, bed size (reads out/dxf)
    ../.venv/bin/python check_camera.py  # camera view vs the turret's own parts
    ../.venv/bin/python feed_sweep.py    # ball escape behind the feed (2,700 cases)
    ../.venv/bin/python camera_tilt_study.py  # best camera tilt (needs out/approach/shoot_map.json)
    ../.venv/bin/python access_check.py  # tool access; in-season adjustments must pass (~5 min)
    ../.venv/bin/python dfm_check.py     # can a cutting service make every aluminum plate?
    ../.venv/bin/python approach_check.py  # 3D: shots from any angle vs the whole HIVE Structure (~5 min; out/approach/)
    ../.venv/bin/python shoot_map.py     # figure 7: where the robot can score from, plan view (~15 min on 11 cores; out/approach/shoot_map.json)
    ../.venv/bin/python height_budget.py
    ../.venv/bin/python belt_test.py     # belt test plates
    ../.venv/bin/python motor_mount_fig.py  # labelled motor-mount drawing on the page
    ../.venv/bin/python proto_print.py   # printed prototype: out/proto (STLs by build step, PROTOTYPE.md) + page table
    ../.venv/bin/python cost_page.py     # "What it costs" from prices.json + scs_quote.json
    ../.venv/bin/python diagrams.py && ../.venv/bin/python pack_meshes.py
