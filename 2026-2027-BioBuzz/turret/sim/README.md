# Field shot simulator

Live: https://crossbotics.com/parts/turret/sim/ (linked from the turret page's menu).
The team plans to keep using and extending it as the rest of the robot is designed.

## Files

| File | What it is |
|---|---|
| `field_sim.html` | The page (three.js r128). Placeholders `/*@SIMCORE@*/`, `/*@GEOM@*/`, `/*@MAP@*/` are filled by `build_sim.py`. |
| `sim_core.js` | Shot physics. A line-by-line port of `approach_check.shots()` / `shoot_map.point()`, plus `fly()` for the animation (same drag model, rough bounces). Runs in the page, in a Web Worker, and in node. |
| `validate.mjs` | Checks `sim_core.js` against the Python shooting map. Run after any model change: `node sim/validate.mjs 16`. Must report 0 mismatches. |
| `build_sim.py` | Inlines the core and data into `../out/sim/index.html`. |
| `../sim_export.py` | Writes `../out/sim/sim_geom.json` from the Python models (every constant, the frame tubes, the other HIVE's bars) and copies `shoot_map.json`. |
| `../../field_cad/export_field.py` | Official field CAD -> `../out/sim/field.glb` (7.4 MB). `compare_cad.py`, `frame_cad.py`, `cell_tag_cad.py` measure the HIVE for the Python models. |

`deploy_site.sh` runs `sim_export.py` and `build_sim.py`, then copies `index.html` and `field.glb` to `turret/sim/`.

## Rules

- The Python models are the source of truth. Change geometry or physics there, rerun `shoot_map.py --fresh`, then `sim_export.py`, `build_sim.py`, and `validate.mjs`.
- `judge()` decides what scores. `fly()` draws the flight and also decides what stays in the CELL (retained weight for tipping) and the shot log's "bounced out"; it is rougher than `judge()`. A sweep of 408 shots (2026-09-29): every clean "in" lands in the CELL, every miss stays out; rim touches sometimes roll in.
- Shots that need better than ±2% speed room are shown as "only in theory" (`PRACTICAL` in the page).

## Aiming modes (2026-09-29)

- Auto: only while `seesTarget()` (port of `camera_coverage.sees_target`, checked on 2,856 spots: 0 mismatches) says the Limelight, yawed at the opening, can read our up-CELL's tag cluster. Then the solver picks angle and speed.
- Otherwise manual (turret heading from the robot's front, launch angle within the hood's range, exit speed with motor rpm). `manualEval()` judges the set shot and every speed at that angle and heading. "Calculate best shot (in theory)" runs the solver and loads the result into the sliders.
- Camera tilt is a setting (20-50 deg, default `T.LL_TILT`).

## Ideas for later (as the robot gets designed)

- Nectar wear sensing (future state, see DESIGN_LOG "Future state: Nectar wear sensing"): the user prefers a swing-arm angle sensor or the flywheel encoder dip; add a "wear sensor" setting that corrects the next shots.

- Robot moving while it shoots (the turret's feed sim already covers accelerations up to 8 m/s²).
- The real base-plate height and launch height from the chassis design (now a fixed 0.33 m).
- Drivetrain footprint and bumpers, intake side, and the indexer's timing (the 0.25 s release rate).
- Other robots as obstacles, and match timing.

## Astra review round 1 (2026-09-29, `../ASTRA_SIM_REVIEW.md`) and fixes

- Turret model: the table is 27 wheel speeds (3.5-10 m/s) x 11 hood angles (0-25 deg, 2.5 apart). `turretShot()` solves inside each valid table cell (damped Newton on the bilinear launch/exit), skips entries the feed sim didn't launch, and takes the lowest wheel speed on worn Pollen's plateau. Hold-out check (`checks/inverse_holdout.py` + `.mjs`, 60 feed runs off the grid): all found; worst 0.26 deg hood, 0.56 % wheel speed, 7.4 rpm spin.
- The solver searches the launch angles the turret reaches for that ball and wear (`turretLaunchRange`), 2 deg apart; Simple mode keeps the Python map's range.
- Unreachable manual shots don't fire; a random error past the turret's reach is capped at what it can throw.
- Manual mode: exact result at the set speed, bisected lower/upper margins, one contiguous suggestion over 3-12 m/s.
- CELL cavity in `fly()`: all 8 edges of the opening's cross-section plus the back wall; leaving through the opening needs a fresh entry.
- HIVE load per alliance and per CELL generation; stale solver jobs dropped by revision; worker failure falls back to the page; one rpm function; camera readout refreshed after "Calculate".
- Known, documented: Shapely's buffered corners vs exact distance differ by < 0.06 mm; JS now truncates the step count like Python. Spin lift (C_L = r*omega/v) and the feed model's spin are uncalibrated.

## Astra round 3 (`../ASTRA_SIM_REVIEW_3.md`) and fixes

- Wear: the turret table has 13 wear levels (stiffness 8000 -> 4000 -> 1500 N/m; quarter steps new->medium, eighth steps medium->worn). The old 3-endpoint blend overstated Nectar (65% vs ~40% direct feed). `checks/wear_check.py` + `.mjs`: 120 direct feed runs at in-between wear; 95% within 0.32 deg / 0.42 % / 8.1 rpm, worst 1.4 deg / 1.15 % / 17 rpm (limits 0.5 / 1 / 10 at 95%, 1.5 / 2 / 20 worst).
- `turretForward()`: exact entries and valid edges handled, no bridging of failed launches (returns null -> "didn't launch"); wheel speed capped at 10 m/s.
- Variation: hood play moves the hood (clamped 0-25) before the lookup; wear reflected at new/worn (no pile-up); burst slowdown applied to exit speed (reference-setting data).
- Hit rate: 400 samples, shown with a 95% range; memo key includes the field-frame mirroring; one verdict drives text and colour; label "single shots, empty CELL, clean entry".
- Only settled balls count as held (up to 5 continuation flights); bursts stop when ball, wear, variation, ball-flight model or gate changes.
- Gates now fail on thin coverage (holdout: >= 60 chosen, <= 5 outside; replay: >= 60 rows).
- Default spot, Realistic, 2000 shots: Pollen 93.2 % (Astra direct feed 93.5 %), Nectar 46.6 % (direct 39.8 %, with launch-angle instead of hood scatter).

## Shot choice (2026-09-29)

The solver still finds every launch angle that scores and its speed room. In the turret model, the page then picks
among them with `pickShot()` (sim_core.js): the angle with the best hit rate under the chosen variation (300 shots,
the same seeded random balls for each angle), preferring the lower exit speed within 3 points. With variation off: the
lowest exit speed within 0.5 points of the widest speed room. Why: at longer range the speed room is nearly flat
across angles, so "widest room" often picked a lob (e.g. Nectar from a corner: 81 deg) that scores less often in
practice (more sensitive to hood error, near the 10 m/s wheel limit). Over 108 spots: lobs (>= 72 deg) 45 -> 23,
hit rate +13 points on average (worst -1.6, sampling noise). The Simple model and validate.mjs still use the widest
room, as in the Python map.
