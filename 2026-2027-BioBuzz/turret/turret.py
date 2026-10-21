"""
FTC BIOBUZZ (2026-27) swivel turret, v2: linked dual flywheels, adjustable hood, Limelight 3A.

What changed from v1
  * The hood rollers are replaced by a second (upper) flywheel on a spring-loaded swing arm.
      - Main shaft -> GT2 belt -> jackshaft. The jackshaft sits on the hood carrier, which
        pivots on the main shaft, so that belt's center distance never changes.
      - Jackshaft -> gear pair -> upper flywheel. The swing arm pivots on the jackshaft, so the
        gear center distance never changes either. The gears also reverse direction, so the
        two wheels counter-rotate, and the 32:24 ratio matches surface speed (96 vs 72mm wheels).
      - Extension spring (posts, 3 preload holes) pulls the arm in; an eccentric printed cam is
        the Pollen hard stop (rotate to tune squeeze +/-2mm). Nectar pushes the arm out.
  * Flywheel motor lies across the front between the side plates, belt to the main shaft outboard.
  * Limelight 3A tilt mount at the front: 20-50 deg in 5 deg steps (the HIVE tags face down).
  * Stiffer, taller side plates with small pockets and two hood guide slots.

Edit PARAMETERS and run:  python turret.py   (outputs in ./out, same layout as v1)

Frame: Z up, origin = turret axis on top of the fixed base plate. Balls rise up the center and
leave toward -Y ("front"). Carrier angles psi are about the main flywheel axis, from +Y toward +Z.
"""
import json
import math
import re
from pathlib import Path

import ezdxf
import numpy as np
from shapely.geometry.polygon import orient
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union
from shapely import affinity
from build123d import (Face, Location, Plane, Wire, Compound, export_step,
                       export_stl, extrude, Rot, Pos, Axis, Box, Solid, Vector, Align)

# ============================== PARAMETERS ==============================
POLLEN_D = 71.1           # 2.8 in (manual 9.8: "approximately")
NECTAR_D = 91.4           # 3.6 in
# Squeeze is set by SPRING FORCE, not by the stop (v3). Worn, softer balls squish further for the same grip.
COMPRESSION = 16.0        # squeeze with the upper wheel on its hard stop (worn Pollen); mm
MIN_SQUEEZE = 4.0         # lightest squeeze (new Nectar pushing the arm out); sets the arm's travel, mm
PINCH_FORCE = 40.0        # target spring preload at the upper wheel, N (sim: keeps exit speed within ~5-13% as balls wear)
WHEEL_SURFACE_SPEED = 8.5 # m/s target; a HIVE lob needs roughly 6-9 m/s at the exit

# v7: the frame plates (side plates, hood carriers, deck, base plate) are two laser-cut 1/8" (3 mm) Baltic birch layers
# glued together, grain crossed. Threads become M4 square nuts pressed into a pocket in one layer; countersunk flat heads
# become button heads sitting in a counterbore in the other layer; 3 dowel holes per pair line the layers up for gluing.
# PLY_LAYER / PLY_GLUE: set them from a caliper measurement of the team's sheets and a glued test piece.
LAMINATE = True
PLY_LAYER, PLY_GLUE = 3.0, 0.2
T_PLATE = 2 * PLY_LAYER + PLY_GLUE if LAMINATE else 6.35
CBORE_D = 8.2                 # counterbore for an M4 button head (ISO 7380: 7.6 dia, 2.2 tall) in the 3 mm head-side layer
SQNUT = (7.0, 2.2)            # M4 square nut, DIN 562 (thin): width, thickness
SQNUT_POCKET = 7.3            # laser-cut square pocket (press fit + a drop of CA glue)
DOWEL_D = 3.3                 # 1/8" (3.18 mm) hardwood dowel, cut flush: lines the two layers up while the glue sets
KERF_HOLE = 0.0
M4, M8 = 4.3, 8.2
BRG_1611 = 14.0           # goBILDA 1611 flanged bearing OD (8mm REX)
PRINT_BRG_1611 = 14.1     # 1611 seat in a PA-CF print: the lifter's bearing fit coupon (2026-10-16, same printer + PA-CF) fit at 14.1
REX_AF, REX_D = 7.0, 8.0
B608_OD, B608_W = 22.0, 7.0
MOTOR_BOLT_SQ, MOTOR_CENTER_HOLE, MOTOR_D = 16.0, 14.0, 37.5
# v6: the 5203 face has 8 M4 holes (goBILDA spec sheet): 4 on the 16 mm square and 4 at +/-12 mm on the axes. v6 mounts
# both motors on the 4 axis holes (r 12): the web to the 14 mm centre hole is 2.85 mm (>= 2.41 cutting-service minimum;
# the 16 mm square left only 2.16 mm).
MOTOR_BOLTS = [(12.0, 0.0), (-12.0, 0.0), (0.0, 12.0), (0.0, -12.0)]

# ---- turret bearing / drive (unchanged from v1) ----
FEED_HOLE_D = 110.0
BASE_HOLE_D = 120.0         # v8: base-plate centre hole: the ball (Nectar 91.4) + a cable ribbon side by side, for any of the
                            # cable methods kept open (DESIGN_LOG v8). The roller bolts' holes start at r 68.9, so 8.9 mm of web.
ROLLER_R = 73.0
N_ROLLERS = 6
TIRE_OD, TIRE_ROOT_D = 38.0, 30.0
TIRE_LIP = 1.0              # center lip in the tire bore: each 608 seats against it (outer race only)
TIRE_H = 2 * B608_W + TIRE_LIP
INNER_RACE_OD = 11.5        # 608 spacers touch the inner race only, never the shields
ROLLER_SPACER_H = 4.0
RING_MODULE, RING_TEETH, PINION_TEETH = 0.8, 280, 20   # goBILDA 2303-4008-0020 steel pinion on the turret motor
GEAR_BACKLASH = 0.15
TURRET_MOTOR_ANGLE = 135      # v8: was -90 (front, right where the indexer comes in under the turret). 135 = back-left
                              # corner, inside the footprint; the lifter's slide is in the back-RIGHT corner.
# v6 (Codex review 2026-09-26): the turret motor's gear mesh must be settable WITH the ring on, and the motor must come
# out without lifting the deck. The inner screw (under the ring) is dropped: 3 screws (the two tangential + the outer).
# The deck and cable drum get access holes that line up over those 3 screws when the turret is turned so that deck
# angle TURRET_ACCESS_ANG sits over the motor. The centre hole shrinks 14 -> 10 mm (only the 8 mm shaft passes): with
# the 1.5 mm mesh slots, 14 mm left a 1.35 mm web to the radial screw slots.
_TB_M90 = [(12.0, 0.0), (-12.0, 0.0), (0.0, -12.0)]   # as laid out for the motor at -90 deg: (0, +12) would be the inner one
_tb = math.radians(TURRET_MOTOR_ANGLE + 90)          # v8: turned with the motor, so the dropped screw is still the inner one
TURRET_BOLTS = [(round(dx * math.cos(_tb) - dy * math.sin(_tb), 6), round(dx * math.sin(_tb) + dy * math.cos(_tb), 6)) for dx, dy in _TB_M90]
TURRET_CENTER_D = 10.0
TURRET_ACCESS_ANG, TURRET_ACCESS_D = -30.0, 8.0             # deck angle of the access holes; hole diameter. v8: -30 (was 0):
                                                            # the deck's pad for them stays inside the start footprint, and
                                                            # heading 165 (inside +/-190) lines them up over the motor at 135
DECK_ID, DECK_OD, DECK_SPACER_R = 166.0, 224.0, 100.0
# v5: 22.5/157.5 moved to 45/135. At 22.5 the bolt head sat under the rear L-bracket foot and its hole was 5 mm
# from the bracket's deck hole (checked by the fastener pass in check.py).
DECK_SPACER_ANGS = [52.0, 67.5, 112.5, 128.0, 202.5, 247.5, 292.5, 337.5]   # v6: 45/135 -> 52/128 (keys hit the rear crossbar)
RING_TAPPED = True     # v6: deck-spacer screws thread into the ring (tap M4 in aluminum, heat-sets if printed): no trapped nuts
DECK_TAPPED = True     # v6: L-bracket foot screws thread into the deck (tap M4): no nuts trapped under the deck
BRKT_FOOT_L, BRKT_FOOT_HOLE = 22.0, 13.0
# v6: foot and side screws staggered so a hex key for the side screws doesn't hit the foot screw heads
BRKT_W, BRKT_FOOT_DY, BRKT_SIDE_DY = 36.0, (-4.0, 4.0), (-12.0, 12.0)   # L-bracket foot length and hole distance from the side plate (v5: was 16 / 8;
                                           # a socket head at 8 dug 0.5 mm into the bracket's own upright)
# ---- fasteners (v5): modelled so check.py sees heads and nuts; flush ones are countersunk ----
SHCS_M4 = (7.0, 4.0)          # socket head: dia, height
BTN_M4 = (7.6, 2.2)           # button head
NYLOC_M4 = (8.1, 5.0)         # across corners, height
HEAD_M8, NYLOC_M8 = (13.0, 5.3), (15.0, 9.0)   # M8 roller bolt head; washer + nyloc
SHCS_M3, NYLOC_M3 = (5.5, 3.0), (6.4, 4.0)
CSK_D, CSK_DEPTH = 9.0, 2.35  # M4 flat head (ISO 10642, 90 deg): countersink dia on the face, depth for a 4.3 hole
TAP_M4 = 3.3                  # tap drill
FW_MOTOR_CSK = False          # v6: button heads instead (v5's countersinks sat 4.3 mm from the motor's centre hole, under a
                              # cutting service's 8.6 mm rule, and nicked it). The motor belt plane moves out MOTOR_BELT_SHIFT so
                              # the 2.2 mm button heads clear the 16T pulley. Holes stay round: fixed, tested centre distance.
MOTOR_BELT_SHIFT = 0.0          # v6: no longer used (the motor screws now sit in the side-plate window)
# v6 (open-item review): parts bolted to a Sonic Hub on their OUTBOARD side (48T pulley, 60T and 48T sync gears) have their
# 4 hub-screw heads on the INBOARD face, facing a bearing. The hub pattern is the 16 mm square (heads at r 11.3, so r 7.5-15.1
# for a button head; the 1611 flange is only r 7.5). With 0.5 mm of end play the heads hit the plate / swing arm. Button
# heads (2.2 mm) and 2.5 mm from the bearing flange clear it (3.5 mm to the plate face).
HUB_HEAD_GAP = 2.5
HUB_SCREWS = [(8.0, 8.0), (-8.0, 8.0), (8.0, -8.0), (-8.0, -8.0)]
# v6 (2026-09-26): ADJUSTABLE flywheel-motor mount, so the motor belt can be tensioned on the assembled turret.
# The motor bolts to a 1/8in aluminum ADAPTER on the INSIDE of the motor-side plate. The side plate has a window for the
# motor-screw heads and slots for 4 clamp bolts; the clamp heads sit OUTSIDE, clear of the 16T pulley and belt. A printed
# LUG (clamped by two of the bolts) and a printed YOKE (bolted to the plate) sit in front of the motor: the JACK SCREW
# goes through the yoke into a heat-set in the lug and pulls the motor FORWARD (key from the front). Belt tension pulls
# the lug back toward the yoke, so the belt can't go slack even if a clamp bolt loosens.
# All offsets below are (Y, Z) from the motor axis; Y is forward-negative.
MOTOR_ADJ = True
ADAPT_T = 3.175                    # 1/8in 6061 (tapped M4 for the 4 clamp bolts)
MOTOR_SLIDE = (-1.0, 0.0)          # slide direction: straight forward. C grows 0.69 mm per mm of slide (v8.1: the slide is 46 deg off the belt line)
MOTOR_TRAVEL = 2.0                 # +/- mm of slide (the 16T pulley goes on WITH the belt, so no install slack is needed)
MOTOR_WIN_R = 16.5                 # side-plate window around the motor-screw heads (r 12 + 3.8 head + 0.7)
JACK_DZ, JACK_H = 5.0, 5.0         # jack screw axis: 5 mm above the motor axis, 5 mm off the side plate's outer face
JACK_SCREW_L = 16.0                # M4 x 16 button head (v8.1: was socket; the lower head keeps the jack inside 201 mm)
LUG_Y, LUG_W, LUG_H = (-29.5, -21.0), 10.5, 10.0          # tension lug: Y range, half-width about the jack axis, height
LUG_BOLTS = [(-25.5, JACK_DZ - 6.5), (-25.5, JACK_DZ + 6.5)]
CLAMP_BOLTS = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for r, a in ((25.0, 110.0), (26.0, 256.0))]
# v7 (Astra round 2): the lower clamp bolt moved from (r 25, 250 deg) so its printed square-nut pocket keeps 1.3 mm of
# plastic to the front-crossbar notch (it had 0.15 mm); 5.4 mm from the motor's own screws.
YOKE_BRIDGE, YOKE_ARM, YOKE_H = (-38.5, -32.0), (11.5, 19.5), 10.0   # bridge Y range; arm span from the jack axis; height
YOKE_ARM_Y = -24.0                 # the arms run back to here
YOKE_SCREWS = [(-28.0, JACK_DZ - 15.5), (-28.0, JACK_DZ + 15.5)]    # M4 x 16 into the tapped side plate
CF_AXLE_D = 9.5               # printed sleeve the spring drum spins on (M4 heat-set in its outer end). v6.1: 8 -> 9.5 so the
                              # 5.6 mm insert pocket keeps a ~2 mm wall
BASE_R, BASE_MOUNT_R = 124.0, 118.0
# v8.1GB: the goBILDA Gear-Driven Turret (3208-0004-0001) replaces our base plate, printed ring gear, 6 rollers and
# the turret pinion (user, 2026-10-04: "let's do v.8.1GB with the GoBilda bearing, leaving 8.1 completely available").
# Sizes from the seller's STEP (gobilda.com/content/step_files/3208-0004-0001.zip, read 2026-10-03): aluminum octagon
# housing 144 mm across the flats, 0-10 mm; turning gear ring (176T MOD 0.8) OD 142.4 at 13-20 mm on an inner ring
# r 52.5-62 (105 mm bore); 64T drive gear (2302-0014-0064, included) at r 96, 13-19 mm, on the included 1-side
# 2-post motor mount (x 72-115, 0-8 mm). z = 0 is the housing's bottom (the robot side); the turret above is
# exactly v8.1's (deck at 21.2), so every part above the deck is the same file.
GB_TURRET = True
GB_SKU = "3208-0004-0001"
GB_ANGLE = 180.0                   # drive (64T + motor) direction: the left side. Front = indexer, back = cables
GB_AF, GB_HOUSING_H, GB_BORE_R = 144.0, 10.0, 52.5
GB_RING = ((52.5, 62.0, 1.0, 13.0), (52.5, 71.2, 13.0, 20.0))   # (r0, r1, z0, z1) of the turning inner ring + gear ring
GB_DRIVE_R, GB_GEAR_OD, GB_GEAR_Z = 96.0, 52.8, (13.0, 19.0)
GB_MOUNT = ((72.0, 115.0), (-28.0, 28.0), (0.0, 8.0))          # (x, y, z) box of the 1-side 2-post mount, drive at +x
GB_RATIO = 176 / 64
GB_MOTOR_RPM, GB_MOTOR_SKU, GB_MOTOR_L = 312, "5203-2402-0019", 116.4   # body length from goBILDA's STEP (140.1 - shaft)
GB_MOTOR_CPR = 537.7               # encoder counts per output turn (goBILDA page, checked 2026-10-03)
GB_DECK_HOLES = [(64.0, 8.0), (64.0, -8.0), (-64.0, 8.0), (-64.0, -8.0), (8.0, 64.0), (-8.0, 64.0), (8.0, -64.0), (-8.0, -64.0)]
                                   # 8 of the gear ring's tapped M4 top holes (8 mm grid, STEP: 3.3 mm tap holes)
GB_SIDE_HOLES = (-16.0, 16.0)      # tapped M4 in each housing flat, 4 mm up, +/-16 (and +/-24) along the flat
GB_SHIM_R = (56.5, 70.5)           # printed deck shim on the ring top (20 -> 21.2): keeps the deck 2.2 mm over the 64T
DECK_ID_GB = 112.0                 # deck's centre opening: the ball (Nectar 91.4) + room; the ring screws sit at r 64.5

# ---- v5 cable management: a reverse-bend ("U-loop") service loop in an annular tray ----
# Four cables cross the turret joint: flywheel motor power, its encoder cable (also carries the Through Bore
# encoder), the hood servo and the Limelight USB-C. The centre is the ball path, so there is no slip ring.
# Instead the turret turns +/-CABLE_TRAVEL deg (software soft limits, homed by a REV magnetic limit switch) and
# the cables lie in a channel as a ribbon standing on edge: along the drum (turns with the turret), a U-turn,
# then back along the fixed outer wall. The U-turn rolls around the channel at rin/(rin+rout) of turret speed.
CABLES = False             # v8: cable management chosen later (see DESIGN_LOG v8); space reserved under the plate
CABLE_TRAVEL = 190.0              # +/- deg from home; 380 deg total, so any heading is reachable
DRUM_R = (131.0, 141.0)           # drum wall (turret): clears the turret pinion tips (r 129) and motor bolts
DRUM_Z0 = 5.0                     # drum bottom (clears the M4 motor-bolt heads on the base plate)
TRAY_R = (108.0, 184.0)           # tray: flange on the base plate (robot-mount bolts at r 118) out to the wall
TRAY_T, TRAY_WALL, TRAY_H = 3.0, 4.0, 24.0   # floor, outer wall thickness, wall height (turret frame starts at z 27.5)
TRAY_NOTCH = 18.0                 # +/- deg flange cut-out around the turret pinion / motor bolts (r < 134)
# v7: the team's printers are a Bambu Lab H2C (~330 x 320 mm) and a Creality K2 Plus (350 x 350 mm). The tray (368 mm
# across) can't be one piece; 3 x 120 deg segments are ~319 mm long and fit either. The drum (282 mm) prints whole.
TRAY_N, DRUM_N = 3, 1
TRAY_SPLITS = [30 + 360 / TRAY_N * k for k in range(TRAY_N)]   # each split sits between robot-mount bolts
DRUM_SPLITS = [0 + 360 / DRUM_N * k for k in range(DRUM_N)]
RIBBON_T, RIBBON_H, RIBBON_Z0 = 6.0, 14.0, 5.0   # the 4 cables stacked on edge (sleeved / heat-shrink bands)
CABLE_MARGIN = 25.0               # deg: closest the U-turn ever gets to either anchor
CABLE_IN_ANG = 80.0               # drum anchor (zip tie) at home, deg (rear)
HOME_ANG = 45.0                   # REV-31-1462 magnetic limit switch on the tray flange; magnet in the drum
SW_DIMS = (41.5, 15.0, 17.0)      # REV-31-1462 datasheet: 41.5 long, 15 wide, 17 tall, 3 mm base, 2x 3.4 holes 32 apart
SW_HOLES = 32.0
SW_OUT_R = DRUM_R[0] - 1.5        # switch side face (side sensor, 5 mm range) 1.5 mm from the drum's inner face
MAG_DIMS = (22.6, 6.1, 9.0)       # included magnet (tabs 16 mm apart, M3), set in a pocket in the drum's inner face
# v8: with no cable tray the home switch sits on a small pad of the base plate, just outside the ring gear (tips r 113), and
# its magnet hangs from an ear of the deck in a printed holder. -45 deg (front-right) keeps the switch clear of the
# robot-mount bolts (-22.5 / -67.5 deg), the holder clear of the deck spacers (r 100 at -22.5 / -67.5 deg) and the L-bracket
# feet (y -20..56), and the pad and the deck's ear inside the start footprint.
HOME_FLAG = not CABLES
if HOME_FLAG:
    HOME_ANG = -45.0                  # front-right: the deck's ear stays inside the start footprint here
    SW_IN_R = 116.0                   # switch inner face: 3 mm outside the ring gear's tips
    SW_RC = SW_IN_R + SW_DIMS[1] / 2  # switch centre line
    MAG_IN_R = SW_IN_R + SW_DIMS[1] + 1.5   # magnet face 1.5 mm outside the switch's outer (sensing) side
    MAG_ZC = 10.5                     # magnet centre above the base-plate top (the switch's sensor is ~9 mm up)
    FLAG_R = (MAG_IN_R - 0.2, 149.0)  # holder: radial span (16.5 mm: an M4 heat-set keeps ~5.5 mm of plastic each side)
    FLAG_W = 30.0                     # tangential width
    FLAG_Z0 = 6.0                     # holder bottom
    EAR_R = 151.0                     # deck ear out to here, carries the holder
if CABLES:
    BASE_R = 132.0                # room for the switch's M3 bolts (through the base plate, nylocs underneath)
    DECK_OD = 2 * DRUM_R[1]       # deck grows to the drum's outer face; drum bolts up under it

# ---- shooter ----
# v5 (measured from goBILDA's STEP files, 2026-09-25): each GripForce Gecko is 24 mm wide with a 38 mm core drilled
# on the 16 mm pattern; three are bolted through their cores into one Sonic Hub with 4x M4 x 80 screws.
GECKO_W, GECKO_N = 24.0, 3
FW1_D, FW1_W = 96.0, GECKO_W * GECKO_N     # main (lower) flywheel stack, 72 mm
FW2_D, FW2_W = 72.0, GECKO_W * GECKO_N     # upper flywheel stack on the swing arm, 72 mm
HUB_W, HUB_R = 7.86, 16.0     # 1309 Sonic Hub body (its 14 mm boss, 2.1 mm tall, sits inside the part's 14 mm bore)
PUL48_W = 12.34               # 3415 48T hub-mount pulley, flange to flange
PUL_SS_W = 12.0               # 3417 16T / 24T set-screw pulleys, flange to flange (belt runs in the middle)
GEAR_T = 4.0                  # 2312 acetal hub-mount gears
WHEEL_SCREW_HEAD = 4.0        # M4 socket heads on the outer Gecko core
FW_Y, FW_Z = -85.0, 98.0      # main flywheel axis (v4: 12mm lower for the 18in height budget)
CORRIDOR = 100.0              # clear width for the ball between hood carriers
PINCH_PSI = 10.0              # v5: 80 deg at hood alpha 0 (the real opening favours 75-80 close, 55 far)
                              # exit elevation = 90 - psi - alpha
HOOD_ALPHA_MAX = 25.0         # v5 hood range (Pollen): 80 deg (close to the HIVE) down to 55 deg (across the field)
SYNC_MODULE, SYNC_TEETH_J, SYNC_TEETH_S = 0.8, 60, 48   # v5: goBILDA 2312-0414-0060 : -0048, 43.2mm apart.
# 60:48 runs the upper wheel's surface ~6% SLOWER than the main wheel -> mild BACKSPIN (helps the ball stop on the
# cell floor instead of rebounding off the back wall). 60:45 would be matched surface speed (no spin).
# goBILDA 5mm HTD, 9mm belts (3412-0009-LLLL). Available pitch lengths, mm:
HTD_BELTS = [215, 225, 245, 265, 275, 295, 315, 320, 340, 360, 370, 380, 390, 410, 420, 425, 445, 460,
             470, 485, 505, 520, 535, 560, 580, 600, 650, 700, 745, 800, 840, 890]
JACK_PULLEY_T = 24            # 3417-4008-0024 on main shaft and jackshaft (1:1)
MOTOR_PULLEY_T = 16           # 3417-4008-0016 on the flywheel motor
MAIN_MOTOR_PULLEY_T = 48      # 3415-0014-0048 + 1309 hub on the main shaft  -> 3:1 reduction
MOTOR_BELT = 315              # 3412-0009-0315. v8.1: 340 -> 315 so the motor (and its jack) sit 13 mm closer to the main shaft
# Belt-fit trims (mm added to the theoretical centre distance). The jackshaft belt runs on a FIXED centre, so cut its
# belt test plate first (belt_test.py), see which engraved position fits, and enter it here. The motor belt is on the
# adjustable mount (MOTOR_ADJ): its trim only re-centres the slide range if its (optional) test shows a big error.
MOTOR_C_TRIM = 0.0            # motor -> main shaft (MOTOR_BELT)
JACK_C_TRIM = 0.0             # main shaft -> jackshaft (410 belt)
# v4: SECOND flywheel motor (the v3 rapid-fire sim showed one 5203 can't recover between Nectar shots).
# Lies across the rear-low corner between the plates, face on the SERVO-side plate, belted outboard on that side.
SECOND_MOTOR = False          # v5: OFF. The robot holds at most 4 balls and 3-4 shots/s is plenty; one motor holds
                              # speed within a few % at that rate (sim_rapid.py). Switch on for faster Nectar volleys.
MOTOR2_BELT = 505             # 3412-0009-0505
MOTOR2_PSI = -15.0            # direction from the main axis (rear, slightly low)
ENCODER = True                # REV Through Bore Encoder V2 (REV-11-3174) on the main shaft, motor side, outboard
# REV-11-3174 drawing (2025-09-26): body 50 x 57.5, 15.3 thick (13.3 body + 2.0 hub), 1/2in hex bore,
# two ears with 5.0mm holes 50.8mm apart on a diagonal through the bore.
ENC_T, ENC_W, ENC_LEN = 15.3, 31.1, 34.5     # thickness; body width; bore-to-body-end length (body points UP)
ENC_EAR_R, ENC_EAR_ANG = 25.4, 40.0          # ear holes: 25.4mm from the bore at 40 and 220 deg
BRKT_T = 4.0                                 # printed bracket plate
BRKT_LEGS = [(58.0, 95.0), (58.0, 150.0)]    # (r, deg about the main axis) legs to the side plate: clear of
                                             # the 48T pulley, motor-1 belt (MOTOR_PSI) and jackshaft belt (22-47 deg)
DRIVE_W = 16.0                # belt/gear layer width (HTD pulley widths unverified: leave room)
ARM_TAIL = 32.0               # constant-force spring attaches this far past the upper-wheel axle
CF_SPRING_N = 11.6            # AndyMark am-4666 constant-force spring: 2.6 lb, 0.5in wide (McMaster 9293K52: same, 2.63 lb)
# v6.1: the geared swing-arm sim (sim_feed.py) showed one spring per side lets a Nectar fling the arm open (to ~75-93 deg)
# into the carrier slot ends. No constant-force spring 0.5 in wide or narrower is sold above ~4 lb (checked 2026-09-28:
# AndyMark, McMaster, Lee, Century, WCP, Misumi, Vulcan), so the springs are STACKED on one drum (wound together, same
# width). 2 per side keeps the arm under ~33 deg in every sim case; the drum and clearances are sized for 3 so the team
# can tune after the real-ball squeeze test.
CF_SPRINGS_PER_SIDE, CF_SPRINGS_MAX = 2, 3
CF_STRIP_T, CF_STRIP_L, CF_COIL_ID = 0.2032, 762.0, 14.9      # am-4666: 0.008 in thick, 30 in, 0.588 in ID

# v6.1 (Astra): Lee's constant-force mounting guidance is a drum 10-20% LARGER than the coil's natural ID (14.9 mm);
# v6 had 14.5. 16.5 mm = +11%.
CF_DRUM_D = 16.5

def cf_coil_od(n):
    """Outer diameter of n stacked 30 in strips wound on the drum (area conservation); 1 strip on a 14.9 mm core -> 20.5 mm
    (McMaster lists 0.82 in)."""
    return math.sqrt(max(CF_COIL_ID, CF_DRUM_D) ** 2 + 4 * n * CF_STRIP_T * CF_STRIP_L / math.pi)

# v6.1 (Astra): heat-set pockets sized for the insert actually on the parts list (set in one place)
INSERT_HOLE_D, INSERT_DEPTH, INSERT_MIN_WALL = 5.6, 5.2, 5.5   # McMaster 94459A150: M4, 4.7 mm long, 5.6 mm hole, >= 5.5 mm of material
CF_DRUM_DIST = 40.0           # drum sits this far from the arm post, toward the main wheel

# ---- spring-loaded idler rollers behind the ball column (press the ball onto the main wheel) ----
IDLER_D = 20.0                # printed roller on two 625 bearings, 5mm axle
IDLERS = False                # v4: OFF. The upper wheel now sits beside the column and holds the ball against the main
                              # wheel itself; the sim showed low idlers push balls out the front (see DESIGN_LOG.md)
IDLER_PSI = (-32.0, -17.0)    # (only used if IDLERS is re-enabled)
IDLER_SQUEEZE = 4.0           # light squeeze at the Pollen stop (the flywheel pinch does the real work)
IDLER_FORCES = [(1.1, 8.6), (1.2, 6.5)]   # N per roller at the front stop / pushed back by Nectar (from the geometry)
IDLER_AXLE = 6.0              # goBILDA 2100-0006-0200 6mm round shaft; rollers on 1611-0514-0006 bearings
# goBILDA 2915-0001-0003 springs (6.5mm OD, 39mm free, 33mm travel), one each side of each roller.
# (direction from the axle in deg, 0 = +Y/rear, 90 = up; anchor distance from the axle, mm). Angled across the slot
# so the forward push stays light: lower roller ~1.1 N at the stop -> 8.6 N for Nectar; upper ~1.2 -> 6.5 N.
IDLER_SPRING_GEOM = [(115.0, 56.0), (105.0, 58.0)]   # both angle up and forward, across their slots

# v6: the hood gear is BOUGHT: goBILDA 2302-0032-0108 aluminum MOD 0.8, 108T, 6 mm thick, 88 OD, 32 mm bore, 4 mm holes
# on the 8 mm grid (measured from goBILDA's STEP). It bolts to the servo-side carrier's OUTER face with 4 M4 flat heads
# countersunk from the ball side (+ nylocs). 108:20 = 5.4:1 (v5 cut a 140T segment into the carrier: 7:1). The small
# MOD 0.8 teeth were too fine for a cutting service in 1/4" aluminum (SendCutSend min bridge 2.41 mm vs 1.26 mm tooth).
SECTOR_MODULE, SECTOR_TEETH_FULL, SERVO_PINION_TEETH = 0.8, 108, 20   # 2303-4008-0020 on a 1922 servo shaft: 5.4:1
HOOD_GEAR_T, HOOD_GEAR_BORE = 6.0, 32.0
HOOD_GEAR_WEB, HOOD_GEAR_RIM_R = 3.0, 40.70   # measured from the STEP: 3 mm web, full 6 mm only outside r 40.70 (v6 said 40.25)
HOOD_GEAR_BOLTS = [(sx * 24.0, sy * 24.0) for sx in (-1, 1) for sy in (-1, 1)]   # 8 mm grid (3,3): r 33.9, carrier frame
FRONT_PLATE = True            # v6: optional flair panel across the front (sticker space); zip-tied to the front crossbar
FRONT_PLATE_T, FRONT_PLATE_H = 3.0, 70.0     # 3 mm basswood / plywood on a diode laser, or printed
STOP_ALONG, CAM_ECC = 11.0, 0.75   # v6: stop cam 11 mm along the arm from the jackshaft, so its screw is reachable from
                              # the ball side (at 24 mm the upper wheel blocked the key). Closer to the pivot = more leverage
                              # on the cam, so the eccentric is halved (0.75 mm) to keep the same ~+/-2.7 mm range at the wheel.
SERVO_TAPPED = True           # v6: tap the 4 hood-servo ear holes in the side plate: no nuts, the servo comes off from outside
CAM_R = 6.5                   # v6 stop cam radius (room for an M4 heat-set at the 1.5 mm eccentric offset)

MOTOR_PSI = 226.0             # flywheel motor direction from the main axis (front, low; clears the deck at FW_Z 98)
SECTOR_PSI = -60.0            # v4: hood gear segment + servo behind/below the main wheel (was straight below)
SERVO_BODY_FWD = True         # servo body extends forward from its spline

LL_DIMS = (72.11, 48.11, 16.80)     # Limelight 3A (docs.limelightvision.io)
LL_FOV = (54.5, 42.0)
LL_PIVOT = (-128.0, 175.0)    # (Y, Z) tilt pivot. v8: 192 -> 175 (user, 2026-10-03: "Drop the camera"). The side plates' top
                              # is the pivot + 18, so the start height drops 209 -> 193 mm above the base plate; no clash with
                              # the hood (0-25 deg) at any tilt, view still clear (check_camera.py), tags face down so a lower
                              # lens sees them as well or better (camera_tilt_study.py)
LL_TILT = 45.0                # default camera tilt above horizontal (v6.2: 35 -> 45; sees our up-CELL's tags from 90% of scoring spots, camera_coverage.py)
LL_TILT_RANGE = (25, 50)          # v8.1: 20 dropped (camera mount hits an encoder-leg screw there at hood 25; 20 deg sees only ~41%)
LL_LOCK_R = (44.0, 58.0)      # 25/35/45 deg row, 20/30/40/50 deg row
LL_PLATE_T = 4.0

FRONT_Y, REAR_Y = -181.0, 104.0 if SECOND_MOTOR else 80.0   # rear only grows to carry a second motor
REAR_BAR = (66.0, 44.0)   # v8: the rear crossbar / rear guard tube. z 40 -> 44: its bottom (r 7) sat below the roller bolts'
                          # ends (33.8) and hit them as the turret turned (check.py + Astra, 2026-10-03; v7 has the same clash)
CROSSBARS = [(-106, 36), (-169, 78)] + ([] if SECOND_MOTOR else [REAR_BAR])   # rear crossbar returns without motor 2
BRACKET_Y = [38, -2]    # v6: spread for the wider (36 mm) staggered brackets; still clear of the hood servo
TAB_Y = [60, -60]
SIDE_EDGE, SIDE_RIB = 13.0, 9.0     # side-plate border and isogrid rib widths
ISO_CELL = 52.0                     # isogrid triangle size (smaller = stiffer, heavier)

OUT = Path(__file__).parent / "out"
# ========================================================================

T = T_PLATE
BASE_TOP = 0.0
ROLLER_Z0 = BASE_TOP + ROLLER_SPACER_H
RING_ZC = ROLLER_Z0 + TIRE_H / 2
RING_Z0 = RING_ZC - T / 2
RING_ID = 2 * (ROLLER_R + TIRE_ROOT_D / 2)
DECK_Z0 = ROLLER_Z0 + TIRE_H + 2.2
DECK_SPACER_H = DECK_Z0 - (RING_Z0 + T)
DECK_TOP = DECK_Z0 + T
# v6.2: REAR GUARD. At a flat hood a slow Nectar can bounce off the upper wheel's underside and drop behind the feed hole
# (into the deck's 166 mm opening, onto the turret ring). The robot also drives while firing, which throws balls fore
# and aft. A printed wall behind the feed column, hung on the rear crossbar, catches them. Side view (turret frame, mm):
# front face at REAR_GUARD_Y, from just above the deck to REAR_GUARD_TOP, which stays >= 4 mm under the upper wheel at
# every hood angle and arm swing (sweep checked in check.py). Segments feed sim_feed's obstacle list (radius = half the
# plate thickness).
REAR_GUARD = True
REAR_GUARD_T = 4.0
REAR_GUARD_Y = 58.0 + REAR_GUARD_T / 2          # plate centre line. Front face at 58: a rising Nectar is pushed back by the main wheel to y 8.7, so its back reaches 54.4 (check.py ball path)
M8_BOLT_TIP = -T + 40.0                 # v8: end of the M8 x 40 roller bolts (head under the base plate): 33.8 mm up
REAR_GUARD_Z = (M8_BOLT_TIP + 2.7, 78.0)  # v8: was DECK_TOP + 2.5 (29.9), which hit the roller bolts' ends as the turret
                                          # turned (found by check.py's rotation test, 2026-10-03). Now 2.7 mm above them;
                                          # 9 mm above the deck, far less than a ball. The top stays >= 6 mm under the upper wheel
REAR_GUARD_BAR = REAR_BAR                       # it replaces the rear crossbar: same tube, same end screws (one shared position)
REAR_GUARD_PIN = (66.0, 54.0)                   # + a second screw each end, so the part can't rotate on the tube
REAR_GUARD_W = CORRIDOR - 2.0                   # wall width: 1 mm clear of each hood carrier's inner face
REAR_GUARD_TAB = 8.0                            # end tabs (at the side plates) that carry the second screw
REAR_GUARD_SEGS = [((REAR_GUARD_Y, REAR_GUARD_Z[0]), (REAR_GUARD_Y, REAR_GUARD_Z[1]), REAR_GUARD_T / 2)] if REAR_GUARD else []
RING_RP = RING_MODULE * RING_TEETH / 2
PIN_RP = RING_MODULE * PINION_TEETH / 2
PIN_CENTER_R = RING_RP + PIN_RP + GEAR_BACKLASH

# ---------- 2D helpers (shapely) ----------
def circ(x, y, d, n=96):
    return Point(x, y).buffer(d / 2 + (KERF_HOLE / 2 if d < 30 else 0), quad_segs=n // 4)

def pol(r, a):
    return (r * math.cos(a), r * math.sin(a))

def wedge(cx, cy, r, a0, a1, n=64):
    a = np.radians(np.linspace(a0, a1, n))
    pts = [(cx, cy)] + [(cx + r * math.cos(t), cy + r * math.sin(t)) for t in a]
    return Polygon(pts)

def slot(p0, p1, w):
    return LineString([p0, p1]).buffer(w / 2, quad_segs=16)

def arc_slot(cx, cy, r, a0, a1, w):
    a = np.radians(np.linspace(a0, a1, 48))
    return LineString([(cx + r * math.cos(t), cy + r * math.sin(t)) for t in a]).buffer(w / 2, quad_segs=16)

def gear(n, m, backlash=GEAR_BACKLASH, pa=20.0, res=10):
    """External involute spur gear outline, centered at origin."""
    pa = math.radians(pa)
    rp = n * m / 2; rb = rp * math.cos(pa); ra = rp + m; rf = rp - 1.25 * m
    inv = lambda a: math.tan(a) - a
    half = math.pi / (2 * n) - backlash / (4 * rp)
    def th(r):
        r = max(r, rb)
        return half + inv(pa) - inv(math.acos(rb / r))
    radii = np.linspace(max(rb, rf), ra, res)
    pts = []
    for k in range(n):
        c = 2 * math.pi * k / n
        if rf < rb: pts.append(pol(rf, c - th(rb)))
        pts += [pol(r, c - th(r)) for r in radii]
        pts += [pol(r, c + th(r)) for r in radii[::-1]]
        if rf < rb: pts.append(pol(rf, c + th(rb)))
    return Polygon(pts).buffer(0)

def rex_bore(clear=0.2):
    af, d = REX_AF + clear, REX_D + clear
    hexa = Polygon([pol(af / 2 / math.cos(math.pi / 6), math.pi / 6 + k * math.pi / 3) for k in range(6)])
    return hexa.intersection(Point(0, 0).buffer(d / 2, quad_segs=24))

def motor_holes(x, y, slot_dir=None, slot_len=0.0, bolts=None, center_d=None):
    """goBILDA motor face: centre hole + M4 screws (default: the 4 axis holes at r 12). Optional slots."""
    bolts = MOTOR_BOLTS if bolts is None else bolts
    center_d = MOTOR_CENTER_HOLE if center_d is None else center_d
    g = [circ(x, y, center_d)]
    for dx, dy in bolts:
        if slot_dir is None:
            g.append(circ(x + dx, y + dy, M4))
        else:
            ux, uy = slot_dir
            p0 = (x + dx - ux * slot_len / 2, y + dy - uy * slot_len / 2)
            p1 = (x + dx + ux * slot_len / 2, y + dy + uy * slot_len / 2)
            g.append(slot(p0, p1, M4))
    if slot_dir is not None:
        ux, uy = slot_dir
        g[0] = slot((x - ux * slot_len / 2, y - uy * slot_len / 2),
                    (x + ux * slot_len / 2, y + uy * slot_len / 2), center_d)
    return unary_union(g)

def truss(outline, keepouts, ribs, edge=8.0, rib=7.0, min_area=120.0, fillet=3.0):
    """Lightening pockets: inset outline minus keep-out zones minus rib lines."""
    inner = outline.buffer(-edge, join_style=2)
    ko = unary_union(keepouts + [LineString(r).buffer(rib / 2) for r in ribs])
    p = inner.difference(ko)
    p = p.buffer(-fillet, join_style=1).buffer(fillet, join_style=1)
    geoms = list(p.geoms) if isinstance(p, MultiPolygon) else [p]
    return unary_union([g for g in geoms if g.area > min_area])

# ---------- shapely -> build123d ----------
def _ring_pts(ring):
    pts = list(ring.coords)[:-1]
    return [(float(x), float(y)) for x, y in pts]

def to_faces(poly):
    geoms = list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]
    faces = []
    for g in geoms:
        g = orient(g.simplify(0.01), 1.0)   # CCW exterior -> face normal +Z
        outer = Wire.make_polygon(_ring_pts(g.exterior), close=True)
        inner = [Wire.make_polygon(_ring_pts(i), close=True) for i in g.interiors]
        faces.append(Face(outer, inner))
    return faces

def plate(poly, t=T):
    faces = to_faces(poly)
    solids = [extrude(f, t) for f in faces]
    return solids[0] if len(solids) == 1 else Compound(solids)

def yz_plate(poly, x0, t=T):
    """Plate drawn in (Y, Z) coordinates, extruded along +X starting at x0."""
    loc = Plane(origin=(x0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location
    return loc * plate(poly, t)

def write_dxf(poly, path, marks=(), notes=()):
    """CUT layer = the laser/CNC profile. marks: (layer, (x, y), dia) circles on non-cut layers, e.g. CSK_M4 (countersink
    dia on the face) or TAP_M4 (hole is cut at tap-drill size). notes: text lines on a NOTES layer."""
    doc = ezdxf.new("R2010"); doc.units = ezdxf.units.MM
    for lay, col in (("CUT", 7), ("CSK_M4", 1), ("TAP_M4", 5), ("NOTES", 3)):
        if lay not in doc.layers:
            doc.layers.add(lay, color=col)
    msp = doc.modelspace()
    geoms = list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]
    for g in geoms:
        for ring in [g.exterior, *g.interiors]:
            msp.add_lwpolyline(_ring_pts(ring), close=True, dxfattribs={"layer": "CUT"})
    for lay, (x, y), d in marks:
        msp.add_circle((x, y), d / 2, dxfattribs={"layer": lay})
    if notes:
        x0, y0, x1, y1 = poly.bounds
        for i, line in enumerate(notes):
            msp.add_text(line, height=3.0, dxfattribs={"layer": "NOTES"}).set_placement((x0, y0 - 8 - 5 * i))
    doc.saveas(path)

def countersunk(solid, pts, inner_z, cone=False):
    """Cut 90-deg M4 countersinks into a plate solid (local coords) at pts, from the face at z = inner_z (0 or T).
    v7 laminated plywood: a cylindrical counterbore through the head-side 3 mm layer instead (button heads sit flush)."""
    from build123d import Solid as _S
    if LAMINATE and not cone:
        for (x, y) in pts:
            z0 = -0.01 if inner_z == 0.0 else inner_z - PLY_LAYER
            solid = solid - _S.make_cylinder(CBORE_D / 2, PLY_LAYER + 0.01, Plane(origin=(x, y, z0), z_dir=(0, 0, 1)))
        return solid
    for (x, y) in pts:
        if inner_z == 0.0:
            cone = _S.make_cone(CSK_D / 2, CSK_D / 2 - CSK_DEPTH - 0.01, CSK_DEPTH + 0.01, Plane(origin=(x, y, -0.01), z_dir=(0, 0, 1)))
        else:
            cone = _S.make_cone(CSK_D / 2, CSK_D / 2 - CSK_DEPTH - 0.01, CSK_DEPTH + 0.01, Plane(origin=(x, y, inner_z + 0.01), z_dir=(0, 0, -1)))
        solid = solid - cone
    return solid

def revolve_profile(r_in, r_out, h):
    """Simple tube along Z."""
    ring = Point(0, 0).buffer(r_out, quad_segs=48).difference(Point(0, 0).buffer(r_in, quad_segs=48))
    return plate(ring, h)

# =====================================================================
#                     TURRET BASE PLATES (same as v1)
# =====================================================================
def base_plate_2d():
    ma = math.radians(TURRET_MOTOR_ANGLE)
    mx, my = pol(PIN_CENTER_R, ma)
    pads = [Point(0, 0).buffer(BASE_R, quad_segs=64), Point(mx, my).buffer(MOTOR_D / 2 + 6, quad_segs=32)]
    if HOME_FLAG:   # v8: pad under the home switch (its base + 3 mm), rounded into the outline
        a = math.radians(HOME_ANG); rc = switch_rc()
        c, t = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
        hw, hl = SW_DIMS[1] / 2 + 3.0, SW_DIMS[0] / 2 + 3.0
        pads.append(Polygon([(rc * c[0] + i * hw * c[0] + j * hl * t[0], rc * c[1] + i * hw * c[1] + j * hl * t[1])
                             for i, j in ((-1, -1), (1, -1), (1, 1), (-1, 1))]).buffer(3.0))
    outer = unary_union(pads).convex_hull
    holes = [circ(0, 0, BASE_HOLE_D)]
    for k in range(N_ROLLERS):
        a = math.radians(30 + 360 * k / N_ROLLERS)
        x, y = pol(ROLLER_R, a)
        if k in (0, N_ROLLERS // 2):   # two slotted rollers to set preload
            holes.append(slot(pol(ROLLER_R - 1.5, a), pol(ROLLER_R + 1.5, a), M8))
        else:
            holes.append(circ(x, y, M8))
    # turret motor, slotted radially for gear mesh adjustment
    holes.append(motor_holes(mx, my, slot_dir=(math.cos(ma), math.sin(ma)), slot_len=1.5, bolts=TURRET_BOLTS, center_d=TURRET_CENTER_D))
    for k in range(8):                     # robot mounting
        a = math.radians(22.5 + 45 * k)
        holes.append(circ(*pol(BASE_MOUNT_R, a), M4))
    if CABLES or HOME_FLAG:
        holes += [circ(*p, 3.4) for p in switch_holes()]
    if not CABLES:
        holes += tie_holes(TIE_BASE)
    return outer.difference(unary_union(holes))

def ring_2d():
    g = gear(RING_TEETH, RING_MODULE)
    g = g.difference(circ(0, 0, RING_ID))
    holes = [circ(*pol(DECK_SPACER_R, math.radians(a)), INSERT_HOLE_D if LAMINATE else (TAP_M4 if RING_TAPPED else M4))
             for a in DECK_SPACER_ANGS]   # v7: the printed ring's heat-set holes (same as the STL)
    return g.difference(unary_union(holes))

def turret_access_pts():
    """Deck/drum access holes over the 3 turret-motor screws (turret frame, heading 0)."""
    mx, my = pol(PIN_CENTER_R, math.radians(TURRET_MOTOR_ANGLE))
    r = math.radians(TURRET_ACCESS_ANG - TURRET_MOTOR_ANGLE)
    return [((mx + dx) * math.cos(r) - (my + dy) * math.sin(r), (mx + dx) * math.sin(r) + (my + dy) * math.cos(r)) for dx, dy in TURRET_BOLTS]

def deck_2d():
    d = Point(0, 0).buffer((GB_RING[1][1] + 1.0) if GB_TURRET else DECK_OD / 2, quad_segs=64)
    if not CABLES:
        # v8: with no cable drum the deck is shaped to what it carries: the ring circle, the L-bracket feet (+4 mm), the
        # turret-motor access holes (+6 mm of wood) and the home-magnet ear. Its convex hull stays inside the start footprint.
        pads = [d]
        for sx in (-1, 1):
            x0, x1 = sorted((sx * SIDE_X_OUT, sx * (SIDE_X_OUT + BRKT_FOOT_L)))
            for by in BRACKET_Y:
                pads.append(box(x0, by - BRKT_W / 2, x1, by + BRKT_W / 2).buffer(4.0))
        pads += [] if GB_TURRET else [Point(x, y).buffer(TURRET_ACCESS_D / 2 + 6.0) for x, y in turret_access_pts()]
        if GB_TURRET:     # v8.1GB: wood around the back zip-tie pair (the ring circle no longer reaches it)
            pads += [Point(*pol(rc, math.radians(a))).buffer(12.0) for a, rc in TIE_DECK]
        for sx in (-1, 1):       # the side plates' tab slots (+5 mm of wood around them)
            xc = sx * (SIDE_X_IN + T / 2)
            pads += [box(xc - T / 2, ty - 10.2, xc + T / 2, ty + 10.2).buffer(5.0) for ty in TAB_Y]
        if HOME_FLAG:   # an ear that carries the home-magnet holder outside the switch
            a = math.radians(HOME_ANG); c, t = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
            r0, hw = (GB_RING[1][1] - 6.0) if GB_TURRET else DECK_OD / 2 - 6.0, FLAG_W / 2 + 3.0
            pads.append(Polygon([(r * c[0] + j * hw * t[0], r * c[1] + j * hw * t[1]) for r, j in ((r0, -1), (EAR_R, -1), (EAR_R, 1), (r0, 1))]))
        d = unary_union(pads).convex_hull
    d = d.difference(circ(0, 0, DECK_ID_GB if GB_TURRET else DECK_ID))
    holes = [circ(x, y, M4) for x, y in GB_DECK_HOLES] if GB_TURRET else [circ(*pol(DECK_SPACER_R, math.radians(a)), M4) for a in DECK_SPACER_ANGS]
    if HOME_FLAG:
        holes += [circ(x, y, M4) for x, y in flag_screw_pts()]
    if not CABLES:
        holes += tie_holes(TIE_DECK)
    holes += [] if GB_TURRET else [circ(x, y, TURRET_ACCESS_D) for x, y in turret_access_pts()]
    for sx in (-1, 1):
        xc = sx * (SIDE_X_IN + T / 2)
        for ty in TAB_Y:
            holes.append(box(xc - (T + 0.3) / 2, ty - 10.2, xc + (T + 0.3) / 2, ty + 10.2))
        for by in BRACKET_Y:
            for dy in BRKT_FOOT_DY:
                holes.append(circ(sx * (SIDE_X_OUT + BRKT_FOOT_HOLE), by + dy, TAP_M4 if DECK_TAPPED else M4))
    if CABLES:
        holes += [circ(*pol(DRUM_BOLT_R, math.radians(a)), M4) for a in DRUM_BOLT_ANGS]
    return d.difference(unary_union(holes))

# =====================================================================
#                 CABLE MANAGEMENT (v5): tray, drum, home switch, U-loop
# =====================================================================
DRUM_BOLT_R = (DRUM_R[0] + DRUM_R[1]) / 2
DRUM_BOLT_ANGS = [22.5 + 45 * k for k in range(8)]      # M4 through the deck into heat-set inserts in the drum
C_RIN = DRUM_R[1] + RIBBON_T / 2 + 0.5                   # ribbon centre-line on the drum side
C_ROUT = TRAY_R[1] - TRAY_WALL - RIBBON_T / 2 - 0.5      # ...and on the outer-wall side
C_RB = (C_ROUT - C_RIN) / 2                              # U-turn bend radius
_S = C_RIN + C_ROUT
C_OUT_ANG = CABLE_IN_ANG + math.degrees(math.radians(CABLE_TRAVEL) * (C_ROUT - C_RIN) / _S)   # fixed exit
_LP = math.radians(CABLE_MARGIN) * _S + C_ROUT * math.radians(CABLE_TRAVEL - (C_OUT_ANG - CABLE_IN_ANG))
CABLE_LEN = _LP + math.pi * C_RB                         # ribbon length inside the channel, drum tie -> tray tie

def cable_u(phi):
    """U-turn angle (deg) for turret heading phi (deg from home); legs = (drum leg, wall leg) in deg."""
    a_i, a_o = math.radians(CABLE_IN_ANG + phi), math.radians(C_OUT_ANG)
    th = (_LP + C_RIN * a_i + C_ROUT * a_o) / _S
    return math.degrees(th), math.degrees(th - a_i), math.degrees(th - a_o)

def sector(a0, a1, r=400.0):
    pts = [(0, 0)] + [pol(r, math.radians(a0 + (a1 - a0) * i / 48)) for i in range(49)]
    return Polygon(pts)

def annulus(r0, r1):
    return Point(0, 0).buffer(r1, quad_segs=96).difference(Point(0, 0).buffer(r0, quad_segs=96))

def switch_rc():
    return SW_RC if HOME_FLAG else SW_OUT_R - SW_DIMS[1] / 2

def switch_base_z():
    return BASE_TOP + (TRAY_T if CABLES else 0.0)

def switch_holes():
    rc = switch_rc()
    a = math.radians(HOME_ANG); c, t = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
    return [(rc * c[0] + k * SW_HOLES / 2 * t[0], rc * c[1] + k * SW_HOLES / 2 * t[1]) for k in (-1, 1)]

def tray_floor_2d():
    g = annulus(TRAY_R[0], TRAY_R[1])
    ma = TURRET_MOTOR_ANGLE
    g = g.difference(sector(ma - TRAY_NOTCH, ma + TRAY_NOTCH).intersection(Point(0, 0).buffer(DRUM_R[0] + 8)))   # v6: +8 clears the outer motor screw (r 132)
    holes = [circ(*pol(BASE_MOUNT_R, math.radians(22.5 + 45 * k)), M4) for k in range(8)]
    holes += [circ(*p, 3.4) for p in switch_holes()]
    a = math.radians(C_OUT_ANG)          # exit slot: the ribbon drops through here to the robot
    t = (-math.sin(a), math.cos(a))
    p0 = pol(C_ROUT, a)
    holes.append(LineString([(p0[0] - 6 * t[0], p0[1] - 6 * t[1]), (p0[0] + 6 * t[0], p0[1] + 6 * t[1])]).buffer(5.0))
    return g.difference(unary_union(holes))

def _radial_holes(r_mid, ang, zc, n=2, pitch=12.0, d=4.0, length=16.0):
    """Zip-tie holes through a curved wall: n radial holes `pitch` apart along the wall at height zc."""
    out = []
    for k in range(n):
        a = math.radians(ang) + (k - (n - 1) / 2) * pitch / r_mid
        c = (math.cos(a), math.sin(a))
        p0 = Vector((r_mid - length / 2) * c[0], (r_mid - length / 2) * c[1], zc)
        out.append(Solid.make_cylinder(d / 2, length, Plane(origin=p0, z_dir=(c[0], c[1], 0))))
    return out

def cable_tray(k):
    """Fixed tray segment k (TRAY_SPLITS): floor + outer wall. Sits on the base plate; the 8 robot-mount bolts hold it."""
    sec = sector(TRAY_SPLITS[k], TRAY_SPLITS[k] + 360 / TRAY_N) if TRAY_N > 1 else Point(0, 0).buffer(1000)
    s = plate(tray_floor_2d().intersection(sec), TRAY_T) + plate(annulus(TRAY_R[1] - TRAY_WALL, TRAY_R[1]).intersection(sec), TRAY_H)
    for h in _radial_holes(TRAY_R[1] - TRAY_WALL / 2, C_OUT_ANG - 9.0, RIBBON_Z0 + RIBBON_H / 2):
        s = s - h
    return s

def cable_drum(k):
    """Drum segment k: turns with the turret, hangs under the deck. Heat-set M4 inserts in the top face."""
    sec = sector(DRUM_SPLITS[k], DRUM_SPLITS[k] + 360 / DRUM_N) if DRUM_N > 1 else Point(0, 0).buffer(1000)
    g = annulus(*DRUM_R).intersection(sec)
    s = Pos(0, 0, DRUM_Z0) * plate(g, DECK_Z0 - DRUM_Z0)
    a = math.radians(HOME_ANG)            # magnet pocket in the inner face, facing the switch at home
    zc = BASE_TOP + TRAY_T + 9.0          # switch side-sensor height (datasheet: 9 mm above its base)
    pocket = Plane(origin=(DRUM_R[0] * math.cos(a), DRUM_R[0] * math.sin(a), zc),
                   x_dir=(-math.sin(a), math.cos(a), 0), z_dir=(math.cos(a), math.sin(a), 0)).location * \
        Box(MAG_DIMS[0] + 0.6, MAG_DIMS[2] + 0.6, 2 * (MAG_DIMS[1] + 0.3))
    s = s - pocket
    for h in _radial_holes(DRUM_R[1] - 5, CABLE_IN_ANG, RIBBON_Z0 + RIBBON_H / 2 + 1):
        s = s - h
    for a in DRUM_BOLT_ANGS:                # M4 heat-set pockets in the top face (5.6 x 7 deep)
        x, y = pol(DRUM_BOLT_R, math.radians(a))
        s = s - Pos(x, y, DECK_Z0 - 7.0) * Solid.make_cylinder(2.8, 7.01)
    for x, y in turret_access_pts():        # v6: hex-key access down to the turret-motor screws
        s = s - Pos(x, y, DRUM_Z0 - 0.01) * Solid.make_cylinder(TURRET_ACCESS_D / 2, DECK_Z0 - DRUM_Z0 + 0.02)
    return s

def home_switch():
    """REF REV-31-1462: base 41.5 x 15 x 3, body ~25 long x 15 x 17 (datasheet), long side toward the drum."""
    a = HOME_ANG
    rc = switch_rc()
    body = Pos(0, 0, 1.5) * Box(SW_DIMS[1], SW_DIMS[0], 3.0) + Pos(0, 0, SW_DIMS[2] / 2) * Box(SW_DIMS[1], 25.0, SW_DIMS[2])
    return Pos(rc * math.cos(math.radians(a)), rc * math.sin(math.radians(a)), switch_base_z()) * Rot(0, 0, a) * body

def home_magnet():
    a = math.radians(HOME_ANG)
    zc = BASE_TOP + (MAG_ZC if HOME_FLAG else TRAY_T + 9.0)
    rm = (MAG_IN_R + MAG_DIMS[1] / 2) if HOME_FLAG else (DRUM_R[0] + MAG_DIMS[1] / 2 + 0.3)
    return Plane(origin=(rm * math.cos(a), rm * math.sin(a), zc),
                 x_dir=(-math.sin(a), math.cos(a), 0), z_dir=(math.cos(a), math.sin(a), 0)).location * \
        Box(MAG_DIMS[0], MAG_DIMS[2], MAG_DIMS[1] - 0.2)

def home_flag():
    """v8: printed holder for the home magnet, hanging under the deck's ear outside the switch. The magnet sits in a pocket
    in its inner face (2x M3 x 20 through its tabs and the switch pad); 2x M4 x 12 come down through the deck into heat-sets in its top."""
    r0, r1 = FLAG_R
    w = FLAG_W
    top = DECK_Z0
    blk = Pos((r0 + r1) / 2, 0, (FLAG_Z0 + top) / 2) * Box(r1 - r0, w, top - FLAG_Z0)
    pocket = Pos(r0 + (MAG_DIMS[1] + 0.4) / 2 - 0.01, 0, MAG_ZC) * Box(MAG_DIMS[1] + 0.4, MAG_DIMS[0] + 0.4, MAG_DIMS[2] + 0.4)
    blk = blk - pocket
    for dy in (-8.0, 8.0):        # magnet tab screws (M3), through to the outer face
        blk = blk - Plane(origin=(r0, dy, MAG_ZC), z_dir=(1, 0, 0)).location * Solid.make_cylinder(1.7, r1 - r0 + 1)
    rc = (r0 + r1) / 2
    for dy in FLAG_SCREW_DY:      # heat-set pockets from the top, + a tip relief so an M4 x 12 (5.8 mm in) can't bottom out
        blk = blk - Pos(rc, dy, top - INSERT_DEPTH) * Solid.make_cylinder(INSERT_HOLE_D / 2, INSERT_DEPTH + 0.01)
        blk = blk - Pos(rc, dy, top - INSERT_DEPTH - FLAG_TIP_RELIEF) * Solid.make_cylinder(2.25, FLAG_TIP_RELIEF + 0.01)
    return Rot(0, 0, HOME_ANG) * blk

FLAG_SCREW_DY = (-8.0, 8.0)
FLAG_TIP_RELIEF = 2.0       # v8 (Astra): M4 x 12 through the 6.2 deck reaches 5.8 into the holder; insert pocket 5.2 + this
MAG_SCREW = "M3 x 20"       # v8 (Astra): holder wall behind the pocket 10.2 + magnet tab 2 + nyloc 4 = 16.2, + 2 threads past

# v8: zip-tie points for whichever cable method is chosen later (DESIGN_LOG v8): pairs of 4 mm holes 8 mm apart,
# radial, so a tie wraps a cable bundle running around the turret. Deck: at the back (between the deck spacers at
# 67.5 / 112.5 deg). Base plate: at the back too, outside the roller tires (r 92).
TIE_DECK = [(90.0, 97.0)]          # (deg, centre radius)
TIE_BASE = [(90.0, 104.0)]
# ---- v8.1GB: goBILDA Gear-Driven Turret (simplified from the seller's STEP) ----
GB_ARM_T = 3.0                       # printed switch arm: floor under the switch, z -3..0 (below the housing's bottom)
GB_LUG = (72.0, 76.0, 10.0)          # its lug on the housing flat at HOME_ANG: r 72..76, up to z 10 (the ring is above 13)

def gb_rot(shape, ang=None):
    return Rot(0, 0, GB_ANGLE if ang is None else ang) * shape

def gb_ring():
    out = None
    for r0, r1, z0, z1 in GB_RING:
        s = Pos(0, 0, z0) * (Solid.make_cylinder(r1, z1 - z0) - Solid.make_cylinder(r0, z1 - z0))
        out = s if out is None else out + s
    return out

def gb_housing():
    from build123d import RegularPolygon, extrude
    oct_ = extrude(Rot(0, 0, 22.5) * RegularPolygon(GB_AF / 2, 8, major_radius=False), GB_HOUSING_H)
    return oct_ - Solid.make_cylinder(GB_RING[0][1] + 1.5, GB_HOUSING_H)

def gb_mount():
    (x0, x1), (y0, y1), (z0, z1) = GB_MOUNT
    m = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)
    return gb_rot(m - Pos(GB_DRIVE_R, 0, 0) * Solid.make_cylinder(7.0, z1 - z0))

def gb_switch_arm():
    """Printed PETG arm: a lug screwed to the housing flat at HOME_ANG (2 M4 into its tapped holes) and a 3 mm floor
    under the home switch, so the switch sits exactly where v8.1's base plate held it (base at z 0)."""
    rc, hl = switch_rc(), SW_DIMS[0] / 2 + 3.0
    r_lo, r_hi, ztop = GB_LUG
    floor = Pos((r_lo + rc + SW_DIMS[1] / 2 + 3.0) / 2, 0, -GB_ARM_T / 2) * Box(rc + SW_DIMS[1] / 2 + 3.0 - r_lo, 2 * hl, GB_ARM_T)
    lug = Pos((r_lo + r_hi) / 2, 0, (ztop - GB_ARM_T) / 2) * Box(r_hi - r_lo, 2 * hl, ztop + GB_ARM_T)
    s = floor + lug
    for t in GB_SIDE_HOLES:
        s = s - Plane(origin=(r_lo - 0.1, t, 4.0), z_dir=(1, 0, 0)).location * Solid.make_cylinder(M4 / 2, r_hi - r_lo + 0.2)
    for k in (-1, 1):
        s = s - Pos(rc, k * SW_HOLES / 2, -GB_ARM_T - 0.1) * Solid.make_cylinder(1.7, GB_ARM_T + 0.2)
    return Rot(0, 0, HOME_ANG) * s

def gb_arm_screws():
    a = math.radians(HOME_ANG); c, t = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
    return [(GB_LUG[1] * c[0] + k * t[0], GB_LUG[1] * c[1] + k * t[1], 0.0) for k in GB_SIDE_HOLES]

def gb_shim():
    r0, r1 = GB_SHIM_R
    s = Pos(0, 0, GB_RING[1][3]) * (Solid.make_cylinder(r1, DECK_Z0 - GB_RING[1][3]) - Solid.make_cylinder(r0, DECK_Z0 - GB_RING[1][3]))
    for x, y in GB_DECK_HOLES:
        s = s - Pos(x, y, GB_RING[1][3] - 0.1) * Solid.make_cylinder(M4 / 2, 2.0)
    return s

def gb_parts():
    """name -> (shape, group, colour, kind). Bought parts are 'ref'."""
    mx, my = pol(GB_DRIVE_R, math.radians(GB_ANGLE))
    return {
        "gb_housing_ref": (gb_housing(), "fixed", "#b9bec5", "ref"),
        "gb_mount_ref": (gb_mount(), "fixed", "#c9ced4", "ref"),
        "gb_gear64_ref": (Pos(mx, my, GB_GEAR_Z[0]) * Solid.make_cylinder(GB_GEAR_OD / 2, GB_GEAR_Z[1] - GB_GEAR_Z[0]), "fixed", "#c9ced4", "ref"),
        "gb_hub_ref": (Pos(mx, my, GB_MOUNT[2][1]) * Solid.make_cylinder(12.0, GB_GEAR_Z[0] - GB_MOUNT[2][1]), "fixed", "#9aa3ad", "ref"),
        "turret_motor_ref": (Pos(mx, my, -GB_MOTOR_L) * ref_cyl(MOTOR_D, GB_MOTOR_L), "fixed", "#9aa0a6", "ref"),
        "switch_arm": (gb_switch_arm(), "fixed", "#e8e8e8", "print"),
        "gb_ring_ref": (gb_ring(), "turret", "#b9bec5", "ref"),
        "deck_shim": (gb_shim(), "turret", "#e8e8e8", "print"),
    }

def tie_holes(spec):
    out = []
    for a, rc in spec:
        for dr in (-4.0, 4.0):
            out.append(circ(*pol(rc + dr, math.radians(a)), 4.0))
    return out

def flag_screw_pts():
    rc = (FLAG_R[0] + FLAG_R[1]) / 2
    a = math.radians(HOME_ANG)
    return [(rc * math.cos(a) - dy * math.sin(a), rc * math.sin(a) + dy * math.cos(a)) for dy in FLAG_SCREW_DY]

def cable_ribbon(phi=0.0):
    """REF the sleeved cable ribbon at heading phi: drum leg, U-turn, wall leg, plus the drops at both anchors."""
    th, _, _ = cable_u(phi)
    a_i = CABLE_IN_ANG + phi
    pts = [pol(C_RIN, math.radians(a_i + (th - a_i) * i / 60)) for i in range(61)]
    t = math.radians(th); er, et = (math.cos(t), math.sin(t)), (-math.sin(t), math.cos(t))
    rc = (C_RIN + C_ROUT) / 2
    pts += [(rc * er[0] + C_RB * (-math.cos(u) * er[0] + math.sin(u) * et[0]),
             rc * er[1] + C_RB * (-math.cos(u) * er[1] + math.sin(u) * et[1])) for u in [math.pi * i / 24 for i in range(1, 24)]]
    pts += [pol(C_ROUT, math.radians(th + (C_OUT_ANG - th) * i / 60)) for i in range(61)]
    band = LineString(pts).buffer(RIBBON_T / 2, cap_style=2)
    s = Pos(0, 0, RIBBON_Z0) * plate(band, RIBBON_H)
    pi_ = pol(C_RIN, math.radians(a_i)); po = pol(C_ROUT, math.radians(C_OUT_ANG))
    s = s + Pos(pi_[0], pi_[1], RIBBON_Z0 + RIBBON_H) * Solid.make_cylinder(2.5, 16.0)       # up to the turret harness
    s = s + Pos(po[0], po[1], -40.0) * Solid.make_cylinder(2.5, 40.0 + RIBBON_Z0)              # down to the Control Hub
    return s

# =====================================================================
#                      V2 SHOOTER GEOMETRY (derived)
# =====================================================================
def unit(v):
    n = math.hypot(*v); return (v[0] / n, v[1] / n)

def add(a, b, k=1.0):
    return (a[0] + k * b[0], a[1] + k * b[1])

def circle_x(c0, r0, c1, r1):
    """Intersections of two circles."""
    dx, dy = c1[0] - c0[0], c1[1] - c0[1]; d = math.hypot(dx, dy)
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d); h = math.sqrt(max(r0 * r0 - a * a, 0))
    xm, ym = c0[0] + a * dx / d, c0[1] + a * dy / d
    return [(xm + h * dy / d, ym - h * dx / d), (xm - h * dy / d, ym + h * dx / d)]

R1, R2 = FW1_D / 2, FW2_D / 2
def pinch_dist(d, c=COMPRESSION):
    return R1 + R2 + d - c

ARM_L = SYNC_MODULE * (SYNC_TEETH_J + SYNC_TEETH_S) / 2        # gear center distance = arm length
def htd_pd(t):
    return t * 5.0 / math.pi
JACK_PD = htd_pd(JACK_PULLEY_T)
_psi0 = math.radians(PINCH_PSI)
_s_nom = (pinch_dist(POLLEN_D) * math.cos(_psi0), pinch_dist(POLLEN_D) * math.sin(_psi0))
_j0 = add(_s_nom, (-math.sin(_psi0), math.cos(_psi0)), ARM_L)  # arm points along exit direction
JACK_BELT = min(HTD_BELTS, key=lambda L: abs((L - math.pi * JACK_PD) / 2 - math.hypot(*_j0)))  # nearest stock belt
JACK_C = (JACK_BELT - math.pi * JACK_PD) / 2 + JACK_C_TRIM
J = add((0, 0), unit(_j0), JACK_C)                              # jackshaft, carrier frame (rel. to M)

def s_pos(d, c=COMPRESSION):
    """Second-flywheel center (carrier frame) for a ball of dia d."""
    return min(circle_x(J, ARM_L, (0, 0), pinch_dist(d, c)),
               key=lambda p: math.hypot(p[0] - _s_nom[0], p[1] - _s_nom[1]))

S_P, S_N = s_pos(POLLEN_D), s_pos(NECTAR_D, MIN_SQUEEZE)
def arm_angle(s):
    return math.atan2(s[1] - J[1], s[0] - J[0])
TH_P, TH_N = arm_angle(S_P), arm_angle(S_N)
U_ARM = unit((S_P[0] - J[0], S_P[1] - J[1]))
N_ARM = (-U_ARM[1], U_ARM[0])
if N_ARM[0] * (0 - S_P[0]) + N_ARM[1] * (0 - S_P[1]) < 0:        # point toward the main flywheel
    N_ARM = (-N_ARM[0], -N_ARM[1])
ARM_HUB = 15.0
SPRING_A = add(S_P, U_ARM, ARM_TAIL)                              # constant-force spring end, on the arm tail
SPRING_B = [add(SPRING_A, N_ARM, CF_DRUM_DIST)]                    # coil drum on the carrier
STOP_E = add(add(J, U_ARM, STOP_ALONG), N_ARM, ARM_HUB + CAM_R + 0.2)   # eccentric hard stop (v6: OD 13)
GUIDE_1 = (86.4 * math.cos(math.radians(-13.14)), 86.4 * math.sin(math.radians(-13.14)))   # below the spring line. v6.1: moved
# 4 mm straight away from the spring drum (was r 90 at -12 deg) so a coil of up to 3 stacked springs clears its 8 mm guide tube
GUIDE_2 = pol(0.62 * math.hypot(*J), math.atan2(J[1], J[0]) - math.radians(18))   # v4: between main axis and jackshaft (keeps height down)
# v6.1: open-side BUMPER stop, mirror of the stop cam: a printed post (PETG core + TPU ring) on the carrier that the swing
# arm's outer edge meets OPEN_STOP_EXTRA past the design Nectar position, before the upper-shaft spacers reach the ends of
# the carrier's arc slot (~7.7 deg past). Flat head from the ball side into a heat-set in the post, like the stop cam.
OPEN_STOP_EXTRA, BUMPER_R, BUMPER_ALONG = 3.0, 7.0, 11.0   # R7: 10 mm PETG core (2.2 mm around the insert) + 2 mm TPU ring
OPEN_STOP_DEG = math.degrees(abs(TH_N - TH_P)) + OPEN_STOP_EXTRA
def _bumper_e():
    a = math.radians(OPEN_STOP_DEG) * (1 if TH_N > TH_P else -1)
    v = (U_ARM[0] * BUMPER_ALONG - N_ARM[0] * (ARM_HUB + BUMPER_R), U_ARM[1] * BUMPER_ALONG - N_ARM[1] * (ARM_HUB + BUMPER_R))
    return (J[0] + v[0] * math.cos(a) - v[1] * math.sin(a), J[1] + v[0] * math.sin(a) + v[1] * math.cos(a))
BUMPER_E = _bumper_e()
CARRIER_CSK = [GUIDE_1, GUIDE_2, SPRING_B[0], STOP_E, BUMPER_E]   # flat heads from the ball side (the carriers wall the ball corridor)
# v6: the stop cam is an in-season ADJUSTMENT, so its screw is reached from the ball side: M4 flat head, countersunk,
# into a heat-set insert in the cam (v5's outboard button head sat behind the jack belt on the motor side).
CARRIER_TAP = []
HOOD_GEAR_CSK = HOOD_GEAR_BOLTS          # servo-side carrier only (the motor-side carrier has the holes, no gear)

def psi_exit(s, alpha=0.0):
    return math.degrees(math.atan2(s[1], s[0])) + alpha

# ---- x layers (each side, measured from center) ----
X_CAR = CORRIDOR / 2                  # hood carrier plate inner face
X_ARM = X_CAR + T + 1.0               # swing arm inner face
X_DRV = X_ARM + T + 1.0               # belt / gear layer inner face
SIDE_X_IN = X_DRV + DRIVE_W + 1.0     # side plate inner face
SIDE_X_OUT = SIDE_X_IN + T

SECTOR_RP = SECTOR_MODULE * SECTOR_TEETH_FULL / 2
SERVO_PIN_RP = SECTOR_MODULE * SERVO_PINION_TEETH / 2
SERVO_AXIS = (FW_Y + (SECTOR_RP + SERVO_PIN_RP + 0.1) * math.cos(math.radians(SECTOR_PSI)),
              FW_Z + (SECTOR_RP + SERVO_PIN_RP + 0.1) * math.sin(math.radians(SECTOR_PSI)))
def _belt_c(L, d1, d2):
    lo, hi = abs(d2 - d1) / 2 + 1, L / 2
    for _ in range(80):                           # open-belt length: 2C + pi(D1+D2)/2 + (D2-D1)^2/(4C)
        c = (lo + hi) / 2
        if 2 * c + math.pi * (d1 + d2) / 2 + (d2 - d1) ** 2 / (4 * c) > L: hi = c
        else: lo = c
    return c
MOTOR_C = _belt_c(MOTOR_BELT, htd_pd(MOTOR_PULLEY_T), htd_pd(MAIN_MOTOR_PULLEY_T)) + MOTOR_C_TRIM
FW_MOTOR = (FW_Y + MOTOR_C * math.cos(math.radians(MOTOR_PSI)), FW_Z + MOTOR_C * math.sin(math.radians(MOTOR_PSI)))
FW_MOTOR_BOLTS = [(FW_MOTOR[0] + dy, FW_MOTOR[1] + dz) for dy, dz in MOTOR_BOLTS]

def motor_rel(p):
    """(Y, Z) offset from the flywheel motor axis -> world (Y, Z)."""
    return (FW_MOTOR[0] + p[0], FW_MOTOR[1] + p[1])

def motor_slide_slot(p, w):
    """Side-plate slot that lets a feature at motor offset p slide +/- MOTOR_TRAVEL."""
    dy, dz = MOTOR_SLIDE[0] * MOTOR_TRAVEL, MOTOR_SLIDE[1] * MOTOR_TRAVEL
    return slot(motor_rel((p[0] - dy, p[1] - dz)), motor_rel((p[0] + dy, p[1] + dz)), w)

def motor_adapter_2d(world_coords=False):
    """v6: 1/8in aluminum plate the flywheel motor bolts to (inside the motor-side plate). Offsets from the motor axis,
    or world (Y, Z). Two lobes, so the front crossbar fits in the notch between them."""
    o = (lambda p: motor_rel(p)) if world_coords else (lambda p: p)
    core = Point(*o((0.0, 0.0))).buffer(19.5, quad_segs=24)
    upper = unary_union([core] + [Point(*o(p)).buffer(6.5, quad_segs=16) for p in LUG_BOLTS + CLAMP_BOLTS[:1]]).convex_hull
    lower = unary_union([core, Point(*o(CLAMP_BOLTS[1])).buffer(6.5, quad_segs=16)]).convex_hull
    holes = ([circ(*o((0.0, 0.0)), MOTOR_CENTER_HOLE)] + [circ(*o(p), M4) for p in MOTOR_BOLTS]
             + [circ(*o(p), M4 if LAMINATE else TAP_M4) for p in LUG_BOLTS + CLAMP_BOLTS])   # v7: printed, square nuts
    # notch around the front crossbar tube (r 7 + 1.5 clearance) over the whole slide range
    fy, fz = min(CROSSBARS, key=lambda c: c[0])
    fy, fz = (fy - FW_MOTOR[0], fz - FW_MOTOR[1]) if not world_coords else (fy, fz)
    dy, dz = MOTOR_SLIDE[0] * MOTOR_TRAVEL, MOTOR_SLIDE[1] * MOTOR_TRAVEL
    notch = slot((fy - dy, fz - dz), (fy + dy, fz + dz), 2 * (7.0 + 1.5))
    out = unary_union([upper, lower]).difference(notch)
    return out.buffer(-1.5, join_style=1).buffer(1.5, join_style=1).difference(unary_union(holes))   # round the notch corners

def motor_lug():
    """v6: printed tension lug, outside the motor-side plate. Two clamp bolts hold it to the adapter through the side
    plate; the jack screw threads into its M4 heat-set (front face)."""
    jz = motor_rel((0.0, JACK_DZ))[1]
    p = box(*motor_rel((LUG_Y[0], JACK_DZ - LUG_W)), *motor_rel((LUG_Y[1], JACK_DZ + LUG_W)))
    p = p.difference(unary_union([circ(*motor_rel(b), M4) for b in LUG_BOLTS]))
    s = yz_plate(p, -(SIDE_X_OUT + LUG_H), LUG_H)
    y0 = motor_rel((LUG_Y[0], 0.0))[0]
    s -= Solid.make_cylinder(2.8, 6.0, Plane(origin=(-(SIDE_X_OUT + JACK_H), y0 - 0.01, jz), z_dir=(0, 1, 0)))   # heat-set
    s -= Solid.make_cylinder(2.25, LUG_Y[1] - LUG_Y[0] + 1, Plane(origin=(-(SIDE_X_OUT + JACK_H), y0 - 0.5, jz), z_dir=(0, 1, 0)))
    return s

def motor_yoke():
    """v6: printed U-shaped jack yoke, bolted to the motor-side plate in front of the lug. The jack screw's head bears on
    its front face."""
    ya, yb = motor_rel((YOKE_BRIDGE[0], 0.0))[0], motor_rel((YOKE_BRIDGE[1], 0.0))[0]
    yarm = motor_rel((YOKE_ARM_Y, 0.0))[0]
    jz = motor_rel((0.0, JACK_DZ))[1]
    p = unary_union([box(ya, jz - YOKE_ARM[1], yb, jz + YOKE_ARM[1]),
                     box(yb - 0.01, jz + YOKE_ARM[0], yarm, jz + YOKE_ARM[1]),
                     box(yb - 0.01, jz - YOKE_ARM[1], yarm, jz - YOKE_ARM[0])])
    p = p.difference(unary_union([circ(*motor_rel(q), M4) for q in YOKE_SCREWS]))
    s = yz_plate(p, -(SIDE_X_OUT + YOKE_H), YOKE_H)
    s -= Solid.make_cylinder(2.25, yb - ya + 1, Plane(origin=(-(SIDE_X_OUT + JACK_H), ya - 0.5, jz), z_dir=(0, 1, 0)))
    return s
MOTOR2_C = _belt_c(MOTOR2_BELT, htd_pd(MOTOR_PULLEY_T), htd_pd(MAIN_MOTOR_PULLEY_T))
FW_MOTOR2 = (FW_Y + MOTOR2_C * math.cos(math.radians(MOTOR2_PSI)), FW_Z + MOTOR2_C * math.sin(math.radians(MOTOR2_PSI)))

def ll_dirs(tilt):
    t = math.radians(tilt)
    return (-math.cos(t), math.sin(t)), (math.sin(t), math.cos(t))   # view dir, plate "up"

def world(p, alpha=0.0):
    """Carrier-frame point -> world (Y, Z) with hood angle alpha."""
    a = math.radians(alpha); c, s = math.cos(a), math.sin(a)
    return (FW_Y + c * p[0] - s * p[1], FW_Z + s * p[0] + c * p[1])

# =====================================================================
#                          V2 PLATES (2D)
# =====================================================================
# ---- idler rollers: horizontal slots in the side plates ----
def _idler_slots():
    """(y_pollen, y_nectar, z) for each idler roller axle, world frame."""
    out = []
    rp = R1 + POLLEN_D + IDLER_D / 2 - IDLER_SQUEEZE
    rn = R1 + NECTAR_D + IDLER_D / 2 - IDLER_SQUEEZE
    for psi in IDLER_PSI:
        a = math.radians(psi)
        z = FW_Z + rp * math.sin(a)
        y_p = FW_Y + rp * math.cos(a)
        y_n = FW_Y + math.sqrt(rn * rn - (z - FW_Z) ** 2)
        out.append((y_p, y_n, z))
    return out
IDLER_SLOTS = _idler_slots() if IDLERS else []
IDLER_TRAVEL = max((yn - yp for yp, yn, _ in IDLER_SLOTS), default=0.0) + 3.0   # slot length past Nectar
# springs hang ~perpendicular to each slot (lower roller: anchor below, upper roller: anchor above)
IDLER_ANCHORS = [(yp + d * math.cos(math.radians(a)), z + d * math.sin(math.radians(a)))
                 for (yp, _, z), (a, d) in zip(IDLER_SLOTS, IDLER_SPRING_GEOM)]

def idler_keepout_carrier(clear=4.0):
    """Region the idler axles sweep, expressed in the carrier frame for every hood angle."""
    polys = []
    for y_p, _, z in IDLER_SLOTS:
        band = LineString([(y_p - 1, z), (y_p + IDLER_TRAVEL + 1, z)]).buffer(2.5 + clear)
        rel = affinity.translate(band, -FW_Y, -FW_Z)
        for a in np.arange(0.0, HOOD_ALPHA_MAX + 0.01, 1.0):
            polys.append(affinity.rotate(rel, -a, origin=(0, 0)))
    return unary_union(polys)

def carrier_2d():
    """Hood carrier, drawn about the main flywheel axis (0,0) at hood angle 0."""
    s_slot = LineString([add(J, (math.cos(t), math.sin(t)), ARM_L)
                         for t in np.linspace(TH_P - math.radians(6), TH_N + math.radians(6), 24)])
    feats = [Point(0, 0).buffer(24), Point(*J).buffer(22), s_slot.buffer(14),
             Point(*GUIDE_1).buffer(10.5), Point(*GUIDE_2).buffer(10.5), Point(*STOP_E).buffer(10.5), Point(*BUMPER_E).buffer(10.5),
             *[Point(*b).buffer(10.5) for b in SPRING_B]]           # v6: >= 8.6 mm around every countersink (cutting-service rule)
    body = unary_union(feats).convex_hull
    gear_pad = Point(0, 0).buffer(44)                         # v6: meat around the bought hood gear's 4 bolts (10 mm edge)
    body = unary_union([body, gear_pad]).buffer(3).buffer(-3)
    body = body.difference(idler_keepout_carrier()).buffer(2).buffer(-2)
    if isinstance(body, MultiPolygon):
        body = max(body.geoms, key=lambda g: g.area)
    holes = [circ(0, 0, BRG_1611), circ(*J, BRG_1611), s_slot.buffer(6.25),   # 12.5 wide: 10mm REX spacers pass through
             circ(*GUIDE_1, M4), circ(*GUIDE_2, M4), circ(*STOP_E, M4), circ(*BUMPER_E, M4),
             *[circ(*b, M4) for b in SPRING_B], *[circ(*b, M4) for b in HOOD_GEAR_BOLTS]]
    keep = [h.buffer(7) for h in holes] + [Point(0, 0).buffer(44)]
    ribs = [[(0, 0), J], [(0, 0), GUIDE_1], [(0, 0), S_P], [J, GUIDE_2], [J, S_P], [(0, 0), SPRING_B[0]]]
    pockets = truss(body, keep, ribs, edge=9, rib=9)
    return body.difference(unary_union(holes)).difference(pockets)

# v6.1: SPRING ANCHOR block on each arm tail (replaces the x-axis post: a constant-force strip's 0.188 in end hole is
# punched through its face, so its screw must be perpendicular to the strip, and a nut had no room: 1 mm to the carrier,
# 1.5 mm to the side plate). The block sits in the drive layer; an M4 flat head from the arm's inner face (countersunk,
# ARM_CSK) threads into its heat-set, and the stacked strip ends are screwed flat to its face with an M4 button head into a
# second heat-set, screw axis along the arm (perpendicular to the strips). Local frame: a along U_ARM from SPRING_A, b along N_ARM.
# v6.1 (Astra): the block starts ON the arm's outer face (v6.1-draft floated 1.5 mm off it) and runs to 0.5 mm short of the
# side plate's drive-layer edge; both insert pockets are modelled and spaced so they can't meet. A printed clamp plate with
# a rounded exit edge sits under the spring-end screw so the strips bend over a controlled radius, not a washer edge.
ANCHOR_SCREW_A = INSERT_DEPTH + INSERT_HOLE_D / 2 + 1.5           # arm-to-block flat head, along the arm from SPRING_A
ANCHOR_A, ANCHOR_B = (0.0, ANCHOR_SCREW_A + INSERT_HOLE_D / 2 + 2.5), (-7.0, 7.0)
ANCHOR_X = 1.0 + 15.5                                             # arm outer face (X_ARM + T) to X_DRV + 15.5
CLAMP_T, CLAMP_R = 3.0, 2.5                                       # clamp plate thickness, exit-edge radius (<= thickness)
TIP_RELIEF = 1.3                                                  # 4.5 mm screw-tip relief past each insert pocket (arm screw needs 5.65 mm, pocket 5.2)
ARM_CSK = [add(SPRING_A, U_ARM, ANCHOR_SCREW_A)]

def _round_corner(rect_poly, cx, cy, sa, sb, r):
    """Round the (cx, cy) corner of an axis-aligned local rectangle; (sa, sb) = +/-1 point from the arc centre to that corner."""
    ctr = (cx - sa * r, cy - sb * r)
    corner = box(min(cx, ctr[0]), min(cy, ctr[1]), max(cx, ctr[0]), max(cy, ctr[1]))
    return rect_poly.difference(corner).union(Point(ctr).buffer(r, quad_segs=24).intersection(corner))

def anchor_2d():
    """Anchor block footprint in the carrier frame (Pollen position). The strip-exit corner (a = 0, b = +7, toward the drum)
    is rounded to CLAMP_R so the strips bend over a radius whichever way the arm swings (v6.1 round 2)."""
    loc = _round_corner(box(ANCHOR_A[0], ANCHOR_B[0], ANCHOR_A[1], ANCHOR_B[1]), ANCHOR_A[0], ANCHOR_B[1], -1, 1, CLAMP_R)
    c = lambda a, b: add(add(SPRING_A, U_ARM, a), N_ARM, b)
    return Polygon([c(a, b) for a, b in loc.exterior.coords])

STRIP_Z = 1.0 + 0.5 + 1.15 + 12.7 / 2        # strip centreline above the arm's outer face (gap + drum offset + half strip)

def anchor_solid(world_frame=False):
    """Anchor block with both insert pockets. Local frame: xy = carrier frame (or world YZ), z = +X from the arm's outer face.
    v6.1 (Astra round 2): each insert pocket continues as a 4.5 mm screw-tip relief so the screws can't bottom out."""
    o = (lambda p: world(p)) if world_frame else (lambda p: p)
    poly = Polygon([o(p) for p in anchor_2d().exterior.coords])
    s = plate(poly, ANCHOR_X)
    xs = o(ARM_CSK[0])                                                    # arm screw: insert at the arm-side face
    s -= Pos(xs[0], xs[1], -0.01) * Solid.make_cylinder(INSERT_HOLE_D / 2, INSERT_DEPTH + 0.01)
    s -= Pos(xs[0], xs[1], -0.01) * Solid.make_cylinder(2.25, INSERT_DEPTH + TIP_RELIEF)
    p0 = o(SPRING_A)                                                      # spring-end screw: insert along +U, strip height
    pl = Plane(origin=(p0[0], p0[1], STRIP_Z), z_dir=(U_ARM[0], U_ARM[1], 0))
    s -= Solid.make_cylinder(INSERT_HOLE_D / 2, INSERT_DEPTH, pl)
    s -= Solid.make_cylinder(2.25, INSERT_DEPTH + TIP_RELIEF, pl)
    return s

def clamp_solid(world_frame=False):
    """Printed clamp plate under the spring-end screw, covering the strip width. Its strip-side edge toward the drum (+N)
    is rounded to CLAMP_R: that is where the strips bend as the arm swings (v6.1 round 2: now actually modelled)."""
    a0, a1 = -0.6 - CLAMP_T, -0.6                                         # along U: screw-head side .. strip side
    rounded = _round_corner(box(a0, -7.0, a1, 7.0), a1, 7.0, 1, 1, CLAMP_R)   # local (a, b); same exit line as the block
    o = (lambda p: world(p)) if world_frame else (lambda p: p)
    c = lambda a, b: o(add(add(SPRING_A, U_ARM, a), N_ARM, b))
    poly = Polygon([c(a, b) for a, b in rounded.exterior.coords])
    s = Pos(0, 0, STRIP_Z - 12.7 / 2 - 1.0) * plate(poly, 12.7 + 2.0)
    p0 = c(a0 - 0.01, 0.0)
    s -= Solid.make_cylinder(M4 / 2, CLAMP_T + 0.02, Plane(origin=(p0[0], p0[1], STRIP_Z), z_dir=(U_ARM[0], U_ARM[1], 0)))
    return s

def arm_2d():
    """Swing arm (x2), drawn in carrier frame at the Pollen position. v6.1: the tail lobe carries the spring anchor's
    countersunk screw (>= 8.6 mm to every edge)."""
    body = unary_union([Point(*J).buffer(ARM_HUB), Point(*S_P).buffer(ARM_HUB),
                        Point(*SPRING_A).buffer(8), Point(*ARM_CSK[0]).buffer(10.5)]).convex_hull
    seat = PRINT_BRG_1611 if LAMINATE else BRG_1611     # 2026-10-20: the arm is printed in PA-CF when LAMINATE
    holes = [circ(*J, seat), circ(*S_P, seat), circ(*ARM_CSK[0], M4)]
    return body.difference(unary_union(holes))

def side_outline(motor_side=True):
    gp = lambda p, a: world(p, a)
    pts = [(FRONT_Y, DECK_TOP), (REAR_Y, DECK_TOP), (REAR_Y, 70)]
    for p in (GUIDE_1, GUIDE_2):
        for a in (0, HOOD_ALPHA_MAX):
            pts.append(gp(add((0, 0), unit(p), math.hypot(*p) + 14), a))
    pts += [(FRONT_Y, LL_PIVOT[1] + 18), (FW_Y - 40, FW_Z + 95)]
    hull = Polygon(pts).convex_hull
    tabs = [box(ty - 10, DECK_Z0, ty + 10, DECK_TOP + 0.01) for ty in TAB_Y]
    # v8.1: the lower motor's window comes within 1.5 mm of the bottom edge, so the plate's front reaches down below
    # it (9.5 mm of wood). Motor side only: on the servo side the deck's home-magnet ear runs out to y -119.
    # 2026-10-05 (user's screenshot): it must also cover the motor adapter and the lower clamp bolt's slot (r 26 at 256 deg,
    # z ~20), which broke out of the bottom edge: 2 mm of wood below the adapter's lowest point
    chin_z = min(FW_MOTOR[1] - MOTOR_WIN_R - 9.5, motor_adapter_2d(True).bounds[1] - 2.0) if MOTOR_ADJ else FW_MOTOR[1] - MOTOR_WIN_R - 9.5
    chin = box(FRONT_Y, chin_z, FW_MOTOR[0] + MOTOR_WIN_R + 12, DECK_TOP + 0.01)
    return unary_union([hull] + ([chin] if motor_side else []) + tabs)

def isogrid(outline, keepouts, cell=ISO_CELL, rib=SIDE_RIB, edge=SIDE_EDGE, min_area=150.0, fillet=2.5):
    """Triangular isogrid pockets: stiff in every direction, no pocket bigger than one cell."""
    inner = outline.buffer(-edge, join_style=2).difference(unary_union(keepouts))
    x0, y0, x1, y1 = outline.bounds
    h = cell * math.sqrt(3) / 2
    tris = []
    for j in range(int((y1 - y0) / h) + 2):
        for i in range(-1, int((x1 - x0) / cell) + 2):
            ox = x0 + i * cell + (cell / 2 if j % 2 else 0); oy = y0 + j * h
            tris.append(Polygon([(ox, oy), (ox + cell, oy), (ox + cell / 2, oy + h)]))
            tris.append(Polygon([(ox + cell / 2, oy + h), (ox + cell * 1.5, oy + h), (ox + cell, oy)]))
    pockets = []
    for t in tris:
        p = t.buffer(-rib / 2, join_style=2).intersection(inner)
        p = p.buffer(-fillet, join_style=1).buffer(fillet, join_style=1)
        for g in (p.geoms if hasattr(p, "geoms") else [p]):
            if g.area > min_area:
                pockets.append(g)
    return unary_union(pockets)

# v7: BIOBUZZ theme: the side plates' weight-saving pockets are a honeycomb instead of triangles. Only whole (or nearly
# whole) hexagons are cut, so the pattern reads as a honeycomb; ribs stay >= HEX_RIB (well above the laser's 2.5 mm).
HEX_POCKETS = LAMINATE
HEX_PITCH, HEX_RIB = 26.0, 6.0      # centre-to-centre across the flats; wood between neighbouring pockets

def honeycomb(outline, keepouts, pitch=HEX_PITCH, rib=HEX_RIB, edge=SIDE_EDGE, keep_frac=0.999, fillet=1.5):
    inner = outline.buffer(-edge, join_style=2).difference(unary_union(keepouts))
    x0, y0, x1, y1 = outline.bounds
    af = pitch - rib                                        # pocket size across the flats
    R = af / math.sqrt(3)                                   # centre to corner
    dx, dy = pitch, pitch * math.sqrt(3) / 2                # rows of flat-topped... pointy-topped hexes
    cells = []
    for j in range(int((y1 - y0) / dy) + 3):
        for i in range(-1, int((x1 - x0) / dx) + 3):
            cx = x0 + i * dx + (dx / 2 if j % 2 else 0); cy = y0 + j * dy
            hexa = Polygon([(cx + R * math.cos(math.radians(90 + 60 * k)), cy + R * math.sin(math.radians(90 + 60 * k))) for k in range(6)])
            cut = hexa.intersection(inner)
            if cut.area >= keep_frac * hexa.area:
                cells.append(cut.buffer(-fillet, join_style=1).buffer(fillet, join_style=1))
    return unary_union(cells) if cells else Polygon()

def servo_insert_2d():
    """v7: the hood servo sits in a printed (PA-CF) insert, because its ear screws are only ~4 mm from its body and a
    plywood web that thin would break. Side-plate coords (Y, Z). Returns (boss, lip, window, ear points): the boss
    fills the opening in both layers; the lip (in the inner layer's plane) is trapped when the layers are glued."""
    sy, sz = SERVO_AXIS
    sdir = -1 if SERVO_BODY_FWD else 1
    window = box(min(sy - sdir * 10.2, sy + sdir * 30.5), sz - 10.3, max(sy - sdir * 10.2, sy + sdir * 30.5), sz + 10.3)
    c = sy + sdir * 10.1
    ears = [(c + e1 * 24.75, sz + e2) for e1 in (-1, 1) for e2 in (-5, 5)]
    edge = SERVO_INS_D / 2 + 5.5                                    # heat-set needs plenty of plastic around it
    boss = box(c - 24.75 - edge, sz - 5 - edge, c + 24.75 + edge, sz + 5 + edge).buffer(3, join_style=1).buffer(-3, join_style=1)
    lip = boss.buffer(5.0, join_style=1).difference(unary_union(       # notched to keep 3 mm of wood around the crossbar screws
        [Point(y, z).buffer(M4 / 2 + 3.0 + 0.15) for (y, z) in CROSSBARS])).buffer(-1, join_style=1).buffer(1, join_style=1)
    return boss, lip, window, ears

def servo_insert():
    """The printed servo insert as a solid in side-plate local coords (z = 0 is the plate's inner face)."""
    boss, lip, window, ears = servo_insert_2d()
    sol = plate(boss.difference(window), T) + plate(lip.difference(boss.buffer(-0.01)), PLY_LAYER)
    for (y, z) in ears:   # blind M3 heat-set pockets from the outer face, 1 mm relief, 3.2 mm hole through
        sol = sol - Pos(y, z, T - SERVO_INS_L - 1.0) * Solid.make_cylinder(SERVO_INS_D / 2, SERVO_INS_L + 1.01)
        sol = sol - Pos(y, z, -0.01) * Solid.make_cylinder(1.6, T + 0.02)
    return sol

SERVO_INSERT = LAMINATE
# v7 (Astra #3): the servo's ear holes are 4.5 mm from its body, so an M4 heat-set (5.6 hole) left ~1.5 mm of plastic to
# the window. M3 screws + M3 heat-sets (4.0 mm hole, 4.0 long) leave ~2.5 mm; the servo's ears take M3 fine.
SERVO_INS_D, SERVO_INS_L = 4.0, 4.3          # McMaster 94459A130: M3, 4.3 long, hole <= 4.0

def side_plate_2d(servo_side):
    out = side_outline(not servo_side)
    if LAMINATE:   # v7: round the outline's sharp corners (a laser-cut plywood tip would char through)
        out = out.buffer(-3.0, join_style=1).buffer(3.0, join_style=1).intersection(side_outline(not servo_side))   # (round-off stays inside)
    holes, keep = [], []
    def hole(g, k=7.0):
        holes.append(g); keep.append(g.buffer(k))
    hole(circ(FW_Y, FW_Z, BRG_1611), 10)
    for (y, z) in CROSSBARS:
        hole(circ(y, z, M4), 8)
    if REAR_GUARD:
        hole(circ(*REAR_GUARD_PIN, M4), 8)
    for by in BRACKET_Y:
        for dy in BRKT_SIDE_DY:
            hole(circ(by + dy, DECK_TOP + 9.5, M4), 7)
    for p in (GUIDE_1, GUIDE_2):
        r = math.hypot(*p); psi = math.degrees(math.atan2(p[1], p[0]))
        hole(arc_slot(FW_Y, FW_Z, r, psi, psi + HOOD_ALPHA_MAX, M4 + 0.4), 7)
    # Limelight tilt: pivot + lock holes every 5 deg
    hole(circ(*LL_PIVOT, M4), 7)
    for tilt in range(LL_TILT_RANGE[0], LL_TILT_RANGE[1] + 1, 5):
        _, up = ll_dirs(tilt)
        r = LL_LOCK_R[1] if tilt % 10 == 0 else LL_LOCK_R[0]   # staggered rows keep webs thick
        hole(circ(*add(LL_PIVOT, up, -r), M4), 4)
    for (y_p, _, z), anchor in zip(IDLER_SLOTS, IDLER_ANCHORS):
        hole(slot((y_p, z), (y_p + IDLER_TRAVEL, z), IDLER_AXLE + 0.3), 7)
        hole(circ(*anchor, M4), 7)
    if servo_side:
        sy, sz = SERVO_AXIS
        sdir = -1 if SERVO_BODY_FWD else 1                      # which way the body runs from the spline
        body = box(min(sy - sdir * 10.2, sy + sdir * 30.5), sz - 10.3, max(sy - sdir * 10.2, sy + sdir * 30.5), sz + 10.3)
        c = sy + sdir * 10.1
        ears = unary_union([circ(c + e1 * 24.75, sz + e2, TAP_M4 if SERVO_TAPPED else M4) for e1 in (-1, 1) for e2 in (-5, 5)])
        if SERVO_INSERT:              # v7: opening for the printed servo insert (the inner layer's lip recess is added per layer)
            boss, lip, _, _ = servo_insert_2d()
            holes.append(boss.buffer(0.15)); keep.append(lip.buffer(4))
        else:
            hole(body, 7); hole(ears, 5)
        if SECOND_MOTOR:
            u2 = unit((FW_MOTOR2[0] - FW_Y, FW_MOTOR2[1] - FW_Z))
            hole(motor_holes(*FW_MOTOR2, slot_dir=u2, slot_len=4.0), 8)
            keep.append(Point(*FW_MOTOR2).buffer(MOTOR_D / 2 + 4))
    else:
        if ENCODER:
            for r, a in BRKT_LEGS:
                ly, lz = pol(r, math.radians(a))
                hole(circ(FW_Y + ly, FW_Z + lz, M4), 6)
        if MOTOR_ADJ:   # window for the motor-screw heads, slots for the 4 clamp bolts, tapped holes for the jack yoke
            hole(motor_slide_slot((0.0, 0.0), 2 * MOTOR_WIN_R), 8)
            for p in LUG_BOLTS + CLAMP_BOLTS:
                hole(motor_slide_slot(p, M4), 6)
            for p in YOKE_SCREWS:
                hole(circ(*motor_rel(p), TAP_M4), 6)
            keep.append(motor_adapter_2d(True).buffer(3))
        else:
            hole(motor_holes(*FW_MOTOR), 8)                 # round: fixed centre distance (belt test plate calibrates it)
            keep.append(Point(*FW_MOTOR).buffer(MOTOR_D / 2 + 4))
    pockets = honeycomb(out, keep) if HEX_POCKETS else isogrid(out, keep)
    inner = out.buffer(-2.5)
    bad = [g.centroid.coords[0] for g in holes if not inner.contains(g)]
    assert not bad, f"side plate ({'servo' if servo_side else 'motor'}): holes within 2.5 mm of (or through) the edge at {[(round(x), round(y)) for x, y in bad]}"
    return out.difference(unary_union(holes)).difference(pockets)

# =====================================================================
#                        V2 PRINTED / REF PARTS
# =====================================================================
def servo_pinion():
    """REF (purchased): 1922-0025-0036 8mm REX servo shaft + 2303-4008-0020 20T MOD 0.8 steel pinion.
    Local Z runs from the pinion's inner face (carrier plane) out to the servo spline."""
    g = plate(gear(SERVO_PINION_TEETH, SECTOR_MODULE).difference(rex_bore()), HOOD_GEAR_T)
    shaft = revolve_profile(0, 4.0, SIDE_X_OUT - 12.0 - (X_CAR + T))
    return g + shaft

def hood_gear():
    """REF (purchased) goBILDA 2302-0032-0108: 108T MOD 0.8 aluminum, 32 mm bore (holes not drawn). Measured from its STEP:
    a 3 mm web (flush with the face against the carrier) out to r 40.25, and the full 6 mm only at the toothed rim."""
    g = plate(gear(SECTOR_TEETH_FULL, SECTOR_MODULE).difference(Point(0, 0).buffer(HOOD_GEAR_BORE / 2)), HOOD_GEAR_T)
    pocket = Pos(0, 0, HOOD_GEAR_WEB) * plate(Point(0, 0).buffer(HOOD_GEAR_RIM_R, quad_segs=48).difference(
        Point(0, 0).buffer(HOOD_GEAR_BORE / 2 - 0.01)), HOOD_GEAR_T - HOOD_GEAR_WEB + 0.01)
    return g - pocket

def htd_pulley(t, w=DRIVE_W - 2):
    """REF (purchased) goBILDA 5mm HTD pulley: toothed OD = PD - 1.14 plus flanges (flange size unverified)."""
    pd = htd_pd(t)
    body = revolve_profile(4.0, pd / 2 - 0.57, w - 2)
    flange = revolve_profile(4.0, pd / 2 + 2.0, 1)
    return flange + Pos(0, 0, 1) * body + Pos(0, 0, w - 1) * flange

# ---- v7: printed drive parts (PA-CF) ----
PRINT_DRIVE = LAMINATE        # printed 24T/48T HTD pulleys and shaft spacers; the 16T motor pulley stays steel
HTD_CLEAR = 0.10              # extra groove clearance for a printed pulley (tune with a test print against the belt)

def htd5_2d(t, clear=HTD_CLEAR):
    """HTD 5M pulley outline: OD = PD - 1.143; each groove is a 1.49 mm (+clear) radius arc 2.06 mm deep (the belt
    tooth's own profile), with 0.4 mm fillets at the tips. Close to the published HTD 5M form; test-fit on the belt."""
    pd = htd_pd(t); ro = pd / 2 - 0.5715
    rg = 1.49 + clear
    cz = ro - 2.06 - clear + rg                    # groove centre radius (groove floor 2.06 + clear below the OD)
    disc = Point(0, 0).buffer(ro, quad_segs=128)
    grooves = unary_union([Point(*pol(cz, 2 * math.pi * k / t)).buffer(rg, quad_segs=24)
                           .union(Polygon([pol(cz, 2 * math.pi * k / t + d) for d in (-rg / cz, rg / cz)] +
                                          [pol(ro + 1, 2 * math.pi * k / t + d) for d in (rg * 1.25 / ro, -rg * 1.25 / ro)]))
                           for k in range(t)])
    return disc.difference(grooves).buffer(-0.4, join_style=1).buffer(0.4, join_style=1)

def printed_pulley(t, w, hub_mount=False):
    """Printed HTD 5M pulley, flange to flange = w (as the bought one). hub_mount: 14 mm boss bore + 4 M4 holes on the
    Sonic Hub's 16 mm square (like goBILDA 3415); else an 8 mm REX bore and an M3 set screw with a captured M3 nut."""
    ro = htd_pd(t) / 2 - 0.5715
    bore = Point(0, 0).buffer(7.1) if hub_mount else rex_bore(0.15)
    holes = unary_union([bore] + ([circ(sx * 8, sy * 8, M4) for sx in (-1, 1) for sy in (-1, 1)] if hub_mount else []))
    body = plate(htd5_2d(t).difference(holes), w - 2)
    flange = plate(Point(0, 0).buffer(ro + 1.5).difference(holes), 1.0)
    sol = flange + Pos(0, 0, 1) * body + Pos(0, 0, w - 1) * flange
    if not hub_mount:     # M3 set screw: radial hole + a nut slot from one flange face, centred on the screw axis
        sol = sol - Pos(0, 0, w / 2) * Solid.make_cylinder(1.6, ro, Plane(origin=(0, 0, 0), z_dir=(0, 1, 0)))
        af, ac, nt = 5.5, 6.35, 2.4          # M3 hex nut: across flats, across corners, thickness (flat face down)
        z0 = w / 2 - af / 2                  # slot floor: the nut's centre sits on the screw axis (v7 Astra rounds 1-2)
        sol = sol - Pos(-(ac + 0.25) / 2, 7.0, z0) * Box(ac + 0.25, nt + 0.3, w - z0 + 0.01, align=(Align.MIN, Align.MIN, Align.MIN))
    return sol

def printed_spacer(L):
    """Printed shaft spacer: 8.2 bore, 10 mm OD (touches only a bearing's inner race), like goBILDA 1522."""
    return revolve_profile(4.1, 5.0, L)

def sync_gear(n):
    """REF (purchased) goBILDA 2312 acetal MOD 0.8 hub-mount gear, 4mm thick, on a 1309-0016-4008 Sonic Hub."""
    return plate(gear(n, SYNC_MODULE).difference(Point(0, 0).buffer(7)), GEAR_T)

def sonic_hub():
    """REF goBILDA 1309-0016-4008 Sonic Hub body envelope: 32 mm across (the real hub is 32 x 25.4), 7.86 thick."""
    return revolve_profile(4.0, HUB_R, HUB_W)

def idler_roller():
    """Printed roller; a goBILDA 1611-0514-0006 flanged bearing (6mm ID x 14mm OD x 5mm) presses into each end."""
    L = CORRIDOR - 2.0
    body = revolve_profile(IDLER_AXLE / 2 + 0.6, IDLER_D / 2, L)
    pocket = revolve_profile(0, 7.05, 5)
    return body - pocket - Pos(0, 0, L - 5) * pocket

def idler_spool():
    """Printed tab clamped on the idler axle end (M3 set screw). The spring's 5mm loop hooks on an M4 bolt in the tab."""
    h = 8.0
    body = Point(0, 0).buffer(7.0).union(box(-5, 0, 5, 14)).union(Point(0, 14).buffer(5.0))
    body = body.difference(Point(0, 0).buffer(IDLER_AXLE / 2 + 0.1)).difference(Point(0, 14).buffer(M4 / 2))
    return plate(body, h)

def idler_spacer():
    """Printed 6mm-ID tube: keeps the idler roller centered between the side plates (rides in the slots' clear zone)."""
    return revolve_profile(IDLER_AXLE / 2 + 0.2, 4.5, SIDE_X_IN - 0.5 - (CORRIDOR / 2 - 1))

def encoder_sleeve():
    """Printed adapter: 8mm REX inside, 1/2in hex (12.7mm across flats, printed 12.6) outside. Drives the encoder rotor."""
    col0, _, _, send = encoder_x()
    af = 12.6
    hexa = Polygon([pol(af / 2 / math.cos(math.pi / 6), k * math.pi / 3) for k in range(6)])
    return plate(hexa.difference(rex_bore(0.2)), send - col0)

def cf_drum():
    """Printed drum the AndyMark am-4666 constant-force coil rides on. v5: spins on a printed sleeve (cf_axle), because
    a shoulder-bolt head has no room either side (ball corridor inside, side plate 2.5 mm outside)."""
    # v6.1: the flange is sized to the largest stack the drum takes (CF_SPRINGS_MAX strips wound together)
    return revolve_profile(CF_AXLE_D / 2 + 0.2, CF_DRUM_D / 2, 14.0) + \
        revolve_profile(CF_AXLE_D / 2 + 0.2, cf_coil_od(CF_SPRINGS_MAX) / 2 + 0.5, 1.0)

def arm_bumper():
    """v6.1: open-side stop post: PETG core (M4 heat-set, flat head from the ball side) + a pressed-on TPU ring.
    Spans the carrier's outer face to the swing arm's outer face."""
    L = X_ARM + T - (X_CAR + T)
    return revolve_profile(0, BUMPER_R, L) - revolve_profile(0, INSERT_HOLE_D / 2, INSERT_DEPTH) - revolve_profile(0, 2.25, L + 0.01)

def arm_bumper_core():
    """The PETG core of arm_bumper(): everything inside the TPU ring."""
    L = X_ARM + T - (X_CAR + T)
    return revolve_profile(0, BUMPER_R - 2.0, L) - revolve_profile(0, INSERT_HOLE_D / 2, INSERT_DEPTH) - revolve_profile(0, 2.25, L + 0.01)

def arm_bumper_ring():
    """The TPU ring of arm_bumper() (shown line-on-line with the core; the printed ring is 0.1 mm tighter, a press fit)."""
    return revolve_profile(BUMPER_R - 2.0, BUMPER_R, X_ARM + T - (X_CAR + T))

CF_AXLE_L = (X_DRV + 0.5 + 14.0 + 0.5) - (X_CAR + T)   # carrier outer face to 0.5 mm past the drum

def cf_axle():
    """Printed sleeve, carrier outer face to just past the drum: M4 heat-set in the outer end, pulled against the
    carrier by an M4 x 30 flat head countersunk from the ball side. The side plate (2 mm away) keeps the drum on."""
    return revolve_profile(0, CF_AXLE_D / 2, CF_AXLE_L) - Pos(0, 0, CF_AXLE_L - 8) * revolve_profile(0, 2.8, 8.01) - \
        revolve_profile(0, 2.25, CF_AXLE_L)

def front_plate_2d():
    """v6: optional flair panel, drawn in (X across, Z up). Sits between the side plates in front of the front crossbar;
    two zip ties around the crossbar through the slot pairs. Flair only: ROBOT SIGNS (R401-R403) go on the chassis."""
    y_c, z_c = min(CROSSBARS, key=lambda c: c[0])                 # the front-most crossbar
    w = 2 * (SIDE_X_IN - 0.5 - (ADAPT_T + 0.5 if MOTOR_ADJ else 0.0))   # v6: clear of the motor adapter plate (kept symmetric)
    z0 = DECK_TOP + 1.0
    outline = box(-w / 2, z0, w / 2, z0 + FRONT_PLATE_H).buffer(-4).buffer(4)
    slots = [box(sx * 40 - 2, z_c + dz - 5, sx * 40 + 2, z_c + dz + 5) for sx in (-1, 1) for dz in (-12, 12)]
    return outline.difference(unary_union(slots)), y_c - 7.0 - 0.3      # plate's back face sits against the crossbar

def eccentric_stop():
    """v6: printed eccentric cam (OD 13) with an M4 heat-set 1.5 mm off-centre. Loosen the flat head from the ball
    side, turn the cam to set the worn-ball squeeze, retighten."""
    body = Point(0, 0).buffer(CAM_R, quad_segs=32).difference(Point(CAM_ECC, 0).buffer(2.8, quad_segs=16))
    return plate(body, T)

def spring_post():
    """Anchor for a goBILDA 2915 spring: two printed half-spacers on an M4 bolt; the loop sits on the bare bolt between."""
    h = DRIVE_W - 1
    half = revolve_profile(M4 / 2, 5.0, (h - 3) / 2)
    return half + Pos(0, 0, (h + 3) / 2) * half

def l_bracket():
    w, t = BRKT_W, 5.0
    vert = box(-w / 2, 0, w / 2, 16).difference(unary_union([circ(dx, 9.5, M4) for dx in BRKT_SIDE_DY]))
    foot = box(-w / 2, 0, w / 2, BRKT_FOOT_L).difference(unary_union([circ(dx, BRKT_FOOT_HOLE, M4) for dx in BRKT_FOOT_DY]))
    v = Plane(origin=(0, t, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)).location * plate(vert, t)
    return v + plate(foot, t)

def crossbar():
    L = 2 * SIDE_X_IN
    tube = revolve_profile(0, 7, L)
    insert = revolve_profile(0, 2.8, 8)
    return tube - insert - Pos(0, 0, L - 8) * insert

def rear_guard():
    """v6.2 rear ball guard, in the turret frame (x across, y back, z up): the rear crossbar tube plus a 4 mm wall just
    behind the feed column, joined by a web; end tabs at the side plates carry a second M4 heat-set each so it can't
    rotate. Print it on its front face."""
    (by, bz), (py, pz) = REAR_GUARD_BAR, REAR_GUARD_PIN
    L = 2 * SIDE_X_IN
    tube = Pos(-SIDE_X_IN, by, bz) * along_x(revolve_profile(0, 7, L))
    wall = Pos(0, REAR_GUARD_Y, (REAR_GUARD_Z[0] + REAR_GUARD_Z[1] + REAR_GUARD_T / 2) / 2) * Box(
        REAR_GUARD_W, REAR_GUARD_T, REAR_GUARD_Z[1] + REAR_GUARD_T / 2 - REAR_GUARD_Z[0])
    web = Pos(0, (REAR_GUARD_Y + by) / 2, bz) * Box(REAR_GUARD_W, by - REAR_GUARD_Y, 12.0)
    body = tube + wall + web
    for sx in (-1, 1):
        x0 = sx * (SIDE_X_IN - REAR_GUARD_TAB / 2)
        tab = Pos(x0, by, (bz + pz) / 2) * Box(REAR_GUARD_TAB, 14.0, pz - bz) + Pos(x0, py, pz) * along_x(
            Pos(0, 0, -REAR_GUARD_TAB / 2) * revolve_profile(0, 7, REAR_GUARD_TAB))
        body = body + tab
        for (yy, zz) in (REAR_GUARD_BAR, REAR_GUARD_PIN):          # heat-set pockets from each end
            pocket = along_x(revolve_profile(0, INSERT_HOLE_D / 2, INSERT_DEPTH + TIP_RELIEF))
            body = body - (Pos(sx * SIDE_X_IN - (INSERT_DEPTH + TIP_RELIEF if sx > 0 else 0), yy, zz) * pocket)
    return body

def ll_mount():
    """Limelight 3A tilt mount. Local: X across, Y = plate 'up', Z = view direction.
    Pivot at the origin; lock inserts at Y = -44 (25/35/45 deg row) and Y = -58 (20/30/40/50 deg row).
    Camera sits on the front face (Z >= 0) centered at Y = -30, on a goBILDA 8mm M4 grid."""
    from build123d import Box, Cylinder
    w = 2 * SIDE_X_IN - 1.0
    def bx(x0, x1, y0, y1, z0, z1):
        return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)
    body = bx(-40, 40, -62, 2, -LL_PLATE_T, 0)
    body += bx(-w / 2, w / 2, -6, 6, -8, 0) + bx(-w / 2, w / 2, -66, -50, -8, 0)
    for sx in (-1, 1):
        body += bx(sx * w / 2 - 10 if sx > 0 else -w / 2, sx * w / 2 if sx > 0 else -w / 2 + 10, -66, 8, -8, 8)
    for x in range(-32, 33, 8):
        for y in range(-46, -13, 8):
            body -= Pos(x, y, -LL_PLATE_T / 2) * Cylinder(M4 / 2, LL_PLATE_T + 1)
    for sx in (-1, 1):
        for y in (0, -LL_LOCK_R[0], -LL_LOCK_R[1]):
            body -= Pos(sx * (w / 2 - 4), y, 0) * Rot(0, 90, 0) * Cylinder(2.8, 8.2)
    return body

def limelight_ref():
    from build123d import Box, Cylinder
    body = Pos(0, 0, LL_DIMS[2] / 2) * Box(LL_DIMS[0], LL_DIMS[1], LL_DIMS[2])
    lens = Pos(0, 4, LL_DIMS[2] + 2) * Cylinder(6, 4)
    return body + lens

def ref_cyl(d, h, d_in=0):
    return revolve_profile(d_in / 2, d / 2, h)

def along_x(shape):
    return Rot(0, 90, 0) * shape

def belt_loop(p0, p1, x0, r0=None, r1=None, w=9.0):
    """HTD 9mm belt as a thin loop around two pulleys (pitch radii r0, r1), for visuals and interference checks."""
    r0 = JACK_PD / 2 if r0 is None else r0
    r1 = r0 if r1 is None else r1
    outer = unary_union([Point(*p0).buffer(r0 + 1.5, quad_segs=32), Point(*p1).buffer(r1 + 1.5, quad_segs=32)]).convex_hull
    inner = unary_union([Point(*p0).buffer(r0 - 0.5, quad_segs=32), Point(*p1).buffer(r1 - 0.5, quad_segs=32)]).convex_hull
    return yz_plate(outer.difference(inner), x0, w)

def tire():
    """Grooved tire over two 608s. Press one bearing in from each end until it seats on the center lip."""
    s = revolve_profile(B608_OD / 2 + 0.05, TIRE_OD / 2, TIRE_H)
    s += Pos(0, 0, B608_W) * revolve_profile(19.5 / 2, B608_OD / 2 + 0.1, TIRE_LIP)
    groove = revolve_profile(TIRE_ROOT_D / 2, TIRE_OD / 2 + 1, T + 0.4)
    return s - Pos(0, 0, TIRE_H / 2 - (T + 0.4) / 2) * groove

def roller_spacer():
    """Between base plate and the lower 608: touches the inner race only."""
    return revolve_profile(M8 / 2, INNER_RACE_OD / 2, ROLLER_SPACER_H)

def guide_spacer():
    """Printed 8 mm tube on each hood guide bolt, carrier outer face to side-plate inner face."""
    return revolve_profile(M4 / 2 + 0.15, 4.0, SIDE_X_IN - X_CAR - T)

def roller_mid_spacer():
    """v5 (Codex review): 1 mm spacer between the two 608s' INNER races, inside the tire lip, so tightening the
    M8 clamps a solid inner-race stack instead of squeezing the bearings together. Or use a 1 mm shim <= 12 mm OD."""
    return revolve_profile(M8 / 2 + 0.1, INNER_RACE_OD / 2, TIRE_LIP)

def roller_cap():
    """On top of the upper 608's inner race, under the M8 washer + nyloc (keeps the washer off the shield)."""
    return revolve_profile(M8 / 2, INNER_RACE_OD / 2, 2.0)

def deck_spacer():
    return revolve_profile(M4 / 2, 5, DECK_SPACER_H)

def turret_pinion():
    """REF (purchased) goBILDA 2303-4008-0020 steel MOD 0.8 20T pinion, 8mm REX, set-screw (face width unverified)."""
    return Pos(0, 0, 4) * plate(gear(PINION_TEETH, RING_MODULE).difference(rex_bore()), 8) + \
        revolve_profile(4.0, 7.0, 4)

UPPER_GEAR_PHASE = 5.55   # deg, set so the upper gear meshes with the 60T at the Pollen position

def at_x(shape, x0):
    """Z-axis part -> axis along +X, starting at x0 (Y/Z set by caller)."""
    return Pos(x0, 0, 0) * along_x(shape)

def on_axis(shape, x0, yz):
    return Pos(x0, yz[0], yz[1]) * along_x(shape)

def ll_location(tilt=LL_TILT):
    v, _ = ll_dirs(tilt)
    return Plane(origin=(0, *LL_PIVOT), x_dir=(1, 0, 0), z_dir=(0, *v)).location

def build():
    """Parts at hood alpha 0, Pollen arm position, default camera tilt.
    group: fixed | turret | carrier | arm | ll ; spin: axis the part spins about (viewer)."""
    parts = {}
    def put(name, shape, group, color, kind, spin=None):
        if LAMINATE and kind == "cut" and not name.startswith("front_plate"):   # v7: plywood looks like wood
            color = "#b07a45" if name.startswith("hood_carrier") else "#d8b98a"
        if filament_of_part(name, kind):          # printed parts: PA-CF black, PETG purple (the filament table above)
            color = FILAMENT_COLOR[filament_of_part(name, kind)]
        parts[name] = (shape, group, color, kind, spin)

    base = None if GB_TURRET else base_plate_2d(); ring = None if GB_TURRET else ring_2d(); deck = deck_2d()
    side_s = side_plate_2d(True); side_m = side_plate_2d(False)
    carr = carrier_2d(); arm = arm_2d()
    plates2d = {**({} if GB_TURRET else {"base_plate": base, "turret_ring": ring}), "deck": deck, "side_plate_servo": side_s,
                "side_plate_motor": side_m, "hood_carrier": carr, "swing_arm": arm}
    # v7 (Astra #4): the 3D plates are built from the same two layers the DXFs are cut from (nut pockets, counterbores,
    # dowel holes, the servo insert's lip recess), so check.py and access_check.py test the parts that get made.
    LAYERS.clear()
    if LAMINATE:
        for fname, poly, nuts, nl, cb, cl, faces, extra in laminate_specs(plates2d):
            LAYERS[fname] = laminate(poly, nuts, nl, cb, cl, extra)
    lam = lambda name: laminated_solid(*LAYERS[name][:2])

    # ---- fixed base ----
    if GB_TURRET:
        for n, (shape, grp, col, kind) in gb_parts().items():
            put(n, shape, grp, col, kind)
    if not GB_TURRET:
      put("base_plate", Pos(0, 0, -T) * (lam("base_plate") if LAMINATE else plate(base)), "fixed", "#2b2f36", "cut")
    for k in range(0 if GB_TURRET else N_ROLLERS):
        x, y = pol(ROLLER_R, math.radians(30 + 360 * k / N_ROLLERS))
        put(f"tire_{k}", Pos(x, y, ROLLER_Z0) * tire(), "fixed", "#d63a2f", "print")
        put(f"roller_spacer_{k}", Pos(x, y, 0) * roller_spacer(), "fixed", "#e8e8e8", "print")
        put(f"roller_cap_{k}", Pos(x, y, ROLLER_Z0 + TIRE_H) * roller_cap(), "fixed", "#e8e8e8", "print")
        put(f"roller_mid_spacer_{k}", Pos(x, y, ROLLER_Z0 + B608_W) * roller_mid_spacer(), "fixed", "#e8e8e8", "print")
    if not GB_TURRET:
        mx, my = pol(PIN_CENTER_R, math.radians(TURRET_MOTOR_ANGLE))
        put("turret_pinion_ref", Pos(mx, my, RING_ZC - 8.0) * turret_pinion(), "fixed", "#c9ced4", "ref")
        put("turret_motor_ref", Pos(mx, my, -T - 107.5) * ref_cyl(MOTOR_D, 107.5), "fixed", "#9aa0a6", "ref")
    if CABLES:
        for k in range(TRAY_N):
            put(f"cable_tray_{k}", cable_tray(k), "fixed", "#3d4450", "print")
        put("home_switch_ref", home_switch(), "fixed", "#15181c", "ref")
        put("cable_loop_ref", cable_ribbon(0.0), "fixed", "#2f6fdd", "ref")
    if HOME_FLAG:
        put("home_switch_ref", home_switch(), "fixed", "#15181c", "ref")

    # ---- rotating turret frame ----
    if not GB_TURRET:
        put("turret_ring", Pos(0, 0, RING_Z0) * plate(ring), "turret", "#2b2f36", "print" if LAMINATE else "cut")
    for k in range(0 if GB_TURRET else 8):
        x, y = pol(DECK_SPACER_R, math.radians(DECK_SPACER_ANGS[k]))
        put(f"deck_spacer_{k}", Pos(x, y, RING_Z0 + T) * deck_spacer(), "turret", "#e8e8e8", "print")
    put("deck", Pos(0, 0, DECK_Z0) * (lam("deck") if LAMINATE else plate(deck)), "turret", "#2b2f36", "cut")
    if CABLES:
        for k in range(DRUM_N):
            put(f"cable_drum_{k}", cable_drum(k), "turret", "#e8e8e8", "print")
        put("home_magnet_ref", home_magnet(), "turret", "#b0b4b8", "ref")
    if HOME_FLAG:
        put("home_flag", home_flag(), "turret", "#e8e8e8", "print")
        put("home_magnet_ref", home_magnet(), "turret", "#b0b4b8", "ref")
    side_s_solid = (Plane(origin=(SIDE_X_IN, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location * lam("side_plate_servo")
                    if LAMINATE else yz_plate(side_s, SIDE_X_IN))
    put("side_plate_servo", side_s_solid, "turret", "#1d1f23", "cut")
    if SERVO_INSERT:
        put("servo_insert", Plane(origin=(SIDE_X_IN, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location * servo_insert(),
            "turret", "#d63a2f", "print")
    put("side_plate_motor", Plane(origin=(-SIDE_X_OUT, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location *
        (lam("side_plate_motor") if LAMINATE else countersunk(plate(side_m), FW_MOTOR_BOLTS if FW_MOTOR_CSK else [], inner_z=0.0)),
        "turret", "#1d1f23", "cut")
    if MOTOR_ADJ:
        put("motor_adapter", Plane(origin=(-SIDE_X_IN, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location *
            (motor_adapter_print(True) if LAMINATE else plate(motor_adapter_2d(True), ADAPT_T)), "turret", "#8a9099", "print" if LAMINATE else "cut")
        put("motor_lug", motor_lug(), "turret", "#d63a2f", "print")
        put("motor_yoke", motor_yoke(), "turret", "#d63a2f", "print")
    for i, (y, z) in enumerate(CROSSBARS):
        if REAR_GUARD and (y, z) == REAR_GUARD_BAR:
            put("rear_guard", rear_guard(), "turret", "#f2f2f2", "print")
        else:
            put(f"crossbar_{i}", on_axis(crossbar(), -SIDE_X_IN, (y, z)), "turret", "#f2f2f2", "print")
    assert not REAR_GUARD or "rear_guard" in parts, "REAR_GUARD is on but the rear guard wasn't placed"
    lb = l_bracket()
    for sx in (-1, 1):
        for j, by in enumerate(BRACKET_Y):
            put(f"l_bracket_{'s' if sx > 0 else 'm'}{j}",
                Pos(sx * SIDE_X_OUT, by, DECK_TOP) * Rot(0, 0, -90 if sx > 0 else 90) * lb, "turret", "#d63a2f", "print")

    M = (FW_Y, FW_Z)
    pw = DRIVE_W - 2
    put("flywheel_main_ref", on_axis(ref_cyl(FW1_D, FW1_W, 8), -FW1_W / 2, M), "turret", "#111111", "ref", "M")
    _ms = shaft_stacks()["Main flywheel shaft"]["items"]
    _x0 = _ms[0][2] if not ENCODER else _ms[1][2]
    _cut = math.floor(_ms[-1][3] - _x0 - 0.5)                       # v6.1: the real 240 mm shaft (its encoder end sits 1.1 mm inside the sleeve)
    put("shaft_main_ref", on_axis(ref_cyl(8, _cut), _ms[-1][3] - _cut, M), "turret", "#b0b4b8", "ref", "M")
    p24 = printed_pulley(JACK_PULLEY_T, PUL_SS_W) if PRINT_DRIVE else htd_pulley(JACK_PULLEY_T, PUL_SS_W)
    p48 = printed_pulley(MAIN_MOTOR_PULLEY_T, PUL48_W, True) if PRINT_DRIVE else htd_pulley(MAIN_MOTOR_PULLEY_T, PUL48_W)
    put("pulley_main_jack", on_axis(p24, -(JACK_BELT_X + PUL_SS_W / 2), M), "turret", "#c9ced4", "print" if PRINT_DRIVE else "ref", "M")
    put("pulley_main_motor", on_axis(p48, -(MOTOR_BELT_X + PUL48_W / 2), M), "turret", "#c9ced4", "print" if PRINT_DRIVE else "ref", "M")
    put("hub_main_pulley", on_axis(sonic_hub(), -(MAIN_HUB_X + HUB_W), M), "turret", "#9aa3ad", "ref", "M")
    put("hub_main_wheels", on_axis(sonic_hub(), FW1_W / 2, M), "turret", "#9aa3ad", "ref", "M")
    put("pulley_motor", on_axis(htd_pulley(MOTOR_PULLEY_T, PUL_SS_W), -(MOTOR_BELT_X + PUL_SS_W / 2), FW_MOTOR), "turret", "#c9ced4", "ref", "motor")
    put("belt_motor_ref", belt_loop(M, FW_MOTOR, -(MOTOR_BELT_X + 4.5),
                                    htd_pd(MAIN_MOTOR_PULLEY_T) / 2, htd_pd(MOTOR_PULLEY_T) / 2), "turret", "#222222", "ref")
    put("flywheel_motor_ref", on_axis(ref_cyl(MOTOR_D, 107.5), -SIDE_X_IN + (ADAPT_T if MOTOR_ADJ else 0.0), FW_MOTOR),
        "turret", "#9aa0a6", "ref")
    if SECOND_MOTOR:   # mirror of motor 1's drive, on the servo side, at the rear
        put("pulley_main_motor2", on_axis(htd_pulley(MAIN_MOTOR_PULLEY_T), SIDE_X_OUT + 2, M), "turret", "#c9ced4", "ref", "M")
        put("pulley_motor2", on_axis(htd_pulley(MOTOR_PULLEY_T), SIDE_X_OUT + 2, FW_MOTOR2), "turret", "#c9ced4", "ref", "motor2")
        put("belt_motor2_ref", belt_loop(M, FW_MOTOR2, SIDE_X_OUT + 2 + pw / 2 - 4.5,
                                         htd_pd(MAIN_MOTOR_PULLEY_T) / 2, htd_pd(MOTOR_PULLEY_T) / 2), "turret", "#222222", "ref")
        put("flywheel_motor2_ref", on_axis(ref_cyl(MOTOR_D, 107.5), SIDE_X_IN - 107.5, FW_MOTOR2), "turret", "#9aa0a6", "ref")
    if ENCODER:
        col0, (b0, b1), (e0, e1), send = encoder_x()
        Mloc = lambda x0: Plane(origin=(x0, *M), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location
        put("encoder_v2_ref", Mloc(-e1) * plate(encoder_2d(), ENC_T), "turret", "#1f2a36", "ref")
        put("encoder_bracket", Mloc(-b1) * plate(encoder_bracket_2d(), BRKT_T), "turret", "#d63a2f", "print")
        for i, (r, a) in enumerate(BRKT_LEGS):
            ly, lz = pol(r, math.radians(a))
            put(f"encoder_bracket_leg_{i}", on_axis(ref_cyl(10.0, b0 - SIDE_X_OUT, M4), -b0, (M[0] + ly, M[1] + lz)), "turret", "#d63a2f", "print")
        put("encoder_sleeve", on_axis(encoder_sleeve(), -send, M), "turret", "#f2c230", "print", "M")
    from build123d import Box
    sy, sz = SERVO_AXIS
    put("hood_servo_ref", Pos(SIDE_X_OUT - 8 + 19, sy + (-1 if SERVO_BODY_FWD else 1) * 10.05, sz) * Box(38, 40.5, 20.2), "turret", "#3a3f47", "ref")
    put("servo_pinion_ref", on_axis(servo_pinion(), X_CAR + T, SERVO_AXIS), "turret", "#c9ced4", "ref", "servo")

    # ---- hood carrier (pivots on the main axis) ----
    cw = lambda p: world(p, 0.0)
    for side, x0 in (("servo", X_CAR), ("motor", -X_CAR - T)):
        put(f"hood_carrier_{side}", Plane(origin=(x0, *M), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location *
            (lam(f"hood_carrier_{side}") if LAMINATE else
             countersunk(plate(carr), CARRIER_CSK + (HOOD_GEAR_CSK if side == "servo" else []), inner_z=0.0 if side == "servo" else T)),
            "carrier", "#d63a2f", "cut")
        if side == "servo":   # v6: the bought hood gear on the carrier's outer face, concentric with the main shaft
            put("hood_gear_108t", on_axis(hood_gear(), X_CAR + T, M), "carrier", "#c9ced4", "ref")
        sgn = 1 if side == "servo" else -1
        for gi, g in enumerate((GUIDE_1, GUIDE_2)):   # shoulder spacer on the guide bolt
            x_a = X_CAR + T if sgn > 0 else -SIDE_X_IN
            put(f"guide_{side}_{gi}", on_axis(guide_spacer(), x_a, cw(g)), "carrier", "#e8e8e8", "print")
        put(f"stop_cam_{side}", on_axis(eccentric_stop(), X_ARM if sgn > 0 else -X_ARM - T, cw(STOP_E)), "carrier", "#f2c230", "print")
    Jw = cw(J)
    put("shaft_jack_ref", on_axis(ref_cyl(8, 2 * (X_DRV + DRIVE_W)), -(X_DRV + DRIVE_W), Jw), "carrier", "#b0b4b8", "ref", "J")
    put("pulley_jack", on_axis(printed_pulley(JACK_PULLEY_T, PUL_SS_W) if PRINT_DRIVE else htd_pulley(JACK_PULLEY_T, PUL_SS_W),
                               -(JACK_BELT_X + PUL_SS_W / 2), Jw), "carrier", "#c9ced4", "print" if PRINT_DRIVE else "ref", "J")
    put("gear_jack_60t", on_axis(sync_gear(SYNC_TEETH_J), X_DRV + HUB_HEAD_GAP, Jw), "carrier", "#e8e8e8", "ref", "J")
    put("belt_jack_ref", belt_loop(M, Jw, -(JACK_BELT_X + 4.5)), "carrier", "#222222", "ref")
    put("hub_jack_gear", on_axis(sonic_hub(), X_DRV + HUB_HEAD_GAP + GEAR_T, Jw), "carrier", "#9aa3ad", "ref", "J")
    for sx, side in ((1, "s"), (-1, "m")):
        x0 = X_DRV + 0.5 if sx > 0 else -(X_DRV + 0.5 + 15.0)
        put(f"cf_drum_{side}", on_axis(cf_drum(), x0, cw(SPRING_B[0])), "carrier", "#f2c230", "print")
        xa = X_CAR + T if sx > 0 else -(X_CAR + T + CF_AXLE_L)
        put(f"cf_axle_{side}", on_axis(cf_axle() if sx > 0 else Pos(0, 0, CF_AXLE_L) * Rot(180, 0, 0) * cf_axle(), xa,
                                       cw(SPRING_B[0])), "carrier", "#e8e8e8", "print")
        # v6.1: the wound coil at the size of the biggest stack the drum takes (the strip is 12.7 mm wide, on the drum body)
        xc0 = x0 + 1.15 if sx > 0 else x0 + 15.0 - 1.15 - 12.7
        put(f"cf_coil_{side}", on_axis(ref_cyl(cf_coil_od(CF_SPRINGS_MAX), 12.7, CF_DRUM_D), xc0, cw(SPRING_B[0])),
            "carrier", "#9aa0a6", "ref")
        # v6.1: open-side bumper stop, carrier outer face to the swing arm's outer face
        Lb = X_ARM + T - (X_CAR + T)
        xb = X_CAR + T if sx > 0 else -(X_CAR + T + Lb)
        # 2026-10-02: core (PETG) and ring (TPU) are separate parts, so each shows (and can be picked) in its filament
        # the insert end faces the carrier on both sides (the motor-side core was drawn unmirrored before 2026-10-02)
        core = arm_bumper_core() if sx > 0 else Pos(0, 0, Lb) * Rot(180, 0, 0) * arm_bumper_core()
        put(f"arm_bumper_core_{side}", on_axis(core, xb, cw(BUMPER_E)), "carrier", "#2f6fdd", "print")
        put(f"arm_bumper_ring_{side}", on_axis(arm_bumper_ring(), xb, cw(BUMPER_E)), "carrier", "#2f6fdd", "print")

    # ---- swing arm (pivots on the jackshaft) ----
    Sw = cw(S_P)
    for side, x0 in (("servo", X_ARM), ("motor", -X_ARM - T)):
        put(f"swing_arm_{side}", Plane(origin=(x0, *M), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location * plate(arm),
            "arm", "#d63a2f", "print" if LAMINATE else "cut")
    put("flywheel_upper_ref", on_axis(ref_cyl(FW2_D, FW2_W, 8), -FW2_W / 2, Sw), "arm", "#111111", "ref", "S")
    put("hub_upper_wheels", on_axis(sonic_hub(), FW2_W / 2, Sw), "arm", "#9aa3ad", "ref", "S")
    put("shaft_upper_ref", on_axis(ref_cyl(8, 2 * (X_DRV + DRIVE_W)), -(X_DRV + DRIVE_W), Sw), "arm", "#b0b4b8", "ref", "S")
    put("gear_upper_48t", on_axis(Rot(0, 0, UPPER_GEAR_PHASE) * sync_gear(SYNC_TEETH_S), X_DRV + HUB_HEAD_GAP, Sw), "arm", "#e8e8e8", "ref", "S")
    put("hub_upper_gear", on_axis(sonic_hub(), X_DRV + HUB_HEAD_GAP + GEAR_T, Sw), "arm", "#9aa3ad", "ref", "S")
    anc, clp = anchor_solid(True), clamp_solid(True)
    to_x = Plane(origin=(X_ARM + T, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location
    for sx, side in ((1, "s"), (-1, "m")):   # v6.1: spring anchor block + clamp plate on each arm tail
        a_, c_ = to_x * anc, to_x * clp
        if sx < 0:
            a_, c_ = a_.mirror(Plane.YZ), c_.mirror(Plane.YZ)
        put(f"cf_anchor_{side}", a_, "arm", "#d63a2f", "print")
        put(f"cf_clamp_{side}", c_, "arm", "#2f6fdd", "print")

    # ---- idler rollers (slide back for Nectar; the viewer/checks shift the 'idler' group) ----
    for i, (y_p, _, z) in enumerate(IDLER_SLOTS):
        put(f"idler_roller_{i}", on_axis(idler_roller(), -(CORRIDOR / 2 - 1), (y_p, z)), "idler", "#f2f2f2", "print")
        put(f"idler_axle_{i}_ref", on_axis(ref_cyl(IDLER_AXLE, 200.0), -100.0, (y_p, z)), "idler", "#b0b4b8", "ref")
        sp_len = SIDE_X_IN - 0.5 - (CORRIDOR / 2 - 1)            # centers the roller between the side plates
        for sx in (-1, 1):
            put(f"idler_spacer_{i}_{'s' if sx > 0 else 'm'}", on_axis(idler_spacer(), CORRIDOR / 2 - 1 if sx > 0 else -(SIDE_X_IN - 0.5), (y_p, z)),
                "idler", "#f2f2f2", "print")
        ay, az = IDLER_ANCHORS[i]
        tab_rot = math.degrees(math.atan2(az - z, ay - y_p))   # point the tab's hook at its anchor
        for sx in (-1, 1):
            x0 = SIDE_X_OUT + 0.5 if sx > 0 else -(SIDE_X_OUT + 8.5)
            put(f"idler_spool_{i}_{'s' if sx > 0 else 'm'}", on_axis(idler_spool(), x0, (y_p, z)).rotate(
                Axis((0, y_p, z), (1, 0, 0)), tab_rot), "idler", "#f2c230", "print")
            xa = SIDE_X_OUT + 0.5 if sx > 0 else -(SIDE_X_OUT + 0.5 + DRIVE_W - 1)
            put(f"idler_anchor_{i}_{'s' if sx > 0 else 'm'}", on_axis(spring_post(), xa, IDLER_ANCHORS[i]), "turret", "#f2c230", "print")

    # ---- shaft hardware from the stacks: bearings, spacers, collars ----
    for sname, st in shaft_stacks().items():
        if st["axis"] is None:
            continue
        key = {"Main flywheel shaft": "main", "Jackshaft": "jack", "Upper flywheel shaft": "upper"}[sname]
        for i, (label, sku, x0, x1, kind) in enumerate(st["items"]):
            if kind == "bearing":
                inboard = "carrier" in label                      # hood-carrier bearings: flange toward x = 0
                flange_low = (x0 >= 0) if inboard else (x0 < 0)   # flange at the low-x end of this item?
                body = ref_cyl(BRG_1611, x1 - x0 - BRG_FL, 8)
                flange = ref_cyl(15.0, BRG_FL, 8)                    # flange: 15 mm OD (goBILDA STEP)
                shape = (flange + Pos(0, 0, BRG_FL) * body) if flange_low else (body + Pos(0, 0, x1 - x0 - BRG_FL) * flange)
                put(f"bearing_{key}_{i}", on_axis(shape, x0, st["axis"]), st["group"], "#2f7fd8", "bearing")
            elif kind == "spacer":
                put(f"spacer_{key}_{i}", on_axis(ref_cyl(10.0 if sku.startswith("1522") else 10.0, x1 - x0, 8), x0, st["axis"]),
                    st["group"], "#9aa3ad", "hardware")
            elif kind == "collar":
                put(f"collar_{key}_{i}", on_axis(ref_cyl(20.0, x1 - x0, 8), x0, st["axis"]), st["group"], "#c9ced4", "hardware")
    for k in range(0 if GB_TURRET else N_ROLLERS):          # 608s inside the turret roller tires
        x, y = pol(ROLLER_R, math.radians(30 + 360 * k / N_ROLLERS))
        for j, z0 in enumerate((ROLLER_Z0, ROLLER_Z0 + B608_W + TIRE_LIP)):
            put(f"bearing_608_{k}_{j}", Pos(x, y, z0) * ref_cyl(B608_OD, B608_W, 8), "fixed", "#2f7fd8", "bearing")
    for i, (y_p, _, z) in enumerate(IDLER_SLOTS):   # 1611 6mm bearings in the idler roller ends
        for x0 in (-(CORRIDOR / 2 - 1), CORRIDOR / 2 - 6):
            put(f"bearing_idler_{i}_{'m' if x0 < 0 else 's'}", on_axis(ref_cyl(14.0, 5.0, 6), x0, (y_p, z)), "idler", "#2f7fd8", "bearing")

    if FRONT_PLATE:
        fp, yb = front_plate_2d()
        put("front_plate", Plane(origin=(0, yb, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)).location *
            Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 0, 1)).location * plate(fp, FRONT_PLATE_T), "turret", "#f2c230", "cut")

    # ---- Limelight ----
    put("ll_mount", ll_location() * ll_mount(), "ll", "#d63a2f", "print")
    put("limelight_ref", ll_location() * Pos(0, -30, 0) * limelight_ref(), "ll", "#20242a", "ref")
    add_fasteners(put)
    add_inserts(parts, put)
    return parts, plates2d

DOWELS = {}            # v7: laminated plate -> its dowel-hole spots (filled by main())
LAYERS = {}            # v7: laminated plate file name -> (layer 1, layer 2, dowel spots), filled by build()
# v7 (Astra #6): button-head bolts from the ball side need 1-2 threads (0.7 mm pitch) past their nylocs.
GUIDE_BOLT_L, GEAR_BOLT_L = 45, 16
# the counterbore goes through layer 1 only, so a head sits on layer 2's face, PLY_LAYER + PLY_GLUE deep (Astra round 2)
GUIDE_TIP = (PLY_LAYER + PLY_GLUE + GUIDE_BOLT_L) - (T + (SIDE_X_IN - X_CAR - T) + T + NYLOC_M4[1])   # past the nyloc
GEAR_TIP = (PLY_LAYER + PLY_GLUE + GEAR_BOLT_L) - (T + HOOD_GEAR_WEB + NYLOC_M4[1])
FASTENER_ACCESS = []   # (name, group, head face centre, outward tool direction, head dia): filled by add_fasteners

def add_inserts(parts, put):
    """2026-10-02: the brass heat-set inserts, found from their holes: every cylindrical hole of an insert's hole size in a
    printed part gets an insert, flush with the hole's open end (both ends open: the upper one). Drawn 0.1 mm under the
    hole size so the collision check doesn't call the pressed-in fit a clash."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_SurfaceOfRevolution
    BRASS = "#c9a227"
    sizes = {round(INSERT_HOLE_D / 2, 3): (4.7, INSERT_HOLE_D - 0.1, 4.0), round(SERVO_INS_D / 2, 3): (SERVO_INS_L, SERVO_INS_D - 0.1, 3.0)}
    for pname, (shape, group, color, kind, spin) in list(parts.items()):
        if kind != "print":
            continue
        seen, k = set(), 0
        if pname.startswith("turret_ring"):
            continue                       # its holes are polygons; added below
        for f in shape.faces():
            srf = BRepAdaptor_Surface(f.wrapped)
            if srf.GetType() == GeomAbs_Cylinder:          # holes cut with make_cylinder
                a = srf.Cylinder().Axis()
            elif srf.GetType() == GeomAbs_SurfaceOfRevolution:   # holes cut with revolve_profile: a constant radius only
                a = srf.AxeOfRevolution()
            else:
                continue
            o = Vector(a.Location().X(), a.Location().Y(), a.Location().Z())
            d = Vector(a.Direction().X(), a.Direction().Y(), a.Direction().Z()).normalized()
            pts = [Vector(v.X, v.Y, v.Z) for v in f.vertices()] + [f.center()]
            rad = [((q - o) - d * (q - o).dot(d)).length for q in pts]
            if max(rad) - min(rad) > 0.02:
                continue
            size = next((v for r0, v in sizes.items() if abs(rad[0] - r0) < 0.02
                         and (r0 != round(SERVO_INS_D / 2, 3) or pname.startswith("servo_insert"))), None)
            if not size:
                continue
            ts = [(q - o).dot(d) for q in pts]
            lo, hi = o + d * min(ts), o + d * max(ts)
            key = tuple(round(c, 1) for c in (lo + hi) * 0.5)
            if key in seen:            # a hole's wall can come as two half faces
                continue
            seen.add(key)
            side = d.cross(Vector(0, 0, 1) if abs(d.Z) < 0.9 else Vector(1, 0, 0)).normalized() * (rad[0] * 0.9)
            open_lo = not shape.is_inside(lo - d * 0.3 + side)
            open_hi = not shape.is_inside(hi + d * 0.3 + side)
            if open_lo and (not open_hi or lo.Z >= hi.Z):
                mouth, inward = lo, d
            elif open_hi:
                mouth, inward = hi, d * -1
            else:
                continue                   # closed both ends: not an insert hole
            L, od, idia = size
            pl = Plane(origin=mouth, z_dir=inward)
            ins = Solid.make_cylinder(od / 2, L, pl) - Solid.make_cylinder(idia / 2, L, pl)
            put(f"heatset_{pname}_{k}", ins, group, BRASS, "ref", spin)
            k += 1
    # parts whose insert holes are polygons (revolve_profile / 2D circles): placed from the design numbers, each checked
    # to sit in an empty hole with plastic around it
    L4, od4, id4 = sizes[round(INSERT_HOLE_D / 2, 3)]
    def explicit(pname, yz, mouth_x, inward):
        if pname not in parts:
            return
        shape, group, _, _, spin = parts[pname]
        pl = Plane(origin=(mouth_x, yz[0], yz[1]), z_dir=(inward, 0, 0))
        mid = Vector(mouth_x + inward * L4 / 2, yz[0], yz[1])
        wall = Vector(mouth_x + inward * L4 / 2, yz[0] + INSERT_HOLE_D / 2 + 0.8, yz[1])
        if shape.is_inside(mid) or not shape.is_inside(wall):
            print(f"WARNING: heat-set insert in {pname} at x {mouth_x:.1f} is not in an insert hole")
        n = sum(1 for q in parts if q.startswith(f"heatset_{pname}_"))
        put(f"heatset_{pname}_{n}", Solid.make_cylinder(od4 / 2, L4, pl) - Solid.make_cylinder(id4 / 2, L4, pl), group, BRASS, "ref", spin)
    cw = lambda p: world(p, 0.0)
    for i, (y, z) in enumerate(CROSSBARS):
        for sx in (1, -1):
            explicit(f"crossbar_{i}", (y, z), sx * SIDE_X_IN, -sx)
    if REAR_GUARD:
        for yz in (REAR_GUARD_BAR, REAR_GUARD_PIN):
            for sx in (1, -1):
                explicit("rear_guard", yz, sx * SIDE_X_IN, -sx)
    for sx, side, sd in ((1, "servo", "s"), (-1, "motor", "m")):
        c = cw(STOP_E)                     # the cam's off-centre hole (its local +x turns to world -z)
        explicit(f"stop_cam_{side}", (c[0], c[1] - CAM_ECC), sx * X_ARM, sx)
        explicit(f"arm_bumper_core_{sd}", cw(BUMPER_E), sx * (X_CAR + T), sx)
        explicit(f"cf_axle_{sd}", cw(SPRING_B[0]), sx * (X_CAR + T + CF_AXLE_L - 1.5), -sx)   # 1.5 mm below the end
    if LAMINATE and "turret_ring" in parts:          # the printed ring's holes are polygons (from its 2D outline)
        _, group, _, _, spin = parts["turret_ring"]
        L, od, idia = sizes[round(INSERT_HOLE_D / 2, 3)]
        for k, ang in enumerate(DECK_SPACER_ANGS):
            x, y = pol(DECK_SPACER_R, math.radians(ang))
            pl = Plane(origin=(x, y, RING_Z0 + T), z_dir=(0, 0, -1))
            put(f"heatset_turret_ring_{k}", Solid.make_cylinder(od / 2, L, pl) - Solid.make_cylinder(idia / 2, L, pl), group, BRASS, "ref", spin)

def add_fasteners(put):
    """v5: bolt heads and nuts as simple cylinders so check.py catches clashes. Countersunk flat heads are flush
    (their countersinks are cut into the plates) and are not added."""
    STEEL = "#6b7078"
    FASTENER_ACCESS.clear()
    def zc(name, x, y, z0, dh, group):              # vertical cylinder from z0 up by dh[1] (negative = down)
        d, h = dh
        put(name, Pos(x, y, min(z0, z0 + h)) * Solid.make_cylinder(d / 2, abs(h)), group, STEEL, "ref")
        FASTENER_ACCESS.append((name, group, (x, y, z0 + h), (0, 0, 1 if h > 0 else -1), d))
    def xc(name, x0, h, yz, d, group):              # cylinder along X from x0, length h (negative = toward -X)
        put(name, Pos(min(x0, x0 + h), yz[0], yz[1]) * along_x(Solid.make_cylinder(d / 2, abs(h))), group, STEEL, "ref")
        FASTENER_ACCESS.append((name, group, (x0 + h, yz[0], yz[1]), (1 if h > 0 else -1, 0, 0), d))
    def yc(name, y0, h, xz, d, group):              # cylinder along Y from y0, length h (negative = toward -Y)
        put(name, Solid.make_cylinder(d / 2, abs(h), Plane(origin=(xz[0], min(y0, y0 + h), xz[1]), z_dir=(0, 1, 0))),
            group, STEEL, "ref")
        FASTENER_ACCESS.append((name, group, (xz[0], y0 + h, xz[1]), (0, 1 if h > 0 else -1, 0), d))
    # countersunk flat heads are flush, but still need a hex key: record their tool paths too
    for sx, sd in ((1, "s"), (-1, "m")):
        for i, p in enumerate(CARRIER_CSK + (HOOD_GEAR_CSK if sx > 0 else [])):
            FASTENER_ACCESS.append((f"csk_carrier_{sd}_{i}", "carrier", (sx * X_CAR, *world(p)), (-sx, 0, 0), CSK_D))
    if FW_MOTOR_CSK:
        for i, p in enumerate(FW_MOTOR_BOLTS):
            FASTENER_ACCESS.append((f"csk_fw_motor_{i}", "turret", (-SIDE_X_OUT, *p), (-1, 0, 0), CSK_D))
    # fixed base
    for k in range(0 if GB_TURRET else N_ROLLERS):
        x, y = pol(ROLLER_R, math.radians(30 + 360 * k / N_ROLLERS))
        zc(f"fastener_m8_head_{k}", x, y, -T, (HEAD_M8[0], -HEAD_M8[1]), "fixed")
        zc(f"fastener_m8_nut_{k}", x, y, ROLLER_Z0 + TIRE_H + 2, NYLOC_M8, "fixed")
        # v8: the bolt's end above the nut (an M8 x 40 from under the base plate), so the turning parts are checked against it
        nut_top = ROLLER_Z0 + TIRE_H + 2 + NYLOC_M8[1]      # (put directly: a bolt end isn't a place for a hex key)
        put(f"fastener_m8_tip_{k}", Pos(x, y, nut_top) * Solid.make_cylinder(4.0, M8_BOLT_TIP - nut_top), "fixed", STEEL, "ref")
    mx, my = pol(PIN_CENTER_R, math.radians(TURRET_MOTOR_ANGLE))
    for i, (dx, dy) in enumerate([] if GB_TURRET else TURRET_BOLTS):
        zc(f"fastener_turret_motor_{i}", mx + dx, my + dy, BASE_TOP, SHCS_M4, "fixed")
    for k in range(0 if GB_TURRET else 8):
        x, y = pol(BASE_MOUNT_R, math.radians(22.5 + 45 * k))
        zc(f"fastener_robot_mount_{k}", x, y, BASE_TOP + (TRAY_T if CABLES else 0.0), BTN_M4, "fixed")
    if CABLES or HOME_FLAG:
        for i, (x, y) in enumerate(switch_holes()):
            zc(f"fastener_switch_head_{i}", x, y, switch_base_z() + 3.0, SHCS_M3, "fixed")
            zc(f"fastener_switch_nut_{i}", x, y, -(GB_ARM_T if GB_TURRET else T), (NYLOC_M3[0], -NYLOC_M3[1]), "fixed")
    # turret frame
    if GB_TURRET:
        for k, (x, y) in enumerate(GB_DECK_HOLES):
            zc(f"fastener_deck_ring_{k}", x, y, DECK_TOP, SHCS_M4, "turret")
        for k, (x, y, d) in enumerate(gb_arm_screws()):
            put(f"fastener_switch_arm_{k}", Pos(x, y, 4.0) * Rot(0, 0, HOME_ANG) * Rot(0, 90, 0) * Solid.make_cylinder(BTN_M4[0] / 2, BTN_M4[1]),
                "fixed", STEEL, "ref")
    for k, a in enumerate([] if GB_TURRET else DECK_SPACER_ANGS):
        x, y = pol(DECK_SPACER_R, math.radians(a))
        zc(f"fastener_deck_spacer_head_{k}", x, y, DECK_TOP, SHCS_M4, "turret")
        if not RING_TAPPED:
            zc(f"fastener_deck_spacer_nut_{k}", x, y, RING_Z0, (NYLOC_M4[0], -NYLOC_M4[1]), "turret")
    if HOME_FLAG:
        for k, (x, y) in enumerate(flag_screw_pts()):
            zc(f"fastener_flag_{k}", x, y, DECK_TOP, SHCS_M4, "turret")
    if CABLES:
        for k, a in enumerate(DRUM_BOLT_ANGS):
            x, y = pol(DRUM_BOLT_R, math.radians(a))
            zc(f"fastener_drum_{k}", x, y, DECK_TOP, SHCS_M4, "turret")
    for sx in (-1, 1):
        sd = "s" if sx > 0 else "m"
        for by in BRACKET_Y:
            for dy in BRKT_FOOT_DY:
                zc(f"fastener_bracket_foot_{sd}_{by + dy:.0f}", sx * (SIDE_X_OUT + BRKT_FOOT_HOLE), by + dy, DECK_TOP + 5.0, SHCS_M4, "turret")
                if not DECK_TAPPED:
                    zc(f"fastener_bracket_foot_nut_{sd}_{by + dy:.0f}", sx * (SIDE_X_OUT + BRKT_FOOT_HOLE), by + dy, DECK_Z0, (NYLOC_M4[0], -NYLOC_M4[1]), "turret")
            for dy in BRKT_SIDE_DY:
                xc(f"fastener_bracket_side_{sd}_{by + dy:.0f}", sx * (SIDE_X_OUT + 5.0), sx * SHCS_M4[1], (by + dy, DECK_TOP + 9.5), SHCS_M4[0], "turret")
                xc(f"fastener_bracket_side_nut_{sd}_{by + dy:.0f}", sx * SIDE_X_IN, -sx * NYLOC_M4[1], (by + dy, DECK_TOP + 9.5), NYLOC_M4[0], "turret")
        for i, (y, z) in enumerate(CROSSBARS):
            xc(f"fastener_crossbar_{sd}_{i}", sx * SIDE_X_OUT, sx * SHCS_M4[1], (y, z), SHCS_M4[0], "turret")
        if REAR_GUARD:
            xc(f"fastener_crossbar_{sd}_guardpin", sx * SIDE_X_OUT, sx * SHCS_M4[1], REAR_GUARD_PIN, SHCS_M4[0], "turret")
        xc(f"fastener_ll_pivot_{sd}", sx * SIDE_X_OUT, sx * SHCS_M4[1], LL_PIVOT, SHCS_M4[0], "turret")
        _, up = ll_dirs(LL_TILT)
        rl = LL_LOCK_R[1] if int(LL_TILT) % 10 == 0 else LL_LOCK_R[0]
        xc(f"fastener_ll_lock_{sd}", sx * SIDE_X_OUT, sx * SHCS_M4[1], add(LL_PIVOT, up, -rl), SHCS_M4[0], "ll")
        for gi, g in enumerate((GUIDE_1, GUIDE_2)):      # guide bolts: flat head inside (countersunk), nyloc outside
            xc(f"fastener_guide_nut_{sd}_{gi}", sx * SIDE_X_OUT, sx * (NYLOC_M4[1] + (GUIDE_TIP if LAMINATE else 0)), world(g), NYLOC_M4[0], "carrier")
        if LAMINATE:   # 2026-10-02: the button heads in each carrier's ball-side counterbores (flush; drawn so they show)
            for k, (what, p) in enumerate((("guide", GUIDE_1), ("guide", GUIDE_2), ("sleeve", SPRING_B[0]), ("cam", STOP_E), ("bumper", BUMPER_E))):
                put(f"fastener_carrier_{what}_{sd}_{k}", Pos(min(sx * X_CAR, sx * (X_CAR + BTN_M4[1])), *world(p)) *
                    along_x(Solid.make_cylinder(BTN_M4[0] / 2, BTN_M4[1])), "carrier", STEEL, "ref")
            if sx > 0:
                for i, b in enumerate(HOOD_GEAR_BOLTS):
                    put(f"fastener_hood_gear_head_{i}", Pos(X_CAR, *world(b)) * along_x(Solid.make_cylinder(BTN_M4[0] / 2, BTN_M4[1])),
                        "carrier", STEEL, "ref")
        if sx > 0:   # v6: nylocs on the bought hood gear's 4 bolts (flat heads countersunk on the ball side)
            for i, b in enumerate(HOOD_GEAR_BOLTS):
                xc(f"fastener_hood_gear_nut_{i}", X_CAR + T + HOOD_GEAR_WEB, NYLOC_M4[1] + (GEAR_TIP if LAMINATE else 0), world(b), NYLOC_M4[0], "carrier")
    if MOTOR_ADJ:   # motor screws: heads on the adapter, inside the side-plate window (reach them with the 16T pulley off)
        for i, p in enumerate(MOTOR_BOLTS):
            xc(f"fastener_fw_motor_{i}", -SIDE_X_IN, -BTN_M4[1], motor_rel(p), BTN_M4[0], "turret")
        for i, p in enumerate(CLAMP_BOLTS):
            xc(f"fastener_motor_clamp_{i}", -SIDE_X_OUT, -BTN_M4[1], motor_rel(p), BTN_M4[0], "turret")
        for i, p in enumerate(LUG_BOLTS):
            xc(f"fastener_motor_clamp_lug_{i}", -(SIDE_X_OUT + LUG_H), -BTN_M4[1], motor_rel(p), BTN_M4[0], "turret")
        for i, p in enumerate(YOKE_SCREWS):
            xc(f"fastener_motor_yoke_{i}", -(SIDE_X_OUT + YOKE_H), -SHCS_M4[1], motor_rel(p), SHCS_M4[0], "turret")
        yc("fastener_motor_jack", motor_rel((YOKE_BRIDGE[0], 0.0))[0], -BTN_M4[1],
           (-(SIDE_X_OUT + JACK_H), motor_rel((0.0, JACK_DZ))[1]), BTN_M4[0], "turret")
    else:
        for i, (dy, dz) in enumerate([] if FW_MOTOR_CSK else MOTOR_BOLTS):   # flywheel motor, from outside
            xc(f"fastener_fw_motor_{i}", -SIDE_X_OUT, -BTN_M4[1], (FW_MOTOR[0] + dy, FW_MOTOR[1] + dz), BTN_M4[0], "turret")
    sy, sz = SERVO_AXIS
    c = sy + (-1 if SERVO_BODY_FWD else 1) * 10.1
    for i, (e1, e2) in enumerate([(e1, e2) for e1 in (-1, 1) for e2 in (-5, 5)]):   # hood servo ears
        hd = SHCS_M3 if SERVO_INSERT else SHCS_M4
        xc(f"fastener_servo_{i}", SIDE_X_OUT + 2.5, hd[1], (c + e1 * 24.75, sz + e2), hd[0], "turret")
        if not SERVO_TAPPED:
            xc(f"fastener_servo_nut_{i}", SIDE_X_IN, -NYLOC_M4[1], (c + e1 * 24.75, sz + e2), NYLOC_M4[0], "turret")
    # v6.1: spring anchors: flat head countersunk in each arm's inner face (flush), and the button head that holds the
    # stacked spring ends to the anchor's side face (axis along the arm)
    for sx, sd in ((1, "s"), (-1, "m")):
        FASTENER_ACCESS.append((f"csk_arm_anchor_{sd}", "arm", (sx * X_ARM, *world(ARM_CSK[0])), (-sx, 0, 0), CSK_D))
        x_mid = sx * (X_ARM + T + STRIP_Z)
        hp = add(SPRING_A, U_ARM, -0.6 - CLAMP_T)                     # on the clamp plate, facing back along the arm
        c0 = world(hp); c1 = world(add(hp, U_ARM, -BTN_M4[1]))
        d = (0.0, c1[0] - c0[0], c1[1] - c0[1]); Ld = math.hypot(d[1], d[2])
        put(f"fastener_spring_end_{sd}", Solid.make_cylinder(BTN_M4[0] / 2, Ld, Plane(origin=(x_mid, *c0), z_dir=d)), "arm", STEEL, "ref")
        FASTENER_ACCESS.append((f"fastener_spring_end_{sd}", "arm", (x_mid, *c1), (0.0, d[1] / Ld, d[2] / Ld), BTN_M4[0]))
    # v6: Sonic Hub screws (button heads on the part's inboard face, toward the bearing)
    for i, (dy, dz) in enumerate(HUB_SCREWS):
        xc(f"fastener_hub_pulley48_{i}", -(SIDE_X_OUT + BRG_FL + HUB_HEAD_GAP), BTN_M4[1], (FW_Y + dy, FW_Z + dz), BTN_M4[0], "turret")
        jw, sw = world(J), world(S_P)
        xc(f"fastener_hub_gear60_{i}", X_DRV + HUB_HEAD_GAP, -BTN_M4[1], (jw[0] + dy, jw[1] + dz), BTN_M4[0], "carrier")
        xc(f"fastener_hub_gear48_{i}", X_DRV + HUB_HEAD_GAP, -BTN_M4[1], (sw[0] + dy, sw[1] + dz), BTN_M4[0], "arm")
    if ENCODER:   # bracket legs: bolt from inside the side plate into the bracket's heat-set inserts
        for i, (r, a) in enumerate(BRKT_LEGS):
            ly, lz = pol(r, math.radians(a))
            xc(f"fastener_encoder_leg_{i}", -SIDE_X_IN, SHCS_M4[1], (FW_Y + ly, FW_Z + lz), SHCS_M4[0], "turret")

# =====================================================================
#                          BILL OF MATERIALS
# SKUs verified on vendor product pages 2026-09-23 (see README). "(verify)" = spec not published.
# =====================================================================
def shaft_sku(need):
    lengths = [24, 32, 40, 43, 48, 52, 54, 56, 64, 72, 80, 88, 96, 104, 112, 120, 144, 168, 192, 216, 240, 264, 288]
    L = min(l for l in lengths if l >= need)
    return L, f"2106-4008-{L * 10:04d}"

# =====================================================================
#                            SHAFT STACKS
# Every item on each shaft, in order, with the gaps filled from goBILDA spacers.
# Convention (v5, after the Codex review): a 1611's 4 mm body sits in the plate's 14 mm hole and its 1 mm flange
# sits OUTSIDE the plate, against the face. Side plates and swing arms: flange on the outer face. Hood carriers:
# flange on the INNER (ball-corridor) face, because the carrier's outer face is only 1 mm from the swing arm and a
# flange there would rub the arm at the jackshaft. Spacers next to bearings are 10mm-OD steel (1522) so they
# touch only the inner race. Parts with set screws (pulleys, pinions, Sonic Hubs) double as end stops.
# Widths marked (verify) come from assumptions: goBILDA doesn't publish them.
# =====================================================================
BRG_W = 5.0            # 1611 bearing width incl. flange (goBILDA: "5mm thickness")
BRG_FL = 1.0           # flange thickness (unverified)
COLLAR_W = 9.5         # 2910-0920-4008 (measured: 9.5)
SPACERS = [(12, "1522-0010-0120"), (9, "1522-0010-0090"), (8, "1522-0010-0080"), (7, "1522-0010-0070"),
           (6, "1522-0010-0060"), (4, "1522-0010-0040"), (3, "1522-0010-0030"), (2, "1522-0010-0020"),
           (1, "1500-0010-0008")]
SPACER_LEN = {sku: L for L, sku in SPACERS}

def fill_gap(g, max_play=0.9):
    """Fewest spacers whose total is within (g - max_play, g]; leaves a little end play."""
    import itertools as it
    lens = [l for l, _ in SPACERS]
    for n in range(0, 5):
        best = None
        for combo in it.combinations_with_replacement(lens, n):
            t = sum(combo)
            if g - max_play < t <= g + 1e-6 and (best is None or t > sum(best)):
                best = combo
        if best is not None:
            return [next((l, sku) for l, sku in SPACERS if l == c) for c in sorted(best, reverse=True)]
    return None

MOTOR_BELT_X = SIDE_X_OUT + BRG_FL + HUB_HEAD_GAP + PUL48_W / 2   # motor belt centre plane (motor side, positive)
MAIN_HUB_X = SIDE_X_OUT + BRG_FL + HUB_HEAD_GAP + PUL48_W   # 48T pulley's hub starts here, outboard of the pulley
JACK_BELT_X = X_DRV + 0.5 + PUL_SS_W / 2                 # jack belt centre plane (motor side)

def encoder_x():
    """x (outboard, as positive distances) of: collar outer face, bracket plate, encoder body, sleeve end."""
    col0 = MAIN_HUB_X + HUB_W + COLLAR_W
    b0, b1 = col0 + 1.0, col0 + 1.0 + BRKT_T
    e0, e1 = b1 + 0.5, b1 + 0.5 + ENC_T
    return col0, (b0, b1), (e0, e1), e1 + 0.5

def encoder_2d():
    """Through Bore Encoder V2 outline in (Y, Z) about the bore; body points up."""
    ears = [pol(ENC_EAR_R, math.radians(a)) for a in (ENC_EAR_ANG, ENC_EAR_ANG + 180)]
    body = unary_union([Point(0, 0).buffer(ENC_W / 2), box(-ENC_W / 2, 0, ENC_W / 2, ENC_LEN)] +
                       [LineString([(0, 0), e]).buffer(5.5).union(Point(*e).buffer(5.5)) for e in ears])
    return body.difference(unary_union([circ(*e, 5.0) for e in ears] + [Point(0, 0).buffer(6.4)]))

def encoder_bracket_2d():
    """Printed plate: carries the encoder's two ears; two legs run back to the motor-side plate."""
    ears = [pol(ENC_EAR_R, math.radians(a)) for a in (ENC_EAR_ANG, ENC_EAR_ANG + 180)]
    legs = [pol(r, math.radians(a)) for r, a in BRKT_LEGS]
    body = unary_union([Point(*p).buffer(7) for p in ears + legs]).convex_hull
    holes = [Point(0, 0).buffer(17.0)] + [circ(*e, M4) for e in ears] + [circ(*l, M4) for l in legs]   # v6.1: ears M4 + nyloc (4 mm is too thin for an insert)
    return body.difference(unary_union(holes))

def shaft_stacks():
    """{shaft: {"axis": (Y,Z) or None, "group": ..., "items": [(label, sku, x0, x1, kind)]}} in turret X."""
    pw = DRIVE_W - 2
    side_r = (SIDE_X_OUT - BRG_W + BRG_FL, SIDE_X_OUT + BRG_FL)   # side plate, flange outside (servo side, +X)
    car_r = (X_CAR - BRG_FL, X_CAR - BRG_FL + BRG_W)              # hood carrier, flange on the ball-corridor face
    arm_r = (X_ARM + T - BRG_W + BRG_FL, X_ARM + T + BRG_FL)      # swing arm, flange outside
    def mirror(a, b): return (-b, -a)
    wstack = lambda d: [(f"4x M4 x 80 socket heads (wheel-stack screws)", "generic", -FW1_W / 2 - WHEEL_SCREW_HEAD, -FW1_W / 2, "hardware"),
                        (f"{GECKO_N}x {d}mm Gecko wheels, bolted through their cores", f"3613-0014-00{d}", -FW1_W / 2, FW1_W / 2, "wheel"),
                        ("Sonic Hub (clamps the shaft; the wheel screws thread into it)", "1309-0016-4008", FW1_W / 2, FW1_W / 2 + HUB_W, "hub")]
    main = [("Collar", "2910-0920-4008", -(MAIN_HUB_X + HUB_W + COLLAR_W), -(MAIN_HUB_X + HUB_W), "collar"),
            ("Sonic Hub (clamps the shaft; boss centers the pulley)", "1309-0016-4008", -(MAIN_HUB_X + HUB_W), -MAIN_HUB_X, "hub"),
            ("48T HTD hub-mount pulley (4x M4 x 20 button heads into the hub; heads toward the bearing)", "3415-0014-0048", -MAIN_HUB_X, -(MAIN_HUB_X - PUL48_W), "pulley"),
            ("1611 bearing in motor-side plate", "1611-0514-4008", *mirror(*side_r), "bearing"),
            ("24T HTD pulley (set screw)", "3417-4008-0024", -(JACK_BELT_X + PUL_SS_W / 2), -(JACK_BELT_X - PUL_SS_W / 2), "pulley"),
            ("1611 bearing in motor-side hood carrier", "1611-0514-4008", *mirror(*car_r), "bearing"),
            *wstack(96),
            ("1611 bearing in servo-side hood carrier", "1611-0514-4008", *car_r, "bearing"),
            ("1611 bearing in servo-side plate", "1611-0514-4008", *side_r, "bearing"),
            *([("48T HTD pulley + Sonic Hub (motor 2)", "3415-0014-0048 + 1309-0016-4008", SIDE_X_OUT + 2, SIDE_X_OUT + 2 + pw, "pulley"),
               ("Collar", "2910-0920-4008", SIDE_X_OUT + 2 + pw, SIDE_X_OUT + 2 + pw + COLLAR_W, "collar")] if SECOND_MOTOR else
              [("Collar", "2910-0920-4008", SIDE_X_OUT + BRG_FL, SIDE_X_OUT + BRG_FL + COLLAR_W, "collar")])]
    if ENCODER:
        col0, (b0, b1), (e0, e1), send = encoder_x()
        main.insert(0, ("Printed 1/2in-hex sleeve (drives the encoder rotor; bracket plate + Through Bore Encoder V2 ride on it)",
                        "encoder_sleeve.stl + REV-11-3174", -send, -col0, "printed"))
        main.insert(0, ("Screw + washer into the shaft's factory threaded end (keep the UNCUT end here; thread: verify)",
                        "generic", -send - 3.0, -send, "nut"))
    jack = [("24T HTD pulley (set screw, end stop)", "3417-4008-0024", -(JACK_BELT_X + PUL_SS_W / 2), -(JACK_BELT_X - PUL_SS_W / 2), "pulley"),
            ("1611 bearing in motor-side swing arm", "1611-0514-4008", *mirror(*arm_r), "bearing"),
            ("1611 bearing in motor-side hood carrier", "1611-0514-4008", *mirror(*car_r), "bearing"),
            ("(open: ball corridor)", "", -car_r[0], car_r[0], "open"),
            ("1611 bearing in servo-side hood carrier", "1611-0514-4008", *car_r, "bearing"),
            ("1611 bearing in servo-side swing arm", "1611-0514-4008", *arm_r, "bearing"),
            ("60T hub-mount gear (4x M4 x 10 button heads into the hub; heads toward the bearing)", "2312-0414-0060", X_DRV + HUB_HEAD_GAP, X_DRV + HUB_HEAD_GAP + GEAR_T, "gear"),
            ("Sonic Hub (clamps the shaft, end stop)", "1309-0016-4008", X_DRV + HUB_HEAD_GAP + GEAR_T, X_DRV + HUB_HEAD_GAP + GEAR_T + HUB_W, "hub")]
    upper = [("Collar", "2910-0920-4008", -arm_r[1] - COLLAR_W, -arm_r[1], "collar"),
             ("1611 bearing in motor-side swing arm", "1611-0514-4008", *mirror(*arm_r), "bearing"),
             *wstack(72),
             ("1611 bearing in servo-side swing arm", "1611-0514-4008", *arm_r, "bearing"),
             ("48T hub-mount gear (4x M4 x 10 button heads into the hub; heads toward the bearing)", "2312-0414-0048", X_DRV + HUB_HEAD_GAP, X_DRV + HUB_HEAD_GAP + GEAR_T, "gear"),
             ("Sonic Hub (clamps the shaft, end stop)", "1309-0016-4008", X_DRV + HUB_HEAD_GAP + GEAR_T, X_DRV + HUB_HEAD_GAP + GEAR_T + HUB_W, "hub")]
    stacks = {"Main flywheel shaft": {"axis": (FW_Y, FW_Z), "group": "turret", "items": main},
              "Jackshaft": {"axis": world(J), "group": "carrier", "items": jack},
              "Upper flywheel shaft": {"axis": world(S_P), "group": "arm", "items": upper}}
    for st in stacks.values():   # insert spacers into every gap
        out, items = [], st["items"]
        for a, b in zip(items, items[1:]):
            out.append(a)
            g = b[2] - a[3]
            if a[4] == "open" or b[4] == "open" or g < 0.5:
                continue
            fill = fill_gap(g)
            x = a[3]
            for l, sku in fill or []:
                out.append((f"Spacer {l}mm" + (" (steel, 10mm OD)" if sku.startswith("1522") else " (8x10x1)"), sku, x, x + l, "spacer"))
                x += l
            if fill is None:
                out.append((f"GAP {g:.2f}mm: no spacer combo", "", a[3], b[2], "gap"))
            elif b[2] - x > 0.05:
                out.append((f"end play {b[2] - x:.2f}mm", "", x, b[2], "play"))
        out.append(items[-1])
        st["items"] = out
        st["length"] = out[-1][3] - out[0][2]
    if IDLERS:
      stacks["Idler axles (x2)"] = {"axis": None, "group": "idler", "length": 200.0, "items": [
          ("Idler tab (M3 set screw), outboard", "idler_spool.stl", -(SIDE_X_OUT + 8.5), -(SIDE_X_OUT + 0.5), "printed"),
          ("slot in motor-side plate", "", -SIDE_X_OUT, -SIDE_X_IN, "plate"),
          ("Idler spacer tube", "idler_spacer.stl", -(SIDE_X_IN - 0.5), -(CORRIDOR / 2 - 1), "spacer"),
          ("1611 bearing (6mm) in roller end", "1611-0514-0006", -(CORRIDOR / 2 - 1), -(CORRIDOR / 2 - 6), "bearing"),
          ("Idler roller body", "idler_roller.stl", -(CORRIDOR / 2 - 6), CORRIDOR / 2 - 6, "printed"),
          ("1611 bearing (6mm) in roller end", "1611-0514-0006", CORRIDOR / 2 - 6, CORRIDOR / 2 - 1, "bearing"),
          ("Idler spacer tube", "idler_spacer.stl", CORRIDOR / 2 - 1, SIDE_X_IN - 0.5, "spacer"),
          ("slot in servo-side plate", "", SIDE_X_IN, SIDE_X_OUT, "plate"),
          ("Idler tab (M3 set screw), outboard", "idler_spool.stl", SIDE_X_OUT + 0.5, SIDE_X_OUT + 8.5, "printed")]}
    stacks["Turret rollers (x6): vertical, mm up from the base-plate top; M8 x 40 bolt"] = {"axis": None, "group": "fixed",
                                                    "length": ROLLER_Z0 + TIRE_H + 11 - (-T - 5), "items": [
        ("M8 x 40 bolt head under base plate", "generic", -T - 5, -T, "bolt"),
        ("Base plate", "base_plate_L1.dxf + base_plate_L2.dxf" if LAMINATE else "base_plate.dxf", -T, 0, "plate"),
        ("Roller spacer (inner race only)", "roller_spacer.stl", 0, ROLLER_SPACER_H, "spacer"),
        ("608 bearing", "1600-0722-0008", ROLLER_Z0, ROLLER_Z0 + B608_W, "bearing"),
        ("tire center lip (outer race only) + inner-race spacer inside it", "roller_tire.stl + roller_mid_spacer.stl", ROLLER_Z0 + B608_W, ROLLER_Z0 + B608_W + TIRE_LIP, "printed"),
        ("608 bearing", "1600-0722-0008", ROLLER_Z0 + B608_W + TIRE_LIP, ROLLER_Z0 + TIRE_H, "bearing"),
        ("Roller cap spacer (inner race only)", "roller_cap.stl", ROLLER_Z0 + TIRE_H, ROLLER_Z0 + TIRE_H + 2, "spacer"),
        ("M8 washer + nyloc", "generic", ROLLER_Z0 + TIRE_H + 2, ROLLER_Z0 + TIRE_H + 11, "nut")]}
    return stacks

def spacer_counts():
    counts = {}
    mult = {"Main flywheel shaft": 1, "Jackshaft": 1, "Upper flywheel shaft": 1}
    for name, st in shaft_stacks().items():
        if name not in mult:
            continue
        for label, sku, x0, x1, kind in st["items"]:
            if kind == "spacer":
                counts[sku] = counts.get(sku, 0) + mult[name]
    return counts

def write_stacks(path):
    lines = ["# Shaft stacks (generated by turret.py)", "",
             "Positions are mm along the shaft from the turret centerline: negative = motor side, positive = servo side.",
             "1611 bearings: flange OUTWARD in the side plates and swing arms; flange toward the BALL CORRIDOR in the hood carriers",
             "(so it can't rub the swing arm). Spacers next to bearings are 10 mm across so they only touch the inner race.",
             "The turret-roller table is vertical: mm up from the base-plate top.", ""]
    for name, st in shaft_stacks().items():
        lines += [f"## {name}", "", f"Stack length {st['length']:.1f} mm", "", "| # | Item | Part / file | From | To | Width |", "|---|---|---|---|---|---|"]
        for i, (label, sku, x0, x1, kind) in enumerate(st["items"], 1):
            lines.append(f"| {i} | {label} | {tag_stl(sku)} | {x0:.2f} | {x1:.2f} | {x1 - x0:.2f} |")
        lines.append("")
    path.write_text("\n".join(lines))

def bom():
    _mi = [i for i in shaft_stacks()['Main flywheel shaft']['items'] if i[4] != "nut"]
    main_cut = math.floor(_mi[-1][3] - _mi[0][2] - 0.5)       # 0.5mm short so the end washer clamps the sleeve
    main_L, main_sku = shaft_sku(main_cut)
    jack_cut = math.floor(2 * (X_DRV + DRIVE_W) - 1)            # must clear the side plates as the hood tilts
    jack_L, jack_sku = shaft_sku(jack_cut)
    n_collars = sum(1 for n_, st in shaft_stacks().items() if n_ in ("Main flywheel shaft", "Jackshaft", "Upper flywheel shaft")
                    for it in st["items"] if it[4] == "collar")
    rows = [  # (section, qty, part, SKU / file, note)
        *([] if LAMINATE else [
        ("Laser / CNC 6.35 mm", 1, "Base plate (fixed)", "base_plate.dxf", ""),
        ("Laser / CNC 1/8in (3.2 mm)", 2, f"Turret ring gear {RING_TEETH}T MOD {RING_MODULE} (two layers, stacked)", "turret_ring.dxf",
         "teeth too fine for a cutting service to cut from 1/4in aluminum -> cut TWO from 1/8in 6061 and stack them, or 3D-print (for testing). "
         "Tap (cut threads in) the 8 deck-spacer holes M4 (use heat-set inserts if printed)"),
        ("Laser / CNC 6.35 mm", 1, "Turret deck", "deck.dxf", "TAP the 8 L-bracket holes M4 (layer TAP_M4)"),
        ("Laser / CNC 6.35 mm", 1, "Side plate, servo side", "side_plate_servo.dxf", "TAP the 4 servo-ear holes M4 (layer TAP_M4)" if SERVO_TAPPED else ""),
        ("Laser / CNC 6.35 mm", 1, "Side plate, motor side", "side_plate_motor.dxf",
         "motor window + 4 clamp slots; TAP the 2 jack-yoke holes M4 (layer TAP_M4)" if MOTOR_ADJ else
         ("COUNTERSINK the 4 flywheel-motor holes on the outside face (layer CSK_M4)" if FW_MOTOR_CSK else "")),
        *([("Laser / CNC 1/8in (3.2 mm)", 1, "Flywheel motor adapter plate", "motor_adapter.dxf",
            "6061; TAP the 4 clamp holes M4 (layer TAP_M4). Slides in the side plate's slots to tighten the motor belt")] if MOTOR_ADJ else []),
        ("Laser / CNC 6.35 mm", 2, "Hood carrier (no teeth; the bought 108T gear bolts on)", "hood_carrier.dxf",
         "COUNTERSINK on the ball-side face (layer CSK_M4): 5 holes on both parts + the 4 gear holes on the servo-side part"),
        ("Laser / CNC 6.35 mm", 2, "Swing arm", "swing_arm.dxf", ""),
        *([("Laser, 3 mm (diode OK)", 1, "Front flair plate (optional)", "front_plate.dxf",
             "3 mm basswood/plywood or print it; 2 zip ties around the front crossbar; sticker space ~150 x 60 mm. Not a ROBOT SIGN")] if FRONT_PLATE else []),
        ("Laser / CNC 6.35 mm", 1, "Belt test plate, motor belt" + (" (optional)" if MOTOR_ADJ else " (cut FIRST)"), "belt_test_motor.dxf",
         "optional: the motor mount is adjustable (+/-1.4 mm of belt center distance). Cut it only to double-check the " + f"{MOTOR_BELT} mm belt"
         if MOTOR_ADJ else "same stock + cutter as the real plates; see 'Belt fit test'"),
        ("Laser / CNC 6.35 mm", 1, "Belt test plate, jackshaft belt (cut FIRST)", "belt_test_jack.dxf", "same stock + cutter as the real plates; see 'Belt fit test'"),
        ]),
        # v7: the frame is laser-cut 1/8in Baltic birch, each plate two layers glued with the grain crossed (see "Plywood frame")
        *([("Laser, 1/8in (3 mm) Baltic birch", 1, "Fit test coupon (cut FIRST)", "fit_test.dxf",
            "bearing holes, square-nut pockets and dowel holes in 0.1 mm steps: pick the sizes that fit YOUR laser, then set them in turret.py"),
           *([] if GB_TURRET else [("Laser, 1/8in (3 mm) Baltic birch", 2, "Base plate (fixed), 2 layers", "base_plate_L1.dxf + base_plate_L2.dxf", "glue, 3 dowels")]),
           ("Laser, 1/8in (3 mm) Baltic birch", 2, "Turret deck, 2 layers", "deck_L1.dxf + deck_L2.dxf",
            "press 8 M4 square nuts into the bottom layer's slots (L-bracket feet), then glue"),
           ("Laser, 1/8in (3 mm) Baltic birch", 2, "Side plate, servo side, 2 layers", "side_plate_servo_L1.dxf + side_plate_servo_L2.dxf",
            "set the printed servo insert's lip into the inner layer's recess, then glue: the insert is trapped"),
           ("Laser, 1/8in (3 mm) Baltic birch", 2, "Side plate, motor side, 2 layers", "side_plate_motor_L1.dxf + side_plate_motor_L2.dxf",
            "press 2 M4 square nuts (jack yoke) into the inner layer, then glue"),
           ("Laser, 1/8in (3 mm) Baltic birch", 2, "Hood carrier, servo side, 2 layers", "hood_carrier_servo_L1.dxf + hood_carrier_servo_L2.dxf",
            "the ball-side layer has 9 button-head counterbores (5 + 4 hood gear)"),
           ("Laser, 1/8in (3 mm) Baltic birch", 2, "Hood carrier, motor side, 2 layers", "hood_carrier_motor_L1.dxf + hood_carrier_motor_L2.dxf",
            "the ball-side layer (L2) has 5 button-head counterbores"),
           *([] if GB_TURRET else [("3D print", 1, f"Turret ring gear {RING_TEETH}T MOD {RING_MODULE} (PA-CF)", "turret_ring_print.stl",
            "8 M4 heat-sets for the deck-spacer screws. Print it flat, hardened 0.4 mm nozzle, 0.12 mm layers, dry filament. "
            "It's a wear part: print a spare. To swap it, take out the two slotted rollers, lift the turret off, and unscrew the 8 deck spacers")]),
           *([("Laser, 3 mm (diode OK)", 1, "Front flair plate (optional)", "front_plate.dxf",
               "3 mm basswood/plywood or print it; 2 zip ties around the front crossbar; sticker space ~150 x 60 mm. Not a ROBOT SIGN")] if FRONT_PLATE else []),
           ("Laser, 1/8in (3 mm) Baltic birch", 2, "Belt test plate, jackshaft belt (cut FIRST)", "belt_test_jack.dxf", "cut it twice and glue the two, like the real plates; see 'Belt fit test'"),
           ("3D print", 2, "Swing arm (PA-CF)", "swing_arm_servo.stl + swing_arm_motor.stl",
            "printed, not cut: its two bearing seats must be exact. The spring-anchor countersink is printed in"),
           ("3D print", 1, "Servo insert (PA-CF)", "servo_insert.stl",
            "4 M3 heat-sets (McMaster 94459A130) for the servo ears; its lip sits in the inner layer's recess, glued, behind the outer layer"),
           *([("3D print", 1, "Flywheel motor adapter (PA-CF)", "motor_adapter.stl",
               "4 M4 square nuts in the pockets on the motor side. Slides in the side plate's slots to tighten the motor belt")] if MOTOR_ADJ else []),
           ] if LAMINATE else []),
        *([("goBILDA", 1, "Gear-Driven Turret (2.75:1, 105 mm ID)", GB_SKU,
            "the turret bearing + its 64T drive gear, 1-side 2-post motor mount and screws. Bolt its housing to the robot through the tapped holes in its flat sides"),
           ("3D print", 1, "Deck shim (on the turret bearing's ring)", "deck_shim.stl", "PA-CF (clamped under the 8 deck screws), 1.2 mm; keeps the deck 2.2 mm above the 64T drive gear"),
           ("3D print", 1, "Home switch arm", "switch_arm.stl",
            "PA-CF (clamped under screws); screws to the bearing housing's flat at -45 deg (2x M4 x 10 button head into its tapped holes); the switch sits on its floor")] if GB_TURRET else [
        ("3D print", 6, "Grooved roller tire (center lip)", "roller_tire.stl", "2x 608 each"),
        ("3D print", 6, "Roller spacer, inner race only", "roller_spacer.stl", ""),
        ("3D print", 6, "Roller cap spacer, inner race only", "roller_cap.stl", ""),
        ("3D print", 6, "Roller inner-race spacer, 1 mm", "roller_mid_spacer.stl", "between each roller's two 608 bearings, inside the tire lip (or a 1 mm shim: 8 mm hole, 12 mm or less across)")]),
        ("3D print", 4, "Guide spacer (8 mm tube)", "guide_spacer.stl", f"{SIDE_X_IN - X_CAR - T:.1f} mm long, on the hood guide bolts"),
        *([] if GB_TURRET else [("3D print", 8, "Deck spacer", "deck_spacer.stl", "")]),
        ("3D print", 2, "Constant-force spring drum", "cf_spring_drum.stl", "spins on the sleeve below"),
        ("3D print", 2, "Spring-drum sleeve", "cf_axle.stl", "M4 heat-set in the outer end; " + ("M4 x 25 button head from the ball side (head in the carrier's counterbore)" if LAMINATE else "M4 x 30 flat head from the ball side")),
        ("3D print", 2, "Eccentric stop cam", "stop_cam.stl", "M4 heat-set insert in the off-center hole; " + ("M4 x 10 button head" if LAMINATE else "flat head") + " from the ball side (adjust without taking anything apart)"),
        ("3D print", 2, "Arm bumper stop, core (PETG)", "arm_bumper_core.stl", "M4 heat-set; " + ("M4 x 10 button head" if LAMINATE else "M4 x 12 flat head") + " from the ball side"),
        ("3D print", 2, "Arm bumper stop, ring (TPU 95A)", "arm_bumper_ring.stl", "presses onto the core; the swing arm lands on it"),
        ("3D print", 4, "L-bracket", "l_bracket.stl", ""),
        ("3D print", 3 - (1 if REAR_GUARD else 0), "Crossbar spacer", "crossbar.stl", "M4 heat-set each end"),
        *([("3D print", 1, "Rear ball guard (it is also the rear crossbar)", "rear_guard.stl",
            "4 M4 heat-sets (2 each end); print it lying on its front face; the wall stops balls dropping behind the feed hole")] if REAR_GUARD else []),
        ("3D print", 2, "Spring clamp plate", "cf_clamp.stl", "goes under the spring-end screw; the spring strips bend over its rounded edge"),
        ("3D print", 2, "Spring anchor block", "cf_anchor.stl",
         "on each arm tail; 2x M4 heat-set (one for the arm screw, one on the side face for the spring ends)"),
        *([("3D print", 1, "Motor tension lug", "motor_lug.stl", "PETG; M4 heat-set in the front face for the jack screw"),
           ("3D print", 1, "Motor jack yoke", "motor_yoke.stl", "PETG; 2x M4 x 16 into the " + ("square nuts in the side plate" if LAMINATE else "tapped side plate"))] if MOTOR_ADJ else []),
        ("3D print", 1, "Limelight 3A tilt mount", "limelight_mount.stl", "M4 heat-set x6"),
        ("goBILDA", 2 if SECOND_MOTOR else 1, "Yellow Jacket 6000 rpm (1:1), flywheels", "5203-2402-0001", "each 3:1 belted -> 2000 rpm max"),
        (("goBILDA", 1, f"Yellow Jacket {GB_MOTOR_RPM} rpm, turret", GB_MOTOR_SKU, f"{GB_RATIO:.2f}:1 (64T to the bearing's 176T) -> {GB_MOTOR_RPM / GB_RATIO:.0f} rpm") if GB_TURRET else
         ("goBILDA", 1, "Yellow Jacket 1150 rpm, turret", "5203-2402-0005", f"{RING_TEETH / PINION_TEETH:.0f}:1 ring -> {1150 * PINION_TEETH / RING_TEETH:.0f} rpm")),
        ("goBILDA", 1, "2000 Series Dual Mode Servo 25-2 (Torque)", "2000-0025-0002", "300 deg mode; hood"),
        ("goBILDA", 1, "8mm REX servo shaft, 25T spline", "1922-0025-0036", ""),
        ("goBILDA", 1 if GB_TURRET else 2, "Steel MOD 0.8 pinion 20T, 8mm REX", "2303-4008-0020", "hood servo" if GB_TURRET else "hood servo + turret"),
        ("goBILDA", 1, "Aluminum MOD 0.8 hub-mount gear 108T, 32mm bore", "2302-0032-0108",
         "hood gear (bought): 4x M4 " + (f"x {GEAR_BOLT_L} button head in counterbores" if LAMINATE else "flat head") + " from the ball side through the servo-side carrier + nylocs; 5.4:1 with the 20T servo pinion"),
        ("goBILDA", 1, "Acetal MOD 0.8 hub-mount gear 60T", "2312-0414-0060", "jackshaft"),
        ("goBILDA", 1, "Acetal MOD 0.8 hub-mount gear 48T", "2312-0414-0048", "upper wheel: 60:48 = ~6% slower surface -> mild backspin"),
        *([] if PRINT_DRIVE else [("goBILDA", 2, "HTD 5mm pulley 24T, 8mm REX", "3417-4008-0024", "main + jackshaft")]),
        ("goBILDA", 2 if SECOND_MOTOR else 1, "HTD 5mm pulley 16T, 8mm REX", "3417-4008-0016", "flywheel motor(s)"),
        *([] if PRINT_DRIVE else [("goBILDA", 2 if SECOND_MOTOR else 1, "HTD 5mm pulley 48T, 14mm hub-mount", "3415-0014-0048", "main shaft, outboard")]),
        *([("3D print", 2, "HTD 5M pulley 24T, 8mm REX (PA-CF)", "pulley_24t.stl",
            "main shaft + jackshaft; an M3 x 6 set screw with an M3 nut in the slot. Backup: goBILDA 3417-4008-0024"),
           ("3D print", 2 if SECOND_MOTOR else 1, "HTD 5M pulley 48T, hub-mount (PA-CF)", "pulley_48t.stl",
            "main shaft, outboard, bolted to its Sonic Hub (4x M4 x 20 button heads). Backup: goBILDA 3415-0014-0048"),
           ("Hardware", 2, "M3 x 6 set screw (cup point) + M3 hex nut", "generic", "one per printed 24T pulley")] if PRINT_DRIVE else []),
        ("goBILDA", 1, f"HTD 5mm belt {JACK_BELT}mm x 9mm", f"3412-0009-{JACK_BELT:04d}", "main -> jackshaft"),
        ("goBILDA", 1, f"HTD 5mm belt {MOTOR_BELT}mm x 9mm", f"3412-0009-{MOTOR_BELT:04d}", "motor 1 -> main (3:1)"),
        *([("goBILDA", 1, f"HTD 5mm belt {MOTOR2_BELT}mm x 9mm", f"3412-0009-{MOTOR2_BELT:04d}", "motor 2 -> main (3:1)")] if SECOND_MOTOR else []),
        *([("REV", 1, "Through Bore Encoder V2", "REV-11-3174", "flywheel speed: 8192 CPR on the main shaft; plugs into flywheel motor 1's encoder port (JST-PH 6->4 pin cable included)"),
           ("3D print", 1, "Encoder sleeve: 8mm REX to 1/2in hex", "encoder_sleeve.stl", "PETG/nylon; adjust the hex size so it slides into the encoder"),
           ("3D print", 1, "Encoder bracket plate", "encoder_bracket.stl", "M4 x 30 + nyloc (locking) nut at each mounting ear (the 4 mm plate is too thin for a heat-set insert)"),
           ("3D print", 2, f"Encoder bracket leg (10 mm tube, {encoder_x()[1][0] - SIDE_X_OUT:.1f} mm long)", "encoder_bracket_leg.stl", "M4 x 50 through it and the bracket plate, nyloc outside"),
           ("Hardware", 1, "Screw + washer for the shaft's threaded end", "generic (thread: verify)", "retains the sleeve")] if ENCODER else []),
        ("goBILDA", 3, "GripForce Gecko 96mm, 30A", "3613-0014-0096", "main flywheel: 3 x 24mm = 72mm stack, bolted to one Sonic Hub"),
        ("goBILDA", 3, "GripForce Gecko 72mm, 30A", "3613-0014-0072", "upper flywheel: 3 x 24mm = 72mm stack, bolted to one Sonic Hub"),
        ("goBILDA", (6 if SECOND_MOTOR else 5) + (1 if GB_TURRET else 0), "Sonic Hub, 8mm REX", "1309-0016-4008", "one each: 48T pulley, 60T gear, 48T gear, main wheel stack, upper wheel stack" + (", turret drive 64T gear (goBILDA's page lists no hub in the turret kit: check the box first)" if GB_TURRET else "") + " (its 14mm raised ring, the boss, centers the part)"),
        ("Hardware", 8, "M4 x 80 socket head screw", "generic", "4 per Gecko stack: through the centers of all three wheels into the Sonic Hub (goBILDA doesn't sell this length)"),
        ("goBILDA", 1, f"8mm REX shaft {main_L}mm, stainless", main_sku,
         f"main flywheel shaft: CUT TO {main_cut:.0f}mm" + (" from the servo end; keep the factory threaded end at the encoder" if ENCODER else "")),
        ("goBILDA", 2, f"8mm REX shaft {jack_L}mm, stainless", jack_sku, f"jackshaft + upper shaft: CUT TO {jack_cut}mm"),
        ("goBILDA", 5, "1611 flanged bearing 8mm REX x 14mm (2-pack)", "1611-0514-4008", "5 packs = 10 bearings: main 4, jack 4, upper 2"),
        *([] if GB_TURRET else [("goBILDA", 6, "Ball bearing 8x22x7 (608 size), 2-pack", "1600-0722-0008", "6 packs = 12 bearings: 2 per turret roller")]),
        ("goBILDA", n_collars, "Clamping collar, 8mm REX", "2910-0920-4008", "shaft ends (count from the shaft stacks)"),
        *([] if PRINT_DRIVE else [("goBILDA", n, f"Spacer {sku}", sku, f"{n} pieces for the shaft stacks: buy {-(-n // (4 if sku.startswith('1522') else 12))} "
          f"pack(s) of {4 if sku.startswith('1522') else 12}") for sku, n in sorted(spacer_counts().items())]),
        *([("3D print", n, f"Shaft spacer {SPACER_LEN[sku]} mm (PA-CF)", f"spacer_{SPACER_LEN[sku]:g}mm.stl",
            f"8.2 bore, 10 mm OD (touches only the bearing's inner race). Backup: goBILDA {sku}") for sku, n in sorted(spacer_counts().items())] if PRINT_DRIVE else []),
        ("goBILDA", 1, "Limelight 3A", "3122-0002-0001", "also ServoCity; was sold out 2026-09-23"),
        ("AndyMark", 2 * CF_SPRINGS_MAX, "Constant-force spring 0.5in x 30in, 2.6 lb", "am-4666",
         f"{CF_SPRINGS_PER_SIDE} STACKED per side on one drum (wind them together); buy {2 * CF_SPRINGS_MAX} to try "
         f"{CF_SPRINGS_MAX} per side after the ball squeeze test. McMaster 9293K52 has the same size and force (2.63 lb) and is rated for 3000 cycles; AndyMark gives no rating"),
        *([("3D print", TRAY_N, f"Cable tray segment, {360 // TRAY_N} deg (fixed)", f"cable_tray_0..{TRAY_N - 1}.stl",
            "PETG; held down by the 8 robot-mount M4 bolts (3 mm longer, button heads under the ring gear)"),
           ("3D print", DRUM_N, "Cable drum (turns with the turret)" if DRUM_N == 1 else f"Cable drum segment, {360 // DRUM_N} deg (turns with the turret)",
            "cable_drum_0.stl" if DRUM_N == 1 else f"cable_drum_0..{DRUM_N - 1}.stl",
            (f"PETG; {len(DRUM_BOLT_ANGS)} M4 heat-sets in the top face; M4 x 12 down through the deck" if DRUM_N == 1 else
             "PETG; 2x M4 heat-set in the top face; M4 x 12 down through the deck")),
           ("REV", 1, "Magnetic Limit Switch (2 magnets included)", "REV-31-1462",
            f"turret home at {HOME_ANG:.0f} deg: 2x M3 x 25 through tray + base plate, nylocs below; magnet in the drum pocket, 2x M3 x 8"),
           ("Hardware", 1, "Heat-shrink bands (1in wide) every ~50 mm + 2 zip ties", "generic",
            f"holds the 4 cables as a flat stack on edge (round sleeving won't bend in the channel); {CABLE_LEN:.0f} mm between the ties, plus 150 mm spare each end")] if CABLES else []),
        *([("REV", 1, "Magnetic Limit Switch (2 magnets included)", "REV-31-1462",
            f"turret home at {HOME_ANG:.0f} deg: on the " + ("printed switch arm" if GB_TURRET else "base plate's pad") + ", 2x M3 x 20 socket head + nyloc below; magnet in the printed holder, 2x {MAG_SCREW} socket head from the pocket side + nyloc on the outer face (fit the magnet on the bench, before the holder goes under the deck)"),
           ("3D print", 1, "Home magnet holder", "home_flag.stl", "hangs under the deck's ear: 2 M4 heat-sets in its top, 2x M4 x 12 socket head down through the deck"),
           ("Hardware", 2, "M4 x 12 socket head (home magnet holder)", "generic", "down through the deck's ear into the holder's heat-sets")] if HOME_FLAG else []),
        *([("Hardware", 2, "M4 x 10 button head (switch arm)", "generic", "into the bearing housing's tapped side holes")] if GB_TURRET else
          [("Hardware", 6, "M8 x 40 bolt + nyloc + washer", "generic", "turret rollers")]),
        ("Hardware", 4, "M4 x 20 button head (48T pulley)", "generic", "48T pulley to its Sonic Hub: heads face the bearing, so button heads (socket heads hit the plate)"),
        ("Hardware", 8, "M4 x 10 button head (sync gears)", "generic", "60T and 48T gears to their Sonic Hubs: heads face the swing-arm bearings"),
        ("Hardware", (20 if CABLES else 12) + (2 if HOME_FLAG else 0) + 2 + 2 + (1 if MOTOR_ADJ else 0) + 2 + 4 + (2 if REAR_GUARD else 0) + (8 if LAMINATE and not GB_TURRET else 0), "M4 heat-set insert, 4.7 mm long (McMaster 94459A150)", "generic",
         "needs a 5.6 mm hole with at least 5.5 mm of plastic around it: crossbars + camera mount + spring-drum sleeves + stop cams" + (" + cable drum" if CABLES else "")
         + (" + motor tension lug" if MOTOR_ADJ else "") + (" + home magnet holder (2)" if HOME_FLAG else "") + " + arm bumpers (2) + spring anchors (4)" + (" + rear guard's anti-turn screws (2)" if REAR_GUARD else "")  + (" + printed turret ring (8)" if LAMINATE and not GB_TURRET else "")),
        *([("Hardware", 4, f"M4 x {GUIDE_BOLT_L} button head (hood guide bolts)", "generic", f"from the ball side, head in the carrier's counterbore; nyloc outside the side plate ({GUIDE_TIP:.1f} mm past it)"),
           ("Hardware", 2, "M4 x 25 button head (spring-drum sleeves)", "generic", "from the ball side into the sleeve's heat-set (press that insert 1.5 mm below the sleeve's end so the screw grips all of it)"),
           ("Hardware", 4, "M4 x 10 button head (stop cams, arm bumpers)", "generic", "from the ball side into the cam's / bumper's heat-set"),
           ("Hardware", 4, f"M4 x {GEAR_BOLT_L} button head (hood gear)", "generic", f"from the ball side through the servo-side carrier and the gear's web, nyloc behind ({GEAR_TIP:.1f} mm past it)"),
           ("Hardware", 8 + (2 if MOTOR_ADJ else 0) + (4 if MOTOR_ADJ else 0), "M4 square nut, DIN 562 (thin: 7 mm across, 2.2 mm thick)", "generic",
            "pressed into the laser-cut / printed pockets (a drop of CA glue): 8 deck (L-bracket feet)" + (", 2 jack yoke, 4 motor adapter" if MOTOR_ADJ else "")),
           ("Hardware", 1, "Wood glue (e.g. Titebond II)", "generic", "for gluing the plywood layers"),
           ("Hardware", 1, "1/8 in hardwood dowel, 36 in", "generic", "cut into 6 mm pins: 3 per glued plate line the layers up")] if LAMINATE else []),
        *([] if LAMINATE else [("Hardware", 10 + 4 + (4 if FW_MOTOR_CSK else 0), "M4 flat head, countersunk 90 deg", "generic",
         "per hood carrier from the ball side: 2x M4 x 45 guide bolts, 1x M4 x 30 drum sleeve, 1x M4 x 12 stop cam, 1x M4 x 12 arm bumper "
         "(a longer one pokes out of the cam); 4x M4 x 16 hood gear (servo side: carrier 6.35 + the gear's 3 mm web + nyloc)" + ("; 4x M4 x 12 flywheel motor" if FW_MOTOR_CSK else ""))]),
        *([("Hardware", 4, "M4 x 12 button head", "generic", "flywheel motor to the motor-side plate (fit before the 16T pulley)")]
          if not FW_MOTOR_CSK and not MOTOR_ADJ else []),
        *([("Hardware", 4, "M4 x 8 button head", "generic", "flywheel motor to the adapter plate (heads sit in the side plate's window)"),
           ("Hardware", 2, "M4 x 10 button head", "generic", "motor clamp bolts: side-plate slots into the adapter's " + ("square nuts" if LAMINATE else "threads")),
           ("Hardware", 2, "M4 x 20 button head", "generic", "motor clamp bolts through the tension lug into the adapter's " + ("square nuts" if LAMINATE else "threads")),
           ("Hardware", 2, "M4 x 16 socket head", "generic", "2 jack-yoke screws"),
           ("Hardware", 1, "M4 x 16 button head", "generic", "the jack screw (tensions the motor belt; 2.5 mm hex key)"),
           ("Hardware", 1, "Blue threadlocker (medium strength)", "generic", "on the motor clamp bolts and the jack screw")] if MOTOR_ADJ else []),
        # v6 (open-item review): the "60 assorted" line, itemised from each joint's grip length
        *([("Hardware", 4, "M3 x 8 socket head (servo)", "generic", "hood servo ears into the M3 heat-sets in the printed servo insert (ears 2.5 + 5.5)"),
           ("Hardware", 4, "M3 heat-set insert, 4.3 mm long (McMaster 94459A130)", "generic", "in the printed servo insert (M3 keeps ~2.5 mm of plastic beside the servo window; an M4 insert would leave ~1.5)")]
          if SERVO_INSERT else [("Hardware", 4, "M4 x 8 socket head (servo)", "generic", "hood servo ears into the tapped side plate (ears 2.5 + 5.5 of thread)")]),
        ("Hardware", 8, "M4 x 12 socket head (L-bracket feet)" if LAMINATE else "M4 x 10 socket head (L-bracket feet)", "generic",
         "through the 5 mm bracket foot and the deck's top layer into the square nuts" if LAMINATE else "through the 5 mm bracket foot into the tapped deck"),
        ("Hardware", (8 if GB_TURRET else 3) + (8 if CABLES else 0) + 6 + 4 + (2 if REAR_GUARD else 0), "M4 x 12 socket head", "generic",
         ("8 deck to the bearing's ring (deck 6.2 + shim 1.2 + 4.6 into its tapped holes), " if GB_TURRET else f"3 turret motor (base {T:.2g} + {12 - T:.2g} into the motor's 10 mm threads), ") + ("8 cable drum (deck + heat-set), " if CABLES else "")
         + 
         "6 crossbars (side plate + heat-set)" + (", 2 rear-guard anti-turn screws (side plate + heat-set)" if REAR_GUARD else "")
         + ", 4 camera pivot + lock (side plate + heat-set)"),
        ("Hardware", 8 if GB_TURRET else 8 + 8, "M4 x 20 socket head", "generic",
         "8 L-bracket sides (bracket 5 + plate 6.2 + nyloc)" if GB_TURRET else (f"8 deck spacers (deck {T:.1f} + spacer {deck_spacer().bounding_box().size.Z:.1f} + into the printed ring's heat-sets), 8 L-bracket sides (bracket 5 + plate {T:.1f} + nyloc)" if LAMINATE
          else "8 deck spacers (deck 6.35 + spacer 6.5 + both ring layers, tapped), 8 L-bracket sides (bracket 5 + plate 6.35 + nyloc)")),
        ("Hardware", 2, "M4 x 12 flat head (spring anchors)", "generic", "countersunk from each arm's inner face into the anchor block's heat-set insert (the tip runs into an empty pocket)"),
        ("Hardware", 2, "M4 x 8 button head (spring ends)", "generic", "through the printed clamp plate (3 mm) and the stacked spring ends' 0.188 in holes into the heat-set insert on the anchor's side (4.4-4.6 mm in, empty pocket behind for the tip)"),
        ("Hardware", 2, "M4 x 50 socket head (encoder legs)", "generic", f"from inside the side plate through the {encoder_x()[1][0] - SIDE_X_OUT:.1f} mm leg and the 4 mm bracket plate, nyloc on the outside (too thin for a heat-set)"),
        ("Hardware", 8, "M4 button head, robot mount", "generic",
         "into the bearing housing's tapped side holes (2 per flat used) through your robot's brackets: length = bracket + 6 mm" if GB_TURRET else
         (f"through the cable tray (3) + base plate ({T:.2g}) + your robot's mounting plate + nyloc: e.g. M4 x 25 for a 1/4 in plate" if CABLES else
          f"through the base plate ({T:.2g}) + your robot's mounting plate + nyloc: e.g. M4 x 20 for a 1/4 in plate (pick it from the real plate's thickness)")),
        ("Hardware", (0 if GB_TURRET else 8) + 8 + 4 + 4 + (4 if ENCODER else 0), "M4 nyloc nut", "generic", ("" if GB_TURRET else "8 robot mount, ") + "8 L-bracket sides, 4 hood guides, 4 hood gear" + (", 2 encoder ears, 2 encoder legs" if ENCODER else "")),
        *([("Hardware", 2, "M4 x 30 socket head (encoder ears)", "2800-0004-0030", "through the encoder's mounting ears and the bracket plate, nyloc behind. REV's drawing doesn't give the ear thickness, so 30 mm covers even a full-thickness (15.3 mm) ear: 4 + 15.3 + 4.8 nyloc = 24.1 mm. Measure the real ear: a shorter screw is fine if 1-2 threads still show past the nyloc")] if ENCODER else []),
    ]
    # printed parts: one section per filament, and every STL name carries its filament (see FILAMENT_STL)
    def _sec(r):
        if r[0] != "3D print":
            return r[0]
        m = next((filament_of_stl(t) for t in re.findall(r"([a-z0-9_]+?)(?:_\d\.\.\d)?\.stl", str(r[3]))), None)
        return f"3D print, {m}" if m else r[0]
    return [(_sec(r), r[1], r[2], tag_stl(r[3]), tag_stl(r[4])) for r in rows]

# ---- filament for every printed part (2026-10-02): one table sets the parts list group, the STL file name and the model
# colour. PA-CF (carbon-fibre nylon) where a part must stay stiff and exact or sits clamped under a bolt (PETG slowly
# squashes, "creeps", under steady load): gears, pulleys, spacer stacks, the stop cams, spring anchors.
# PETG where a part takes knocks, is large, or carries little load. TPU for the one soft part. No PLA: it softens near
# 55-60 C (a closed car, a warm motor) and creeps under bolts.
FILAMENT_STL = [   # STL file stem prefix -> filament (first match wins)
    ("turret_ring_print", "PA-CF"), ("swing_arm_", "PA-CF"), ("servo_insert", "PA-CF"), ("motor_adapter", "PA-CF"),
    ("pulley_", "PA-CF"), ("spacer_", "PA-CF"), ("roller_spacer", "PA-CF"), ("roller_cap", "PA-CF"),
    ("roller_mid_spacer", "PA-CF"), ("deck_spacer", "PA-CF"), ("guide_spacer", "PA-CF"), ("cf_axle", "PA-CF"),
    ("stop_cam", "PA-CF"), ("cf_anchor", "PA-CF"), ("cf_clamp", "PA-CF"),
    # 2026-10-02 (cost): the three biggest "nice to have stiff" parts are PETG: ~160 g less PA-CF, so a 0.5 kg spool does
    ("l_bracket", "PETG"), ("crossbar", "PETG"), ("limelight_mount", "PETG"),
    ("roller_tire", "PETG"), ("rear_guard", "PETG"), ("cf_spring_drum", "PETG"), ("encoder_", "PETG"),
    ("motor_lug", "PETG"), ("motor_yoke", "PETG"), ("arm_bumper_core", "PETG"), ("cable_tray", "PETG"),
    ("cable_drum", "PETG"), ("front_plate", "PETG"), ("spring_post", "PETG"), ("home_flag", "PETG"), ("deck_shim", "PA-CF"), ("switch_arm", "PA-CF"),
    ("arm_bumper_ring", "TPU"),
]
FILAMENT_MODEL = [  # 3D-model part name prefix -> filament (printed parts only; the bought motor pulley stays grey)
    ("turret_ring", "PA-CF"), ("swing_arm", "PA-CF"), ("servo_insert", "PA-CF"), ("motor_adapter", "PA-CF"),
    ("pulley_main", "PA-CF"), ("pulley_jack", "PA-CF"), ("spacer_", "PA-CF"), ("roller_spacer", "PA-CF"),
    ("roller_cap", "PA-CF"), ("roller_mid_spacer", "PA-CF"), ("deck_spacer", "PA-CF"), ("guide_", "PA-CF"),
    ("cf_axle", "PA-CF"), ("stop_cam", "PA-CF"), ("cf_anchor", "PA-CF"), ("cf_clamp", "PA-CF"),
    ("l_bracket", "PETG"), ("crossbar", "PETG"), ("ll_mount", "PETG"),
    ("tire_", "PETG"), ("rear_guard", "PETG"), ("cf_drum", "PETG"), ("encoder_bracket", "PETG"), ("encoder_sleeve", "PETG"),
    ("deck_shim", "PA-CF"), ("switch_arm", "PA-CF"), ("motor_lug", "PETG"), ("motor_yoke", "PETG"), ("arm_bumper_ring", "TPU"), ("arm_bumper", "PETG"), ("cable_tray", "PETG"), ("cable_drum", "PETG"), ("home_flag", "PETG"),
]
FILAMENT_COLOR = {"PA-CF": "#1e1f22", "PETG": "#7e4fd0", "TPU": "#2f6fdd"}   # black, purple; TPU keeps its blue

def filament_of_stl(stem):
    return next((m for pre, m in FILAMENT_STL if stem.startswith(pre)), None)

def filament_of_part(name, kind=None):
    if not LAMINATE or kind in ("ref", "bearing") or (name.startswith("spacer_") and not PRINT_DRIVE):
        return None
    return next((m for pre, m in FILAMENT_MODEL if name.startswith(pre)), None)

def stl_file(stem):
    """The published STL name: filament first, so a folder sorts into print batches (PA-CF_turret_ring_print.stl)."""
    m = filament_of_stl(stem)
    return f"{m}_{stem}.stl" if m else f"{stem}.stl"

_STL_RE = re.compile(r"(?<![A-Za-z0-9_-])([a-z0-9_]+?)((?:_\d\.\.\d)?)\.stl\b")
def tag_stl(text):
    """Every bare STL name in a parts-list / shaft-stack string gets its filament prefix."""
    if not isinstance(text, str):
        return text
    return _STL_RE.sub(lambda mo: (f"{filament_of_stl(mo.group(1))}_" if filament_of_stl(mo.group(1)) else "") + mo.group(1) + mo.group(2) + ".stl", text)

CAT_CUT = "Laser-cut plywood" if LAMINATE else "Laser-cut plates"
PART_LABELS = [  # (name prefix, label, category) - first match wins
    ("base_plate", "Base plate", CAT_CUT), ("turret_ring", "Turret ring gear", "3D-printed" if LAMINATE else CAT_CUT),
    ("deck_spacer", "Deck spacers", "3D-printed"), ("deck_shim", "Deck shim", "3D-printed"), ("switch_arm", "Home switch arm", "3D-printed"),
    ("gb_gear64", "Turret drive gear 64T", "Wheels & drive"), ("gb_hub", "Turret drive gear hub", "Wheels & drive"),
    ("gb_", "goBILDA turret bearing", "Bearings"), ("deck", "Turret deck", CAT_CUT),
    ("side_plate_servo", "Side plate, servo side", CAT_CUT), ("side_plate_motor", "Side plate, motor side", CAT_CUT),
    ("hood_carrier", "Hood carriers", CAT_CUT), ("swing_arm", "Swing arms", "3D-printed" if LAMINATE else CAT_CUT),
    ("tire_", "Turret roller tires", "3D-printed"), ("roller_spacer", "Roller spacers", "3D-printed"), ("roller_cap", "Roller caps", "3D-printed"),
    ("roller_mid_spacer", "Roller inner-race spacers", "3D-printed"),
    ("rear_guard", "Rear ball guard", "3D-printed"), ("servo_insert", "Servo insert", "3D-printed"), ("crossbar", "Crossbars", "3D-printed"), ("l_bracket", "L-brackets", "3D-printed"), ("guide_", "Guide pin spacers", "3D-printed"),
    ("stop_cam", "Stop cams", "3D-printed"), ("cf_drum", "Constant-force spring drums", "3D-printed"),
    ("heatset_", "Heat-set inserts", "Heat-set inserts"), ("arm_bumper_ring", "Arm bumper rings (TPU)", "3D-printed"), ("arm_bumper", "Arm bumper stops", "3D-printed"), ("cf_clamp", "Spring clamp plates", "3D-printed"), ("cf_coil", "Constant-force spring coils (stacked)", "Wheels & drive"),
    ("idler_roller", "Idler rollers", "3D-printed"), ("idler_spacer", "Idler spacer tubes", "3D-printed"),
    ("idler_spool", "Idler axle tabs", "3D-printed"), ("idler_anchor", "Idler spring anchors", "3D-printed"),
    ("ll_mount", "Limelight mount", "3D-printed"),
    ("fastener_", "Bolt heads & nuts", "Fasteners"), ("hood_gear", "Hood gear 108T (bought)", "Wheels & drive"),
    ("motor_adapter", "Motor adapter plate", "3D-printed" if LAMINATE else CAT_CUT), ("motor_lug", "Motor tension lug", "3D-printed"),
    ("motor_yoke", "Motor jack yoke", "3D-printed"),
    ("front_plate", "Front flair plate (optional)", CAT_CUT), ("cf_axle", "Spring-drum sleeves", "3D-printed"),
    ("cable_tray", "Cable tray (fixed)", "Cable management"), ("cable_drum", "Cable drum (turns with turret)", "Cable management"),
    ("cable_loop", "Cable ribbon (shown at home)", "Cable management"), ("home_switch", "Home switch REV-31-1462", "Motors, servo, camera" if HOME_FLAG else "Cable management"),
    ("home_flag", "Home magnet holder", "3D-printed"),
    ("home_magnet", "Home magnet", "Motors, servo, camera" if HOME_FLAG else "Cable management"),
    ("bearing_608", "608 bearings (turret rollers)", "Bearings"), ("bearing_main", "1611 bearings, main shaft", "Bearings"),
    ("bearing_jack", "1611 bearings, jackshaft", "Bearings"), ("bearing_upper", "1611 bearings, upper shaft", "Bearings"),
    ("bearing_idler", "1611 6mm bearings, idler rollers", "Bearings"),
    ("spacer_", "REX spacers", "Shaft hardware"), ("collar_", "Collars", "Shaft hardware"),
    ("shaft_main", "Main flywheel shaft", "Shaft hardware"), ("shaft_jack", "Jackshaft", "Shaft hardware"),
    ("shaft_upper", "Upper flywheel shaft", "Shaft hardware"), ("idler_axle", "Idler axles", "Shaft hardware"),
    ("cf_anchor", "Spring anchor blocks", "3D-printed"),
    ("flywheel_main", "Main flywheel (96mm Geckos)", "Wheels & drive"), ("flywheel_upper", "Upper flywheel (72mm Geckos)", "Wheels & drive"),
    ("hub_", "Sonic Hubs", "Wheels & drive"), ("pulley_main", "HTD pulleys (printed)", "Wheels & drive"), ("pulley_jack", "HTD pulleys (printed)", "Wheels & drive"), ("pulley_", "HTD pulleys", "Wheels & drive"), ("belt_", "HTD belts", "Wheels & drive"), ("gear_", "Sync gears 60T/48T", "Wheels & drive"),
    ("servo_pinion", "Hood servo shaft + pinion", "Wheels & drive"), ("turret_pinion", "Turret pinion", "Wheels & drive"),
    ("flywheel_motor2", "Flywheel motor 2", "Motors, servo, camera"), ("flywheel_motor", "Flywheel motor 1", "Motors, servo, camera"),
    ("encoder_v2", "Through Bore Encoder V2", "Motors, servo, camera"), ("encoder_bracket", "Encoder bracket", "3D-printed"),
    ("encoder_sleeve", "Encoder sleeve (REX to 1/2in hex)", "3D-printed"), ("turret_motor", "Turret motor", "Motors, servo, camera"),
    ("hood_servo", "Hood servo", "Motors, servo, camera"), ("limelight", "Limelight 3A", "Motors, servo, camera"),
]
DXF_MARKS = {   # extra layers so post-cut machining isn't missed
    "hood_carrier": ([("CSK_M4", p, CSK_D) for p in CARRIER_CSK + HOOD_GEAR_CSK],
                     ["COUNTERSINK (layer CSK_M4): 90 deg to 9.0 dia, on the face toward the ball.",
                      "Both parts: the 5 outer holes (guides, drum sleeve, stop cam, arm bumper). Servo-side part only: also the 4 gear holes.",
                      "Cut 2. Servo-side carrier: countersink the face as drawn. Motor-side: countersink the back face."]),
    "side_plate_servo": ([("TAP_M4", (SERVO_AXIS[0] + (-1 if SERVO_BODY_FWD else 1) * 10.1 + e1 * 24.75, SERVO_AXIS[1] + e2), TAP_M4)
                          for e1 in (-1, 1) for e2 in (-5, 5)] if SERVO_TAPPED else [],
                         ["TAP the 4 hood-servo ear holes M4 (layer TAP_M4, cut at 3.3): the servo unbolts from outside."] if SERVO_TAPPED else []),
    "turret_ring": ([("TAP_M4", pol(DECK_SPACER_R, math.radians(a)), TAP_M4) for a in DECK_SPACER_ANGS] if RING_TAPPED else [],
                    ["Cut TWO from 1/8in 6061 and stack them (teeth are too fine for 1/4in), or 3D-print for testing.",
                     "TAP the 8 deck-spacer holes M4 (cut at 3.3); if printed, use M4 heat-set inserts instead."]),
    "deck": ([("TAP_M4", (sx * (SIDE_X_OUT + BRKT_FOOT_HOLE), by + dy), TAP_M4) for sx in (-1, 1) for by in BRACKET_Y for dy in BRKT_FOOT_DY]
             if DECK_TAPPED else [], ["TAP the 8 L-bracket foot holes M4 (layer TAP_M4, cut at 3.3)."]),
    "side_plate_motor": (([("TAP_M4", motor_rel(p), TAP_M4) for p in YOKE_SCREWS] if MOTOR_ADJ else [])
                         + [("CSK_M4", p, CSK_D) for p in (FW_MOTOR_BOLTS if FW_MOTOR_CSK else [])],
                         ["TAP the 2 jack-yoke holes M4 (layer TAP_M4, cut at 3.3). The big window and 4 slots are the",
                          "adjustable flywheel-motor mount: the adapter plate slides behind them."] if MOTOR_ADJ else
                         (["COUNTERSINK 4x M4 (layer CSK_M4) on the OUTSIDE face (away from the motor): flywheel motor,",
                           "M4 x 12 flat heads. Socket heads would hit the 16T motor pulley."] if FW_MOTOR_CSK else [])),
    "swing_arm": ([("CSK_M4", p, CSK_D) for p in ARM_CSK],
                  ["COUNTERSINK the spring-anchor hole (layer CSK_M4) on the face toward the ball corridor.",
                   "Cut 2: the servo-side arm as drawn, the motor-side arm countersunk on the other face."]),
    "motor_adapter": ([("TAP_M4", p, TAP_M4) for p in LUG_BOLTS + CLAMP_BOLTS],
                      ["1/8in 6061. TAP the 4 clamp holes M4 (layer TAP_M4, cut at 3.3). The motor bolts on with 4x M4 x 8",
                       "button heads on the side-plate face. Same orientation as side_plate_motor.dxf (notch toward the crossbar)."]),
}

# v5: what each part is for, shown when you double-click it in the viewer. keys = substrings that find its
# parts-list row(s) in the BOM "part" or "SKU / file" field.
PART_DETAILS = {
    "Base plate": ("The fixed plate that bolts to the robot. It holds the six turret rollers and the turret motor; the turret spins on top of it.", ["base_plate.dxf"]),
    "Turret roller tires": ("Printed tires on skateboard bearings. The ring gear rides in their groove, so the turret can spin but can't lift or slide.", ["roller_tire.stl"]),
    "Roller spacers": ("Lift each roller's lower bearing off the base plate. They touch only the bearing's inner ring, so the outer ring can spin freely.", ["roller_spacer.stl"]),
    "Roller caps": ("Sit on each roller's upper bearing, under the washer and nut, touching only the inner ring.", ["roller_cap.stl"]),
    "Roller inner-race spacers": ("A 1 mm spacer between each roller's two bearings. Tightening the bolt then clamps only the inner rings, instead of squeezing the bearings and making them stiff.", ["roller_mid_spacer.stl"]),
    "Turret pinion": ("The small steel gear on the turret motor. It drives the big ring gear: 20 teeth to 280, so 14:1.", ["2303-4008-0020"]),
    "Turret motor": (("A 312 rpm Yellow Jacket on the turret bearing's motor mount (left side, hanging below). It turns the 64T gear that drives the bearing's 176T ring (2.75:1). Its encoder tells the code which way the turret points.", [GB_MOTOR_SKU]) if GB_TURRET else
                     ("A 1150 rpm Yellow Jacket under the base plate that turns the turret. Its encoder tells the code which way the turret points.", ["5203-2402-0005"])),
    "goBILDA turret bearing": ("goBILDA's Gear-Driven Turret: an aluminum housing with a steel ball bearing and a 176-tooth gear ring on top. The housing bolts to the robot; the deck screws onto the ring. The ball comes up through its 105 mm bore.", [GB_SKU]),
    "Turret drive gear 64T": ("Comes with the turret bearing. It sits on the turret motor's shaft (on a Sonic Hub) and meshes with the bearing's 176T ring: 2.75:1.", [GB_SKU]),
    "Turret drive gear hub": ("A Sonic Hub that holds the 64T drive gear on the turret motor's 8 mm REX shaft (check the turret kit first: goBILDA's page lists no hub).", ["1309-0016-4008"]),
    "Deck shim": ("A thin printed ring between the bearing's gear ring and the deck, so the deck clears the 64T drive gear by 2.2 mm.", ["deck_shim.stl"]),
    "Home switch arm": ("A printed arm screwed to the bearing housing's front-right flat. The home switch sits on its floor, where v8.1's base plate held it.", ["switch_arm.stl"]),
    "Cable tray (fixed)": ("A fixed ring-shaped channel. The cable ribbon lies in it with a U-turn that rolls as the turret turns, so the wires never tangle.", [f"cable_tray_0..{TRAY_N - 1}.stl"]),
    "Home switch REV-31-1462": (("A magnetic sensor on the base plate. When the magnet under the deck's ear passes it, the code knows the turret is at home and resets its angle." if HOME_FLAG else
                                 "A magnetic sensor. When the magnet in the cable drum passes it, the code knows the turret is at home and resets its angle."), ["REV-31-1462"]),
    "Home magnet holder": ("A small printed block under the deck's ear that carries the home magnet past the switch.", ["home_flag.stl"]),
    "Cable ribbon (shown at home)": ("The four turret cables (flywheel motor power, its encoder, the hood servo and the camera's USB) banded into a flat ribbon. Shown at the home position.", ["Heat-shrink"]),
    "Turret ring gear": ("The laser-cut 280-tooth gear the whole turret sits on. The rollers hold it in place and the pinion turns it.", ["turret_ring.dxf"]),
    "Deck spacers": ("Short tubes that hold the deck just above the ring gear.", ["deck_spacer.stl"]),
    "Turret deck": ("The plate the side plates stand on. It turns with the turret.", ["deck.dxf"]),
    "Cable drum (turns with turret)": ("A ring under the deck that turns with the turret. The inner run of the cable ribbon lies along it.", ["cable_drum_0.stl" if DRUM_N == 1 else f"cable_drum_0..{DRUM_N - 1}.stl"]),
    "Home magnet": ("The magnet set into the cable drum. It trips the home switch.", ["REV-31-1462"]),
    "Side plate, servo side": ("One of the shooter's two main walls. It holds a main-shaft bearing, the hood servo and the camera mount.", ["side_plate_servo.dxf"]),
    "Side plate, motor side": ("The other main wall. It holds a main-shaft bearing, the encoder bracket and the adjustable flywheel-motor mount (the window and four slots).", ["side_plate_motor.dxf"]),
    "Crossbars": ("Printed tubes that tie the two side plates together, so they stay parallel.", ["crossbar.stl"]),
    "Servo insert": ("A printed PA-CF frame the hood servo bolts into. Plywood that close to the servo's screw holes would be too thin, so the insert fills an opening in the servo-side plate; its lip is trapped between the plate's two glued layers. The servo's 4 screws go into heat-set inserts, so it still unbolts from outside.", ["servo_insert.stl", "M4 x 8"]),
    "Rear ball guard": ("A printed wall just behind the feed hole, built onto the rear crossbar. With the hood flat, or while the robot is speeding up or braking, a ball can bounce off the underside of the upper wheel and fall behind the feed hole, into the turret. The wall stops it, so it drops back down the feed hole and the indexer can try again. A second screw at each end keeps it from turning.", ["rear_guard.stl", "M4 x 12"]),
    "Motor adapter plate": (("A thin printed PA-CF plate" if LAMINATE else "A thin aluminum plate") + " the flywheel motor bolts to, just inside the side plate. Loosen its 4 clamp bolts (outside) and it slides forward or back to tighten or loosen the motor belt.", ["motor_adapter.stl" if LAMINATE else "motor_adapter.dxf"]),
    "Motor tension lug": ("A printed block clamped to the motor's adapter plate through the side plate. The jack screw threads into it and pulls the motor forward to tighten the belt.", ["motor_lug.stl"]),
    "Motor jack yoke": ("A printed U-shaped block bolted to the side plate in front of the motor. The jack screw's head pushes against it; turn the screw clockwise to tighten the motor belt.", ["motor_yoke.stl", "M4 x 16 button"]),
    "L-brackets": ("Bolt the side plates down to the deck.", ["l_bracket.stl"]),
    "Main flywheel (96mm Geckos)": ("The lower wheel stack that throws the ball: three 96 mm Gecko wheels, each 24 mm wide. Four M4 × 80 screws go through the wheels' centers into a Sonic Hub.", ["3613-0014-0096", "M4 x 80"]),
    "Sonic Hubs": ("A hub that clamps onto the 8 mm REX shaft. The gear, pulley or wheel stack bolts to its face with M4 screws. Its raised ring (the boss) fits the part's 14 mm center hole to keep it centered.", ["1309-0016-4008"]),
    "Main flywheel shaft": ("The 8 mm REX shaft for the main wheel. It also carries both drive pulleys and the encoder.", ["2106-4008-2400"]),
    "HTD pulleys": ("Toothed pulleys for the belts: motor to main shaft (16T to 48T, a 3:1 slow-down) and main shaft to jackshaft (24T to 24T). The 16T and 24T lock on with set screws (small screws that press on the shaft). The 48T bolts to a Sonic Hub.", ["3417-4008-0024", "3417-4008-0016", "3415-0014-0048"]),
    "HTD belts": (f"Toothed belts: {MOTOR_BELT} mm from the motor to the main shaft, 410 mm from the main shaft to the jackshaft.", [f"3412-0009-{MOTOR_BELT:04d}", "3412-0009-0410"]),
    "Flywheel motor 1": ("The 6000 rpm Yellow Jacket that spins both flywheels through a 3:1 belt. It bolts to the adapter plate, which slides to set the belt tension.", ["5203-2402-0001"]),
    "Through Bore Encoder V2": ("Measures flywheel speed very finely (8192 counts per turn), so the code can fire only when the wheels are exactly at speed.", ["REV-11-3174"]),
    "Encoder bracket": ("A printed plate and two legs that hold the encoder on the end of the main shaft.", ["encoder_bracket.stl", "encoder_bracket_leg"]),
    "Encoder sleeve (REX to 1/2in hex)": ("A printed adapter from the 8 mm REX shaft to the encoder's 1/2 in hex hole.", ["encoder_sleeve.stl"]),
    "Hood servo": ("Tilts the hood, which sets the launch angle, through a 20T pinion and the bought 108T gear: 5.4:1.", ["2000-0025-0002"]),
    "Hood servo shaft + pinion": ("The servo's shaft and 20T steel pinion that drive the bought 108T hood gear.", ["1922-0025-0036", "2303-4008-0020"]),
    "Hood gear 108T (bought)": ("A goBILDA 108-tooth aluminum gear bolted to the servo-side hood carrier. The servo's 20-tooth pinion turns it to tilt the hood (5.4:1). " + ("Bought, because it takes the servo's full force (printed teeth this small would break if the servo pushed against an end stop) and it's hard to reach." if LAMINATE else "Bought, because teeth this small are too fine to have cut from 1/4 in aluminum."), ["2302-0032-0108"]),
    "Front flair plate (optional)": ("A flat panel across the front for stickers. Zip-tied to the front crossbar, so it swaps in seconds. Decoration only, not a ROBOT SIGN (it's narrower than a sign's 165 mm, and it turns with the turret); we put the signs on the chassis.", ["front_plate.dxf", "front_plate.stl"]),
    "Hood carriers": ("The tilting hood: two plates that pivot on the main shaft and carry the jackshaft and the upper wheel's arm. The bought 108T gear bolts to the servo-side carrier; the servo drives it.", ["hood_carrier.dxf"]),
    "Guide pin spacers": ("Tubes on the guide bolts. They slide in curved slots in the side plates, take load off the servo, and stop the hood at each end.", ["guide_spacer.stl"]),
    "Stop cams": ("Adjustable off-center stops. They set how close the upper wheel can come to the main wheel, which sets the squeeze on worn balls. Loosen, turn, retighten.", ["stop_cam.stl"]),
    "Jackshaft": ("The in-between shaft on the hood. It takes drive from the main shaft by belt and passes it to the upper wheel by gears.", ["2106-4008-1680"]),
    "Sync gears 60T/48T": ("The 60T gear on the jackshaft drives the 48T on the upper wheel. The gears reverse the direction of spin, and make the upper wheel's surface about 6% slower, for gentle backspin.", ["2312-0414-0060", "2312-0414-0048"]),
    "Constant-force spring drums": ("The coiled constant-force springs sit on these. The springs pull the upper wheel's arm in with a steady force.", ["cf_spring_drum.stl", "am-4666"]),
    "Spring clamp plates": ("a small printed plate under the screw that holds the spring ends. The strips bend over its rounded edge as the arm swings, instead of over a sharp washer.", ["cf_clamp.stl"]),
    "Heat-set inserts": ("Brass threaded sleeves pressed into a printed part with a hot soldering iron (an insert tip). The plastic melts around their knurls and locks them in, so screws thread into metal instead of plastic and can be undone many times.", ["94459A150", "94459A130"]),
    "Arm bumper rings (TPU)": ("The soft TPU ring pressed onto each bumper core. The swing arm lands on it, so the stop is cushioned instead of a hard plastic hit.", ["arm_bumper_ring.stl"]),
    "Arm bumper stops": ("a cushioned post on each hood carrier. When the gear-driven upper wheel grips a Nectar, the drag can throw the arm open. The bumper catches it 3° past the Nectar position, before the spinning shaft spacers could reach the carrier.", ["arm_bumper_core.stl", "arm_bumper_ring.stl"]),
    "Constant-force spring coils (stacked)": ("The springs' coils, drawn at the size of the biggest stack the drum holds, so the collision check (which makes sure no parts hit) includes them. The design stacks two am-4666 per side (three fit).", ["am-4666"]),
    "Spring-drum sleeves": ("Printed axles the spring drums spin on, held by a " + ("button-head bolt in a counterbore" if LAMINATE else "countersunk bolt") + " from the ball side.", ["cf_axle.stl"]),
    "Swing arms": ("Hold the upper wheel. They pivot on the jackshaft, so a bigger ball can push the wheel out.", ["swing_arm.dxf"]),
    "Upper flywheel (72mm Geckos)": ("The upper wheel stack that squeezes the ball against the main wheel: three 72 mm Gecko wheels, bolted through their centers into a Sonic Hub.", ["3613-0014-0072", "M4 x 80"]),
    "Upper flywheel shaft": ("The 8 mm REX shaft for the upper wheel stack.", ["2106-4008-1680"]),
    "Spring anchor blocks": ("printed blocks on the swing arms' tails. The ends of the stacked constant-force springs screw flat to their side face. A countersunk (flush) screw through the arm holds each block.", ["cf_anchor.stl", "am-4666"]),
    "Collars": ("Clamp onto a shaft end so nothing slides off.", ["2910-0920-4008"]),
    "REX spacers": ("Short tubes that hold parts at exact positions along a shaft." + (" Printed in PA-CF: 8.2 mm hole, 10 mm across, so they only touch a bearing's inner ring." if PRINT_DRIVE else ""),
                    ["Shaft spacer"] if PRINT_DRIVE else ["1522-0010"]),
    "1611 bearings, main shaft": ("Flanged ball bearings (with a lip that stops them sliding through the hole) that let the main shaft spin in the plates.", ["1611-0514-4008"]),
    "1611 bearings, jackshaft": ("Flanged ball bearings for the jackshaft, in the hood carriers and swing arms.", ["1611-0514-4008"]),
    "1611 bearings, upper shaft": ("Flanged ball bearings for the upper wheel's shaft, in the swing arms.", ["1611-0514-4008"]),
    "608 bearings (turret rollers)": ("Skateboard-size ball bearings, two inside each turret roller.", ["1600-0722-0008"]),
    "Limelight mount": ("The printed mount that holds the camera and sets its tilt, 20–50° in 5° steps.", ["limelight_mount.stl"]),
    "Limelight 3A": ("The smart camera that finds the HIVE's AprilTags so the turret can aim.", ["3122-0002-0001"]),
    "Bolt heads & nuts": ("A bolt head or nut, included in the model so the collision check can make sure nothing hits it.", []),
}
if LAMINATE:   # v7: part cards point at the files that get made now
    # keys must match the BOM rows' file field (part_details() matches by substring), so use the real file names
    _V7_FILES = {"base_plate.dxf": ["base_plate_L1.dxf"], "deck.dxf": ["deck_L1.dxf"], "side_plate_servo.dxf": ["side_plate_servo_L1.dxf"],
                 "side_plate_motor.dxf": ["side_plate_motor_L1.dxf"], "hood_carrier.dxf": ["hood_carrier_servo_L1.dxf", "hood_carrier_motor_L1.dxf"],
                 "swing_arm.dxf": ["swing_arm_servo.stl"], "turret_ring.dxf": ["turret_ring_print.stl"],
                 "motor_adapter.dxf": ["motor_adapter.stl"], "M4 x 8": ["M3 x 8 socket head (servo)"] }
    def _v7_keys(lab, fs):
        out = []
        for f in fs:
            if f == "M4 x 8" and lab != "Servo insert" and "servo" not in lab.lower():
                out.append(f); continue
            out += _V7_FILES.get(f, [f])
        return out
    PART_DETAILS = {k: (w.replace("The laser-cut 280-tooth gear", "The printed (PA-CF) 280-tooth gear"), _v7_keys(k, fs))
                    for k, (w, fs) in PART_DETAILS.items()}
    PART_DETAILS["HTD pulleys"] = (PART_DETAILS.get("HTD pulleys", ("", []))[0], ["pulley_24t.stl", "pulley_48t.stl", "3417-4008-0016"])
    PART_DETAILS["HTD pulleys (printed)"] = (PART_DETAILS["HTD pulleys"][0], ["pulley_24t.stl", "pulley_48t.stl"])
    PART_DETAILS["Servo insert"] = (PART_DETAILS["Servo insert"][0], ["servo_insert.stl", "M3 x 8 socket head (servo)", "M3 heat-set insert"])
FASTENER_INFO = [  # name prefix -> what this fastener is
    ("fastener_flag", "M4 × 12 socket head: holds the home magnet holder under the deck's ear"),
    ("fastener_carrier_guide", f"M4 × {GUIDE_BOLT_L} button head: a hood guide bolt, its head in the carrier's counterbore on the ball side"),
    ("fastener_carrier_sleeve", "M4 × 25 button head: holds the spring-drum sleeve, threaded into the sleeve's heat-set insert"),
    ("fastener_carrier_cam", "M4 × 10 button head: holds the eccentric stop cam (into its insert). Loosen it to turn the cam"),
    ("fastener_carrier_bumper", "M4 × 10 button head: holds the arm bumper's core, threaded into the core's heat-set insert"),
    ("fastener_hood_gear_head", f"M4 × {GEAR_BOLT_L} button head: a hood gear bolt, head on the ball side"),
    ("fastener_m8_head", "M8 × 40 bolt head, a turret roller's axle (under the base plate)"),
    ("fastener_m8_nut", "M8 washer + nyloc (locking) nut on a turret roller"),
    ("fastener_m8_tip", "The end of a turret roller's M8 × 40 bolt, above its nut"),
    ("fastener_turret_motor", "M4 socket head: turret motor mount (3 screws). Turn the turret until the deck's 3 access holes sit over them to set the gear mesh or swap the motor"),
    ("fastener_robot_mount", "M4 button head: base plate to the robot (also holds the cable tray down)"),
    ("fastener_switch_head", "M3 socket head: home switch"), ("fastener_switch_nut", "M3 nyloc: home switch (under the base plate)"),
    ("fastener_deck_ring", "M4 × 12 socket head: deck to the turret bearing's gear ring (through the shim)"), ("fastener_switch_arm", "M4 × 10 button head: home switch arm to the bearing housing"),
    ("fastener_deck_spacer_head", "M4 socket head: deck spacer bolt"), ("fastener_deck_spacer_nut", "M4 nyloc: deck spacer bolt (under the ring gear)"),
    ("fastener_drum", "M4 socket head: cable drum to deck, into a heat-set insert"),
    ("fastener_bracket_foot_nut", "M4 nyloc: L-bracket to deck"), ("fastener_bracket_foot", "M4 socket head: L-bracket to deck"),
    ("fastener_bracket_side_nut", "M4 nyloc: L-bracket to side plate"), ("fastener_bracket_side", "M4 socket head: L-bracket to side plate"),
    ("fastener_crossbar", "M4 socket head: side plate to crossbar, into a heat-set insert"),
    ("fastener_ll_pivot", "M4: camera mount pivot"), ("fastener_ll_lock", "M4: camera tilt lock (move it to another hole to change the tilt)"),
    ("fastener_guide_nut", "M4 nyloc: hood guide bolt " + ("(its button head sits in a counterbore on the ball side)" if LAMINATE else "(its flat head is countersunk on the ball side)")),
    ("fastener_hood_gear_nut", "M4 nyloc: hood gear bolt " + ("(its button head sits in a counterbore on the ball side)" if LAMINATE else "(its flat head is countersunk on the ball side)")),
    ("fastener_servo_nut", "M4 nyloc: hood servo mount"), ("fastener_servo", ("M3" if SERVO_INSERT else "M4") + ": hood servo mount"),
    ("fastener_encoder_leg", "M4 × 50: encoder bracket leg, nyloc on the bracket plate"),
    ("fastener_spring_end", "M4 × 8 button head: holds the stacked spring ends to the anchor block, under a printed clamp plate"),
    ("fastener_hub_pulley48", "M4 × 20 button head: 48T pulley to its Sonic Hub (fit before the pulley goes on the shaft)"),
    ("fastener_hub_gear", "M4 × 10 button head: sync gear to its Sonic Hub (fit before the gear goes on the shaft)"),
    ("fastener_fw_motor", "M4 × 8 button head: flywheel motor to its adapter plate (reach it through the side-plate window with the 16T pulley off)"),
    ("fastener_motor_clamp_lug", "M4 × 20 button head: motor clamp bolt through the tension lug (loosen to adjust the belt)"),
    ("fastener_motor_clamp", "M4 × 10 button head: motor clamp bolt (loosen to adjust the belt)"),
    ("fastener_motor_yoke", "M4 × 16 socket head: jack yoke to the side plate"),
    ("fastener_motor_jack", "M4 × 16 button head: the jack screw. Turn clockwise to tighten the motor belt"),
]

# v5: parts that share a label but are different parts get their own title, description and BOM keys
PART_OVERRIDES = [  # (name prefix, title, what, BOM keys) - longest prefix wins
    ("pulley_main_motor", "48T HTD pulley, main shaft", "The big pulley the motor's belt drives, outside the motor-side plate. 16T on the motor to 48T here is a 3:1 slow-down (6000 → 2000 rpm). It bolts to a Sonic Hub.", ["pulley_48t.stl" if PRINT_DRIVE else "3415-0014-0048"]),
    ("pulley_motor", "16T HTD pulley, flywheel motor", "The small pulley on the flywheel motor's shaft. It drives the 48T on the main shaft through the " + f"{MOTOR_BELT} mm belt, and locks on with set screws.", ["3417-4008-0016"]),
    ("pulley_main_jack", "24T HTD pulley, main shaft", "Drives the jackshaft through the 410 mm belt (24T to 24T, same speed). Locks on with set screws.", ["pulley_24t.stl" if PRINT_DRIVE else "3417-4008-0024"]),
    ("pulley_jack", "24T HTD pulley, jackshaft", "Driven by the main shaft through the 410 mm belt, so the jackshaft turns at main-shaft speed. Locks on with set screws.", ["pulley_24t.stl" if PRINT_DRIVE else "3417-4008-0024"]),
    ("belt_motor", f"{MOTOR_BELT} mm HTD belt, motor → main shaft", "Carries the motor's power to the main shaft: 16T to 48T, a 3:1 slow-down.", [f"3412-0009-{MOTOR_BELT:04d}"]),
    ("belt_jack", "410 mm HTD belt, main shaft → jackshaft", "Carries drive up to the jackshaft on the hood. The hood pivots on the main shaft, so this belt never loosens as the hood tilts.", ["3412-0009-0410"]),
    ("gear_jack_60t", "60T gear, jackshaft", "Drives the 48T gear on the upper wheel. Gears reverse the direction of spin, so the two wheels spin opposite ways. Bolted to a Sonic Hub.", ["2312-0414-0060"]),
    ("gear_upper_48t", "48T gear, upper wheel", "Driven by the 60T on the jackshaft: the upper wheel's surface ends up about 6% slower than the main wheel's, for gentle backspin. Bolted to a Sonic Hub.", ["2312-0414-0048"]),
    ("hub_main_pulley", "Sonic Hub for the 48T pulley", "Clamps onto the main shaft. The 48T pulley bolts to it, and its raised ring (boss) centers the pulley.", ["1309-0016-4008"]),
    ("hub_main_wheels", "Sonic Hub for the main wheel stack", "Clamps onto the main shaft. Four M4 × 80 screws run through the centers of the three Gecko wheels into this hub.", ["1309-0016-4008", "M4 x 80"]),
    ("hub_upper_wheels", "Sonic Hub for the upper wheel stack", "Clamps onto the upper shaft. Four M4 × 80 screws run through the centers of the three Gecko wheels into this hub.", ["1309-0016-4008", "M4 x 80"]),
    ("hub_jack_gear", "Sonic Hub for the 60T gear", "Clamps onto the jackshaft. The 60T gear bolts to it. It also keeps the shaft from sliding sideways.", ["1309-0016-4008"]),
    ("hub_upper_gear", "Sonic Hub for the 48T gear", "Clamps onto the upper shaft. The 48T gear bolts to it. It also keeps the shaft from sliding sideways.", ["1309-0016-4008"]),
    ("encoder_bracket_leg", "Encoder bracket leg", "A printed 10 mm tube from the side plate out to the encoder bracket plate. An M4 × 50 bolt runs through the side plate, the tube and the bracket plate, with a nyloc nut on the outside.", ["encoder_bracket_leg"]),
    ("encoder_bracket", "Encoder bracket plate", "The printed plate that holds the Through Bore Encoder's two ears on the end of the main shaft.", ["encoder_bracket.stl"]),
]
FASTENER_BOM = [("fastener_deck_ring", "M4 x 12 socket"), ("fastener_switch_arm", "M4 x 10 button head (switch arm)"), ("fastener_flag", "M4 x 12 socket head (home magnet holder)"), ("fastener_carrier_guide", f"M4 x {GUIDE_BOLT_L} button head (hood guide"), ("fastener_carrier_sleeve", "M4 x 25 button head (spring-drum"),
                ("fastener_carrier_cam", "M4 x 10 button head (stop cams"), ("fastener_carrier_bumper", "M4 x 10 button head (stop cams"),
                ("fastener_hood_gear_head", f"M4 x {GEAR_BOLT_L} button head (hood gear)"),
                ("fastener_m8", "M8 x 40"), ("fastener_hood_gear_nut", f"M4 x {GEAR_BOLT_L} button head (hood gear)" if LAMINATE else "M4 flat head"),
                ("fastener_guide_nut", f"M4 x {GUIDE_BOLT_L} button head (hood guide" if LAMINATE else "M4 flat head"),
                ("fastener_hub_pulley48", "M4 x 20 button head (48T"), ("fastener_hub_gear", "M4 x 10 button head (sync"),
                ("fastener_spring_end", "M4 x 8 button head (spring ends)"),
                ("fastener_fw_motor", "M4 x 8 button" if MOTOR_ADJ else "M4 x 12 button"), ("fastener_motor_clamp_lug", "M4 x 20 button"),
                ("fastener_motor_clamp", "M4 x 10 button"), ("fastener_motor_yoke", "M4 x 16 socket"), ("fastener_motor_jack", "M4 x 16 button"),
                ("fastener_switch", "REV-31-1462"), ("fastener_drum", "M4 x 12 socket"), ("fastener_turret_motor", "M4 x 12 socket"),
                ("fastener_crossbar", "M4 x 12 socket"), ("fastener_ll_", "M4 x 12 socket"), ("fastener_encoder_leg", "M4 x 50 socket"),
                ("fastener_servo", "M3 x 8 socket" if SERVO_INSERT else "M4 x 8 socket"), ("fastener_bracket_foot", "M4 x 12 socket" if LAMINATE else "M4 x 10 socket"), ("fastener_bracket_side_nut", "M4 nyloc"),
                ("fastener_bracket_side", "M4 x 20 socket"), ("fastener_deck_spacer", "M4 x 20 socket"),
                ("fastener_robot_mount", "robot mount"), ("fastener_", "M4 nyloc")]

def part_details(parts, bom_rows, parts_kind=None):
    """Per-part info for the viewer's double-click card: what it does, its BOM rows, its shaft-stack slot."""
    parts_kind = parts_kind or {}
    stacks = shaft_stacks()
    shaft_of = {"main": "Main flywheel shaft", "jack": "Jackshaft", "upper": "Upper flywheel shaft"}
    out = {}
    find = lambda keys, exact=False: [i for i, r in enumerate(bom_rows)
                                       if any(k and ((k == str(r[3])) if exact else (k in str(r[3]) or k in str(r[2]))) for k in keys)]
    for name in parts:
        lab, _ = part_label(name)
        what, keys = PART_DETAILS.get(lab, ("", []))
        d = {"what": what}
        if parts_kind.get(name) == "cut":   # stock for the viewer's card
            d["stock"] = ("1/8 in (3.175 mm) 6061, two layers stacked" if name.startswith("turret_ring") else
                          "1/8 in (3.175 mm) 6061" if name.startswith("motor_adapter") else
                          "3 mm wood or plastic" if name.startswith("front_plate") else
                          f"two 1/8 in (3 mm) Baltic birch layers, glued (~{T:.1f} mm)" if LAMINATE else "6.35 mm (1/4 in) 6061")
        elif filament_of_part(name, parts_kind.get(name)):
            d["stock"] = f"3D-printed {filament_of_part(name, parts_kind.get(name))}"
        ov = max((o for o in PART_OVERRIDES if name.startswith(o[0])), key=lambda o: len(o[0]), default=None)
        if ov:
            d["title"], d["what"], keys = ov[1], ov[2], ov[3]
        side = re.search(r"_(servo|motor|s|m)(_?\d+)?$", name)
        if side and not ov and "side" not in lab and not name.startswith(("fastener_", "pulley_", "hub_")):
            d["title"] = lab.rstrip("s") + (", servo side" if side.group(1) in ("servo", "s") else ", motor side") \
                if lab.endswith("s") and not lab.endswith("ss") else lab + (", servo side" if side.group(1) in ("servo", "s") else ", motor side")
        if name.startswith("fastener_"):
            d["title"] = next((t for pre, t in FASTENER_INFO if name.startswith(pre)), "Fastener").split(":")[0]
            d["what"] = next((t for pre, t in FASTENER_INFO if name.startswith(pre)), what) + \
                ". Included in the model so the collision check can make sure nothing hits it."
            keys = [next(k for pre, k in FASTENER_BOM if name.startswith(pre))]
        rows = find(keys)
        m = re.match(r"^(bearing|spacer|collar)_(main|jack|upper)_(\d+)$", name)
        if m:
            st = stacks[shaft_of[m.group(2)]]
            it = st["items"][int(m.group(3))]
            d["stack"] = {"shaft": shaft_of[m.group(2)], "item": it[0], "sku": it[1], "x0": round(it[2], 2), "x1": round(it[3], 2)}
            d["title"] = it[0][0].upper() + it[0][1:]
            rows = find([it[1]], exact=True) or rows          # the exact spacer / bearing / collar in this slot
        if rows:
            d["bom"] = rows
        out[name] = d
    return out

def part_label(name, kind=None):
    lab, cat = next(((lab, cat) for pre, lab, cat in PART_LABELS if name.startswith(pre)), (name, "Other"))
    m = filament_of_part(name, kind)
    return (lab, f"3D-printed {m}") if m else (lab, cat)

# ---- v7: laminated plywood layers ----
def _sq(p, w):
    return box(p[0] - w / 2, p[1] - w / 2, p[0] + w / 2, p[1] + w / 2)

def dowel_pts(poly, avoid=(), n=3, clear=4.0):
    """n alignment-dowel spots, spread as far apart as possible, >= clear mm of wood around each."""
    zone = poly.buffer(-(DOWEL_D / 2 + clear))
    if avoid:
        zone = zone.difference(unary_union([Point(*a).buffer(DOWEL_D / 2 + clear + 4.5) for a in avoid]))
    x0, y0, x1, y1 = poly.bounds
    cand = [(x, y) for x in np.arange(x0, x1, 2.0) for y in np.arange(y0, y1, 2.0) if zone.contains(Point(x, y))]
    if len(cand) < n:
        return []
    c = np.array(cand)
    pick = [int(np.argmax(np.linalg.norm(c - c.mean(0), axis=1)))]
    for _ in range(n - 1):
        d = np.min(np.linalg.norm(c[:, None, :] - c[pick][None, :, :], axis=2), axis=1)
        pick.append(int(np.argmax(d)))
    return [tuple(c[i]) for i in pick]

def laminate(poly, nuts=(), nut_layer=1, cbores=(), cbore_layer=1, extra=None):
    """Split a plate into its two 3 mm layers. nuts: threaded holes -> clearance hole in the head-side layer, square
    nut pocket in nut_layer. cbores: button-head counterbores in cbore_layer (plain M4 hole in the other layer)."""
    L = {1: poly, 2: poly}
    head = 3 - nut_layer
    if nuts:
        L[head] = L[head].difference(unary_union([circ(*p, M4) for p in nuts]))
        pockets = unary_union([_sq(p, SQNUT_POCKET) for p in nuts])
        pockets = pockets.buffer(1.6, join_style=2).buffer(-1.6, join_style=2)   # pockets < 3.2 mm apart merge into one slot
        L[nut_layer] = L[nut_layer].difference(pockets)
    if cbores:
        L[cbore_layer] = L[cbore_layer].difference(unary_union([circ(*p, CBORE_D) for p in cbores]))
    for k, g in (extra or {}).items():            # e.g. the servo insert's lip recess in the inner layer
        L[k] = L[k].difference(g)
    dw = dowel_pts(L[1].intersection(L[2]), list(nuts) + list(cbores))
    if dw:
        L = {k: v.difference(unary_union([circ(*p, DOWEL_D) for p in dw])) for k, v in L.items()}
    return L[1], L[2], dw

def laminate_specs(plates2d):
    """(file name, 2D plate, nuts, nut layer, counterbores, counterbore layer, what layer 1 / layer 2 face)."""
    tap = lambda name: [m[1] for m in DXF_MARKS.get(name, ((), ()))[0] if m[0] == "TAP_M4"]
    return [
        ("side_plate_servo", plates2d["side_plate_servo"], [] if SERVO_INSERT else tap("side_plate_servo"), 1, [], 1,
         ("inner (ball-corridor side)", "outer (servo side)"), {1: servo_insert_2d()[1].buffer(0.15)} if SERVO_INSERT else None),
        ("side_plate_motor", plates2d["side_plate_motor"], tap("side_plate_motor"), 2, [], 1, ("outer (motor-pulley side)", "inner"), None),
        ("deck", plates2d["deck"], tap("deck"), 1, [], 1, ("bottom", "top"), None),
        *([] if GB_TURRET else [("base_plate", plates2d["base_plate"], [], 1, [], 1, ("bottom", "top"), None)]),
        ("hood_carrier_servo", plates2d["hood_carrier"], [], 1, CARRIER_CSK + HOOD_GEAR_CSK, 1, ("ball side", "outer (hood-gear side)"), None),
        ("hood_carrier_motor", plates2d["hood_carrier"], [], 1, CARRIER_CSK, 2, ("outer", "ball side"), None),
    ]

def fit_test_2d():
    """v7: a 1/8in plywood coupon to tune the laser before cutting the plates. Rows (left to right, -0.2 .. +0.2 mm):
    1611 bearing holes around BRG_1611, square-nut pockets around SQNUT_POCKET, dowel holes around DOWEL_D, and M4
    clearance holes around M4. Returns (coupon, [(text, (x, y))] labels for an ENGRAVE layer)."""
    steps = [-0.2, -0.1, 0.0, 0.1, 0.2]
    W, H = 5 * 22 + 8, 4 * 20 + 14
    part = box(0, 0, W, H).buffer(3, join_style=1).buffer(-3, join_style=1)
    holes, labels = [], []
    rows = [("bearing", BRG_1611, "c"), ("nut", SQNUT_POCKET, "s"), ("dowel", DOWEL_D, "c"), ("M4", M4, "c")]
    for r, (name, nom, kind) in enumerate(rows):
        y = H - 14 - r * 20
        labels.append((f"{name} {nom:.1f}", (2, y + 9)))
        for k, d in enumerate(steps):
            x = 15 + k * 22
            holes.append(circ(x, y, nom + d) if kind == "c" else _sq((x, y), nom + d))
            labels.append((f"{d:+.1f}", (x - 4, y - 9)))
    return part.difference(unary_union(holes)), labels

def laminated_solid(l1, l2):
    """v7: a glued plywood plate as two 3 mm layers and the glue line between them (where both layers have wood)."""
    glue = l1.intersection(l2).buffer(0)
    glue = unary_union([g for g in getattr(glue, "geoms", [glue]) if g.geom_type in ("Polygon", "MultiPolygon") and g.area > 0.01])
    return plate(l1, PLY_LAYER) + Pos(0, 0, PLY_LAYER) * plate(glue, PLY_GLUE) + Pos(0, 0, PLY_LAYER + PLY_GLUE) * plate(l2, PLY_LAYER)

def motor_adapter_print(world_coords=False):
    """v7: the motor adapter is printed (PA-CF). Its 4 clamp holes get M4 square-nut pockets from the motor side. The
    0.8 mm floor under each nut is clamped flat against the side plate by that same bolt, so it only sees compression."""
    o = (lambda p: motor_rel(p)) if world_coords else (lambda p: p)
    outline = motor_adapter_2d(world_coords)
    sol = plate(outline, ADAPT_T)
    for p0 in LUG_BOLTS + CLAMP_BOLTS:
        p = o(p0)
        sq, wall = adapter_pocket(outline, p)
        sol = sol - Pos(0, 0, ADAPT_T - SQNUT[1] - 0.2) * plate(sq, SQNUT[1] + 0.25)
    return sol

def adapter_pocket(outline, p):
    """Square-nut pocket turned to leave the most plastic between it and the adapter's edge (v7 Astra round 2: the
    lower clamp pocket left 0.15 mm to the crossbar notch). Returns (pocket, wall)."""
    ext = unary_union([outline.exterior] + [r for r in outline.interiors if not Polygon(r).contains(Point(*p))])
    best = None
    for a in range(0, 90, 3):
        sq = affinity.rotate(_sq(p, SQNUT_POCKET), a, origin=tuple(p))
        wall = round(ext.distance(sq), 2)            # rounded, so the print and the assembly pick the same angle
        if best is None or wall > best[1]:
            best = (sq, wall)
    return best

def main():
    for sub in ("dxf", "stl", "step", "viewer"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)
    parts, plates2d = build()
    finished = {**({} if LAMINATE else {   # v7: laminated plates export from their layers (below)
                "hood_carrier_servo": countersunk(plate(plates2d["hood_carrier"]), CARRIER_CSK + HOOD_GEAR_CSK, 0.0),
                "hood_carrier_motor": countersunk(plate(plates2d["hood_carrier"]), CARRIER_CSK, T),
                "side_plate_motor_finished": countersunk(plate(plates2d["side_plate_motor"]), FW_MOTOR_BOLTS if FW_MOTOR_CSK else [], 0.0)}),
                "swing_arm_servo": countersunk(plate(plates2d["swing_arm"]), ARM_CSK, 0.0, cone=True),
                "swing_arm_motor": countersunk(plate(plates2d["swing_arm"]), ARM_CSK, T, cone=True)}
    for name, sol in finished.items():      # plate + countersinks, as they'll be after machining
        export_step(sol, str(OUT / "step" / f"{name}.step"))
    for f in (OUT / "dxf").glob("*.dxf"):
        f.unlink()
    LAMINATED = {"side_plate_servo", "side_plate_motor", "deck", "base_plate", "hood_carrier"} if LAMINATE else set()
    PRINTED_PLATES = {"swing_arm", "turret_ring"} if LAMINATE else set()   # v7: both printed in PA-CF
    for name, poly in plates2d.items():
        if name in LAMINATED or name in PRINTED_PLATES:
            continue
        marks, notes = DXF_MARKS.get(name, ((), ()))
        write_dxf(poly, OUT / "dxf" / f"{name}.dxf", marks, notes)
    if LAMINATE:
        DOWELS.clear()
        for fname, poly, nuts, nl, cb, cl, faces, extra in laminate_specs(plates2d):
            l1, l2, dw = LAYERS[fname]                    # the same layers build() made the 3D plates from
            export_step(laminated_solid(l1, l2), str(OUT / "step" / f"{fname}.step"))
            DOWELS[fname] = dw
            for k, lp in ((1, l1), (2, l2)):
                extra = []
                if nuts and k == nl:
                    extra.append(f"{len(nuts)} square pockets: press an M4 square nut (DIN 562) into each before gluing.")
                if cb and k == cl:
                    extra.append(f"{len(cb)} 8.2 mm counterbores: M4 button heads sit in them, flush with this face.")
                write_dxf(lp, OUT / "dxf" / f"{fname}_L{k}.dxf", (), [
                    f"{fname}, layer {k} of 2: the {faces[k - 1]} face. 1/8 in (3 mm) Baltic birch.",
                    "Glue to the other layer (wood glue, grain crossed 90 deg), 1/8 in dowels in the 3 small holes, clamp flat overnight."] + extra)
    if LAMINATE:
        cp, labels = fit_test_2d()
        write_dxf(cp, OUT / "dxf" / "fit_test.dxf", (), [
            "Fit test, 1/8 in Baltic birch: cut this first. Bearing: a 1611 should press in by hand and not wobble.",
            "Nut: an M4 square nut presses in and can't turn. Dowel: 1/8 in dowel is a snug push fit. Pick the best",
            "column in each row and set BRG_1611 / SQNUT_POCKET / DOWEL_D / M4 in turret.py (or tell the design lead)."])
        doc = ezdxf.readfile(OUT / "dxf" / "fit_test.dxf")
        if "ENGRAVE" not in doc.layers:
            doc.layers.add("ENGRAVE", color=4)
        for t, (x, y) in labels:
            doc.modelspace().add_text(t, height=2.5, dxfattribs={"layer": "ENGRAVE"}).set_placement((x, y))
        doc.saveas(OUT / "dxf" / "fit_test.dxf")
    if MOTOR_ADJ:
        if LAMINATE:
            export_step(motor_adapter_print(), str(OUT / "step" / "motor_adapter.step"))
        else:
            marks, notes = DXF_MARKS["motor_adapter"]
            write_dxf(motor_adapter_2d(), OUT / "dxf" / "motor_adapter.dxf", marks, notes)
            export_step(plate(motor_adapter_2d(), ADAPT_T), str(OUT / "step" / "motor_adapter.step"))
    if FRONT_PLATE:
        write_dxf(front_plate_2d()[0], OUT / "dxf" / "front_plate.dxf", (),
                  ["Optional flair plate, 3 mm (basswood/plywood on a diode laser, or print).",
                   "Zip-tie to the front crossbar through the slot pairs. Decoration only, not a ROBOT SIGN."])
        export_step(plate(front_plate_2d()[0], FRONT_PLATE_T), str(OUT / "step" / "front_plate.step"))
    for name, poly in plates2d.items():     # every plate as STEP (v6 fix: only the last one was exported)
        if name not in ("hood_carrier", "side_plate_motor") and not (LAMINATE and name in ("side_plate_servo", "deck", "base_plate")):
            export_step(plate(poly), str(OUT / "step" / f"{name}.step"))
    printed = {**({} if GB_TURRET else {"roller_tire": tire(), "roller_spacer": roller_spacer(), "roller_cap": roller_cap(), "deck_spacer": deck_spacer(),
                   "roller_mid_spacer": roller_mid_spacer()}),
               "cf_spring_drum": cf_drum(), "cf_axle": cf_axle(), "stop_cam": eccentric_stop(),
               "guide_spacer": guide_spacer(),
               **({"encoder_sleeve": encoder_sleeve(), "encoder_bracket": plate(encoder_bracket_2d(), BRKT_T)} if ENCODER else {}),
               "spring_post": spring_post(), "l_bracket": l_bracket(), "crossbar": crossbar(),
               **({"rear_guard": rear_guard()} if REAR_GUARD else {}),
               **({"servo_insert": servo_insert()} if SERVO_INSERT else {}),
               **({"pulley_24t": printed_pulley(JACK_PULLEY_T, PUL_SS_W), "pulley_48t": printed_pulley(MAIN_MOTOR_PULLEY_T, PUL48_W, True),
                   **{f"spacer_{SPACER_LEN[k]:g}mm": printed_spacer(SPACER_LEN[k]) for k in spacer_counts()}} if PRINT_DRIVE else {}),
               **({"swing_arm_servo": countersunk(plate(plates2d["swing_arm"]), ARM_CSK, 0.0, cone=True),
                   "swing_arm_motor": countersunk(plate(plates2d["swing_arm"]), ARM_CSK, T, cone=True),
                   **({"motor_adapter": motor_adapter_print()} if MOTOR_ADJ else {})} if LAMINATE else {}),
               "limelight_mount": ll_mount(),
               "cf_anchor": anchor_solid(False),
               "cf_clamp": clamp_solid(False),
               # v6.1 bumper: PETG core (heat-set bore) + TPU ring pressed on
               "arm_bumper_core": revolve_profile(0, BUMPER_R - 2.0, X_ARM - X_CAR) - revolve_profile(0, INSERT_HOLE_D / 2, INSERT_DEPTH)
                                  - revolve_profile(0, 2.25, X_ARM - X_CAR + 0.01),
               "arm_bumper_ring": revolve_profile(BUMPER_R - 2.0 - 0.1, BUMPER_R, X_ARM - X_CAR),
               **({"encoder_bracket_leg": ref_cyl(10.0, encoder_x()[1][0] - SIDE_X_OUT, M4)} if ENCODER else {}),
               **({} if GB_TURRET else {"turret_ring_print": plate(gear(RING_TEETH, RING_MODULE).difference(circ(0, 0, RING_ID)).difference(
                   unary_union([circ(*pol(DECK_SPACER_R, math.radians(a)), INSERT_HOLE_D) for a in DECK_SPACER_ANGS])), T if LAMINATE else 2 * 3.175)}),
               **({"switch_arm": Rot(0, 0, -HOME_ANG) * gb_switch_arm(), "deck_shim": gb_shim()} if GB_TURRET else {}),
               **({"front_plate": plate(front_plate_2d()[0], FRONT_PLATE_T)} if FRONT_PLATE else {}),
               **({n: (lambda s: Pos(-s.bounding_box().min.X, -s.bounding_box().min.Y, -s.bounding_box().min.Z) * s)(f())
                   for n, f in (("motor_lug", motor_lug), ("motor_yoke", motor_yoke))} if MOTOR_ADJ else {}),
               **({f"cable_tray_{k}": cable_tray(k) for k in range(TRAY_N)} if CABLES else {}),
               **({"home_flag": Rot(0, 0, -HOME_ANG) * home_flag()} if HOME_FLAG else {}),
               **({f"cable_drum_{k}": cable_drum(k) for k in range(DRUM_N)} if CABLES else {})}
    for f in (OUT / "stl").glob("*.stl"):       # names carry the filament now: clear the old ones
        f.unlink()
    for name, s in printed.items():
        export_stl(s, str(OUT / "stl" / stl_file(name)))
        export_step(s, str(OUT / "step" / f"{name}.step"))

    Mw = (FW_Y, FW_Z)
    meta = {
        "parts": [], "fw": list(Mw), "jack": list(world(J)), "s_pollen": list(world(S_P)), "s_nectar": list(world(S_N)),
        "arm_delta": TH_N - TH_P, "r1": R1, "r2": R2, "compression": COMPRESSION, "min_squeeze": MIN_SQUEEZE, "cf_springs": CF_SPRINGS_PER_SIDE, "open_stop": OPEN_STOP_DEG,
        "spring_a": list(world(SPRING_A)), "spring_b": list(world(SPRING_B[0])),
        "spring_x": -(X_DRV + DRIVE_W / 2), "alpha_max": HOOD_ALPHA_MAX,
        "psi_pollen": psi_exit(S_P), "psi_nectar": psi_exit(S_N),
        "balls": {"pollen": POLLEN_D, "nectar": NECTAR_D}, "servo_axis": list(SERVO_AXIS),
        "sector_ratio": SECTOR_TEETH_FULL / SERVO_PINION_TEETH, "sync_ratio": SYNC_TEETH_J / SYNC_TEETH_S, "motor_ratio": MAIN_MOTOR_PULLEY_T / MOTOR_PULLEY_T,
        "turret_pinion": list(pol(GB_DRIVE_R, math.radians(GB_ANGLE))) if GB_TURRET else list(pol(PIN_CENTER_R, math.radians(TURRET_MOTOR_ANGLE))),
        "turret_ratio": GB_RATIO if GB_TURRET else RING_TEETH / PINION_TEETH, "fw_motor": list(FW_MOTOR), "fw_motor2": list(FW_MOTOR2) if SECOND_MOTOR else None,
        "ll_pivot": list(LL_PIVOT), "ll_tilt": LL_TILT, "ll_range": list(LL_TILT_RANGE), "ll_fov": list(LL_FOV),
        "belts": {"jack": JACK_BELT, "motor": MOTOR_BELT}, "sweep_r": 0.0, "base_z": 0.0 if GB_TURRET else -T, "width": 2 * SIDE_X_OUT,
        "idler_slots": [list(v) for v in IDLER_SLOTS], "idler_anchors": [list(a) for a in IDLER_ANCHORS],
        "idler_x": SIDE_X_OUT + 4.5,
        "cable": {"travel": CABLE_TRAVEL, "rin": C_RIN, "rout": C_ROUT, "in_ang": CABLE_IN_ANG, "out_ang": C_OUT_ANG,
                  "len": CABLE_LEN, "home": HOME_ANG} if CABLES else None}
    assembly = []
    for name, (shape, group, color, kind, spin) in parts.items():
        export_stl(shape, str(OUT / "viewer" / f"{name}.stl"), tolerance=0.05, angular_tolerance=0.2)
        lab, cat = part_label(name, kind)
        meta["parts"].append({"name": name, "group": group, "color": color, "kind": kind, "spin": spin, "label": lab, "cat": cat})
        assembly.append(shape)
        if group != "fixed":
            bb = shape.bounding_box()
            for x in (bb.min.X, bb.max.X):
                for y in (bb.min.Y, bb.max.Y):
                    meta["sweep_r"] = max(meta["sweep_r"], math.hypot(x, y))
    meta["bom"] = [list(r) for r in bom()]
    for pm, det in zip(meta["parts"], part_details([p["name"] for p in meta["parts"]], meta["bom"],
                                                   {p["name"]: p["kind"] for p in meta["parts"]}).values()):
        pm["info"] = det
    write_stacks(OUT / "shaft_stacks.md")
    meta["stacks"] = {k: {"length": v["length"], "items": [[i[0], tag_stl(i[1]), *i[2:]] for i in v["items"]]} for k, v in shaft_stacks().items()}
    import csv
    with open(OUT / "bom.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["section", "qty", "part", "sku_or_file", "note"]); w.writerows(bom())
    (OUT / "viewer" / "viewer_meta.json").write_text(json.dumps(meta, indent=1))
    export_step(Compound(assembly), str(OUT / "step" / "turret_assembly.step"))

    top = max(p[0].bounding_box().max.Z for p in parts.values())
    if CABLES:
        lo, hi = cable_u(-CABLE_TRAVEL), cable_u(CABLE_TRAVEL)
        print(f"cables: +/-{CABLE_TRAVEL:.0f} deg, ribbon {CABLE_LEN:.0f} mm in the channel (r {C_RIN:.0f}/{C_ROUT:.0f}, bend r {C_RB:.1f}); "
              f"drum anchor {CABLE_IN_ANG:.0f}, exit {C_OUT_ANG:.1f} deg; U-turn {lo[0]:.0f}..{hi[0]:.0f} deg; "
              f"closest to an anchor {min(lo[1], lo[2], hi[1], hi[2]):.1f} deg; tray OD {2 * TRAY_R[1]:.0f} mm")
    print(f"jackshaft belt HTD {JACK_BELT} mm (C={JACK_C:.1f}); motor belt HTD {MOTOR_BELT} mm (C={MOTOR_C:.1f}, 3:1)")
    print(f"arm {ARM_L:.0f} mm, swing Pollen->Nectar {math.degrees(TH_N - TH_P):.1f} deg")
    print(f"exit elevation: Pollen {90-psi_exit(S_P)-HOOD_ALPHA_MAX:.0f}..{90-psi_exit(S_P):.0f}, "
          f"Nectar {90-psi_exit(S_N)-HOOD_ALPHA_MAX:.0f}..{90-psi_exit(S_N):.0f} deg")
    print("idler rollers (y pollen -> nectar @ z):", [(round(a), round(b), round(z)) for a, b, z in IDLER_SLOTS])
    print(f"width {2*SIDE_X_OUT:.0f} mm, swing radius {meta['sweep_r']:.0f} mm, top z {top:.0f} mm")
    return parts, plates2d

if __name__ == "__main__":
    main()
