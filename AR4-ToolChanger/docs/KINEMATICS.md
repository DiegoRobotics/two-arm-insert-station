# Kinematics

## Verified MK3 parameters
Sources: Annin HMI 6.0 `ARbot.cal` and `kinematics.cpp` (read-only), and `ycheng517/ar4_ros_driver` `annin_ar4_description/config/mk3.yaml` plus the URDF.

| Parameter | Annin HMI 6.0 | ROS MK3 URDF | Match |
|---|---|---|---|
| Base to J2 height (d1) | 169.77 mm | 0.16977 m | ✓ |
| J1 to J2 offset (a) | 64.2 mm | 0.0642 m | ✓ |
| Upper arm (a) | 305 mm | 0.305 m | ✓ |
| Forearm (d4) | 222.63 mm | 0.22263 m | ✓ |
| Flange (d6) | 41 mm | 0.041 m | ✓ (Ekumen: 36.25 ✗) |
| J1 limits | ±170° | ±170° | ✓ |
| J2 limits | −42 / +90° | −42 / +90° | ✓ |
| J3 limits | −89 / +52° | −89 / +52° | ✓ |
| J4 limits | ±180° | ±180° | ✓ |
| J5 limits | ±105° | ±105° | ✓ |
| J6 limits | ±180° | ±180° | ✓ |

Annin uses modified DH (`kinematics.cpp`): α = [0, −90°, 0, −90°, 90°, −90°] and θ offsets = [0, 0, −90°, 0, 0, 180°].
The wrist appears spherical (J4–J6 axes intersect), so a closed-form IK is possible. To be checked in Phase 2.

## Model facts (from the URDF, checked with Pinocchio)
- At J1 = 0 the MK3 faces **−Y**. Home (all joints 0) puts the flange at (−0.007, −0.328, 0.475) m, which equals d1 + a2 = 0.4748 high and a1 + d4 + d6 = 0.3278 out (pytest ✓).
- **Home is a wrist singularity**: with J5 = 0, J4 and J6 are aligned and manipulability is about 0 (pytest ✓). Avoid ending Cartesian moves there.
- TCP (assumed): 36 mm along flange z, at the gripper jaw joint origin.

## Validation method
MoveIt IK (KDL, from upstream `kinematics.yaml`) → Pinocchio FK of the solution → accept only if all of these hold: within joint limits, ≤ 1 mm, ≤ 0.5°, and manipulability ≥ 1e-4. The thresholds are proposed and provisional.

## Results (2026-10-21)
| Test | Result |
|---|---|
| pytest `test_kinematics.py` | 6/6 pass (limits = Annin, link lengths = Annin, pose-error units, singularity, validator rejects, TCP round trip) |
| IK benchmark, 200 random reachable flange poses, seed = home | **195/200 solved (97.5 %)**, 0 rejected by FK check; error mean 0.00035 mm / max 0.0054 mm, max 0.00055°; 3 ms per pose |
| `goto_pose 0.0 -0.30 0.25 0 180 0` | REACHED, final TCP error 0.485 mm / 0.085° |
| `goto_pose 0.0 -0.80 0.25 …` (out of reach) | Correctly refused: "no collision-free IK solution", exit 1 |

## KDL vs pick_ik (2026-10-22, 500 random reachable flange poses, seed = home, mock)
| Solver | Solved + validated | Max pos error | Max orient. error | Mean / max time |
|---|---|---|---|---|
| **KDL** (upstream settings) | **490/500 (98.0 %)** | 0.0079 mm | 0.0006° | **0.002 / 0.024 s** |
| pick_ik (global, 50 ms, 0.1 mm threshold) | 446/500 (89.2 %) | 0.0997 mm | 0.011° | 0.038 / 0.157 s |
| pick_ik tuned (global, 250 ms, 0.01 mm threshold) | 464/500 (92.8 %) | 0.0100 mm | 0.001° | 0.111 / 1.907 s |

**Decision: KDL stays the default** (`ik:=kdl`). pick_ik remains selectable (`ik:=pick_ik`, `config/kinematics_pick_ik.yaml`). Both are set from our launch file; upstream is untouched.

## Cross-check against Annin HMI 6.0 (2026-10-22)
Method: compiled a **copy** of Annin's `kinematics.cpp`/`bindings.cpp` in WSL (GCC needs `-fpermissive`), loaded the DH table the HMI stores in `ARbot.cal`, and compared with the URDF (Pinocchio) on 500 random joint sets. Repeat with `tools/annin_fk_check/annin_fk_check.sh`.

| Check | Result |
|---|---|
| Annin FK copy vs the HMI's own saved pose (J = 0, −41.832, −4.284, 0, 89.981, 0) | X 44.668 / Y 0.000 / Z 529.075 mm, **exact match** |
| Joint mapping | **Annin J1 = −URDF J1**; J2–J6 identical; no offsets |
| Base frames | Annin X = −URDF Y, Annin Y = URDF X, Z equal |
| Flange frames | Annin flange = URDF `ee_link` rotated +90° about z; constant within 0.0036° |
| **Position difference** | **7.000 mm constant, sideways** (min 6.998, max 7.001) |

Cause of the 7 mm: the URDF's J3 origin is `xyz="0 -0.305 0.007"`, an offset along the J3 axis. Annin's DH table has none. **Unresolved:** which matches the physical arm. Measure it on the robot before using HMI coordinates in ROS or vice versa.
Note: Annin's orientation output is a rotation vector (rad). Its float32 conversion becomes unstable within about 0.05 rad of 180° (an Annin-side issue).
Note: `robot_set()` defaults in `kinematics.cpp` (α6 = 180°) do **not** match the HMI; the HMI uses the `ARbot.cal` table (α6 = −90°).
