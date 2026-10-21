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

The 5 IK misses are probably KDL starting from the singular home seed. Next step: try pick_ik, overriding the kinematics plugin in our own launch file (not by editing upstream).
Not yet done: comparing FK against Annin's own `kinematics.cpp` on the same joint angles, which is needed before using HMI coordinates.
