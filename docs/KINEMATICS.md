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

The parameter match was checked by comparing values only. No FK comparison has been run yet. The joint-zero convention and the sign of each joint between Annin and the URDF still need to be verified in Phase 2.
