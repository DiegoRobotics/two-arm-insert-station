# Research — Phase 1

Repos inspected 2026-10-20 (shallow clones of default branch).

## Repository comparison

| | ongdexter/ar3_core | Ekumen-OS/ar4 | ycheng517/ar4_ros_driver *(added)* | wheresryan22/anatomy |
|---|---|---|---|---|
| Last commit | 2021-08 | 2025-12 | 2026-04 | 2026-10 |
| ROS | ROS 1 Melodic | ROS 2 **Humble** (Dockerfile `osrf/ros:humble-desktop-full`) | ROS 2 **Jazzy** (main), `humble` branch | n/a (Node ≥20.10) |
| Robot | AR3 | AR4 — model choice `ar4` only | AR4 **mk1 / mk2 / mk3** (`ar_model`, default `mk3`) | n/a |
| MK3 params/meshes | no | **no** — `j4 ±165°`, `j6 ±155°`, `l6_length 0.03625` (MK1-era) | yes — `config/mk3.yaml`: `j4/j6 ±180°`, `l6_length 0.041`, `ar4_mk3` meshes | n/a |
| Gripper | no | no | Annin servo gripper (Arduino Nano) | n/a |
| Sims | Gazebo Classic | Gazebo, MuJoCo (pkg exists; top README says "not yet available", bringup README has launch steps), Isaac Sim | Gazebo | n/a |
| MoveIt | MoveIt 1 | MoveIt 2, KDL IK, group `arm` → `link_6` | MoveIt 2, KDL IK, group `ar_manipulator` | n/a |
| Firmware | AR3 Teensy | Teensy driver in C++, **no firmware in repo** | Teensy + Nano sketches included | n/a |
| License | — | BSD-3 / MIT | MIT | MIT |

Ekumen's `ar4_description` and `ar4_hardware_interface` say they are derived from `ycheng517/ar4_ros_driver`, so I added that repo to the comparison.

## Findings

1. **Ekumen doesn't model the MK3.** Its joint limits and flange length match the older AR4. Using it unchanged would put the TCP about 4.75 mm off along the flange axis and limit J4/J6 too much. The URDF origins for J1–J3 match ycheng's.
2. **Both ROS 2 drivers need ROS-specific Teensy firmware.** It replaces Annin's stock firmware. ycheng ships the sketch; Ekumen doesn't. Flashing it means the Annin ARCS software can't drive the robot until you reflash the stock firmware, and the robot has to be recalibrated after each flash or power cycle. MK3 also needs the DIP-switch settings introduced after ARCS v6.1 (`docs/ar4_dip_switch_settings.png`). **This only matters in Phase 5.**
3. **Humble reaches end of life in May 2027.** Jazzy is supported until 2029 and runs on Ubuntu 24.04.
4. **KDL IK is the default in both drivers.** For a 6-DOF arm it's slow and finds only one solution near the seed. pick_ik or TRAC-IK is better, and the analytical wrist solution should be checked in Phase 2.
5. **Windows checkout breaks Ekumen's MuJoCo meshes.** The repo has both `link_6.STL` and `link_6.stl`, which collide on a case-insensitive file system. Clone inside a Linux file system (WSL ext4), not under `C:\`.
6. **Anatomy is a Claude skill, not CAD.** It draws interactive isometric SVG/WebGL explainer figures. Use it to communicate coupling and locking concepts and sequences, not for dimensions. It's already installed in this Claude Code environment. SolidWorks stays the source of truth for geometry.
7. **ar3_core is reference only.** It's ROS 1, AR3 hardware and unmaintained. Its hardware-interface layering is the only part worth borrowing as a pattern.

## User constraints (2026-10-21)
- Robot: AR4 MK3, Teensy 4.1, operated with **Annin HMI 6.0**, which must stay unchanged. So ROS drives the simulation only. Driving the real robot (Phase 5) needs a separate decision: flash the ROS firmware with an easy way back, or write a bridge that speaks Annin's serial protocol.
- Gripper: Annin servo gripper. The user is open to a simpler or better design for the tool-changer tools.
- CAD: SolidWorks 2026 Student Edition.
- No GitHub. A local git repo lives in `AR4 MK3 Engiworks Gael/AR4-ToolChanger`.

## Decisions (approved 2026-10-21)

| Topic | Choice | Why |
|---|---|---|
| Robot model and driver | **ycheng517/ar4_ros_driver (Jazzy), pinned commit**, used unmodified as an upstream dependency | Only stack with MK3 parameters and meshes, the gripper and matching firmware |
| Ekumen | Borrow selectively (MuJoCo MJCF pipeline, launch layering), port to MK3 if needed | Preferred by user, but Humble-only and no MK3 model |
| Primary sim | **MoveIt 2 + RViz with mock hardware** → **Gazebo Harmonic** | Tool changing only needs kinematic attach/detach for now. Physics comes later |
| Contact physics (optional, Phase 4) | MuJoCo | Better for checking coupling insertion and contact. Not needed for the first milestone |
| IK (planning) | MoveIt plugin: **pick_ik** (fallback TRAC-IK, then KDL) | Packaged for Jazzy, multiple solutions, tunable |
| IK validation | **Pinocchio** (FK, Jacobian, manipulability), independent of MoveIt | Cross-checks the solver against limits, pose error and singularities |
| Platform | **WSL2 Ubuntu 24.04 + ROS 2 Jazzy installed with apt** (no Docker at first) | Fewest layers. WSLg provides GUI and GPU. usbipd-win passes the Teensy USB later |
| Isaac Sim | Rejected | Heavy, and adds nothing here |

Package availability on Jazzy (pick_ik, Pinocchio, Gazebo Harmonic) still has to be confirmed at install time.
