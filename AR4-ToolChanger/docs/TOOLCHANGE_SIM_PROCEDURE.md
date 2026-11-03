# Simulated Tool Change — Procedure

## Current version (2026-10-22): tool-changer model, no gripper
| Item | What changed |
|---|---|
| Robot model | `urdf/ar4tc.urdf.xacro`: upstream MK3 arm + **tool-changer master** (Ø50 × 20 mm cylinder on the flange) + `tc_mating` frame at its face. **No gripper.** SRDF `srdf/ar4tc.srdf.xacro` adds the master's collision pairs. Used in mock and bridge modes (Gazebo keeps the upstream gripper model and refuses `run:=true`) |
| Dock | Base plate + **two cradle posts per slot**. Each tool's coupling plate rests on its posts with the body hanging between them |
| Tools | `tool_a` "Gripper tool": plate + box body + 2 fingers, orange. `tool_b` "Generic tool (probe)": plate + cylinder + tip, blue. Each has its own **TCP** (`tcp` TF frame follows the held tool) and payload (logged) |
| Collision checking | **On for every move, including docking.** The coupling face stops `dock_gap` = 1 mm above the plate; docked tools rest 1 mm above their posts |
| Sequence | `sequence:` list in the config (`home`, `{pick: tool}`, `{place: tool}`) |
| HOME | = HMI `Home.ar4` (J5 = 90°, flange down) |
| RViz | `rviz/ar4tc.rviz`: upstream MoveIt view + slot labels + `tc_mating`/`tcp` axes. Dock and tools are coloured scene objects |
| Recording | Every run writes `~/ar4tc_reports/program_*.json` for the HMI export (HMI_EXPORT.md) |
| Configs | `extends:` lets a config override another one (e.g. `toolchanger_real_test.yaml` = +100 mm dock, 10 % speed) |

Results (mock, 2026-10-22): **DONE 15/15** in 52.7–59.8 s, twice in a row in one simulation; coupling-face error ≤ 0.46 mm (free moves) and 0.000 mm (straight docking moves); placement error 0.000 mm. Through the bridge emulator: DONE, 1,078 MoveIt points → 30 RJ segments. Raised-dock config: DONE, 120.2 s.

Run it: `ros2 launch ar4tc_toolchanger toolchange_demo.launch.py` (RViz + sequence), or `bringup.launch.py` then `ros2 run ar4tc_toolchanger toolchange_demo [config.yaml]`.

---
## Earlier version (2026-10-21): kept for the record
(It used the upstream servo-gripper model, placeholder box tools and collision checking off during docking. Superseded by the version above.)

Goal: in simulation, the MK3 goes HOME → picks **tool_a** → HOME → returns tool_a to its slot → picks **tool_b** → HOME. Every step is checked independently.

## 1. Run it
In the **Ubuntu** app:
```bash
bash "/mnt/c/Users/gael1/OneDrive/Documents/AR4 MK3 Engiworks Gael/AR4-ToolChanger/setup/build.sh"   # sync + build + tests
ros2 launch ar4tc_toolchanger toolchange_demo.launch.py          # RViz opens; the sequence starts by itself
```
Other commands (start the sim first with `ros2 launch ar4tc_toolchanger toolchange_demo.launch.py run:=false`):
```bash
ros2 run ar4tc_toolchanger toolchange_demo          # rerun the sequence (restart the sim first so tools are back in slots)
ros2 run ar4tc_toolchanger goto_pose 0.0 -0.30 0.25 0 180 0   # X Y Z (m) roll pitch yaw (deg), TCP in base_link
ros2 run ar4tc_toolchanger ik_benchmark 200
```
Each sequence writes a JSON report to `~/ar4tc_reports/`.

## 2. What was built and why
| Step | What | Why |
|---|---|---|
| 1 | Generated the MK3 URDF (`xacro fake_ar.urdf.xacro ar_model:=mk3`) and inspected frames with Pinocchio | To learn the real geometry before choosing positions. **Found:** at J1 = 0 the MK3 faces **−Y**; home (all joints 0) puts the flange at (−0.007, −0.328, 0.475) m; the flange z-axis points out of the tool face; the gripper jaws sit **36 mm** along flange z. |
| 2 | `kinematics.py`: Pinocchio FK, Jacobian manipulability, limit check, pose error | IK answers from MoveIt are re-checked by separate code, so a solver bug can't move the robot unnoticed. The upstream URDF has `effort="-1"`, which Pinocchio rejects, so it's patched in memory only. |
| 3 | `motion.py`: wrapper for MoveIt services and actions (`/compute_ik`, `/move_action`, `/compute_cartesian_path`, `/execute_trajectory`, planning scene, gripper action) | One place for all MoveIt calls. Swapping in the real robot later changes nothing here. |
| 4 | `scene.py`: dock and two tool boxes as MoveIt collision objects; attach/detach | MoveIt plans around the dock and tools and carries the attached tool's collision shape with the arm. |
| 5 | `toolchange_demo.py`: state machine (HOME, APPROACH, OPEN, DESCEND, COUPLE, RELEASE, RETREAT, DONE/FAULT) | Each state has an action plus a verification. Any failed check goes to FAULT and stops. |
| 6 | `config/toolchanger.yaml` | All positions, tolerances and speeds are in one editable file, not in code. |
| 7 | `test/test_kinematics.py` (6 pytest tests) | Checks that the model matches Annin HMI 6.0 limits and link lengths, and that the singularity and error checks work. |

## 3. Layout (placeholder, not from CAD)
Robot's right = **−X** (it faces −Y). Dock 100 × 300 × 60 mm centred at (−0.28, −0.13, 0.03) m. Tools are 30 × 30 × 70 mm boxes standing on the dock:
- tool_a slot (−0.28, −0.20)
- tool_b slot (−0.28, −0.06)

To move them, edit `slot_xy` in the YAML and rebuild.

## 4. How one tool pick works
1. **APPROACH:** TCP target = 100 mm above the coupling pose, pointing straight down. Yaw follows the slot direction so the wrist stays natural. MoveIt IK (collision-aware) runs first, then the Pinocchio check: limits, FK error ≤ 1 mm / 0.5°, manipulability ≥ 1e-4. Then a joint-space plan and execution (OMPL) at 30 % speed, and finally the measured TCP error is checked.
2. **OPEN:** gripper to 14 mm; jaw position confirmed from `/joint_states`.
3. **DESCEND:** 100 mm straight-line move down to the coupling pose (TCP 15 mm below the tool's top). Collision checking is **off for this segment only**, because the docking move is meant to touch the tool.
4. **COUPLE:** gripper closes, then the tool is attached to the flange in the planning scene. Confirmed by reading the scene back: the tool is in the attached list and gone from the world.
5. **RETREAT:** straight-line move 100 mm up, then the TCP error is checked.

**Place** is the same with RELEASE instead of COUPLE. RELEASE detaches the tool and confirms it's back in the world within 2 mm of its slot centre. **HOME** is a joint goal (all zeros), checked to within 0.5° per joint.

## 5. Results (3 identical runs, mock hardware)
| Check | Result |
|---|---|
| Sequence | **DONE, 17/17 steps**, 52.7 s, ends at HOME holding tool_b |
| Joint-space approach, TCP error | 0.24–0.38 mm, ≤ 0.10° (limited by MoveIt's 0.001 rad joint goal tolerance) |
| Straight-line moves, TCP error | 0.000 mm / 0.000° |
| tool_a placement error | 0.000 mm (scene read-back: tool_a at its slot centre, −0.28, −0.20, 0.095) |
| tool_b carried | Attached to `ee_link` at z = 56 mm (= 36 TCP + 15 depth + 5 to centre ✓) |
| Manipulability at the approach poses | 0.010–0.015 (well above 1e-4) |
| Unit tests | 6/6 pass |

## 5b. Gazebo run (2026-10-22): `toolchange_demo.launch.py mode:=gazebo`
| Check | Result |
|---|---|
| Sequence | **DONE, 17/17**, 59.0 s (report `docs/reports/toolchange_gazebo_20261022_092816.json`) |
| TCP error, joint-space approach | 0.23–0.42 mm, ≤ 0.15° |
| TCP error, straight-line moves | 0.005–0.12 mm, ≤ 0.08° |
| tool_a placement error | 1.58 mm (limit 2 mm) |
| Jaw position reached | 13.1 mm open (target 14 mm, 1 mm tolerance); 4.9–5.0 mm closed |

First Gazebo attempt failed at OPEN (jaw still moving). That's a real timing effect the mock hid; fixed by waiting for the jaw (TROUBLESHOOTING.md).

## 5c. Launch options (2026-10-22)
`bringup.launch.py mode:=mock|gazebo|annin|annin_emu ik:=kdl|pick_ik run:=true|false rviz:=true|false include_gripper:=True|False`
`toolchange_demo.launch.py` = bringup with `run:=true`. `mode:=annin` (real robot) refuses `run:=true`. The earlier `mode:=real` (ROS firmware) was removed on 2026-10-22 (REAL_ROBOT.md).
Real-robot dry-run config: `config/toolchanger_real_test.yaml` (10 % speed, dock +100 mm, no gripper). Mock run: DONE 17/17, 110.9 s.

## 6. Limits of this result
- **Kinematic simulation only.** Mock hardware follows commands perfectly. There's no physics, no contact forces and no real servo lag. The 0.000 mm figures show the logic is consistent, not that the real robot will be accurate to that level.
- Dock and tool dimensions and the 36 mm TCP are **assumptions**. They get replaced by SolidWorks geometry (Phase 4) and a measured TCP.
- Collision checking is off during the 100 mm docking moves. On hardware this segment will need slow speed and force or position monitoring.
- Gazebo checks the arm's control and dynamics only. The dock and tools exist in MoveIt, not in the Gazebo world, so no contact physics is simulated yet.
