# Simulated Tool Change — Procedure (Phase 2, 2026-10-21)

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

## 6. Limits of this result
- **Kinematic simulation only.** Mock hardware follows commands perfectly. There's no physics, no contact forces and no real servo lag. The 0.000 mm figures show the logic is consistent, not that the real robot will be accurate to that level.
- Dock and tool dimensions and the 36 mm TCP are **assumptions**. They get replaced by SolidWorks geometry (Phase 4) and a measured TCP.
- Collision checking is off during the 100 mm docking moves. On hardware this segment will need slow speed and force or position monitoring.
- Gazebo isn't used yet. RViz with mock hardware is enough for this milestone.
