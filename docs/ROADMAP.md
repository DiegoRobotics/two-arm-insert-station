# Roadmap

| Phase | Status |
|---|---|
| 1 Research and environment setup | **Done** 2026-10-21 (Gazebo GUI not yet opened) |
| 2 Digital twin and kinematics | **Milestone 1 met** 2026-10-21 (`goto_pose`, IK benchmark; see KINEMATICS.md) |
| 3 Virtual tool changer | **First demo working** 2026-10-21: 2 placeholder tools, pick/return/pick sequence (TOOLCHANGE_SIM_PROCEDURE.md) |
| 4 Mechanical development | Not started |
| 5 Physical integration | Bridge to Annin v6.0 built and emulator-tested; **first hardware session pending** (REAL_ROBOT.md) |

## Phase 1 checklist
- [x] User approves stack (see RESEARCH.md)
- [x] MK3 parameters cross-checked against Annin HMI 6.0 (KINEMATICS.md)
- [x] Setup script written (`setup/setup_wsl_ros.sh`) and run
- [x] WSL 3.0.3 + Ubuntu 24.04.5 installed (no reboot needed)
- [x] ROS 2 Jazzy, MoveIt 2, Gazebo (ros_gz), pick_ik, Pinocchio installed
- [x] ar4_ros_driver @ 4b88db1 built (4 packages; deprecation warnings only)
- [x] Smoke test 2026-10-21 (below)
- [x] Backup: `C:\Users\gael1\WSL-backups\Ubuntu-24.04_ros-jazzy_ar4mk3_2026-10-21.tar` (6.81 GB, 194,714 entries, archive listing OK)

## Smoke test results (2026-10-21, `demo.launch.py ar_model:=mk3`, mock hardware)
| Check | Result |
|---|---|
| Hardware plugin | `mock_components/GenericSystem`. No serial or robot connection |
| move_group ready | About 4 s |
| Controllers | joint_trajectory, gripper, joint_state_broadcaster all `active` |
| FK→IK→FK round trip (KDL, 1 pose, q=[0.3,0.4,−0.3,0.5,0.6,−0.4] rad) | IK success; 0.0000 mm / 0.0000° |
| MoveGroup plan + execute to a joint goal | SUCCESS, 29 points, final joint error ≤ 0.05° |
| Known harmless error | `No 3D sensor plugin(s) defined for octomap updates` (no camera configured) |

Not yet tested: Gazebo launch, pick_ik plugin, Cartesian pose targets, and multi-pose statistics. These are Phase 2 work.

## Next
- [x] pick_ik vs KDL comparison: KDL kept (KINEMATICS.md), 2026-10-22
- [x] FK cross-check against Annin `kinematics.cpp`: match except a 7 mm URDF J3 offset (KINEMATICS.md)
- [x] Gazebo run of the sequence: DONE 17/17 (TOOLCHANGE_SIM_PROCEDURE.md)
- [x] Real robot route chosen: **B**, bridge to Annin v6.0 (2026-10-22)
- [x] `annin_bridge` built; emulator tests: sequence DONE 17/17, fault test 14/14 (ANNIN_BRIDGE.md)
- [ ] First hardware session following REAL_ROBOT.md (confirm unknowns: J1 sign, USB attach, CL, timing)
- [ ] Measure the 7 mm J3 offset on the physical arm
- [ ] Per-tool TCP and payload switching (tool registry), tool-specific collision shapes
- [ ] Replace placeholder dock and tools with SolidWorks geometry (Phase 4)

## Milestone 1 (Phase 2) — acceptance criteria (met; this section is kept as the reference)
`ros2 run ar4tc_motion goto_pose 0.25 0.0 0.30 180 0 0` (example only, reachability not checked) shows the simulated MK3 reaching the pose, and the log reports:
- IK solution within limits, manipulability above the threshold
- FK pose error below the targets (proposed: 1 mm, 0.5°; to be set from test data)
- Collision-free plan executed
- Pytest: FK↔IK round trip on N random reachable poses passes
