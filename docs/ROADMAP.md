# Roadmap

| Phase | Status |
|---|---|
| 1 Research and environment setup | **Done** 2026-10-21 (Gazebo GUI not yet opened) |
| 2 Digital twin and kinematics | Not started |
| 3 Virtual tool changer | Not started |
| 4 Mechanical development | Not started |
| 5 Physical integration | Not started. Don't command the real robot until sim and safety checks pass |

## Phase 1 remaining
- [x] User approves stack (see RESEARCH.md)
- [x] MK3 parameters cross-checked against Annin HMI 6.0 (KINEMATICS.md)
- [x] Setup script written (`setup/setup_wsl_ros.sh`), not yet run
- [ ] Install WSL2 (needs admin, so the user runs it) + Ubuntu 24.04 + ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic
- [ ] Build pinned ar4_ros_driver; `moveit.launch.py` with mock hardware shows the MK3 in RViz

## Milestone 1 (Phase 2) — acceptance criteria
`ros2 run ar4tc_motion goto_pose 0.25 0.0 0.30 180 0 0` (example only, reachability not checked) shows the simulated MK3 reaching the pose, and the log reports:
- IK solution within limits, manipulability above the threshold
- FK pose error below the targets (proposed: 1 mm, 0.5°; to be set from test data)
- Collision-free plan executed
- Pytest: FK↔IK round trip on N random reachable poses passes
