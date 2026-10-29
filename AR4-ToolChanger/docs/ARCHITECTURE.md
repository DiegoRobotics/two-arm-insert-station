# Architecture

**Implemented so far (2026-10-21):** one package, `ros_ws/src/ar4tc_toolchanger`, with separate modules: `kinematics.py` (Pinocchio, no ROS), `motion.py` (MoveIt client), `scene.py` (dock/tools), `toolchange_demo.py` (state machine), `goto_pose.py`, `ik_benchmark.py`, and `config/toolchanger.yaml`. HMI path: `export_hmi.py` (program JSON → .ar4 Move R), `hmi_program.py` (.ar4 read/write), `hmi_check.py` (replay .ar4 in sim), `annin_kinematics.py` (HMI FK, HMI coordinates). Model: `urdf/ar4tc.urdf.xacro`, `srdf/ar4tc.srdf.xacro`, `rviz/ar4tc.rviz`. Optional ROS hardware path: `annin_bridge.py` (ROS node: FollowJointTrajectory + /joint_states over Annin v6.0 serial), `annin_protocol.py` (no ROS), `annin_emulator.py` (pty stand-in). See ANNIN_BRIDGE.md. The modules will be split into the packages below once they grow.
Source lives in OneDrive (git). `setup/build.sh` copies it with rsync into `~/ar4tc_ws/src/ar4tc` and builds there.

## Target layout

ROS 2 Jazzy colcon workspace. Upstream code is pinned in `ar4tc.repos` and never edited. All project packages use the `ar4tc_` prefix.

```
ws/src/
  upstream/ar4_ros_driver        # MK3 URDF, meshes, ros2_control driver, firmware (pinned)
  ar4tc_description              # xacro: MK3 + tool flange + coupler master + tools + dock + world
  ar4tc_kinematics               # pure Python, no ROS: Pinocchio FK/Jacobian, IK validation, pytest
  ar4tc_motion                   # MoveIt (moveit_py) wrapper: goto_pose, planning scene, Cartesian moves
  ar4tc_tools                    # tool registry YAML (TCP, mass, CoM, collision mesh) + attach/detach
  ar4tc_toolchanger              # finite-state machine for the change sequence, with verification at every step
  ar4tc_safety                   # workspace/speed limits, e-stop supervision, dry-run gate (real hardware only)
  ar4tc_bringup                  # launch: sim:=mock|gazebo|real, tool:=gripper|tool2
```

| Concern | Owner | Notes |
|---|---|---|
| Robot model and config | ar4tc_description | Wraps the upstream MK3 xacro; adds frames `tool0`, `coupler`, `tcp` |
| FK/IK | MoveIt pick_ik + ar4tc_kinematics | Every IK result is checked by FK (position and orientation error), limits and manipulability |
| Planning and collision | MoveIt 2 (OMPL, Pilz LIN for docking moves) | Straight-line approach and retreat into the dock with Pilz LIN |
| Sim backend | ros2_control mock → gz_ros2_control | Same controllers as the real robot, so code doesn't change between sim and hardware |
| Tool manager and TCP | ar4tc_tools | On change: update attached collision object, TCP frame and payload |
| State machine | ar4tc_toolchanger | Plain Python FSM; library choice deferred to Phase 3 |
| Hardware | upstream annin_ar4_driver | Selected only through `sim:=real`; ar4tc_safety must be running |
| Logging | rosbag2 + structured logs per FSM transition | |

Data flow: `goto_pose(x,y,z,r,p,y, frame=tcp)` → IK → validate → plan → execute → measure the final pose error.
