# AR4 MK3 Tool Changer

Digital twin and automatic tool changer for an Annin AR4 MK3 (Teensy 4.1, Annin HMI 6.0).
Status: **Phase 1 done; Phase 2 milestone met; first simulated tool-change sequence working.**

Docs: [HMI EXPORT](docs/HMI_EXPORT.md) · [TOOL-CHANGE SIM PROCEDURE](docs/TOOLCHANGE_SIM_PROCEDURE.md) · [REAL ROBOT](docs/REAL_ROBOT.md) · [ANNIN BRIDGE](docs/ANNIN_BRIDGE.md) · [RESEARCH](docs/RESEARCH.md) · [ARCHITECTURE](docs/ARCHITECTURE.md) · [KINEMATICS](docs/KINEMATICS.md) · [ROADMAP](docs/ROADMAP.md) · [TROUBLESHOOTING](docs/TROUBLESHOOTING.md)

## Quick start (Ubuntu app)
```bash
bash "/mnt/c/Users/gael1/Documents/AR4 MK3 Engiworks Gael/AR4-ToolChanger/setup/build.sh"
ros2 launch ar4tc_toolchanger toolchange_demo.launch.py          # simulate the tool change in RViz
ros2 run ar4tc_toolchanger export_hmi -o "<path>/My program.ar4"  # then: hmi_check "<path>/My program.ar4"
```

**Main workflow (2026-10-22):** develop in the simulation → `export_hmi` → `hmi_check` → run the `.ar4` program in **Annin HMI 6.0** (no ROS on the robot side). Ready-made programs: `hmi_programs/`.
Optional: drive the robot from ROS through `annin_bridge` (`mode:=annin`, emulator `mode:=annin_emu`).

## Ground rules
- The robot runs on Annin's HMI 6.0 (`../6.0/...`). This project **never modifies** those files or the Teensy firmware.
- Simulation only until the Phase 5 safety gate.

## Layout
```
AR4-ToolChanger/        (this folder, Documents, git)  docs, setup scripts, ros_ws/src source, CAD exports later
~/ar4tc_ws              (inside WSL, ext4)            build copy (setup/build.sh rsyncs ros_ws/src here)
~/ar4tc_reports         (inside WSL)                  JSON report per tool-change run
```

## Setup
1. **Admin** PowerShell: `wsl --install --web-download -d Ubuntu-24.04`. Use `--web-download`, because the Store download stalled on this PC.
2. Copy `setup/.wslconfig` to `%UserProfile%\.wslconfig` (caps WSL RAM at 10 GB), then run `wsl --shutdown`.
   On this PC, the default user `gael` was created by script (`/etc/wsl.conf`: systemd on, default user gael).
3. In Ubuntu:
   ```bash
   bash "/mnt/c/Users/gael1/Documents/AR4 MK3 Engiworks Gael/AR4-ToolChanger/setup/setup_wsl_ros.sh"
   ```
   Takes about 30–60 min and 10–15 GB.
4. Smoke test: `ros2 launch annin_ar4_moveit_config moveit.launch.py ar_model:=mk3 include_gripper:=True` should open RViz with the MK3. Drag the end-effector, then click Plan & Execute.

### ROS 2 Humble (Ubuntu 22.04) instead of Jazzy
The same `ros_ws/src` builds on both. Differences are handled in code:
- Upstream driver: `humble` branch pinned at `b9adb2a` (MK3 meshes, DH and joint limits are identical to `main`; only joint `effort` values differ).
- Gazebo: Ignition Fortress (`ign_ros2_control`) instead of Harmonic. `mode:=gazebo` includes upstream's launch, so nothing to change.
- MoveIt: upstream's Humble-syntax `ompl_planning.yaml`. There's no Pilz config, so `bringup.launch.py` loads Pilz only when it exists (we don't use Pilz).
- `/compute_cartesian_path` on Humble has no speed-scaling fields, so `motion.move_linear` stretches the path itself (`motion.slow_down`).
- Gazebo Fortress's GUI crashes on WSLg GPU rendering, so `mode:=gazebo` on Humble uses software GL (TROUBLESHOOTING.md).

**Installed 2026-11-03** as WSL distro `Ubuntu-22.04` (Jazzy stays in `Ubuntu-24.04`, still the default). Open it with `wsl -d Ubuntu-22.04`.
Verified on Humble: 35 tests pass, the mock tool-change sequence completes all 15 steps (59.7 s, the same as Jazzy), and Gazebo starts its controllers and executes a trajectory.

Steps: install a **second** distro next to Jazzy (`wsl --install --web-download -d Ubuntu-22.04`), then in it run
`setup/setup_wsl_ros_humble.sh`, then `setup/build.sh` (it detects the distro). Needs about 12–15 GB free on C:.
