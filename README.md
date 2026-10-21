# AR4 MK3 Tool Changer

Digital twin and automatic tool changer for an Annin AR4 MK3 (Teensy 4.1, Annin HMI 6.0).
Status: **Phase 1, environment setup in progress.**

Docs: [RESEARCH](docs/RESEARCH.md) · [ARCHITECTURE](docs/ARCHITECTURE.md) · [KINEMATICS](docs/KINEMATICS.md) · [ROADMAP](docs/ROADMAP.md) · [TROUBLESHOOTING](docs/TROUBLESHOOTING.md)

## Ground rules
- The robot runs on Annin's HMI 6.0 (`../6.0/...`). This project **never modifies** those files or the Teensy firmware.
- Simulation only until the Phase 5 safety gate.

## Layout
```
AR4-ToolChanger/        (this folder, OneDrive, git)  docs, setup scripts, CAD exports later
~/ar4tc_ws              (inside WSL, ext4)            ROS 2 workspace, built code
```

## Setup
1. **Admin** PowerShell: `wsl --install --web-download -d Ubuntu-24.04`. Use `--web-download`, because the Store download stalled on this PC.
2. Copy `setup/.wslconfig` to `%UserProfile%\.wslconfig` (caps WSL RAM at 10 GB), then run `wsl --shutdown`.
   On this PC, the default user `gael` was created by script (`/etc/wsl.conf`: systemd on, default user gael).
3. In Ubuntu:
   ```bash
   bash "/mnt/c/Users/gael1/OneDrive/Documents/AR4 MK3 Engiworks Gael/AR4-ToolChanger/setup/setup_wsl_ros.sh"
   ```
   Takes about 30–60 min and 10–15 GB.
4. Smoke test: `ros2 launch annin_ar4_moveit_config moveit.launch.py ar_model:=mk3 include_gripper:=True` should open RViz with the MK3. Drag the end-effector, then click Plan & Execute.
