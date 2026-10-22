# Troubleshooting

## Physical AR4 MK3 — history before this project
Source: Annin HMI 6.0 `ErrorLog`, read-only. Status on 2026-10-20: all joints calibrated and homed, and no current errors according to the user.

| Date | Logged events | Notes |
|---|---|---|
| 2026-10-15 | Repeated `UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1` then `COMMUNICATIONS STARTED` | The connection came up after retries. The exact fix isn't recorded |
| 2026-10-15 | Many `J1 Axis Limit`, `J6 Axis Limit`, `J2 Axis Limit` | Software axis-limit errors before calibration |
| 2026-10-15 | `Auto Calibration Stage 1 Successful`, `J5 CALIBRATION ERROR`, `Stage 2 Failed`; then J4, J5, J6 calibrated individually | Calibrating joints one at a time worked when auto-calibration failed |
| 2026-10-15 | `J2`, `J3 Axis Limit` | |
| 2026-10-20 | J1, J2, J3 and J5 calibrated individually; J5 failed twice at 11:10, then succeeded at 11:11 | J5 limit-switch or calibration is the least reliable joint |
| 2026-10-20 | `J2`, `J3 Axis Limit` at 11:26–11:30 | Last entries before the successful home |
| Ongoing | `UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD` | Expected if no IO board/Nano is connected. **Matters for the servo gripper**, which uses the Nano |

Open questions: what physically fixed the communication and J5 problems (cable, COM port, limit-switch adjustment)? Add them here when known.

## Rules
- Annin HMI files (`6.x/…`) are **read-only** for this project. The simulation never writes to them, and nothing reflashes the Teensy without explicit approval (Phase 5).

## Environment
- Ekumen repo: `link_6.STL` and `link_6.stl` collide on Windows. Clone inside WSL ext4.
- WSL install needs an **admin** PowerShell (fails from a normal shell with "not installed").
- 2026-10-21: `wsl --install -d Ubuntu-24.04` stalled at 49.2% (network down to about 0 MB/s). **Fix:** kill `wsl.exe`, then run `wsl --install --web-download -d Ubuntu-24.04 --no-launch`, which downloads from GitHub instead of the Store. It finished, and WSL 3.0.3 / kernel 6.18 worked **without a reboot**.
- Linux user `gael` was created by script and has **no password** yet. Set one with `wsl -u root passwd gael`. `sudo` won't work until you do (the temporary no-password sudo rule was removed after setup).
- `setup_wsl_ros.sh` stopped at `source /opt/ros/jazzy/setup.bash` with `AMENT_TRACE_SETUP_FILES: unbound variable`, because ROS setup files fail under `set -u`. **Fix:** `set +u` before sourcing (now in the script).
- Pinocchio `ValueError: Some components of min_effort are greater than max_effort`: the upstream URDF has `effort="-1"`. **Fix:** `kinematics.py` replaces it with 1000 in memory (the upstream file is unchanged).
- A planning-scene call hung until timeout when made right after the node started (service not discovered yet). **Fix:** `wait_for_service` before scene calls.
- Launch error `launch configuration 'tf_prefix' does not exist`: upstream controller YAML uses `$(var tf_prefix)`. **Fix:** `bringup.launch.py` declares `tf_prefix`.
- Gazebo: the gripper action returned before the jaw finished moving (jaw at 9.7 of 14 mm), so the state machine raised FAULT. **Fix:** `motion.gripper()` polls `/joint_states` for up to 3 s.
- MoveGroup code −10 (start state in collision) after attaching a tool: `touch_links` lacked the gripper links. **Fix:** always list them (harmless if absent).
- Compiling Annin `kinematics.cpp` on Linux: GCC rejects MSVC-style template lookups. **Fix:** `-fpermissive`.
- `PYTHONPATH=dir python3` drops ROS paths (pinocchio not found). **Fix:** `PYTHONPATH=dir:$PYTHONPATH`.
- **Hardware compatibility (not a bug):** Annin v6.0 = old microstepping; ROS driver mk3 and Annin v6.1+ = doubled steps/°. See REAL_ROBOT.md.
- Restore backup: `wsl --unregister Ubuntu-24.04` (deletes the current install), then `wsl --import Ubuntu-24.04 <install dir> <backup .tar>`.
