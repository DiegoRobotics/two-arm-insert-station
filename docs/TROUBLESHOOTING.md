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
- annin_bridge crashed on a rejected goal: `ValueError: Logger severity cannot be changed between calls`. rclpy forbids one call site logging at different levels. **Fix:** separate `error()`/`info()` calls. Found by `tools/bridge_fault_test.py`.
- Fault test hung: child processes kept the shell pipe open after `timeout`. **Fix:** write to a log file, use `timeout -k`.
- Real robot rest pose J2 = −41.832° is within 1° of the −42° limit. The bridge applies its 1° margin to the **target only**.
### First hardware session (2026-10-22, bridge to Annin v6.0)
- Bridge connected first time. The position read (0, 0, 0, 0.020, 89.981, 0) matched the HMI's saved pose exactly. usbipd attach and closing the HMI did **not** reset the Teensy.
- **Unexpected large move:** RViz started while `/dev/ttyACM0` was not attached yet, so its goal state was all zeros. Changing J1 by 5° sent (5.24°, 0, 0, 0, 0, 0), and the robot also moved **J5 89.98° → 0°** (and J4 0.02° → 0), over 51 s at 5 % speed. It finished correctly (0.007° error). Cause: the goal state wasn't reset to `<current>`, and my step-6 instructions didn't say to. **Fixes:** (1) REAL_ROBOT.md step 6 now says reset Goal State → `<current>`, then Plan, check, Execute; (2) bridge `max_travel_deg` (default 15°) refuses larger trajectories, and can be changed live with `ros2 param set`.
- Later Plan & Execute clicks did nothing: move_group logged "Goal constraints are already satisfied" (the goal equalled the current pose).
- Bridge bug found while testing the fix: the 1° target margin refused any move while J2 sat at the rest pose (−41.83°). The margin now applies only to joints that move.
- **Two bridges at once → USB errors → Teensy reset (10:41–10:49):** the first launch (10:22) was never stopped, and a second was started at 10:41. Both bridges opened `/dev/ttyACM0` and both served MoveIt's trajectory action. The log showed "device reports readiness to read but returned no data (… multiple access on port?)". At 10:48:15 the USB link dropped (usbipd "connection reset by peer"). At re-attach (10:49:36) the Teensy reported **all joints 0.000**: it had restarted and lost calibration. Last good reading: 0.473 / 4.950 / 0 / 0.020 / 0 / 0.045. The exact cause of the reset isn't proven; the interleaved commands are the prime suspect. **Fixes:** exclusive serial lock; every bridge that sees a second `annin_bridge` node locks motion; connection loss → motion disabled; after a reconnect, an all-zero position (reset) or a jump > 1° **locks** motion until the bridge is restarted after recalibration. usbipd now runs with `--auto-attach`.
### Simulation / HMI export (2026-10-22)
- `hmi_check` second run in the same sim: "goal rejected by /joint_trajectory_controller": a zero-length Move R (already at HOME) produced a trajectory with all times 0. **Fix:** skip zero-length moves.
- `toolchange_demo` second run: an already-satisfied HOME goal returns an empty trajectory. **Fix:** don't record empty motions.
- Emulator run FAULT on HOME: the bridge's 15° first-session travel limit. **Fix:** `mode:=annin_emu` sets `max_travel_deg: 0`; the real `mode:=annin` keeps 15°.
- Exported HOME had goal-tolerance noise (J5 90.005). **Fix:** the exporter writes the exact configured HOME.
- `Move R` with J1 = −0.0 printed "-0.000". **Fix:** normalised.
- Restore backup: `wsl --unregister Ubuntu-24.04` (deletes the current install), then `wsl --import Ubuntu-24.04 <install dir> <backup .tar>`.
