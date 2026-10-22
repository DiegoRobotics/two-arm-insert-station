# Running the Tool-Change Motion on the Real AR4 MK3

> **Update 2026-10-22:** the main route to the real robot is now the **HMI 6.0 program export** (HMI_EXPORT.md): the HMI runs the robot, ROS is only used to develop and check programs. The ROS bridge below stays available as an option.

**Decision 2026-10-22: route B.** Annin HMI/sketch **v6.0 is the only version used**. The robot (firmware, DIP switches, calibration) is **not changed**. ROS drives it through `annin_bridge` (how it works: ANNIN_BRIDGE.md).
Status: bridge tested against the emulator only. **Not yet run on hardware.**

## Why not the published ROS driver
It needs its own Teensy firmware and the v6.1+ DIP-switch microstepping (2× steps/°). Your robot runs v6.0 microstepping, and changing it means taking the robot apart. Running that driver as-is would move joints 2× the commanded angle, so the launch mode for it was **removed**.

## One-time setup (Windows, admin PowerShell)
```powershell
winget install --exact dorssel.usbipd-win
```

Done 2026-10-22: usbipd-win installed. The Teensy (`16c0:0483`, COM5) is at **BUSID 2-5** and bound ("Shared"). The bind persists, so each session only needs `usbipd attach --wsl --busid 2-5`. The attach is lost if WSL shuts down (`wsl --shutdown` or idle timeout); re-run attach.

## Procedure for each session
1. **Prepare in the HMI 6.0 (as usual):** power the robot, connect, calibrate/home, and move to a safe pose with the arm raised. **Write down the 6 joint angles the HMI shows.**
2. **Release the port:** close the HMI. Keep the robot **powered**; calibration lives in the Teensy's memory.
3. **Pass the USB port to Ubuntu** (PowerShell): `usbipd list` and note the Teensy's BUSID. The first time only, as admin: `usbipd bind --busid <BUSID>`. Then: `usbipd attach --wsl --busid <BUSID>`.
4. **Start ROS** (Ubuntu): `ls /dev/ttyACM*`, then
   ```bash
   ros2 launch ar4tc_toolchanger bringup.launch.py mode:=annin serial_port:=/dev/ttyACM0
   ```
   The log prints `robot position, Annin/HMI degrees J1..J6: …`. **It must match step 1** (within about 0.1°). If it doesn't, or nothing is printed, **stop** (see "Unknowns" below).
5. **Enable motion** (second Ubuntu terminal):
   ```bash
   ros2 service call /annin_bridge/enable std_srvs/srv/SetBool "{data: true}"
   ```
6. **First moves from RViz** (hand on the E-stop). The bridge refuses any trajectory moving a joint more than **15°** (`max_travel_deg`).
   - Planning tab: **Velocity Scaling 0.05, Acceleration Scaling 0.05**.
   - **Always reset the goal first:** Planning tab → *Goal State* → choose **`<current>`** (choose it again even if it's already shown). The orange goal robot must overlap the real robot model.
   - Joints tab: change **one joint by about 5°** (type the value, press Enter). Only that joint of the orange robot should move.
   - Click **Plan** (not Plan & Execute). Watch the preview: only that joint should move. Then click **Execute**.
   - Check every joint moves the way RViz shows. **J1:** RViz +5° should make the HMI angle −5°. If any joint moves the wrong way, E-stop and report it.
   - Then drag the end-effector marker a few cm, Plan, check, Execute.
   - Before step 7, lift the travel limit: `ros2 param set /annin_bridge max_travel_deg 0.0`
7. **Tool-change dry run** (same pattern as the simulation, virtual dock raised 100 mm, 10 % speed, no gripper):
   ```bash
   ros2 run ar4tc_toolchanger toolchange_demo $(ros2 pkg prefix ar4tc_toolchanger)/share/ar4tc_toolchanger/config/toolchanger_real_test.yaml
   ```
   Any failed check stops the robot (FAULT) and disables motion. The JSON report is saved in `~/ar4tc_reports/`.
8. **Finish:** Ctrl-C both terminals, then `usbipd detach --busid <BUSID>` (PowerShell). Open the HMI again and check its displayed angles look right.

## If the USB drops or the bridge says MOTION LOCKED
1. Ctrl+C in **every** Ubuntu window that runs a launch. Check with `pgrep -af annin_bridge` (must print nothing).
2. PowerShell: close the minimized `usbipd … --auto-attach` window, then `usbipd detach --busid 2-5`.
3. HMI 6.0: recalibrate (all joints), move to a safe pose, write down the angles, close the HMI.
4. `usbipd attach --wsl --busid 2-5 --auto-attach` (leave the window open), then start **one** launch (step 4).

## Safety rules
- **Only one launch at a time.** Stop the old one (Ctrl+C) before starting a new one. Two bridges caused a Teensy reset on 2026-10-22.
- Clear workspace. Nothing where the virtual dock is (about 280 mm to the robot's right, 130–200 mm up). E-stop within reach. Stay next to the robot.
- The bridge caps joints at **20°/s** and refuses targets outside the firmware limits. That's not a substitute for supervision.
- A move in progress **can't be stopped over USB**. The physical E-stop is the only immediate stop.
- **Never connect the HMI and ROS at the same time.**
- ROS (URDF) and the HMI disagree by **7 mm sideways** (KINEMATICS.md). Don't copy HMI XYZ positions into ROS until this is measured.

## Unknowns to confirm on the first session (record results in TROUBLESHOOTING.md)
| Item | Why it matters | Check |
|---|---|---|
| HMI sends `CL` on exit (`Serial.end()` on the Teensy)? | The bridge might get no reply | Step 4 prints a position. If not: don't power-cycle (calibration would be lost); reopen the HMI and report |
| Does usbipd attach reset the Teensy? | It would lose calibration | Step 4 angles = HMI angles |
| J1 sign / joint directions | From model comparison, never seen on hardware | Step 6 |
| Firmware `Ss` timing (≈ 1.2 × seconds?) | Motion duration only | Watch `speed_violation` warnings and move times |
| Accel/decel 20 % / 20 % / ramp 20 | Assumed, not from HMI | Smoothness at step 6; tune in `config/annin_bridge.yaml` |
