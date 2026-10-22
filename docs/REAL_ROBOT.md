# Running the Tool-Change Motion on the Real AR4 MK3

Status 2026-10-22: **not run on hardware.** Everything below has been checked in simulation only.

## 1. The blocker (verified from your files)
| | Your robot now | ROS driver (`ar4_ros_driver`, `ar_model:=mk3`) |
|---|---|---|
| Teensy firmware | Annin sketch **v6.0** + HMI 6.0 | Its own sketch (`annin_ar4_firmware/AR4_teensy`, v2.1.0) |
| Motor steps/° J1…J6 | 44.44 / 55.56 / 55.56 / 49.78 / 21.86 / 22.22 (`ARbot.cal`, v6.0 sketch) | **88.89 / 111.11 / 111.11 / 99.56 / 43.72 / 44.44** (driver docs: needs the DIP-switch setting introduced with Annin v6.1) |

Annin's own v6.1, v6.2 and v6.3 sketches (in your folders) also use the doubled values. So the ROS driver as published needs:
1. Your driver **DIP switches changed** to the v6.1+ setting, and
2. Your Teensy **reflashed** with the ROS firmware.

Running the mk3 driver on your current switches would move every joint **2× the commanded angle**. Running it as `mk2` matches the step values but calibrates J4/J6 against the wrong limits (±165/±155° instead of ±180°). **Don't do either.**

## 2. Two routes
| | **A. ROS firmware (available now)** | **B. Bridge to Annin v6.0 (recommended, not built yet)** |
|---|---|---|
| Changes to your robot | DIP switches + Teensy firmware. Going back to Annin means **HMI/sketch 6.1–6.3**, not 6.0 | **None.** A ROS node talks to the existing v6.0 firmware over USB (`RJ` = move joints, `RP` = read position) |
| Motion quality | Streams trajectories, smooth | Point-to-point segments, a bit slower, fine for docking moves |
| Annin HMI | Usable only after reflashing an Annin sketch | Usable any time. Just close it, or release the COM port, while ROS runs |
| Effort | Setup only | I build and test the bridge (sim protocol emulator first) |
| Fits "don't change Annin's operation" | **No** | **Yes** |

Joint mapping for route B is already measured (KINEMATICS.md): Annin J1 = −ROS J1, J2–J6 identical, degrees.

## 3. Steps common to both: USB from Windows into WSL
1. Admin PowerShell, once: `winget install --exact dorssel.usbipd-win`
2. Close the Annin HMI. Plug in the robot USB.
3. `usbipd list`: note the BUSID of the Teensy (USB Serial)
4. Admin, once per device: `usbipd bind --busid <BUSID>`
5. Each session: `usbipd attach --wsl --busid <BUSID>`. In Ubuntu, `ls /dev/ttyACM*` should show `/dev/ttyACM0`.
6. To give it back to the HMI: `usbipd detach --busid <BUSID>`

## 4. Route A procedure (only if you accept the firmware/DIP change)
1. **Record first:** photo of the current DIP switches; note that your working v6.0 sketch is `6.0/AR4_teensy41_sketch_v6.0`.
2. Set DIP switches to the v6.1+ MK3 setting (`ar4_ros_driver/docs/ar4_dip_switch_settings.png`, Annin v6.1 docs).
3. Flash `annin_ar4_firmware/AR4_teensy/AR4_teensy.ino` with Arduino IDE + Teensyduino (add the Bounce2 library).
4. Attach USB (section 3). Then in Ubuntu:
   ```bash
   ros2 launch ar4tc_toolchanger bringup.launch.py mode:=real include_gripper:=False calibrate:=True
   ```
   The robot **moves by itself to calibrate** (limit switches). Keep your hand on the E-stop.
5. In RViz (MotionPlanning panel): set Velocity/Accel scaling to **0.05**. Plan & Execute small single-joint moves first. Check each joint's direction and angle against reality before going on.
6. Tool-change dry run (virtual dock raised 100 mm, 10 % speed, no gripper), in a second Ubuntu terminal:
   ```bash
   ros2 run ar4tc_toolchanger toolchange_demo $(ros2 pkg prefix ar4tc_toolchanger)/share/ar4tc_toolchanger/config/toolchanger_real_test.yaml
   ```
   Watch it in RViz and on the robot. Any failed check stops it (FAULT). `run:=true` is refused in real mode on purpose.
7. **Back to Annin:** flash `6.3/AR4_teensy41_sketch_v6.3` (matches the new DIP setting), use HMI 6.3, and recalibrate there. Restoring v6.0 means also restoring the old DIP switches.

## 5. Safety rules for the first real runs
- Clear the workspace. Nothing on the table where the virtual dock is (around 280 mm to the robot's right). E-stop within reach.
- 5–10 % speed, single moves before sequences. Stay next to the robot.
- No real tools or dock until the motion has been verified empty-handed.
- The ROS (URDF) model and Annin's model differ by **7 mm sideways** (URDF J3 offset). Don't mix HMI XYZ coordinates with ROS coordinates until this is measured on the robot.
- Annin HMI and ROS must never be connected at the same time.

## 6. Already in place (tested in sim)
- `bringup.launch.py mode:=real` wraps the upstream driver with our MoveIt/IK setup; `include_gripper:=False` is supported.
- `config/toolchanger_real_test.yaml`: mock run DONE 17/17, 110.9 s.
- Gripper steps are skipped when `use_gripper: false`.
