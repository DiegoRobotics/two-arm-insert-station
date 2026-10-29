# Annin Bridge: How It Works

`annin_bridge` lets MoveIt/RViz drive the **real AR4 MK3 without changing anything on the robot**. It talks to the Teensy running Annin's **sketch v6.0** with the same serial commands the Annin HMI 6.0 uses. Firmware, DIP switches, calibration and the HMI all stay as they are.

Code: `ros_ws/src/ar4tc_toolchanger/ar4tc_toolchanger/annin_bridge.py` (node), `annin_protocol.py` (protocol + math, no ROS), `annin_emulator.py` (test stand-in). Parameters: `config/annin_bridge.yaml`.

## 1. Where it sits
```
RViz / toolchange_demo
        │  plan (MoveIt, KDL IK, OMPL / Cartesian)
        ▼
move_group ── FollowJointTrajectory ──► annin_bridge ── USB serial "RJ…/RP" ──► Teensy (Annin v6.0) ──► motors
        ▲                                   │
        └──────────── /joint_states ◄───────┘ (from "RP" replies)
```
The bridge presents the **same interfaces as the ROS controller** used in simulation (`/joint_trajectory_controller/follow_joint_trajectory`, `/joint_states`), so MoveIt, RViz and the tool-change state machine run unchanged.

## 2. Protocol used (read from `AR4_teensy41_sketch_v6.0.ino`)
| Command | Sent | Firmware reply |
|---|---|---|
| Read position | `RP\n` | `A<j1>B<j2>C<j3>D<j4>E<j5>F<j6>G<x>H<y>I<z>J<rz>K<ry>L<rx>M<speedViol>N<debug>O<flag>P<j7>Q<j8>R<j9>` |
| Move joints | `RJA<j1>B…F<j6>J7<j7>J8<j8>J9<j9>Ss<seconds>Ac<acc%>Dc<dcc%>Rm<ramp%>WF Lm000000\n` | Same position line **after the move finishes**, or an alarm |

- Angles are degrees in Annin's convention. `Ss<seconds>` = "complete this move in N seconds"; the firmware times its steps (with accel/decel) to fit. If that's too fast for the motors, it caps the speed and sets `M1`.
- `Lm000000` = **closed loop** on all joints. After each move the firmware compares encoders with the commanded steps (tolerance 25 steps). A mismatch sets flag `EC…` (collision / lost steps).
- Alarms: `EL<9 bits>` = axis limit (move refused), `ER` = kinematic error. Flag `EB` = E-stop (sent immediately by the E-stop interrupt).
- **Never sent:** calibration (`LL`), parameter updates (`UP`), `CL` (it calls `Serial.end()`), drive resets. Calibration stays the HMI's job.

## 3. Conversions
| ROS (MoveIt) | Annin firmware |
|---|---|
| radians | degrees |
| J1 | **−J1** (sign inverted) |
| J2–J6 | identical |

Measured by comparing the URDF with Annin's own kinematics (KINEMATICS.md). **Check it on the robot before trusting it** (REAL_ROBOT.md, step 6).

## 4. What happens to one MoveIt trajectory
1. **Accept or reject:** the goal is rejected unless motion is **enabled** (`~/enable`), the robot position is known, and nothing else is running.
2. **Check:** joint names; every point within the firmware limits; joints that move must end ≥ 1° inside their limits; the trajectory start within **1°** of the measured position; no joint travels more than `max_travel_deg` (default **15°**, 0 = off, changeable live: `ros2 param set /annin_bridge max_travel_deg <deg>`).
3. **Reduce:** MoveIt sends dense trajectories (e.g. 64–171 points). The firmware moves all joints **linearly in step space** between targets, so the bridge keeps only the points needed to stay within **0.2°** of the planned path (Douglas-Peucker). In tests, 1,193 points became 30 RJ commands.
4. **Time:** each segment uses its planned duration, stretched if any joint would exceed **20°/s** (hard cap), minimum 0.4 s.
5. **Execute:** one blocking `RJ` per segment. The firmware can't answer while moving, so during a segment `/joint_states` shows a **linear estimate**. Each RJ reply provides the **measured** position.
6. **Verify:** every reply is parsed. Any alarm, `EC`, `EB`, timeout or serial error → abort, report to MoveIt, **disable motion**. After the last segment the measured position must be within **0.5°** of the target.
7. **Cancel:** checked between segments. A running segment can't be interrupted over serial, so **use the physical E-stop for emergencies.**

While idle, it polls `RP` at 10 Hz for `/joint_states` and watches for an unsolicited E-stop line.

**Connection safety (added after the first hardware session):**
- The serial port is opened **exclusively**, so no second process can use it.
- If another `annin_bridge` node appears, motion **locks**.
- Losing the connection switches motion off.
- After reconnecting, an **all-zero position** (sketch v6.0's power-on state, meaning uncalibrated) or a jump of **> 1°** from the last reading **locks** motion. `~/enable` then refuses until the bridge is restarted after recalibrating in the HMI.

## 5. Differences from the simulation controller
- **Stop-and-go:** each RJ segment accelerates and decelerates (Ac/Dc 20 %). Joint-space moves are usually one segment; straight-line docking moves were 4–5 segments in testing. Annin's spline mode (`SL`/`MS`/`SS`) could smooth this later.
- MoveIt's execution-time monitoring is **off** in bridge modes (the bridge enforces its own timeouts, `seconds × 3 + 5 s`), because stop-and-go segments take longer than MoveIt's plan.
- **No gripper:** the bridge drives J1–J6 only (the Annin servo gripper is on the Arduino IO board, which isn't connected).
- **The acceleration values (Ac/Dc/Rm = 20 %) are assumptions.** They aren't taken from the HMI; tune them on the robot.

## 6. Launch
```bash
ros2 launch ar4tc_toolchanger bringup.launch.py mode:=annin_emu                 # emulator, no hardware
ros2 launch ar4tc_toolchanger bringup.launch.py mode:=annin_emu run:=true       # + tool-change sequence
ros2 launch ar4tc_toolchanger bringup.launch.py mode:=annin serial_port:=/dev/ttyACM0   # REAL ROBOT (see REAL_ROBOT.md)
ros2 service call /annin_bridge/enable std_srvs/srv/SetBool "{data: true}"      # allow motion
ros2 topic echo /annin_bridge/status                                            # last event / fault
```
`mode:=annin` starts **disabled** and refuses `run:=true`.

## 7. Test evidence (2026-10-22, emulator only; no hardware yet)
| Test | Result |
|---|---|
| Unit tests (`test_annin_protocol.py`): reply parsing, alarms, EB/EC flags, RJ format parsed back with the sketch's own field logic, J1 sign, limits, path reduction, speed cap | 10/10 pass (16/16 with the kinematics tests) |
| Full tool-change sequence, MoveIt → bridge → emulator (`toolchanger_real_test.yaml`) | **DONE 17/17**, 154.5 s, final joint errors ≤ 0.01°, TCP ≤ 0.39 mm |
| Fault test (`tools/bridge_fault_test.py`): disabled gate, collision EC, E-stop EB, limit violation, start mismatch, recovery | **14/14 pass** |

The emulator copies the sketch's command parsing, limit check and replies. It is **not** the real firmware: real timing, encoder behaviour and USB behaviour are only confirmed on the robot.
