# From Simulation to Annin HMI 6.0

**Goal:** develop and validate the tool change in the ROS simulation, then run it on the robot **with the Annin HMI 6.0 only**: no ROS, no firmware change. The HMI stays in charge, so its camera/vision features can be added to the same programs later.

```
config/toolchanger.yaml ──► toolchange_demo (MoveIt, collision-checked) ──► program_*.json
                                                                              │
                                                     export_hmi ◄─────────────┘
                                                        │
                                          "AR4TC tool change.ar4" (HMI program)
                                                        │
                                hmi_check (replays the .ar4 in the simulation, collision-checked) ──► PASS
                                                        │
                                          copy into the HMI folder ──► HMI 6.0 runs it
```

## 1. What the exporter writes
Plain HMI 6.0 program lines, in the same format as Annin's own `Home.ar4` / `Stack Blocks.ar4`:

| Simulation event | HMI line(s) |
|---|---|
| Any motion (free move or straight docking move) | `Move R [*] J1 … J6 … Sp <%> Ac 20 Dc 20 Rm 100 $ N`, i.e. **joint moves only** |
| Lock tool (COUPLE) | `##TC COUPLE tool_a`, `Out On = 8`, `Wait Time = 0.5` |
| Release tool | `##TC RELEASE tool_a`, `Out Off = 8`, `Wait Time = 0.5` |
| Info | `##` comments: config used, slot names, docking poses in **HMI coordinates** |

**Why only `Move R`:** it sends joint angles, which the HMI passes straight to the Teensy (`RJ`). The robot repeats **exactly the joint path validated in simulation**. It doesn't depend on the HMI's tool-frame setting, its IK solution choice, or the 7 mm URDF/HMI model difference (KINEMATICS.md).

**Reducing the path:** each simulated motion (dozens of points) is reduced to the fewest `Move R` points whose joint-linear path stays within **0.2°** of the simulated one, the same method the bridge uses. A straight docking descent becomes about 4 lines at `Sp 10`. Free moves use `Sp 25`. Speeds, lock output and wait time are set in `hmi:` in the config.

**Joint convention:** HMI J1 = −(simulation J1), all others equal (KINEMATICS.md). HOME is written exactly as `Home.ar4`: `J1 0 J2 0 J3 0 J4 0 J5 90 J6 0`.

## 2. How the program is checked (`hmi_check`)
It reads the **.ar4 file itself** (not the simulation recording) and plays it in the simulation the way the firmware moves: joint-linear, sampled every ≤ 0.5°. Each sample is checked against the firmware joint limits and collision-checked against the dock, cradles and tools, including the tool held on the master. At each `##TC COUPLE/RELEASE` it also checks that the coupling face is within 2 mm of that tool's docking pose.

Proof that it catches mistakes (2026-10-22):

| Broken copy of the program | hmi_check result |
|---|---|
| Release of tool_a deleted (robot carries tool_a down onto tool_b) | FAIL: `tool_b<->tool_a` collisions, "COUPLE tool_b while already holding tool_a" |
| Last docking line shifted 2° in J2 | FAIL: `tool_a<->tc_master` collision, "coupling face is 10.0 mm / 2.0° from its docking pose" |

## 3. Make a program (Ubuntu)
```bash
ros2 launch ar4tc_toolchanger bringup.launch.py                      # simulation + RViz
ros2 run ar4tc_toolchanger toolchange_demo                           # 2nd window: run the sequence (config: toolchanger.yaml)
ros2 run ar4tc_toolchanger export_hmi -o "/mnt/c/Users/gael1/OneDrive/Documents/AR4 MK3 Engiworks Gael/AR4-ToolChanger/hmi_programs/My program.ar4"
ros2 run ar4tc_toolchanger hmi_check "/mnt/c/…/hmi_programs/My program.ar4" --speedup 4   # must print RESULT: PASS
```
Change the layout, tools or sequence in `config/toolchanger.yaml` (or a file with `extends: toolchanger.yaml`), rebuild (`setup/build.sh`), and repeat. Pass another config with `toolchange_demo myconfig.yaml`.

## 4. Ready-made programs (`hmi_programs/`)
| File | Content | Check |
|---|---|---|
| `AR4TC tool change.ar4` | HOME → pick tool_a → HOME → return tool_a → pick tool_b → HOME (30 Move R) | PASS |
| `AR4TC tool change - raised dock dry run.ar4` | Same, dock 100 mm higher, for the first empty run | PASS |
| `sim_run_*.json` | The simulation recordings they were exported from | |

## 5. Run it in the HMI (once the robot is repaired)
1. **Copy** the `.ar4` file into the HMI folder (`6.0\AR4-MK3-HMI-interface-6.0-exe\AR4-MK3 HMI interface 6.0 exe\`, next to `Home.ar4`). The project never writes there itself.
2. In the HMI: calibrate, then load the program the same way you open `Home.ar4`.
3. **First run = raised-dock dry run, empty.** No dock or tools, low speed, hand on the E-stop. Step through it line by line if the HMI allows.
4. **Check the joint convention on the first docking line.** At the line before `##TC COUPLE tool_a`, the HMI position display (tool frame 0) should read about **X 205.6, Y −275.8, Z 233.0 mm** for the raised version (Z 133.0 for the normal one). The arm should be on the robot's **right**. If it's on the left (Y ≈ +275), J1's sign is wrong. Stop and report.
5. **Tool lock output:** `Out On/Off = 8` is a placeholder. Set `hmi.lock_output` to your real wiring and re-export. Without the Arduino IO board connected, delete or comment out the `Out` lines for dry runs.
6. **Place the real dock** using the comments in the file: `##  docking pose, HMI coords: … coupling face X … Y … Z …`. That's where each tool's coupling plate top must be (+1 mm gap). Measure it with the HMI's position display, and re-export if your dock ends up elsewhere.

## 6. Adding vision later
The output is ordinary HMI code. You can add `Cam On`, `Vis Find …`, `Move Vis …` or `Call Program` lines around it in the HMI, as in Annin's `Stack Blocks.ar4`. `hmi_check` simply reports lines it doesn't simulate as "not simulated".

## 7. Limits
- Simulation only so far: no physical run. Geometry (dock, tools, master 20 mm, TCPs) is placeholder until CAD.
- Joint-linear `Move R` steps stop briefly between points (4 short stops per docking move). That's fine for docking, but `Move L` would be smoother. It wasn't used because it depends on the HMI's tool frame and its 7 mm model difference.
- `Sp` timing on the real robot isn't the same as in the simulation; only the path is reproduced.

## 8. goto_pose on the real robot with held joints (`goto_pose_hmi`, 2026-10-28)
For testing the arm while J5/J6 are unreliable. `python3 -m ar4tc_toolchanger.goto_pose_hmi X Y Z --start J1..J6 -o "<file>.ar4"` (run from `ros_ws/src/ar4tc_toolchanger`, no ROS needed).
- Target = flange X Y Z in **HMI coordinates** (mm, tool frame 0: what the HMI display shows). IK runs on Annin's own kinematics, so there's no 7 mm URDF offset and no J1 sign conversion.
- J4, J5, J6 are held by default (`--hold`). They're written with exactly the `--start` values in every line; the tool checks that the firmware's angle→step formula (`int((angle + limNeg) * stepDeg)`) then gives a step difference of 0.
- Position only: the tool tilt follows from J2/J3 and is printed.
- Refuses: unreachable targets, targets within 1° of a limit, travel > `--max-travel` (30°), any elbow/wrist/flange frame below `--min-z` (150 mm) along the joint-linear path.
- Program: start pose → target (`Sp 10`) → `Wait Time` → back to start. **Only valid if the HMI shows exactly the `--start` angles**, otherwise held joints would move to them.
- Not checked: collisions with objects around the robot (none modelled in HMI coordinates); not replayed with `hmi_check`.

## 9. Tool-change loop with held joints (`hold_joints_hmi`, 2026-10-28)
`python3 -m ar4tc_toolchanger.hold_joints_hmi "<exported>.ar4" --start J1..J6 --loop -o "<out>.ar4"` converts an exported tool-change program: every Move R target becomes its flange position and is re-solved with J1–J3 (`goto_pose_hmi.solve`); straight docking moves (Sp 10) are split every 10 mm. `Out` lines are dropped and each COUPLE/RELEASE becomes a comment + `Wait Time`. `--loop` wraps it in `Tab Number 1` … `Jump Tab-1` (syntax from Annin's `Stack Blocks.ar4`), so it runs until stopped in the HMI.
Result for the raised-dock program: 30 → 93 Move R lines; flange exactly at the source dock positions; J1 0…−76.5°, J2 0…32.4°, J3 0…30.8°. **Without J5 the tool can't stay vertical: it's tilted 49° (tool_a) and 57° (tool_b) from straight down at the docks, so this is a motion dry run only: no dock, no tools, no real coupling.**
