# HMI 6.0 programs generated from the simulation

Copy an `.ar4` file into the HMI 6.0 folder (next to `Home.ar4`) to run it with the Annin HMI.
Read `../docs/HMI_EXPORT.md` first, especially §5 (first run = raised-dock dry run, check the J1 direction, set the lock output).

| File | Sequence | Simulation check |
|---|---|---|
| AR4TC tool change.ar4 | HOME → pick tool_a → HOME → return tool_a → pick tool_b → HOME | hmi_check PASS (2026-10-22) |
| AR4TC tool change - raised dock dry run.ar4 | same, dock +100 mm | hmi_check PASS (2026-10-22) |
| AR4TC goto test 1 - J2 J3 only.ar4 | goto_pose_hmi: HMI flange (286.8, 0, 433.8) → (330, 0, 350) → back; J1, J4, J5, J6 held | goto_pose_hmi checks (2026-10-28) |
| AR4TC goto test 2 - J1 J2 J3.ar4 | goto_pose_hmi: → (300, −120, 380) → back; J4, J5, J6 held | goto_pose_hmi checks (2026-10-28) |

| AR4TC tool change loop - J1 J2 J3 only (dry run).ar4 | hold_joints_hmi from the raised-dock dry run: same flange dock positions with J4–J6 held (tool tilts 49–57°, no coupling, no outputs), loops (`Tab Number 1` / `Jump Tab-1`) until stopped | hold_joints_hmi checks (2026-10-28) |

The goto tests and the loop assume the robot starts at J1 0, J2 0, J3 0, J4 0, J5 89.981, J6 0 (ARbot.cal, 2026-10-28).
If the HMI shows anything else, regenerate with `python3 -m ar4tc_toolchanger.goto_pose_hmi X Y Z --start <6 HMI readings> -o "<file>.ar4"` (no ROS needed).

Not yet run on the physical robot.
