# HMI 6.0 programs generated from the simulation

Copy an `.ar4` file into the HMI 6.0 folder (next to `Home.ar4`) to run it with the Annin HMI.
Read `../docs/HMI_EXPORT.md` first, especially §5 (first run = raised-dock dry run, check the J1 direction, set the lock output).

| File | Sequence | Simulation check |
|---|---|---|
| AR4TC tool change.ar4 | HOME → pick tool_a → HOME → return tool_a → pick tool_b → HOME | hmi_check PASS (2026-10-22) |
| AR4TC tool change - raised dock dry run.ar4 | same, dock +100 mm | hmi_check PASS (2026-10-22) |

Not yet run on the physical robot.
