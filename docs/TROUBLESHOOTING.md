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
