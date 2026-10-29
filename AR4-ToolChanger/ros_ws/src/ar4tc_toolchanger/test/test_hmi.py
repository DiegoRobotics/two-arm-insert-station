"""HMI 6.0 export tests: Annin FK vs poses saved by the HMI, .ar4 format, exporter output."""
import os

import numpy as np
import pytest

from ar4tc_toolchanger import annin_kinematics as ak
from ar4tc_toolchanger import hmi_program as hp
from ar4tc_toolchanger.export_hmi import build_lines

HMI_DIR = "/mnt/c/Users/gael1/OneDrive/Documents/AR4 MK3 Engiworks Gael/6.0/AR4-MK3-HMI-interface-6.0-exe/AR4-MK3 HMI interface 6.0 exe"


@pytest.mark.parametrize("q,xyz", [
    ([0, -41.832, -4.284, 0, 89.981, 0], [44.668, 0.0, 529.075]),     # ARbot.cal, 2026-10-20
    ([0, 0, 0, 0.020, 89.981, 0], [286.844, 0.014, 433.770]),         # ARbot.cal, 2026-10-22 10:21
])
def test_annin_fk_matches_hmi_saved_poses(q, xyz):
    assert np.allclose(ak.xyz(q), xyz, atol=0.002)


def test_ar4_roundtrip(tmp_path):
    p = tmp_path / "t.ar4"
    hp.write(p, ["##hello", "##TC COUPLE tool_a", hp.out(True, 8), hp.wait(0.5),
                 hp.move_r([1, -2.5, 3, 0, 90, -1], 25, 20, 20, 100)])
    raw = p.read_bytes()
    assert raw.startswith(b"##BEGINNING OF PROGRAM##\r\n") and b"\r\n" in raw and b"\n\n" not in raw
    ops = hp.parse(p)
    assert [o.kind for o in ops] == ["comment", "couple", "out", "wait", "move"]
    assert ops[1].tool == "tool_a" and ops[2].on and ops[2].value == 8 and ops[3].value == 0.5
    assert ops[4].q_deg == [1, -2.5, 3, 0, 90, -1] and ops[4].speed == 25


@pytest.mark.skipif(not os.path.isdir(HMI_DIR), reason="Annin HMI folder not available")
@pytest.mark.parametrize("name,expect", [("Home.ar4", [0, 0, 0, 0, 90, 0]), ("Rest.ar4", [0, 0, -89, 0, 0, 0])])
def test_parse_annin_samples(name, expect):
    ops = [o for o in hp.parse(os.path.join(HMI_DIR, name)) if o.kind == "move"]
    assert len(ops) == 1 and ops[0].q_deg == expect


def test_our_move_r_line_matches_annin_layout():
    with open(os.path.join(HMI_DIR, "Off Position.ar4")) if os.path.isdir(HMI_DIR) else _skip() as f:
        annin = [l.strip() for l in f if l.startswith("Move R")][0]
    ours = hp.move_r([0, -41.832, -4.284, 0, 89.981, 0], 25, 20, 20, 100)
    # same tokens in the same order; only the trailing wrist flag differs ($ F vs $ N, unused for joint moves)
    assert ours.split()[:-1] == annin.split()[:-1]


def _skip():
    pytest.skip("Annin HMI folder not available")


def test_export_build_lines():
    cfg = {"tools": {"tool_a": {"label": "A"}}, "home": [0, 0, 0, 0, float(np.radians(90)), 0],
           "mating_offset": [0, 0, 0.02],
           "hmi": {"speed_free": 25, "speed_dock": 10, "acc": 20, "dcc": 20, "ramp": 100,
                   "path_tolerance_deg": 0.2, "lock_output": 8, "lock_on_is_locked": True, "lock_wait_s": 0.5}}
    line = [[0, 0, 0, 0, np.radians(90) * s, 0] for s in np.linspace(0, 1, 30)]
    prog = {"config": cfg, "config_name": "x.yaml", "result": "DONE", "start": [0] * 6, "items": [
        {"type": "move", "kind": "free", "step": "home", "times": list(range(30)),
         "points": line[:-1] + [[0.0002, 0, 0, 0, np.radians(90.004), 0]]},   # goal-tolerance noise -> snapped
        {"type": "couple", "tool": "tool_a"},
        {"type": "move", "kind": "dock", "step": "onto tool_a", "times": [0, 1], "points": [line[-1], line[-1]]},
        {"type": "release", "tool": "tool_a"}]}
    lines, n = build_lines(prog)
    moves = [l for l in lines if l.startswith("Move R")]
    assert n == len(moves) == 2                         # straight joint path -> 1 point; second move -> 1 point
    assert moves[0].startswith("Move R [*] J1 0.000 J2 0.000 J3 0.000 J4 0.000 J5 90.000") and " Sp 25 " in moves[0]
    assert " Sp 10 " in moves[1]
    i = lines.index("##TC COUPLE tool_a")
    assert lines[i + 1] == "Out On = 8" and lines[i + 2] == "Wait Time = 0.5"
    assert lines[lines.index("##TC RELEASE tool_a") + 1] == "Out Off = 8"
    assert any("coupling face" in l for l in lines)
