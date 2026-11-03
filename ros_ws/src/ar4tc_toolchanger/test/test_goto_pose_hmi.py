"""goto_pose_hmi: position IK on Annin kinematics with held joints, step-exact hold, generated program."""
import subprocess
import sys

import numpy as np
import pytest

from ar4tc_toolchanger import annin_kinematics as ak
from ar4tc_toolchanger import hmi_program as hp
from ar4tc_toolchanger.goto_pose_hmi import check_path, solve, steps

START = np.array([0, 0, 0, 0, 89.981, 0])     # ARbot.cal 2026-10-28 09:04


@pytest.mark.parametrize("xyz", [[330, 0, 350], [300, -120, 380], [300, 120, 380], [250, 50, 420]])
def test_solve_reaches_target_and_holds_wrist(xyz):
    q = solve(np.array(xyz, float), START, [0, 1, 2])
    assert q is not None
    assert np.allclose(ak.xyz(q), xyz, atol=0.01)
    assert np.all(q[3:] == START[3:])


def test_unreachable_is_refused():
    assert solve(np.array([600.0, 0, 300]), START, [0, 1, 2]) is None


def test_held_values_give_zero_step_change():
    # the HMI's 3-decimal reading of J5 maps back to the same firmware step
    assert steps(START)[4] == steps([0, 0, 0, 0, 89.9809, 0])[4]
    assert steps(START)[5] == steps([0, 0, 0, 0, 89.981, 0.0])[5]


def test_min_z_check():
    low = np.array([0, 60, 50, 0, 89.981, 0], float)
    assert check_path(START, low, min_z=150)
    assert not check_path(START, START, min_z=150)


def test_cli_program(tmp_path):
    out = tmp_path / "g.ar4"
    r = subprocess.run([sys.executable, "-m", "ar4tc_toolchanger.goto_pose_hmi", "330", "0", "350",
                        "--start", *map(str, START), "-o", str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    moves = [o.q_deg for o in hp.parse(out) if o.kind == "move"]
    assert len(moves) == 3 and moves[0] == moves[2] == START.tolist()
    assert all(m[3:] == [0, 89.981, 0] for m in moves)
    assert np.allclose(ak.xyz(moves[1]), [330, 0, 350], atol=0.05)
    r = subprocess.run([sys.executable, "-m", "ar4tc_toolchanger.goto_pose_hmi", "250", "0", "250",
                        "--start", *map(str, START), "-o", str(tmp_path / "x.ar4")], capture_output=True, text=True)
    assert r.returncode != 0 and "max-travel" in r.stdout and not (tmp_path / "x.ar4").exists()


def test_hold_joints_loop_program(tmp_path):
    src, out = tmp_path / "s.ar4", tmp_path / "o.ar4"
    home = [0, 0, 0, 0, 90, 0]
    hp.write(src, ["##home", hp.move_r(home, 25, 20, 20, 100),
                   hp.move_r([-30, 10, 5, 0, 70, 1], 25, 20, 20, 100),
                   hp.move_r([-30, 15, 15, 0, 60, 1], 10, 20, 20, 100),
                   "##TC COUPLE tool_a", hp.out(True, 8), hp.wait(0.5),
                   "##home", hp.move_r(home, 25, 20, 20, 100)])
    r = subprocess.run([sys.executable, "-m", "ar4tc_toolchanger.hold_joints_hmi", str(src), "--loop",
                        "--start", *map(str, START), "-o", str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    ops = hp.parse(out)
    moves = [o.q_deg for o in ops if o.kind == "move"]
    assert all(m[3:] == [0, 89.981, 0] for m in moves) and moves[0] == moves[-1] == START.tolist()
    assert [o.text for o in ops if o.kind == "other"] == ["Tab Number 1", "Jump Tab-1"]
    assert not any(o.kind == "out" for o in ops)
    src_dock = ak.xyz([-30, 15, 15, 0, 60, 1])
    assert any(np.allclose(ak.xyz(m), src_dock, atol=0.05) for m in moves)
