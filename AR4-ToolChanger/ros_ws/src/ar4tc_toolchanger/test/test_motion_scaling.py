"""slow_down(): the Humble fallback that applies velocity/acceleration scaling to Cartesian paths."""
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ar4tc_toolchanger.motion import slow_down


def _traj():
    t = JointTrajectory()
    for sec, ns, v, a in [(0, 0, 0.0, 2.0), (0, 500_000_000, 1.0, 0.0), (1, 0, 0.0, -2.0)]:
        p = JointTrajectoryPoint(positions=[0.0], velocities=[v], accelerations=[a])
        p.time_from_start.sec, p.time_from_start.nanosec = sec, ns
        t.points.append(p)
    return t


def test_velocity_limited():
    t = _traj()
    k = slow_down(t, 0.25, 1.0)
    assert math.isclose(k, 4.0)
    assert (t.points[2].time_from_start.sec, t.points[2].time_from_start.nanosec) == (4, 0)
    assert math.isclose(t.points[1].velocities[0], 0.25)
    assert math.isclose(t.points[0].accelerations[0], 2.0 / 16)


def test_acceleration_limited():
    t = _traj()
    k = slow_down(t, 1.0, 0.04)   # 1/sqrt(0.04) = 5
    assert math.isclose(k, 5.0)
    assert (t.points[1].time_from_start.sec, t.points[1].time_from_start.nanosec) == (2, 500_000_000)


def test_never_speeds_up():
    t = _traj()
    assert slow_down(t, 1.0, 1.0) == 1.0
    assert t.points[2].time_from_start.sec == 1
