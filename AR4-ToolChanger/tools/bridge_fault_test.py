"""Fault-handling test for annin_bridge against annin_emulator (no hardware).

Run (Ubuntu):  python3 tools/bridge_fault_test.py
Starts the emulator with an injected collision on RJ #2 and E-stop on RJ #4, starts the bridge,
then sends trajectories directly (no MoveIt) and checks each outcome.
"""
import math
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f"joint_{i}" for i in range(1, 7)]
R = FollowJointTrajectory.Result
PORT = "/tmp/ttyANNIN_TEST"


def main():
    procs = [subprocess.Popen(["ros2", "run", "ar4tc_toolchanger", "annin_emulator", "--link", PORT,
                               "--speedup", "4", "--collision-at-move", "2", "--estop-at-move", "4"])]
    time.sleep(1.5)
    procs.append(subprocess.Popen(["ros2", "run", "ar4tc_toolchanger", "annin_bridge", "--ros-args",
                                   "-p", f"port:={PORT}", "-p", "max_joint_speed_deg_s:=60.0"]))
    rclpy.init()
    n = rclpy.create_node("bridge_fault_test")
    js, status = {}, []
    n.create_subscription(JointState, "/joint_states", lambda m: js.update(zip(m.name, m.position)), 10)
    n.create_subscription(String, "/annin_bridge/status", lambda m: status.append(m.data), 10)
    ac = ActionClient(n, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
    en = n.create_client(SetBool, "/annin_bridge/enable")
    ac.wait_for_server(20)
    en.wait_for_service(20)
    t_end = time.time() + 5
    while time.time() < t_end and len(js) < 6:
        rclpy.spin_once(n, timeout_sec=0.1)

    def spin(fut, t=60):
        rclpy.spin_until_future_complete(n, fut, timeout_sec=t)
        return fut.result()

    def enable(on):
        return spin(en.call_async(SetBool.Request(data=on))).success

    def send(target_deg, start_offset_deg=0.0):
        """Trajectory from the current (fresh) position to an absolute ROS-convention target in degrees."""
        t_end = time.time() + 0.6
        while time.time() < t_end:          # drain queued joint_states so the start is current
            rclpy.spin_once(n, timeout_sec=0.05)
        q0 = [js[j] for j in JOINTS]
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = JOINTS
        p0 = JointTrajectoryPoint(positions=[q + math.radians(start_offset_deg) for q in q0], time_from_start=Duration())
        p1 = JointTrajectoryPoint(positions=[math.radians(d) for d in target_deg], time_from_start=Duration(sec=1))
        g.trajectory.points = [p0, p1]
        h = spin(ac.send_goal_async(g), 10)
        if not h.accepted:
            return "REJECTED"
        return spin(h.get_result_async()).result.error_code

    mid = [10, 20, 10, 10, 60, 10]      # well inside all limits
    mid2 = [15, 25, 5, 15, 65, 15]
    checks = [
        ("goal while disabled is rejected", lambda: send(mid), "REJECTED"),
        ("enable", lambda: enable(True), True),
        ("RJ #1 normal move succeeds", lambda: send(mid), R.SUCCESSFUL),
        ("RJ #2 collision flag aborts", lambda: send(mid2), R.PATH_TOLERANCE_VIOLATED),
        ("bridge disabled itself after fault", lambda: send(mid), "REJECTED"),
        ("re-enable", lambda: enable(True), True),
        ("RJ #3 normal move succeeds", lambda: send(mid), R.SUCCESSFUL),
        ("RJ #4 E-stop aborts", lambda: send(mid2), R.PATH_TOLERANCE_VIOLATED),
        ("re-enable", lambda: enable(True), True),
        ("target beyond J2 limit (+95) is refused before sending", lambda: send([10, 95, 10, 10, 60, 10]), R.INVALID_GOAL),
        ("re-enable", lambda: enable(True), True),
        ("start 3 deg away from robot is refused", lambda: send(mid, start_offset_deg=3.0), R.INVALID_GOAL),
        ("re-enable", lambda: enable(True), True),
        ("RJ #5 normal move after all faults succeeds", lambda: send(mid2), R.SUCCESSFUL),
    ]
    failed = 0
    for name, fn, want in checks:
        got = fn()
        ok = got == want
        failed += not ok
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: got {got}, want {want}", flush=True)
    rclpy.spin_once(n, timeout_sec=0.3)
    print("status messages seen:", *[s for s in status if "FAULT" in s], sep="\n  ")
    for p in procs:
        p.terminate()
    print("RESULT:", "ALL PASS" if not failed else f"{failed} FAILED")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
