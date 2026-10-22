"""Play an Annin HMI 6.0 program in the simulation and check it before running it on the robot.

Every Move R is played as the firmware moves (joint-linear), sampled <= 0.5 deg apart, and each sample is
checked: firmware joint limits + MoveIt collision check against the dock, cradles and tools (including a
tool held on the master). "##TC COUPLE/RELEASE <tool>" markers attach/detach the tool after checking that
the coupling face is at that tool's docking pose. Watch it in RViz.

usage (simulation running: ros2 launch ar4tc_toolchanger bringup.launch.py):
  ros2 run ar4tc_toolchanger hmi_check "<file>.ar4" [--config toolchanger.yaml] [--speedup 4]
"""
import argparse
import re

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from . import annin_protocol as ap
from . import hmi_program as hp
from . import scene
from .common import connect, load_config
from .kinematics import pose_error


def main():
    a = argparse.ArgumentParser()
    a.add_argument("file")
    a.add_argument("--config")
    a.add_argument("--deg-per-s-per-pct", type=float, default=1.2, help="playback speed: deg/s per HMI Sp %%")
    a.add_argument("--speedup", type=float, default=1.0, help="play faster than real time (e.g. 4)")
    args, _ = a.parse_known_args(rclpy.utilities.remove_ros_args()[1:])

    ops = hp.parse(args.file)
    cfg_name = args.config
    for o in ops:  # the exporter writes the config name into the header
        if o.kind == "comment" and (m := re.match(r"^##ar4tc config: (\S+)", o.text)):
            cfg_name = cfg_name or m.group(1)
    cfg = load_config(cfg_name)

    rclpy.init()
    node = rclpy.create_node("ar4tc_hmi_check")
    mv, kin = connect(node, cfg)
    log = node.get_logger()
    validity = node.create_client(GetStateValidity, "/check_state_validity")
    validity.wait_for_service(10)
    scene.reset(mv, cfg)            # start condition: all tools in their cradles
    traj = ActionClient(node, FollowJointTrajectory, cfg["trajectory_action"])

    def valid(q_ros):
        req = GetStateValidity.Request(group_name=cfg["group"])
        req.robot_state = RobotState(is_diff=True, joint_state=JointState(name=cfg["arm_joints"], position=list(q_ros)))
        res = mv._wait(validity.call_async(req))
        return res.valid, [f"{c.contact_body_1}<->{c.contact_body_2}" for c in res.contacts]

    def play(path, speed_deg_s):
        if float(np.max(np.abs(path[-1] - path[0]))) < 1e-3:
            return True              # already there (e.g. HOME when starting at HOME): nothing to play
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = cfg["arm_joints"]
        dt = float(np.max(np.abs(path[-1] - path[0]))) / speed_deg_s / (len(path) - 1)
        for k, p in enumerate(path):
            t = k * dt
            g.trajectory.points.append(JointTrajectoryPoint(
                positions=ap.annin_to_ros(p).tolist(), time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
        traj.wait_for_server(10)
        return mv._action(traj, g, 120.0).error_code == 0

    q = ap.ros_to_annin(mv.joint_positions())
    problems, samples, moves, held = [], 0, 0, None
    for o in ops:
        if o.kind == "move":
            moves += 1
            target = np.array(o.q_deg)
            if not ap.within_limits(target):
                problems.append(f"line {o.line_no}: target outside firmware limits {np.round(target, 2).tolist()}")
            n = max(2, int(np.ceil(np.max(np.abs(target - q)) / 0.5)) + 1)
            path = [q + (target - q) * s for s in np.linspace(0, 1, n)]
            for p in path[1:]:
                samples += 1
                ok, contacts = valid(ap.annin_to_ros(p))
                if not ok:
                    problems.append(f"line {o.line_no}: collision at {np.round(p, 2).tolist()} ({', '.join(sorted(set(contacts))) or 'invalid state'})")
                    break
            # Play the segment so the scene (and any held tool) follows the program, visible in RViz.
            if not play(path, max(o.speed * args.deg_per_s_per_pct * args.speedup, 1.0)):
                problems.append(f"line {o.line_no}: playback failed")
            q = target
        elif o.kind == "couple":
            err = pose_error(kin.fk(ap.annin_to_ros(q)), scene.mating_target(cfg, o.tool))
            if err[0] > cfg["tolerances"]["place_mm"] or err[1] > 1.0:
                problems.append(f"line {o.line_no}: COUPLE {o.tool} but coupling face is {err[0]:.1f} mm / {err[1]:.1f} deg from its docking pose")
            if held:
                problems.append(f"line {o.line_no}: COUPLE {o.tool} while already holding {held}")
            mv.joint_positions(0.3)           # let the scene catch up with the played robot pose
            mv.apply_scene(scene.attach(cfg, o.tool))
            held = o.tool
        elif o.kind == "release":
            err = pose_error(kin.fk(ap.annin_to_ros(q)), scene.mating_target(cfg, o.tool))
            if err[0] > cfg["tolerances"]["place_mm"] or err[1] > 1.0:
                problems.append(f"line {o.line_no}: RELEASE {o.tool} but coupling face is {err[0]:.1f} mm / {err[1]:.1f} deg from its cradle pose")
            if held != o.tool:
                problems.append(f"line {o.line_no}: RELEASE {o.tool} while holding {held}")
            mv.joint_positions(0.3)
            mv.apply_scene(scene.detach(cfg, o.tool))
            held = None
        elif o.kind == "other":
            log.warn(f"line {o.line_no}: not simulated: {o.text}")

    print(f"\nHMI program check: {args.file}")
    print(f"  {moves} Move R lines, {samples} path samples checked (<= 0.5 deg apart), holding at end: {held}")
    for p in problems:
        print("  PROBLEM:", p)
    print("RESULT:", "PASS" if not problems else f"FAIL ({len(problems)} problems)")
    rclpy.shutdown()
    raise SystemExit(0 if not problems else 1)


if __name__ == "__main__":
    main()
