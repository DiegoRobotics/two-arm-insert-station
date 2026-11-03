"""Simulated tool change driven by the `sequence` in the config (default:
HOME -> pick tool_a -> HOME -> return tool_a -> pick tool_b -> HOME).

Every step is a state with an action and a verification. Any failed check moves the machine to FAULT
and stops (no automatic recovery). Every executed motion is recorded to a program JSON that
`export_hmi` turns into an Annin HMI 6.0 program.

usage: ros2 run ar4tc_toolchanger toolchange_demo [config.yaml]
"""
import json
import os
import sys
import time
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from . import scene
from .common import connect, load_config
from .kinematics import pose_error


class S(Enum):
    HOME = "HOME"
    APPROACH = "APPROACH"      # joint-space move to the pre-dock pose above the slot
    DESCEND = "DESCEND"        # straight-line move down to the coupling pose (dock_gap above contact)
    COUPLE = "COUPLE"          # lock tool to the master (attach in the scene)
    RELEASE = "RELEASE"        # unlock tool in its cradle (detach)
    RETREAT = "RETREAT"        # straight-line move back up
    DONE = "DONE"
    FAULT = "FAULT"


class StepFailed(Exception):
    pass


class ToolChanger:
    def __init__(self, node, cfg, mv, kin):
        self.node, self.cfg, self.mv, self.kin = node, cfg, mv, kin
        self.tol = cfg["tolerances"]
        self.held = None
        self.records = []
        self.program = []              # executed motions + tool actions, for export_hmi
        self.tf = StaticTransformBroadcaster(node)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.labels = node.create_publisher(MarkerArray, "~/labels", latched)
        self._publish_tcp()

    # ---- visual aids ----------------------------------------------------
    def _publish_tcp(self):
        """tc_mating -> tcp for the tool currently held (identity without a tool)."""
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "tc_mating", "tcp"
        off = self.cfg["tools"][self.held]["tcp"] if self.held else [0.0, 0.0, 0.0]
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, off)
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)

    def publish_labels(self):
        arr = MarkerArray()
        for i, (name, t) in enumerate(self.cfg["tools"].items()):
            m = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, id=i, ns="tool_slots", text=f"{name}: {t['label']}")
            m.header.frame_id = self.cfg["base_frame"]
            p = scene.docked_tool_pose(self.cfg, name).translation
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p[0]), float(p[1]), float(p[2]) + 0.05
            m.pose.orientation.w, m.scale.z = 1.0, 0.018
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            arr.markers.append(m)
        self.labels.publish(arr)

    # ---- helpers --------------------------------------------------------
    def _run(self, state, label, fn):
        t0 = time.time()
        try:
            ok, info = fn()
        except Exception as e:  # ROS timeouts, rejected goals
            ok, info = False, f"{type(e).__name__}: {e}"
        rec = {"state": state.value, "step": label, "ok": ok, "info": info, "seconds": round(time.time() - t0, 2)}
        self.records.append(rec)
        self.node.get_logger().info(f"[{'OK' if ok else 'FAIL'}] {state.value:8s} {label}: {info}")
        if not ok:
            self.records.append({"state": S.FAULT.value, "step": label, "ok": False, "info": info, "seconds": 0})
            raise StepFailed(f"{state.value} {label}: {info}")

    def _record_motion(self, kind, label):
        jt = self.mv.last_trajectory
        if jt is None or not jt.points:  # goal was already satisfied: nothing moved
            return
        order = [jt.joint_names.index(j) for j in self.cfg["arm_joints"]]
        self.program.append({"type": "move", "kind": kind, "step": label,
                             "times": [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points],
                             "points": [[p.positions[i] for i in order] for p in jt.points]})

    def _reached(self, target):
        q = self.mv.joint_positions()
        pos, rot = pose_error(self.kin.fk(q), target)
        ok = pos <= self.tol["pos_mm"] and rot <= self.tol["rot_deg"]
        return ok, f"coupling-face error {pos:.3f} mm / {rot:.3f} deg"

    def _approach(self, target, label):
        q0 = self.mv.joint_positions()
        q = self.mv.compute_ik(self.kin.tcp_to_flange(target), q0)
        if q is None:
            return False, "no collision-free IK solution"
        chk = self.kin.validate(q, target, self.tol["pos_mm"], self.tol["rot_deg"], self.tol["min_manipulability"])
        if not chk.ok:
            return False, "IK rejected: " + "; ".join(chk.reasons)
        ok, info = self.mv.move_to_joints(q)
        if not ok:
            return False, info
        self._record_motion("free", label)
        ok, err = self._reached(target)
        return ok, f"IK q(deg)={np.round(np.degrees(q), 1).tolist()} w={chk.manipulability:.4f}; {err}"

    def _linear(self, target, label):
        # Collision checking stays ON: the geometry leaves dock_gap / float clearance at the docked pose.
        ok, info = self.mv.move_linear(self.kin.tcp_to_flange(target), avoid_collisions=True)
        if not ok:
            return False, info
        self._record_motion("dock", label)
        ok, err = self._reached(target)
        return ok, f"{info}; {err}"

    def _scene_state(self):
        sc = self.mv.get_scene()
        attached = {a.object.id for a in sc.robot_state.attached_collision_objects}
        world = {o.id: o for o in sc.world.collision_objects}
        return attached, world

    def _couple(self, tool):
        if not self.mv.apply_scene(scene.attach(self.cfg, tool)):
            return False, "attach rejected by planning scene"
        attached, world = self._scene_state()
        if tool not in attached or tool in world:
            return False, f"attach not confirmed (attached={sorted(attached)})"
        self.held = tool
        self._publish_tcp()
        self.program.append({"type": "couple", "tool": tool})
        t = self.cfg["tools"][tool]
        return True, f"{tool} ({t['label']}) locked; TCP {t['tcp']} m, payload {t['mass_kg']} kg"

    def _release(self, tool):
        if not self.mv.apply_scene(scene.detach(self.cfg, tool)):
            return False, "detach rejected by planning scene"
        attached, world = self._scene_state()
        if tool in attached or tool not in world:
            return False, f"detach not confirmed (attached={sorted(attached)})"
        p = world[tool].pose.position
        err = float(np.linalg.norm(np.array([p.x, p.y, p.z]) - scene.tool_center(self.cfg, tool)) * 1000)
        self.held = None
        self._publish_tcp()
        self.program.append({"type": "release", "tool": tool})
        return err <= self.tol["place_mm"], f"{tool} back in cradle, placement error {err:.3f} mm"

    # ---- sequences ------------------------------------------------------
    def home(self):
        def go():
            ok, info = self.mv.move_to_joints(self.cfg["home"])
            if ok:
                self._record_motion("free", "home")
            q = self.mv.joint_positions()
            err = float(np.max(np.abs(np.degrees(q - np.array(self.cfg["home"])))))
            return ok and err <= self.tol["home_joint_deg"], f"{info}; max joint error {err:.3f} deg"
        self._run(S.HOME, "home", go)

    def pick(self, tool):
        if self.held:
            raise StepFailed(f"cannot pick {tool}: already holding {self.held}")
        dock = scene.mating_target(self.cfg, tool)
        above = scene.mating_target(self.cfg, tool, self.cfg["approach_height"])
        self._run(S.APPROACH, f"above {tool}", lambda: self._approach(above, f"above {tool}"))
        self._run(S.DESCEND, f"onto {tool}", lambda: self._linear(dock, f"onto {tool}"))
        self._run(S.COUPLE, tool, lambda: self._couple(tool))
        self._run(S.RETREAT, f"lift {tool}", lambda: self._linear(above, f"lift {tool}"))

    def place(self, tool):
        if self.held != tool:
            raise StepFailed(f"cannot place {tool}: holding {self.held}")
        dock = scene.mating_target(self.cfg, tool)
        above = scene.mating_target(self.cfg, tool, self.cfg["approach_height"])
        self._run(S.APPROACH, f"above cradle {tool}", lambda: self._approach(above, f"above cradle {tool}"))
        self._run(S.DESCEND, f"{tool} into cradle", lambda: self._linear(dock, f"{tool} into cradle"))
        self._run(S.RELEASE, tool, lambda: self._release(tool))
        self._run(S.RETREAT, f"clear of {tool}", lambda: self._linear(above, f"clear of {tool}"))

    def run_sequence(self, steps):
        for step in steps:
            if step == "home":
                self.home()
            elif isinstance(step, dict) and "pick" in step:
                self.pick(step["pick"])
            elif isinstance(step, dict) and "place" in step:
                self.place(step["place"])
            else:
                raise StepFailed(f"unknown sequence step {step!r}")


def main():
    rclpy.init()
    node = rclpy.create_node("ar4tc_toolchange_demo")
    args = [a for a in rclpy.utilities.remove_ros_args(sys.argv)[1:] if a.endswith(".yaml")]
    cfg_name = args[0] if args else None
    cfg = load_config(cfg_name)
    mv, kin = connect(node, cfg)
    if not scene.reset(mv, cfg):
        node.get_logger().error("could not add dock/tools to planning scene")
        sys.exit(1)
    tc = ToolChanger(node, cfg, mv, kin)
    tc.publish_labels()
    start_q = mv.joint_positions().tolist()
    t0, result = time.time(), S.DONE
    try:
        tc.run_sequence(cfg["sequence"])
    except StepFailed as e:
        result = S.FAULT
        node.get_logger().error(f"FAULT: {e}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.expanduser("~/ar4tc_reports")
    os.makedirs(out_dir, exist_ok=True)
    report = {"result": result.value, "held_tool": tc.held, "total_seconds": round(time.time() - t0, 1),
              "config": cfg, "steps": tc.records}
    out = f"{out_dir}/toolchange_{stamp}.json"
    with open(out, "w") as f:
        json.dump(report, f, indent=2)
    prog = f"{out_dir}/program_{stamp}.json"
    with open(prog, "w") as f:
        json.dump({"result": result.value, "config_name": cfg_name or "toolchanger.yaml", "config": cfg,
                   "joint_names": cfg["arm_joints"], "start": start_q, "items": tc.program}, f, indent=1)
    print(f"\n{'#':>2} {'STATE':9} {'STEP':22} {'OK':4} {'s':>5}  INFO")
    for i, r in enumerate(tc.records, 1):
        print(f"{i:>2} {r['state']:9} {r['step'][:22]:22} {str(r['ok']):4} {r['seconds']:>5}  {r['info']}")
    print(f"\nRESULT: {result.value}  holding: {tc.held}  time: {report['total_seconds']} s\n"
          f"report: {out}\nprogram (for export_hmi): {prog}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result is S.DONE else 1)


if __name__ == "__main__":
    main()
