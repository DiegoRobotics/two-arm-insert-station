"""Simulated tool change: HOME -> pick tool_a -> HOME -> return tool_a -> pick tool_b -> HOME.

Every step is a state with an action and a verification. Any failed check moves the
machine to FAULT and stops the robot where it is (sim only, no automatic recovery).
"""
import json
import math
import os
import sys
import time
from enum import Enum

import numpy as np
import rclpy

from . import scene
from .common import connect, load_config
from .kinematics import pose_error


class S(Enum):
    HOME = "HOME"
    APPROACH = "APPROACH"      # joint-space move to pre-dock pose above the slot
    OPEN = "OPEN"              # open gripper
    DESCEND = "DESCEND"        # straight-line move down onto the tool
    COUPLE = "COUPLE"          # close gripper + attach tool to flange
    RELEASE = "RELEASE"        # open gripper + detach tool into its slot
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

    def _reached(self, target):
        q = self.mv.joint_positions()
        pos, rot = pose_error(self.kin.fk(q), target)
        ok = pos <= self.tol["pos_mm"] and rot <= self.tol["rot_deg"]
        return ok, f"TCP error {pos:.3f} mm / {rot:.3f} deg"

    def _approach(self, target):
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
        ok, err = self._reached(target)
        return ok, f"IK q(deg)={np.round(np.degrees(q), 1).tolist()} w={chk.manipulability:.4f}; {err}"

    def _linear(self, target):
        # Docking moves intentionally touch the tool/dock, so collision checking is off for this short segment.
        ok, info = self.mv.move_linear(self.kin.tcp_to_flange(target), avoid_collisions=False)
        if not ok:
            return False, info
        ok, err = self._reached(target)
        return ok, f"{info}; {err}"

    def _scene_state(self):
        sc = self.mv.get_scene()
        attached = {a.object.id for a in sc.robot_state.attached_collision_objects}
        world = {o.id: o for o in sc.world.collision_objects}
        return attached, world

    def _grip(self, key):
        if not self.cfg.get("use_gripper", True):
            return True, "gripper disabled in config (skipped)"
        return self.mv.gripper(self.cfg["gripper"][key])

    def _couple(self, tool):
        ok, info = self._grip("closed")
        if not ok:
            return False, f"gripper: {info}"
        if not self.mv.apply_scene(scene.attach(self.cfg, tool)):
            return False, "attach rejected by planning scene"
        attached, world = self._scene_state()
        if tool not in attached or tool in world:
            return False, f"attach not confirmed (attached={sorted(attached)})"
        self.held = tool
        return True, f"{tool} attached to {self.cfg['tip_link']}; {info}"

    def _release(self, tool):
        ok, info = self._grip("open")
        if not ok:
            return False, f"gripper: {info}"
        if not self.mv.apply_scene(scene.detach(self.cfg, tool)):
            return False, "detach rejected by planning scene"
        attached, world = self._scene_state()
        if tool in attached or tool not in world:
            return False, f"detach not confirmed (attached={sorted(attached)})"
        p = world[tool].pose.position
        err = float(np.linalg.norm(np.array([p.x, p.y, p.z]) - scene.tool_center(self.cfg, tool)) * 1000)
        self.held = None
        return err <= self.tol["place_mm"], f"{tool} back in slot, placement error {err:.3f} mm"

    # ---- sequences ------------------------------------------------------
    def home(self):
        def go():
            ok, info = self.mv.move_to_joints(self.cfg["home"])
            q = self.mv.joint_positions()
            err = float(np.max(np.abs(np.degrees(q - np.array(self.cfg["home"])))))
            return ok and err <= self.tol["home_joint_deg"], f"{info}; max joint error {err:.3f} deg"
        self._run(S.HOME, "home", go)

    def pick(self, tool):
        if self.held:
            raise StepFailed(f"cannot pick {tool}: already holding {self.held}")
        grasp = scene.tcp_target(self.cfg, tool)
        above = scene.tcp_target(self.cfg, tool, self.cfg["approach_height"])
        self._run(S.APPROACH, f"above {tool}", lambda: self._approach(above))
        self._run(S.OPEN, "gripper open", lambda: self._grip("open"))
        self._run(S.DESCEND, f"onto {tool}", lambda: self._linear(grasp))
        self._run(S.COUPLE, tool, lambda: self._couple(tool))
        self._run(S.RETREAT, f"lift {tool}", lambda: self._linear(above))

    def place(self, tool):
        if self.held != tool:
            raise StepFailed(f"cannot place {tool}: holding {self.held}")
        grasp = scene.tcp_target(self.cfg, tool)
        above = scene.tcp_target(self.cfg, tool, self.cfg["approach_height"])
        self._run(S.APPROACH, f"above slot {tool}", lambda: self._approach(above))
        self._run(S.DESCEND, f"{tool} into slot", lambda: self._linear(grasp))
        self._run(S.RELEASE, tool, lambda: self._release(tool))
        self._run(S.RETREAT, f"clear of {tool}", lambda: self._linear(above))


def main():
    rclpy.init()
    node = rclpy.create_node("ar4tc_toolchange_demo")
    cfg = load_config(sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].endswith(".yaml") else None)
    mv, kin = connect(node, cfg)
    if not mv.apply_scene(scene.initial_scene(cfg)):
        node.get_logger().error("could not add dock/tools to planning scene")
        sys.exit(1)
    tc = ToolChanger(node, cfg, mv, kin)
    t0, result = time.time(), S.DONE
    try:
        tc.home()
        tc.pick("tool_a")
        tc.home()
        tc.place("tool_a")
        tc.pick("tool_b")
        tc.home()
    except StepFailed as e:
        result = S.FAULT
        node.get_logger().error(f"FAULT: {e}")

    report = {"result": result.value, "held_tool": tc.held, "total_seconds": round(time.time() - t0, 1),
              "config": cfg, "steps": tc.records}
    out = os.path.expanduser(f"~/ar4tc_reports/toolchange_{time.strftime('%Y%m%d_%H%M%S')}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\n{'#':>2} {'STATE':9} {'STEP':22} {'OK':4} {'s':>5}  INFO")
    for i, r in enumerate(tc.records, 1):
        print(f"{i:>2} {r['state']:9} {r['step'][:22]:22} {str(r['ok']):4} {r['seconds']:>5}  {r['info']}")
    print(f"\nRESULT: {result.value}  holding: {tc.held}  time: {report['total_seconds']} s  report: {out}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result is S.DONE else 1)


if __name__ == "__main__":
    main()
