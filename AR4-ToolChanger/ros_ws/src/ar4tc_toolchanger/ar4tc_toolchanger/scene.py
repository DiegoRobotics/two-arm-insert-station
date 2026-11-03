"""Planning-scene geometry for the dock, cradles and tools, plus attach/detach messages.

Tool frame: origin at the top centre of the coupling plate, +z down into the tool (equal to tc_mating
when coupled). A docked tool hangs in its cradle with that frame pointing straight down.
"""
import math

import numpy as np
import pinocchio as pin
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

R_DOWN = np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])  # z down; MK3 wrist-natural at J1 = 0


def _pose(M: pin.SE3) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in M.translation)
    q = pin.Quaternion(M.rotation)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q.x, q.y, q.z, q.w
    return p


def _prim(kind, dims):
    t = {"box": SolidPrimitive.BOX, "cylinder": SolidPrimitive.CYLINDER}[kind]
    return SolidPrimitive(type=t, dimensions=[float(d) for d in dims])


def _obj(obj_id, frame, M, parts):
    """parts: list of (primitive, SE3 relative to M)."""
    o = CollisionObject(id=obj_id, operation=CollisionObject.ADD)
    o.header.frame_id = frame
    o.pose = _pose(M)
    o.primitives = [p for p, _ in parts]
    o.primitive_poses = [_pose(T) for _, T in parts]
    return o


def _at(x=0.0, y=0.0, z=0.0):
    return pin.SE3(np.eye(3), np.array([x, y, z]))


def tool_parts(t):
    """Primitives of one tool in the tool frame (z down)."""
    ph = t["plate"][2]
    parts = [(_prim("box", t["plate"]), _at(z=ph / 2))]
    b = t["body"]
    if b["type"] == "box":
        parts.append((_prim("box", b["size"]), _at(z=ph + b["size"][2] / 2)))
        bottom = ph + b["size"][2]
    else:
        parts.append((_prim("cylinder", [b["length"], b["radius"]]), _at(z=ph + b["length"] / 2)))
        bottom = ph + b["length"]
    if "fingers" in t:
        f = t["fingers"]
        for s in (-1, 1):
            parts.append((_prim("box", f["size"]), _at(x=s * f["spacing"] / 2, z=bottom + f["size"][2] / 2)))
        bottom += f["size"][2]
    if "tip" in t:
        parts.append((_prim("cylinder", [t["tip"]["length"], t["tip"]["radius"]]),
                      _at(z=bottom + t["tip"]["length"] / 2)))
        bottom += t["tip"]["length"]
    return parts, bottom


def body_half_width(t):
    b = t["body"]
    w = b["size"][0] / 2 if b["type"] == "box" else b["radius"]
    if "fingers" in t:
        w = max(w, t["fingers"]["spacing"] / 2 + t["fingers"]["size"][0] / 2)
    return w


def slot_rotation(t):
    x, y = t["slot_xy"]
    return pin.rpy.rpyToMatrix(0, 0, math.atan2(y, x) + math.pi / 2) @ R_DOWN


def post_height(cfg, t):
    return tool_parts(t)[1] + 0.010   # posts taller than the hanging tool, 10 mm under it


def docked_tool_pose(cfg, tool):
    """World pose of the tool frame (plate top centre) when the tool sits in its cradle."""
    t, d = cfg["tools"][tool], cfg["dock"]
    base_top = d["base"]["center"][2] + d["base"]["size"][2] / 2
    plate_top = base_top + post_height(cfg, t) + d["float"] + t["plate"][2]
    return pin.SE3(slot_rotation(t), np.array([*t["slot_xy"], plate_top]))


def mating_target(cfg, tool, above=0.0):
    """Pose for tc_mating to couple with (or deliver) a tool: dock_gap above the plate, plus `above`."""
    M = docked_tool_pose(cfg, tool)
    return pin.SE3(M.rotation, M.translation + np.array([0, 0, cfg["dock_gap"] + above]))


def tool_center(cfg, tool):
    return docked_tool_pose(cfg, tool).translation


def initial_scene(cfg):
    s = PlanningScene(is_diff=True)
    frame, d = cfg["base_frame"], cfg["dock"]
    base = _obj("dock_base", frame, _at(*d["base"]["center"]), [(_prim("box", d["base"]["size"]), _at())])
    s.world.collision_objects.append(base)
    s.object_colors.append(ObjectColor(id="dock_base", color=ColorRGBA(r=0.35, g=0.35, b=0.38, a=1.0)))
    base_top = d["base"]["center"][2] + d["base"]["size"][2] / 2
    p = d["post"]
    for name, t in cfg["tools"].items():
        h = post_height(cfg, t)
        off = body_half_width(t) + p["clearance"] + p["width"] / 2
        cradle = pin.SE3(slot_rotation(t), np.array([*t["slot_xy"], base_top]))
        # posts sit on the base, either side of the tool body (tool-frame x), cradle frame z down -> place at -h/2
        posts = [(_prim("box", [p["width"], p["depth"], h]), _at(x=s_ * off, z=-h / 2)) for s_ in (-1, 1)]
        s.world.collision_objects.append(_obj(f"cradle_{name}", frame, cradle, posts))
        s.object_colors.append(ObjectColor(id=f"cradle_{name}", color=ColorRGBA(r=0.25, g=0.25, b=0.28, a=1.0)))
        parts, _ = tool_parts(t)
        s.world.collision_objects.append(_obj(name, frame, docked_tool_pose(cfg, name), parts))
        s.object_colors.append(ObjectColor(id=name, color=ColorRGBA(**dict(zip("rgba", map(float, t["color"]))))))
    return s


def reset(mv, cfg):
    """Put the scene in its start state (every tool in its cradle), whatever a previous run left behind."""
    ids = ["dock_base"] + [f"cradle_{t}" for t in cfg["tools"]] + list(cfg["tools"])
    for t in cfg["tools"]:
        mv.apply_scene(detach(cfg, t))          # attached -> world (no-op if not attached)
    rm = PlanningScene(is_diff=True)
    for i in ids:
        o = CollisionObject(id=i, operation=CollisionObject.REMOVE)
        o.header.frame_id = cfg["base_frame"]
        rm.world.collision_objects.append(o)
    mv.apply_scene(rm)
    return mv.apply_scene(initial_scene(cfg))


def attach(cfg, tool):
    """Move a world object onto the flange (MoveIt keeps its current geometry and pose)."""
    a = AttachedCollisionObject(link_name=cfg["tip_link"], touch_links=cfg["touch_links"])
    a.object.id, a.object.operation = tool, CollisionObject.ADD
    s = PlanningScene(is_diff=True)
    s.robot_state.is_diff = True
    s.robot_state.attached_collision_objects = [a]
    return s


def detach(cfg, tool):
    """Release an attached object back into the world at its current pose."""
    a = AttachedCollisionObject(link_name=cfg["tip_link"])
    a.object.id, a.object.operation = tool, CollisionObject.REMOVE
    s = PlanningScene(is_diff=True)
    s.robot_state.is_diff = True
    s.robot_state.attached_collision_objects = [a]
    return s
