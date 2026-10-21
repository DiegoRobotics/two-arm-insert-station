"""Planning-scene objects for the dock and tools, plus attach/detach messages."""
import math

import numpy as np
import pinocchio as pin
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


def _box(obj_id, frame, center, size):
    o = CollisionObject()
    o.id, o.header.frame_id, o.operation = obj_id, frame, CollisionObject.ADD
    o.pose.position.x, o.pose.position.y, o.pose.position.z = (float(v) for v in center)
    o.pose.orientation.w = 1.0
    o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])]
    p = Pose()
    p.orientation.w = 1.0
    o.primitive_poses = [p]
    return o


def tool_center(cfg, tool):
    t = cfg["tools"][tool]
    dock = cfg["dock"]
    top = dock["center"][2] + dock["size"][2] / 2
    return np.array([t["slot_xy"][0], t["slot_xy"][1], top + t["size"][2] / 2])


def tcp_target(cfg, tool, height_above_grasp=0.0):
    """TCP pose for coupling with a tool: pointing straight down onto its top face.

    Yaw follows the slot's azimuth so the wrist stays in its natural orientation
    (at J1 = 0 the MK3 faces -Y and a downward flange has x = -X, y = +Y).
    """
    t = cfg["tools"][tool]
    c = tool_center(cfg, tool)
    top = c[2] + t["size"][2] / 2
    z = top - cfg["grasp_depth"] + height_above_grasp
    azimuth = math.atan2(c[1], c[0])
    R0 = np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])
    R = pin.rpy.rpyToMatrix(0, 0, azimuth + math.pi / 2) @ R0
    return pin.SE3(R, np.array([c[0], c[1], z]))


def initial_scene(cfg):
    s = PlanningScene()
    frame = cfg["base_frame"]
    s.world.collision_objects.append(_box("dock", frame, cfg["dock"]["center"], cfg["dock"]["size"]))
    for name, t in cfg["tools"].items():
        s.world.collision_objects.append(_box(name, frame, tool_center(cfg, name), t["size"]))
    return s


def attach(cfg, tool):
    """Move a world object onto the flange (MoveIt keeps its current geometry and pose)."""
    a = AttachedCollisionObject()
    a.link_name = cfg["tip_link"]
    a.object.id, a.object.operation = tool, CollisionObject.ADD
    a.touch_links = cfg["touch_links"]
    s = PlanningScene()
    s.robot_state.attached_collision_objects = [a]
    return s


def detach(cfg, tool):
    """Release an attached object back into the world at its current pose."""
    a = AttachedCollisionObject()
    a.link_name = cfg["tip_link"]
    a.object.id, a.object.operation = tool, CollisionObject.REMOVE
    s = PlanningScene()
    s.robot_state.attached_collision_objects = [a]
    return s
