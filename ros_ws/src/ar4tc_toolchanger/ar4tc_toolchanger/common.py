import os

import yaml
from ament_index_python.packages import get_package_share_directory

from .kinematics import KinematicsValidator
from .motion import MoveItClient


def load_config(path=None):
    path = path or os.path.join(get_package_share_directory("ar4tc_toolchanger"), "config", "toolchanger.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


def connect(node, cfg):
    """Wait for MoveIt + controllers, then build the Pinocchio validator from the live URDF."""
    mv = MoveItClient(node, cfg)
    mv.wait_ready()
    kin = KinematicsValidator(mv.robot_description(), cfg["tip_link"], cfg["tcp_offset"], len(cfg["arm_joints"]))
    return mv, kin
