import os

import yaml
from ament_index_python.packages import get_package_share_directory

from .kinematics import KinematicsValidator
from .motion import MoveItClient


def config_path(name):
    if os.path.isabs(name) or os.path.exists(name):
        return name
    return os.path.join(get_package_share_directory("ar4tc_toolchanger"), "config", name)


def _merge(base, over):
    out = dict(base)
    for k, v in over.items():
        out[k] = _merge(base[k], v) if isinstance(v, dict) and isinstance(base.get(k), dict) else v
    return out


def load_config(path=None):
    """Load a config; `extends: other.yaml` deep-merges this file over the other one."""
    with open(config_path(path or "toolchanger.yaml")) as f:
        cfg = yaml.safe_load(f)
    parent = cfg.pop("extends", None)
    return _merge(load_config(parent), cfg) if parent else cfg


def connect(node, cfg):
    """Wait for MoveIt + controllers, then build the Pinocchio validator from the live URDF.
    The validator's 'TCP' is the coupling face (tc_mating) unless cfg['tcp_offset'] overrides it."""
    mv = MoveItClient(node, cfg)
    mv.wait_ready()
    offset = cfg.get("tcp_offset", cfg.get("mating_offset", [0.0, 0.0, 0.0]))
    kin = KinematicsValidator(mv.robot_description(), cfg["tip_link"], offset, len(cfg["arm_joints"]))
    return mv, kin
