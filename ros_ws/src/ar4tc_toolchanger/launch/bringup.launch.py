"""MK3 + MoveIt + RViz with a selectable backend and IK solver.

  mode:=mock       ros2_control mock hardware (default)
  mode:=gazebo     Gazebo Harmonic physics (upstream annin_ar4_gazebo)
  mode:=annin      REAL ROBOT through annin_bridge -> unchanged Annin v6.0 firmware (serial_port:=/dev/ttyACM0)
  mode:=annin_emu  annin_bridge -> firmware emulator on a pseudo-terminal (no hardware)
  ik:=kdl|pick_ik
  run:=true        start the tool-change sequence automatically (refused in mode:=annin)
  demo_config:=<file in config/>  sequence config (default: toolchanger_real_test.yaml for annin modes)
The upstream ROS-firmware driver mode was removed: it needs DIP/firmware changes this robot can't take (REAL_ROBOT.md).

Move-group setup mirrors upstream demo.launch.py / moveit.launch.py; only the kinematics plugin is ours.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def _yaml(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def _xacro(pkg, rel, **args):
    cmd = [PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
           PathJoinSubstitution([FindPackageShare(pkg), rel])]
    for k, v in args.items():
        cmd += [" ", f"{k}:={v}"]
    return ParameterValue(Command(cmd), value_type=str)


def _setup(context):
    a = {k: context.launch_configurations[k] for k in
         ["mode", "ik", "run", "rviz", "ar_model", "serial_port", "include_gripper", "demo_config"]}
    mode, model = a["mode"], a["ar_model"]
    if mode not in ("mock", "gazebo", "annin", "annin_emu"):
        raise RuntimeError(f"mode must be mock|gazebo|annin|annin_emu, got {mode}")
    if mode == "gazebo" and a["run"] == "true":
        raise RuntimeError("the tool-change sequence needs the tool-changer model; Gazebo uses the upstream "
                           "gripper model. Use mode:=mock (or annin_emu).")
    if mode == "annin" and a["run"] == "true":
        raise RuntimeError("run:=true is refused with mode:=annin (real robot); start the sequence by hand (REAL_ROBOT.md)")
    bridge = mode.startswith("annin")
    gazebo = mode == "gazebo"
    sim_time = {"use_sim_time": gazebo}

    if gazebo:  # upstream Gazebo model (servo gripper, no tool-changer master)
        grip = a["include_gripper"]
        robot_description = {"robot_description": _xacro(
            "annin_ar4_description", "urdf/ar.urdf.xacro", ar_model=model, include_gripper=grip)}
        semantic = {"robot_description_semantic": _xacro(
            "annin_ar4_moveit_config", "srdf/ar.srdf.xacro", name=model, include_gripper=grip)}
    else:       # our model: MK3 arm + tool-changer master, no gripper (mock ros2_control only in mock mode)
        robot_description = {"robot_description": _xacro(
            "ar4tc_toolchanger", "urdf/ar4tc.urdf.xacro", ar_model=model, mock=str(mode == "mock").lower())}
        semantic = {"robot_description_semantic": _xacro("ar4tc_toolchanger", "srdf/ar4tc.srdf.xacro")}
    kinematics = {"robot_description_kinematics": _yaml("ar4tc_toolchanger", f"config/kinematics_{a['ik']}.yaml")}
    joint_limits = ParameterFile(PathJoinSubstitution(
        [FindPackageShare("annin_ar4_moveit_config"), "config/joint_limits.yaml"]), allow_substs=True)
    planning = {"default_planning_pipeline": "ompl", "planning_pipelines": ["ompl", "pilz"],
                "ompl": _yaml("annin_ar4_moveit_config", "config/ompl_planning.yaml"),
                "pilz": _yaml("annin_ar4_moveit_config", "config/pilz_planning.yaml")}
    if gazebo:
        controllers = ParameterFile(PathJoinSubstitution(
            [FindPackageShare("annin_ar4_moveit_config"), "config/controllers.yaml"]), allow_substs=True)
    else:
        controllers = _yaml("ar4tc_toolchanger", "config/moveit_controllers_arm_only.yaml")
    execution = {"moveit_manage_controllers": False,
                 "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                 "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                 "trajectory_execution.allowed_goal_duration_margin": 0.5,
                 "trajectory_execution.allowed_start_tolerance": 0.01,
                 # The bridge runs stop-and-go RJ segments and enforces its own timeouts and speed cap.
                 "trajectory_execution.execution_duration_monitoring": not bridge}
    scene_monitor = {"publish_planning_scene": True, "publish_geometry_updates": True,
                     "publish_state_updates": True, "publish_transforms_updates": True}

    nodes = [Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                  parameters=[robot_description, semantic, kinematics, joint_limits, planning,
                              execution, controllers, scene_monitor, sim_time])]
    if a["rviz"] == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          arguments=["-d", os.path.join(get_package_share_directory("ar4tc_toolchanger"),
                                                        "rviz", "ar4tc.rviz")],
                          parameters=[robot_description, semantic, kinematics, planning, sim_time]))

    if mode == "mock":
        ros2_controllers = ParameterFile(PathJoinSubstitution(
            [FindPackageShare("annin_ar4_driver"), "config", "controllers.yaml"]), allow_substs=True)
        nodes += [Node(package="robot_state_publisher", executable="robot_state_publisher",
                       parameters=[robot_description]),
                  Node(package="controller_manager", executable="ros2_control_node",
                       parameters=[robot_description, ros2_controllers, {"tf_prefix": ""}])]
        nodes += [Node(package="controller_manager", executable="spawner", arguments=[c, "-c", "/controller_manager"])
                  for c in ("joint_state_broadcaster", "joint_trajectory_controller")]
    elif mode == "gazebo":
        nodes.append(IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("annin_ar4_gazebo"), "launch", "gazebo.launch.py"]),
            launch_arguments={"ar_model": model}.items()))
    else:  # annin / annin_emu: our bridge to the unchanged Annin v6.0 firmware
        emu = mode == "annin_emu"
        port = "/tmp/ttyANNIN" if emu else a["serial_port"]
        nodes.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                          parameters=[robot_description]))
        if emu:
            nodes.append(Node(package="ar4tc_toolchanger", executable="annin_emulator", output="screen",
                              arguments=["--link", port]))
        nodes.append(Node(package="ar4tc_toolchanger", executable="annin_bridge", name="annin_bridge", output="screen",
                          parameters=[os.path.join(get_package_share_directory("ar4tc_toolchanger"), "config",
                                                   "annin_bridge.yaml"),
                                      # emulator: no first-session travel limit, so whole sequences run
                                      {"port": port, "start_enabled": emu, **({"max_travel_deg": 0.0} if emu else {})}]))

    if a["run"] == "true":
        cfg = a["demo_config"] or ("toolchanger_real_test.yaml" if bridge else "toolchanger.yaml")
        nodes.append(Node(package="ar4tc_toolchanger", executable="toolchange_demo", output="screen",
                          arguments=[os.path.join(get_package_share_directory("ar4tc_toolchanger"), "config", cfg)],
                          parameters=[sim_time]))
    return nodes


def generate_launch_description():
    args = [("mode", "mock"), ("ik", "kdl"), ("run", "false"), ("rviz", "true"), ("ar_model", "mk3"),
            ("serial_port", "/dev/ttyACM0"), ("include_gripper", "True"), ("demo_config", ""),
            ("tf_prefix", "")]  # referenced as $(var tf_prefix) inside upstream controller YAML
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args] + [OpaqueFunction(function=_setup)])
