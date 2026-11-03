"""Shortcut: bringup.launch.py with run:=true (simulation only).

ros2 launch ar4tc_toolchanger toolchange_demo.launch.py               # mock hardware, KDL IK
ros2 launch ar4tc_toolchanger toolchange_demo.launch.py mode:=gazebo  # Gazebo physics
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [("mode", "mock"), ("ik", "kdl"), ("run", "true"), ("rviz", "true")]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args] + [
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("ar4tc_toolchanger"), "launch", "bringup.launch.py"]),
            launch_arguments={k: LaunchConfiguration(k) for k, _ in args}.items())])
