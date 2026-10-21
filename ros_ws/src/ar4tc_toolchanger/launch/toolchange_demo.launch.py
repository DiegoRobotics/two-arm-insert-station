"""MK3 + MoveIt + RViz on mock hardware, then run the tool-change sequence.

ros2 launch ar4tc_toolchanger toolchange_demo.launch.py            # sim + sequence
ros2 launch ar4tc_toolchanger toolchange_demo.launch.py run:=false # sim only (then use goto_pose)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("annin_ar4_moveit_config"), "launch", "demo.launch.py"]),
        launch_arguments={"ar_model": "mk3"}.items(),
    )
    # The node waits for move_group and the controllers itself, so no timer is needed.
    demo = Node(package="ar4tc_toolchanger", executable="toolchange_demo", output="screen",
                condition=IfCondition(LaunchConfiguration("run")))
    return LaunchDescription([DeclareLaunchArgument("run", default_value="true"), sim, demo])
