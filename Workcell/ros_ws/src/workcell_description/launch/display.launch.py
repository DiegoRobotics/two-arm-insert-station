import os

import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('workcell_description')

    # Expand the xacro into plain URDF text (robot_state_publisher wants the text, not the path)
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'workcell.urdf.xacro')
    robot_description = xacro.process_file(xacro_file_path).toxml()

    # Create a Node action to launch the robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Load the saved RViz config if it exists (step 3); otherwise RViz opens with defaults
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'workcell.rviz')
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    # Create a Node action to launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    # Return the LaunchDescription with both nodes
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
