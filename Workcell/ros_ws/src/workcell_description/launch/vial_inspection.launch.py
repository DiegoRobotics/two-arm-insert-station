import os

import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('workcell_description')

    # Expand the vial inspection xacro into plain URDF text
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'vial_inspection.urdf.xacro')
    robot_description = xacro.process_file(xacro_file_path).toxml()

    # Publishes the TF of every link from the URDF + joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Sliders for the myCobot joints (the AR4 is static, so it has none)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_config_path = os.path.join(pkg_share, 'rviz', 'vial_inspection.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
