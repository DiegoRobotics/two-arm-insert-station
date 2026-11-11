"""RViz + joint sliders for the JetCobot, forwarded to the real arm, plus the USB camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    speed = LaunchConfiguration('speed')
    port = LaunchConfiguration('port')
    camera = LaunchConfiguration('camera')
    video_device = LaunchConfiguration('video_device')

    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='25', description='Arm speed 1-100'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('jetcobot_description'), 'launch', 'display.launch.py'])),

        Node(
            package='jetcobot_placer',
            executable='joint_bridge',
            output='screen',
            parameters=[{'port': port, 'speed': speed}],
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            condition=IfCondition(camera),
            parameters=[{
                'video_device': video_device,
                'image_width': 640,
                'image_height': 480,
            }],
        ),
    ])
