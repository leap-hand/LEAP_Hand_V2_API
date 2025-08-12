import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leap_v2',
            executable='leap_v2_node.py',
            name='leaphand_v2_node',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FD016668-if00'}
            ]
        ),
        Node(
            package='leap_v2',
            executable='ros2_example.py',
            name='ros2_example',
            emulate_tty=True,
            output='screen'
        )
    ])
