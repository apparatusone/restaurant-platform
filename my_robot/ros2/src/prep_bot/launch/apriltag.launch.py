#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('prep_bot')
    config_file = os.path.join(pkg_share, 'config', 'tags.yaml')
    
    return LaunchDescription([
        # AprilTag detector node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            arguments=['--ros-args', '--log-level', 'INFO'],
            remappings=[
                ('image_rect', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
            ],
            parameters=[config_file],
            output='screen'
        )
    ])
