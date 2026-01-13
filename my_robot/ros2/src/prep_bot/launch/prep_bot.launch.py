#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('prep_bot')
    apriltag_launch = os.path.join(pkg_share, 'launch', 'apriltag.launch.py')
    rviz_config = os.path.join(pkg_share, 'rviz', 'main.rviz')
    
    return LaunchDescription([
        # place the camera in the world
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_link',
            arguments=['0', '0', '0.5', '-1.57079632679', '0', '-1.57079632679', 'world', 'camera_link'],
            output='screen'
        ),
        
        # camera optical frame orientation (down)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_frame',
            arguments=['0', '0', '0', '0', '-3.1415926536', '-1.57079632679', 
                      'camera_link', 'camera_color_frame'],
            output='screen'
        ),
        
        # Record3D camera bridge
        Node(
            package='prep_bot',
            executable='record3d_bridge',
            name='record3d_bridge',
            output='screen'
        ),
        
        # Include AprilTag detection launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(apriltag_launch)
        ),
        
        # Pick and place controller
        Node(
            package='prep_bot',
            executable='pick_place',
            name='pick_place',
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        ),
    ])
