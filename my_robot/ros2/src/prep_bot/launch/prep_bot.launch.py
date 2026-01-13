from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('prep_bot')
    apriltag_launch = os.path.join(pkg_share, 'launch', 'apriltag.launch.py')
    rviz_config = os.path.join(pkg_share, 'rviz', 'main.rviz')
    robot_control_config = os.path.join(pkg_share, 'config', 'robot_control.yaml')
    
    # MoveIt config
    moveit_config = MoveItConfigsBuilder("prep_bot", package_name="robot_moveit_config").to_moveit_configs()
    
    # MoveIt config package
    moveit_config_pkg = get_package_share_directory('robot_moveit_config')
    moveit_launch = os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
    
    return LaunchDescription([
        # Publish robot_description for RViz
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[moveit_config.robot_description]
        ),
        
        # World to base_link transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            output='screen'
        ),
        
        # Record3D camera bridge
        Node(
            package='prep_bot',
            executable='record3d_bridge',
            name='record3d_bridge',
            output='screen'
        ),
        
        # Camera calibration node (publishes base_link -> camera_color_frame)
        Node(
            package='prep_bot',
            executable='camera_calibration',
            name='camera_calibration',
            parameters=[robot_control_config],
            output='screen'
        ),
        
        # Include AprilTag detection launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(apriltag_launch)
        ),
        
        # Include MoveIt move_group
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={'log_level': 'error'}.items()
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
