from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('prep_bot')
    apriltag_launch = os.path.join(pkg_share, 'launch', 'apriltag.launch.py')
    api_bridge_launch = os.path.join(pkg_share, 'launch', 'api_bridge.launch.py')
    rviz_config = os.path.join(pkg_share, 'rviz', 'main.rviz')
    robot_control_config = os.path.join(pkg_share, 'config', 'robot_control.yaml')
    
    enable_api_arg = DeclareLaunchArgument(
        'enable_api',
        default_value='true',
        description='Enable API bridge (HTTP server)'
    )
    
    # MoveIt config
    moveit_config = MoveItConfigsBuilder("prep_bot", package_name="robot_moveit_config").to_moveit_configs()
    
    return LaunchDescription([
        enable_api_arg,
        
        # Publish robot_description for RViz
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[moveit_config.robot_description]
        ),
        
        # World to base_link transform
        # https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html#the-proper-way-to-publish-static-transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'world', '--child-frame-id', 'base_link'],
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
        
        # Move group node (launched directly like restaurant_robot)
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                moveit_config.robot_description_kinematics,
                {'use_sim_time': False},
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        
        # Scene manager (collision objects)
        Node(
            package='prep_bot',
            executable='scene_manager',
            name='scene_manager',
            output='screen'
        ),
        
        # Motion controller (MoveIt planning)
        Node(
            package='prep_bot',
            executable='motion_controller',
            name='motion_controller',
            parameters=[robot_control_config],
            output='screen'
        ),
        
        # Task controller (high-level task logic)
        Node(
            package='prep_bot',
            executable='task_controller',
            name='task_controller',
            parameters=[robot_control_config],
            output='screen'
        ),
        
        # Hardware interface (STM32 communication, publishes /joint_states)
        Node(
            package='prep_bot',
            executable='hardware_interface',
            name='hardware_interface',
            parameters=[robot_control_config],
            output='screen'
        ),
        
        # Trajectory executor (executes MoveIt trajectories via STM32)
        Node(
            package='prep_bot',
            executable='trajectory_executor',
            name='trajectory_executor',
            parameters=[robot_control_config],
            output='screen'
        ),
        
        # API Bridge (conditionally included)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(api_bridge_launch),
            condition=IfCondition(LaunchConfiguration('enable_api'))
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        ),
    ])
