from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('prep_bot')
    robot_control_config = os.path.join(pkg_share, 'config', 'robot_control.yaml')
    
    return LaunchDescription([
        # API Bridge (HTTP server for external control)
        Node(
            package='prep_bot',
            executable='api_bridge',
            name='api_bridge',
            parameters=[robot_control_config],
            output='screen'
        ),
    ])
