from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('prep_bot')
    rviz_config = os.path.join(pkg_share, 'rviz', 'main.rviz')
    
    return LaunchDescription([
        Node(
            package='prep_bot',
            executable='record3d_bridge',
            name='record3d_bridge',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
