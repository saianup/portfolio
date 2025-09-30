# ekf_localization_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('agvrobot_description')
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_local',
            output='screen',
            parameters=[ekf_config_path], 
            remappings=[
                ('/odometry/filtered', '/odometry/filtered')
            ]
        )
    ])
