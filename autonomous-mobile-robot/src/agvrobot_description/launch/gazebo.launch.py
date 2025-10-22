from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from launch_ros.descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_share = FindPackageShare('agvrobot_description').find('agvrobot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'agvrobot.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'controller.yaml')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str
    )

    gazebo_pkg_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'agvrobot'
            ],
            output='screen'
        ),


        # ROS2 Control Node (controller manager)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_file],
            remappings=[('/controller_manager/robot_description', '/robot_description')],
            output='screen'
        ),

        # Spawner for joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Spawner for diff_drive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Include Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
        ),
    ])