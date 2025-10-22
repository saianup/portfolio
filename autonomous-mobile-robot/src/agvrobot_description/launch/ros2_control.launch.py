from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('agvrobot_description'),
                             'urdf', 'agvrobot.xacro')
    rviz_config_path = os.path.join(get_package_share_path('agvrobot_description'),
                                    'config', 'display.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # Controller Manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            os.path.join(get_package_share_path('agvrobot_description'), 'config', 'controller.yaml')
        ],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Diff Drive Controller Spawner
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # RViz2
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        rviz2_node
    ])