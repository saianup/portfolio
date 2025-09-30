import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Paths for different files and directories
    share_dir = get_package_share_directory('agvrobot_description')  # Adjust this to your package name
    xacro_file = os.path.join(share_dir, 'urdf', 'agvrobot.xacro')  # Adjust this to your URDF file
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')  # Adjust this to your RVIZ config

    # Localization EKF Config File (Update path to your config file)
    ekf_config_file = os.path.join(share_dir, 'config', 'ekf.yaml')

    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Whether to start the GUI for joint_state_publisher'
    )
    
    show_gui = LaunchConfiguration('gui')

    # Declare localization configuration arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) time if true'
    )

    # Start robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )

    # Start the joint_state_publisher node (GUI or non-GUI based)
    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Start EKF for robot localization
    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_file, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',  # Match your Arduino code
            'world_frame': 'odom'
    }],
    output='screen'
)
    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Return the LaunchDescription with all nodes and arguments
    return LaunchDescription([
        gui_arg,
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        ekf_localization_node,
        rviz_node
    ])
