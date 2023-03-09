#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_directory = get_package_share_directory('r2mini_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file_path = LaunchConfiguration('urdf_file_path')
    urdf_file_name = 'r2mini.urdf'

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file_path',
        default_value=os.path.join(share_directory, 'urdf', urdf_file_name),
        description='urdf file path'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file_path],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(urdf_file_arg)
    ld.add_action(robot_state_publisher_node)

    return ld
