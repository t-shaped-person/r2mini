#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ROBOT_MODEL = os.environ['ROBOT_MODEL']
    share_directory = get_package_share_directory('r2mini_description')
    
    urdf_file = LaunchConfiguration(
        'urdf_file',
        default=os.path.join(share_directory, 'urdf', ROBOT_MODEL + '.urdf'),
    )

    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='false',
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=urdf_file,
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(urdf_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher_node)

    return ld
