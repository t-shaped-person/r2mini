#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_directory = get_package_share_directory('r2mini_navigation')
    rviz_file = LaunchConfiguration('rviz_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_file_arg = DeclareLaunchArgument(
        'rviz_file',
        default_value=os.path.join(share_directory, 'rviz', 'navigation.rviz'),
        description='Argument for rviz_file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Argument for use_sim_time'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(rviz_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_node)

    return ld