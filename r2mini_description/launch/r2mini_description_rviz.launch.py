#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_directory = get_package_share_directory('r2mini_description')
    rviz_dir = os.path.join(share_directory, 'rviz', 'model.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(rviz)

    return ld