#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    publish_period_sec_arg = DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec)

    resolution = LaunchConfiguration('resolution', default='0.05')
    resolution_arg = DeclareLaunchArgument('resolution', default_value=resolution)
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(publish_period_sec_arg)
    ld.add_action(resolution_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(occupancy_grid_node)

    return ld
