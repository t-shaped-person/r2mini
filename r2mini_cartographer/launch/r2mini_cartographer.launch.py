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
    ROBOT_MODEL = os.environ['ROBOT_MODEL']

    cartographer_dir = get_package_share_directory('r2mini_cartographer')

    configuration_basename = LaunchConfiguration('configuration_basename', default=ROBOT_MODEL+'.lua')
    configuration_basename_arg = DeclareLaunchArgument('configuration_basename', default_value=configuration_basename)

    configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(cartographer_dir, 'config'))
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value=configuration_directory)

    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    publish_period_sec_arg = DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec)

    resolution = LaunchConfiguration('resolution', default='0.05')
    resolution_arg = DeclareLaunchArgument('resolution', default_value=resolution)

    rviz_file = LaunchConfiguration('rviz_file', default=os.path.join(cartographer_dir, 'rviz', 'cartographer.rviz'))
    rviz_file_arg = DeclareLaunchArgument('rviz_file', default_value=rviz_file)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)    

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=['-configuration_directory', configuration_directory, '-configuration_basename', configuration_basename],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    occupancy_grid_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        launch_arguments={'publish_period_sec': publish_period_sec, 'resolution': resolution, 'use_sim_time': use_sim_time}.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(configuration_basename_arg)
    ld.add_action(configuration_directory_arg)
    ld.add_action(publish_period_sec_arg)
    ld.add_action(resolution_arg)
    ld.add_action(rviz_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld
