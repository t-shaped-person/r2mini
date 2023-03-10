#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ROBOT_MODEL = os.environ['ROBOT_MODEL']
    share_directory = get_package_share_directory('r2mini_cartographer')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    resolution = LaunchConfiguration('resolution')
    use_sim_time = LaunchConfiguration('use_sim_time')

    configuration_directory_arg = DeclareLaunchArgument(
        'configuration_directory',
        default_value=os.path.join(share_directory, 'config'),
        description='Argument for configuration_directory'
    )

    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value=ROBOT_MODEL + '.lua',
        description='Argument for configuration_basename'
    )

    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Argument for publish_period_sec'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Argument for resolution'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Argument for use_sim_time'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
        output='screen'
    )

    occupancy_grid_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                          'publish_period_sec': publish_period_sec}.items(),
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/r2mini_cartographer_rviz.launch.py']),
    )

    ld = LaunchDescription()
    ld.add_action(configuration_directory_arg)
    ld.add_action(configuration_basename_arg)
    ld.add_action(publish_period_sec_arg)
    ld.add_action(resolution_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld
