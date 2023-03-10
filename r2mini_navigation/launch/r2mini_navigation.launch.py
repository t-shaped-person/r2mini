#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ROBOT_MODEL = os.environ['ROBOT_MODEL']
    share_directory = get_package_share_directory('r2mini_navigation')
    map = LaunchConfiguration('map')
    navigation_package = LaunchConfiguration('navigation_package')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(share_directory, 'map', ROBOT_MODEL + '_world.yaml'),
        description='Argument for map_dir'
    )

    navigation_package_arg = DeclareLaunchArgument(
        'navigation_package',
        default_value=os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
        description='Argument for navigation_package'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_directory, 'param', ROBOT_MODEL + '.yaml'),
        description='Argument for params_file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Argument for use_sim_time'
    )

    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_package, '/bringup_launch.py']),
        launch_arguments={'map': map,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file}.item(),
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/r2mini_navigation_rviz.launch.py']),
    )

    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(navigation_package_arg)
    ld.add_action(params_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(navigation_node)
    ld.add_action(rviz_node)

    return ld
