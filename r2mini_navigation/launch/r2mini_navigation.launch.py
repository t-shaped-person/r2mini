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
    
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    map = LaunchConfiguration('map', default=os.path.join(ThisLaunchFileDir(), 'map', ROBOT_MODEL+'_world.yaml'))
    map_arg = DeclareLaunchArgument('map', default_value=map)

    params_file = LaunchConfiguration('params_file', default=os.path.join(ThisLaunchFileDir(), 'config', ROBOT_MODEL+'.yaml'))
    params_file_arg = DeclareLaunchArgument('params_file', default_value=params_file)

    rviz_file = LaunchConfiguration('rviz_file', default=os.path.join(ThisLaunchFileDir(), 'rviz', 'navigation.rviz'))
    rviz_file_arg = DeclareLaunchArgument('rviz_file', default_value=rviz_file)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch, '/bringup_launch.py']),
        launch_arguments={'map': map, 'params_file': params_file, 'use_sim_time': use_sim_time}.item(),
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
    ld.add_action(map_arg)
    ld.add_action(params_file_arg)
    ld.add_action(rviz_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(navigation_node)
    ld.add_action(rviz_node)

    return ld
