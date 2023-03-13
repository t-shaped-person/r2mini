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
    LIDAR_MODEL = os.environ['LIDAR_MODEL']
    ROBOT_MODEL = os.environ['ROBOT_MODEL']

    bringup_dir = get_package_share_directory('r2mini_bringup')
    description_dir = get_package_share_directory('r2mini_description')
    robot_dir = get_package_share_directory('r2mini_robot')
    description_launch = os.path.join(description_dir, 'launch', 'robot_state_publisher.launch.py')
    robot_launch = os.path.join(robot_dir, 'launch', 'r2mini_robot.launch.py')

    lidar_yaml = LaunchConfiguration('lidar_yaml', default=os.path.join(bringup_dir, 'config', LIDAR_MODEL+'.yaml'))
    lidar_yaml_arg = DeclareLaunchArgument('lidar_yaml', default_value=lidar_yaml)

    robot_yaml = LaunchConfiguration('robot_yaml', default=os.path.join(robot_dir, 'config', ROBOT_MODEL+'.yaml'))
    robot_yaml_arg = DeclareLaunchArgument('robot_yaml', default_value=robot_yaml)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    with open(os.path.join(description_dir, 'urdf', ROBOT_MODEL+'.urdf'), 'r') as infp:
        robot_description = infp.read()

    robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_launch]),
        launch_arguments={'robot_yaml': robot_yaml}.items(),
    )

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir, '/launch/r2mini_lidar.launch.py']),
        launch_arguments={'lidar_yaml': lidar_yaml}.items(),
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch]),
        launch_arguments={'robot_description': robot_description, 'use_sim_time': use_sim_time}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(lidar_yaml_arg)
    ld.add_action(robot_yaml_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_node)
    ld.add_action(lidar_node)
    ld.add_action(robot_state_publisher_node)

    return ld
