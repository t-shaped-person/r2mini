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

    bringup_dir = get_package_share_directory('r2mini_bringup')

    lidar_yaml = LaunchConfiguration('lidar_yaml', default=os.path.join(bringup_dir, 'config', LIDAR_MODEL+'.yaml'))
    lidar_yaml_arg = DeclareLaunchArgument('lidar_yaml', default_value=lidar_yaml)

    lidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[lidar_yaml],
        # emulate_tty=True,
        # namespace='/',
    )

    ld = LaunchDescription()
    ld.add_action(lidar_yaml_arg)
    ld.add_action(lidar_node)

    return ld
