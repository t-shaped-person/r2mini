#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    LIDAR_MODEL = os.environ['LIDAR_MODEL']
    share_directory = get_package_share_directory('r2mini_bringup')

    lidar_config = LaunchConfiguration(
        'lidar_config',
        default=os.path.join(share_directory, 'config', LIDAR_MODEL + '.yaml'),
    )

    lidar_config_arg = DeclareLaunchArgument(
        'lidar_config',
        default_value=lidar_config,
    )

    lidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[lidar_config],
        # emulate_tty=True,
        # namespace='/',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(lidar_config_arg)
    ld.add_action(lidar_node)

    return ld