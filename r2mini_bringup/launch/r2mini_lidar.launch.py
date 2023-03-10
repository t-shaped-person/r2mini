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
    lidar_param = LaunchConfiguration('lidar_param')

    lidar_param_arg = DeclareLaunchArgument(
        'lidar_param',
        default_value=os.path.join(share_directory, 'param', 'ydlidar_' + LIDAR_MODEL + '.yaml'),
        description='Argument for lidar_param'
    )

    lidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[lidar_param],
        # emulate_tty=True,
        # namespace='/',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(lidar_param_arg)
    ld.add_action(lidar_node)

    return ld