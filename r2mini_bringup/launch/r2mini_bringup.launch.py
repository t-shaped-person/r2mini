#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ROBOT_MODEL = os.environ['ROBOT_MODEL']
    share_directory = get_package_share_directory('r2mini_bringup')
    mcu_package = LaunchConfiguration('mcu_package')
    mcu_param = LaunchConfiguration('mcu_param')
    robot_state_publisher = LaunchConfiguration('robot_state_publisher')
    

    mcu_package_arg = DeclareLaunchArgument(
        'mcu_package',
        default_value=os.path.join(get_package_share_directory('r2mini_node'), 'launch'),
        description='Argument for mcu_package'
    )

    mcu_param_arg = DeclareLaunchArgument(
        'mcu_param',
        default_value=os.path.join(get_package_share_directory('r2mini_node'), 'param', ROBOT_MODEL + '_mcu.yaml'),
        description='Argument for mcu_param'
    )

    robot_state_publisher_arg = DeclareLaunchArgument(
        'robot_state_publisher',
        default_value=os.path.join(get_package_share_directory('r2mini_description'), 'launch'),
        description='Argument for robot_state_publisher'
    )

    mcu_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([mcu_package, '/r2mini_node.launch.py']),
        launch_arguments={'mcu_param': mcu_param}.items()
    )

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([share_directory, '/launch/r2mini_lidar.launch.py']),
    )

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_state_publisher, '/r2mini_description.launch.py']),
    )

    ld = LaunchDescription()
    ld.add_action(mcu_package_arg)
    ld.add_action(mcu_param_arg)
    ld.add_action(robot_state_publisher_arg)
    ld.add_action(mcu_node)
    ld.add_action(lidar_node)
    ld.add_action(robot_state_publisher_node)

    return ld
