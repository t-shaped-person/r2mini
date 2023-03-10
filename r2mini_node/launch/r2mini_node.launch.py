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
    share_directory = get_package_share_directory('r2mini_node')
    mcu_param = LaunchConfiguration('mcu_param')    

    mcu_param_arg = DeclareLaunchArgument(
        'mcu_param',
        default_value=os.path.join(share_directory, 'param', ROBOT_MODEL + '_mcu.yaml'),
        description='Argument for mcu_param'
    )

    mcu_node = Node(
        package='r2mini_node',
        executable='r2mini_mcu',
        name='r2mini_mcu',
        parameters=[mcu_param],
        # emulate_tty=True,
        # namespace='/',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(mcu_param_arg)
    ld.add_action(mcu_node)

    return ld
