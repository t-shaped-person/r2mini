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
    
    robot_dir = get_package_share_directory('r2mini_robot')
    
    robot_yaml = LaunchConfiguration('robot_yaml', default=os.path.join(robot_dir, 'config', ROBOT_MODEL+'.yaml'))
    robot_yaml_arg = DeclareLaunchArgument('robot_yaml', default_value=robot_yaml)
    
    robot_control_node = Node(
        package='r2mini_robot',
        executable='robot_control',
        name='robot_control',
        output='screen',
        parameters=[robot_yaml],
        # emulate_tty=True,
        # namespace='',
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_yaml_arg)
    ld.add_action(robot_control_node)
    
    return ld
