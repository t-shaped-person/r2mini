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
    
    teleop_keyboard = ExecuteProcess(
        cmd=['ros2', 'run', 'r2mini_teleop', 'teleop_keyboard'],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(teleop_keyboard)

    return ld
