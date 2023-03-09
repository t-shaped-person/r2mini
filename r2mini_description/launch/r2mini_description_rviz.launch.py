#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()

    robot_state_publisher = Node(
        executable='robot_state_publisher',
        package='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[],
        arguments=[]
    )
    ld.add_action(robot_state_publisher)