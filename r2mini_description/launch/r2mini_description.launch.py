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

    rviz_file = LaunchConfiguration('rviz_file', default=os.path.join(ThisLaunchFileDir(), 'rviz', 'description.rviz'))
    rviz_file_arg = DeclareLaunchArgument('rviz_file', default_value=rviz_file)
    
    urdf_file = LaunchConfiguration('urdf_file', default=os.path.join(ThisLaunchFileDir(), 'urdf', ROBOT_MODEL + '.urdf'))
    urdf_file_arg = DeclareLaunchArgument('urdf_file', default_value=urdf_file)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    with open(os.path.join(ThisLaunchFileDir(), 'urdf', ROBOT_MODEL+'.urdf'), 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/robot_state_publisher.launch.py']),
        launch_arguments={'robot_description': robot_description, 'use_sim_time': use_sim_time}.items(),
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
    ld.add_action(rviz_file_arg)
    ld.add_action(urdf_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
