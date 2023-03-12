#r2mini

##package create

###r2mini
'''
ros2 pkg create r2mini --description "Meta package for r2mini." --license "Apache License, Version 2.0" --build-type ament_python --dependencies r2mini_bringup r2mini_cartographer r2mini_description r2mini_navigation r2mini_robot r2mini_teleop --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r2mini_bringup --done
'''
ros2 pkg create r2mini_bringup --description "Launch scripts for starting the r2mini." --license "Apache License, Version 2.0" --build-type ament_python --dependencies r2mini_description r2mini_node robot_state_publisher rviz2 ydlidar_ros2_driver --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r2mini_cartographer --done
'''
ros2 pkg create r2mini_cartographer --description "Launch scripts for cartographer." --license "Apache License, Version 2.0" --build-type ament_python --dependencies cartographer_ros rviz2 --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r2mini_description --done
'''
ros2 pkg create r2mini_description --description "3D models of simulation and visualization for r2mini." --license "Apache License, Version 2.0" --build-type ament_python --dependencies robot_state_publisher rviz2 urdf --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r2mini_navigation
'''
ros2 pkg create r2mini_navigation --description "Launch scripts for navigation2." --license "Apache License, Version 2.0" --build-type ament_python --dependencies nav2_bringup rviz2 --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K"
'''

###r2mini_robot
'''
ros2 pkg create r2mini_robot --description "Driver for r2mini." --license "Apache License, Version 2.0" --build-type ament_python --dependencies geometry_msgs message_filters rosidl_default_runtime tf2_ros_py --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K" --node-name robot_control
'''
--dependencies geometry_msgs message_filters nav_msgs rclpy sensor_msgs std_msgs std_srvs tf2_ros_py

###r2mini_teleop (launch file error fix)
'''
ros2 pkg create r2mini_teleop --description "Teleoperation node using keyboard for r2mini." --license "Apache License, Version 2.0" --build-type ament_python --dependencies rclpy geometry_msgs --maintainer-email "kucira00@gmail.com" --maintainer-name "Dr.K" --node-name teleop_keyboard
'''


##ros topic, service, action list
###normal
node        None
topic       /parameter_events
            /rosout
service     None
action      None
param       None

###r2mini_teleop
node        /teleop_keyboard
topic       /cmd_vel
            <!-- /parameter_events
            /rosout -->
service     /teleop_keyboard/describe_parameters
            /teleop_keyboard/get_parameter_types
            /teleop_keyboard/get_parameters
            /teleop_keyboard/list_parameters
            /teleop_keyboard/set_parameters
            /teleop_keyboard/set_parameters_atomically
action      None
parma       None

###r2mini_description


##rviz
###rviz default
1. Global Options
2. Global Status
3. Grid
###rviz slam
1. Global Options
2. Global Status
3. Grid
4. TF
5. Laser scan
6. Map
7. odometry
8. navigation
9. catographer
###rviz navigation
1. Global Options
2. Global Status
3. Grid
4. TF
5. Laser scan
6. Map
7. Robotmodel
8. Bumper Hit
9. Amcl particle swarm
10. Global Planner
11. Controller
12. Realsense
13. Marker Array