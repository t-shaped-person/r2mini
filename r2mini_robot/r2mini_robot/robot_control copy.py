import os
import rclpy
from .robot_driver import *
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, Pose, TransformStamped

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port.name', None),
                ('port.baudrate', None),
                ('wheel.separation', None),
                ('wheel.radius', None),
                ('motor.gear_ratio', None),
                ('motor.max_lin_vel', None),
                ('motor.max_ang_vel', None),
                ('sensor.old_enc_pulse', None),
                ('sensor.new_enc_pulse', None),
            ]
        )
        
        port_name = self.get_parameter_or('port.name', Parameter('port.name', Parameter.Type.STRING, '/dev/ttyMCU')).get_parameter_value().string_value
        port_baudrate = self.get_parameter_or('port.baudrate', Parameter('port.baudrate', Parameter.Type.INTEGER, 115200)).get_parameter_value().integer_value
        self.wheel_separation = self.get_parameter_or('wheel.separation', Parameter('wheel.separation', Parameter.Type.DOUBLE, 0.17)).get_parameter_value().double_value
        self.wheel_radius = self.get_parameter_or('wheel.radius', Parameter('wheel.radius', Parameter.Type.DOUBLE, 0.0335)).get_parameter_value().double_value
        self.motor_gear_ratio = self.get_parameter_or('motor.gear_ratio', Parameter('motor.gear_ratio', Parameter.Type.DOUBLE, 21.3)).get_parameter_value().double_value
        self.motor_max_lin_vel = self.get_parameter_or('motor.max_lin_vel', Parameter('motor.max_lin_vel', Parameter.Type.DOUBLE, 2.0)).get_parameter_value().double_value
        self.motor_max_ang_vel = self.get_parameter_or('motor.max_ang_vel', Parameter('motor.max_ang_vel', Parameter.Type.DOUBLE, 2.5)).get_parameter_value().double_value
        if os.environ['MOTOR_MODEL'] == 'old':
            self.enc_pulse = self.get_parameter_or('sensor.old_enc_pulse', Parameter('sensor.old_enc_pulse', Parameter.Type.DOUBLE, 44.0)).get_parameter_value().double_value
        elif os.environ['MOTOR_MODEL'] == 'new':
            self.enc_pulse = self.get_parameter_or('sensor.new_enc_pulse', Parameter('sensor.new_enc_pulse', Parameter.Type.DOUBLE, 1440.0)).get_parameter_value().double_value
        self.distance_per_pulse = 2 * math.pi * self.wheel_radius / self.enc_pulse / self.motor_gear_ratio
        print(f'port.name:\t\t{port_name}')
        print(f'port.baudrate:\t{port_baudrate}')
        print(f'wheel.separation:\t{self.wheel_separation}')
        print(f'wheel.radius:\t\t{self.wheel_radius}')
        print(f'motor.gear_ratio:\t{self.motor_gear_ratio}')
        print(f'motor.max_lin_vel:\t{self.motor_max_lin_vel}')
        print(f'motor.max_ang_vel:\t{self.motor_max_ang_vel}')
        print(f'sensor.enc_pulse:\t{self.enc_pulse}')
        print(f'distance per pulse:\t{self.distance_per_pulse}')
        self.print('display info')

        self.ph = PacketHandler(port_name, port_baudrate)
        self.ph.set_periodic_info(50)
        self.print('packet handler start')
        self.ph.update_battery_state()
        self.print('update battery')

        # self.calc_yaw = ComplementaryFilter()# modify
        self.odom_pose = Odometry()
        self.print('odometry call')
        self.odom_pose.header.stamp = self.get_clock().now().to_msg()
        self.odom_pose_header_pre_stamp = self.odom_pose.header.stamp
        self.print('stamp set')

        qos_profile = QoSProfile(depth=10)
        self.print('qos set')
        self.sub_twist = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile)
        self.print('create sub')
        self.pub_jointState = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.print('create pub joint')
        self.pub_odometry = self.create_publisher(Odometry, 'odom', qos_profile)
        self.print('create pub odometry')
        self.pub_pose = self.create_publisher(Pose, 'pose', qos_profile)
        self.print('create pub pose')
        self.pub_tf = TransformBroadcaster(self)
        self.print('create pub tf')
        self.timer_ = self.create_timer(0.02, self.update_robot)
        self.print('create timer')
    
    def print(self, str_info):
        self.get_logger().info(str_info)

    def cmd_vel_callback(self, msg):
        self.print('sub callback start')
        lin_vel_x = msg.linear.x
        ang_vel_z = msg.angular.z
        lin_vel_x = max(-self.motor_max_lin_vel, min(self.motor_max_lin_vel, lin_vel_x))
        ang_vel_z = max(-self.motor_max_ang_vel, min(self.motor_max_ang_vel, ang_vel_z))
        self.ph.vw_command(lin_vel_x * 1000, ang_vel_z * 1000)                                          # form m/s & rad/s to mm/s & mrad/s
    
    def update_robot(self):
        self.print('update_robot func start')
        self.ph.read_packet()
        self.print('read_packet func start')
        odo_lh = self.ph.odo[0]
        odo_rh = self.ph.odo[1]
        lin_vel = self.ph.vw[0]
        ang_vel = self.ph.vw[1]
        # vel_z = self.ph.gyro[2]
        pose_roll = self.ph.pose[0]
        pose_pitch = self.ph.pose[1]
        pose_yaw = self.ph.pose[2]
        self.print('ph data saved')
        self.update_odometry_(odo_lh, odo_rh, lin_vel, ang_vel)
        self.print('pass23')
        self.update_jointstate(odo_lh, odo_rh, lin_vel, ang_vel)
        self.print('pass24')
        self.update_pose(pose_roll, pose_pitch, pose_yaw)

    def update_odometry_(self, odo_lh, odo_rh, lin_vel, ang_vel):
        odo_lh /= 1000.                                                                                 # form mm/s & mrad/s to m/s & rad/s
        odo_rh /= 1000.
        lin_vel /= 1000.
        ang_vel /= 1000.
        self.odom_pose.header.stamp = self.get_clock().now().to_msg()
        # dt = (self.odom_pose.header.stamp - self.odom_pose_header_pre_stamp).nanoseconds * 1e-9
        dt = (self.odom_pose.header.stamp.nanosec() - self.odom_pose_header_pre_stamp.nanosec())
        self.print(dt, type(dt))
        self.print('pass1')
        self.odom_pose_header_pre_stamp = self.odom_pose.header.stamp
        self.odom_pose.twist.twist.angular.z += (ang_vel * dt)
        self.odom_pose.pose.pose.position.x += (lin_vel * math.cos(self.odom_pose.twist.twist.angular.z) * dt)
        self.odom_pose.pose.pose.position.y += (lin_vel * math.sin(self.odom_pose.twist.twist.angular.z) * dt)
        self.print('pass2')
        q = quaternion_from_euler(0, 0, self.odom_pose.twist.twist.angular.z)
        self.print('pass3')
        odometry_ = Odometry()
        odometry_.header.frame_id = "odom"
        odometry_.child_frame_id = "base_footprint"
        odometry_.header.stamp = self.get_clock().now().to_msg()
        odometry_.pose.pose.position.x = self.odom_pose.pose.pose.position.x
        odometry_.pose.pose.position.y = self.odom_pose.pose.pose.position.y
        odometry_.pose.pose.position.z = 0.0
        odometry_.pose.pose.orientation = q
        odometry_.twist.twist.linear.x = lin_vel
        odometry_.twist.twist.linear.y = 0.0
        odometry_.twist.twist.angular.z = ang_vel
        self.pub_odometry.publish(odometry_)
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odometry_.header.frame_id
        odom_tf.child_frame_id = odometry_.child_frame_id
        odom_tf.header.stamp = odometry_.header.stamp
        odom_tf.transform.translation = odometry_.pose.pose.position
        odom_tf.transform.rotation = odometry_.pose.pose.orientation
        self.pub_tf.sendTransform(odom_tf)
        self.print('pass4')

    def update_jointstate(self, odo_lh, odo_rh, lin_vel, ang_vel):
        odo_lh /= 1000.                                                                                 # form mm/s & mrad/s to m/s & rad/s
        odo_rh /= 1000.
        wheel_lh_pos = odo_lh / self.wheel_radius                                                       # pos means angle
        wheel_rh_pos = odo_rh / self.wheel_radius
        wheel_lh_vel = (lin_vel - (self.wheel_separation / 2.0) * ang_vel) / self.wheel_radius          # vel means angular velocity
        wheel_rh_vel = (lin_vel + (self.wheel_separation / 2.0) * ang_vel) / self.wheel_radius
        jointstate = JointState()
        jointstate.header.frame_id = "base_link"
        jointstate.header.stamp = self.get_clock().now().to_msg()
        jointstate.name = ['wheel_left_joint', 'wheel_right_joint']
        jointstate.position = [wheel_lh_pos, wheel_rh_pos]
        jointstate.velocity = [wheel_lh_vel, wheel_rh_vel]
        jointstate.effort = []
        self.pub_jointState.publish(jointstate)

    def update_pose(self, pose_roll, pose_pitch, pose_yaw):
        pose = Pose()
        pose.orientation.x = pose_roll
        pose.orientation.y = pose_pitch
        pose.orientation.z = pose_yaw
        self.pub_pose.publish(pose)

def main(args=None):
    print('robot_control main func start')
    rclpy.init(args=args)
    print('rclpy init end')
    node = RobotControl()
    node.print('robotcontrol class init end')
    try:
        node.print('node start...............')
        rclpy.spin(node)
    except Exception as e:
        node.print('exception...............')
        print(e)
    finally:
        node.ph.close_port()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
