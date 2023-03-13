import os
import rclpy
from .robot_driver import *
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, Pose, TransformStamped

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        print('where am i_3')
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
        print('where am i_4')
        port_name = self.get_parameter_or('port.name', Parameter('port.name', Parameter.Type.STRING, '/dev/ttyMCU1')).get_parameter_value().string_value
        port_baudrate = self.get_parameter_or('port.baudrate', Parameter('port.baudrate', Parameter.Type.INTEGER, 1152001)).get_parameter_value().integer_value
        print('where am i_5')
        self.wheel_separation = self.get_parameter_or('wheel.separation', Parameter('wheel.separation', Parameter.Type.DOUBLE, 0.171)).get_parameter_value().double_value
        self.wheel_radius = self.get_parameter_or('wheel.radius', Parameter('wheel.radius', Parameter.Type.DOUBLE, 0.03351)).get_parameter_value().double_value
        print('where am i_6')
        self.gear_ratio = self.get_parameter_or('motor.gear_ratio', Parameter('motor.gear_ratio', Parameter.Type.DOUBLE, 21.31)).get_parameter_value().double_value
        self.max_lin_vel = self.get_parameter_or('motor.max_lin_vel', Parameter('motor.max_lin_vel', Parameter.Type.DOUBLE, 2.01)).get_parameter_value().double_value
        self.max_ang_vel = self.get_parameter_or('motor.max_ang_vel', Parameter('motor.max_ang_vel', Parameter.Type.DOUBLE, 2.51)).get_parameter_value().double_value
        print('where am i_7')
        if os.environ['MOTOR_MODEL'] == 'old':
            self.enc_pulse = self.get_parameter_or('sensor.old_enc_pulse', Parameter('sensor.old_enc_pulse', Parameter.Type.DOUBLE, 44.01)).get_parameter_value().double_value
        elif os.environ['MOTOR_MODEL'] == 'new':
            self.enc_pulse = self.get_parameter_or('sensor.new_enc_pulse', Parameter('sensor.new_enc_pulse', Parameter.Type.DOUBLE, 1440.01)).get_parameter_value().double_value
        print('where am i_8')
        
        print(f'port.name:\t\t{port_name}')
        print(f'port.baudrate:\t{port_baudrate}')
        print(f'wheel.separation:\t{self.wheel_separation}')
        print(f'wheel.radius:\t\t{self.wheel_radius}')
        print(f'motor.gear_ratio:\t{self.gear_ratio}')
        print(f'motor.max_lin_vel:\t{self.max_lin_vel}')
        print(f'motor.max_ang_vel:\t{self.max_ang_vel}')
        print(f'sensor.enc_pulse:\t{self.enc_pulse}')

        self.distance_per_pulse = 2 * math.pi * self.wheel_radius / self.enc_pulse / self.gear_ratio
        print(f'distance per pulse:\t{self.distance_per_pulse}')
        
        # Packet handler
        self.ph = PacketHandler(port_name, port_baudrate)
        self.calc_yaw = ComplementaryFilter()
        self.ph.robot_state = {
            "VW" : [0., 0.],
            "ODO" : [0., 0.],
            "ACCL" : [0., 0., 0.],
            "GYRO" : [0., 0., 0.],
            "POSE" : [0., 0., 0.],
            "BAT" : [0., 0., 0.],
        }
        
        self.odom_pose = OdomPose()
        self.odom_pose.timestamp = self.get_clock().now()
        self.odom_pose.pre_timestamp = self.get_clock().now()
        self.odom_vel = OdomVel()
        self.joint = Joint()
        # Set subscriber
        self.subCmdVelMsg = self.create_subscription(Twist, 'cmd_vel', self.cbCmdVelMsg, 10)
        # Set publisher
        self.pub_JointStates = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_IMU = self.create_publisher(Imu, 'imu', 10)
        self.pub_Odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_OdomTF = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(Pose, 'pose', 10)
        # Set Periodic data
        self.ph.set_periodic_info(50)
        self.ph.update_battery_state()
        # Set timer proc
        self.timerProc = self.create_timer(0.01, self.update_robot)

    def update_odometry(self, odo_l, odo_r, trans_vel, orient_vel, vel_z):
        odo_l /= 1000.
        odo_r /= 1000.
        trans_vel /= 1000.
        orient_vel /= 1000.
        self.odom_pose.timestamp = self.get_clock().now()
        dt = (self.odom_pose.timestamp - self.odom_pose.pre_timestamp).nanoseconds * 1e-9
        self.odom_pose.pre_timestamp = self.odom_pose.timestamp
        if self.use_gyro:
            self.calc_yaw.wheel_ang += orient_vel * dt
            self.odom_pose.theta = self.calc_yaw.calc_filter(vel_z*math.pi/180., dt)
            self.get_logger().info('R1mini state : whl pos %1.2f, %1.2f, gyro : %1.2f, whl odom : %1.2f, robot theta : %1.2f' 
                        %(odo_l, odo_r, vel_z,
                        self.calc_yaw.wheel_ang*180/math.pi, 
                        self.d_odom_pose['theta']*180/math.pi ))
        else:
            self.odom_pose.theta += orient_vel * dt
        d_x = trans_vel * math.cos(self.odom_pose.theta) 
        d_y = trans_vel * math.sin(self.odom_pose.theta) 
        self.odom_pose.x += d_x * dt
        self.odom_pose.y += d_y * dt
        q = quaternion_from_euler(0, 0, self.odom_pose.theta)
        self.odom_vel.x = trans_vel
        self.odom_vel.y = 0.
        self.odom_vel.w = orient_vel
        timestamp_now = self.get_clock().now().to_msg()
        # Set odometry data
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.header.stamp = timestamp_now
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = trans_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = orient_vel
        self.pub_Odom.publish(odom)
        # Set odomTF data
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odom.header.frame_id
        odom_tf.child_frame_id = odom.child_frame_id
        odom_tf.header.stamp = timestamp_now
        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = odom.pose.pose.position.z
        odom_tf.transform.rotation = odom.pose.pose.orientation
        self.pub_OdomTF.sendTransform(odom_tf)

    def updatePoseStates(self, roll, pitch, yaw):
        #Added to publish pose orientation of IMU
        pose = Pose()
        pose.orientation.x = roll
        pose.orientation.y = pitch
        pose.orientation.z = yaw
        self.pub_pose.publish(pose)

    def updateJointStates(self, odo_l, odo_r, trans_vel, orient_vel):
        odo_l /= 1000.
        odo_r /= 1000.
        wheel_ang_left = odo_l / self.wheel_radius
        wheel_ang_right = odo_r / self.wheel_radius
        wheel_ang_vel_left = (trans_vel - (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius
        wheel_ang_vel_right = (trans_vel + (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius
        self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
        self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]
        timestamp_now = self.get_clock().now().to_msg()
        joint_states = JointState()
        joint_states.header.frame_id = "base_link"
        joint_states.header.stamp = timestamp_now
        joint_states.name = self.joint.joint_name
        joint_states.position = self.joint.joint_pos
        joint_states.velocity = self.joint.joint_vel
        joint_states.effort = []
        self.pub_JointStates.publish(joint_states)

    def update_robot(self):
        self.ph.read_packet()
        odo_l = self.ph._wodom[0]
        odo_r = self.ph._wodom[1]
        trans_vel = self.ph._vel[0]
        orient_vel = self.ph._vel[1]
        vel_z = self.ph._gyro[2]
        roll_imu = self.ph._imu[0]
        pitch_imu = self.ph._imu[1]
        yaw_imu = self.ph._imu[2]
        self.update_odometry(odo_l, odo_r, trans_vel, orient_vel, vel_z)
        self.updateJointStates(odo_l, odo_r, trans_vel, orient_vel)
        self.updatePoseStates(roll_imu, pitch_imu, yaw_imu)

    def cbCmdVelMsg(self, cmd_vel_msg):
        lin_vel_x = cmd_vel_msg.linear.x
        ang_vel_z = cmd_vel_msg.angular.z
        lin_vel_x = max(-self.max_lin_vel, min(self.max_lin_vel, lin_vel_x))
        ang_vel_z = max(-self.max_ang_vel, min(self.max_ang_vel, ang_vel_z))
        self.ph.vw_command(lin_vel_x*1000, ang_vel_z*1000)

def main(args=None):
    print('where am i_1')
    rclpy.init(args=args)
    print('where am i_2')
    node = RobotControl()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        node.ph.close_port()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
