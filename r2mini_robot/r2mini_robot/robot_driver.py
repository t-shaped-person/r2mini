#!/usr/bin/env python3
# import io
import math
import time
import serial

class PacketHandler:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate)
        print("serial port open")
        # self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, 1), newline = '\r', line_buffering = True)
        self.ser.flush()
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.item_list = ['GYRO', 'ODO', 'POSE', 'VW']
        self.bat = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.odo = [0.0, 0.0]
        self.pose = [0.0, 0.0, 0.0]
        self.vw = [0.0, 0.0]

    def write_data(self, tx_string):
        if self.ser.isOpen() == True:
            self.ser.write((tx_string + "\r\n").encode())

    def close_port(self):
        self.write_data("$cPEEN,0")
        self.ser.close()
        print("serial port close")

    def update_battery_state(self):
        self.write_data("$qBAT")

    def odometry_reset(self):
        self.write_data("$cODO,0")

    def vw_command(self, lin_vel, ang_vel):
        self.write_data('$cVW,{:.0f},{:.0f}'.format(lin_vel, ang_vel)) # mm/s, mrad/s

    def set_periodic_info(self, millisecond):
        for idx, item in enumerate(self.item_list):
            self.write_data("$cREGI," + str(idx) + "," + item)
            time.sleep(0.01)
        self.write_data("$cPERI," + str(millisecond))
        time.sleep(0.01)
        self.write_data("$cPEEN,1")
        time.sleep(0.01)

    def read_packet(self):
        if self.ser.isOpen() == True:
                whole_packet = (self.ser.readline().split(b'\r')[0]).decode("utf-8").strip()
                if whole_packet:
                    packet = whole_packet.split(',')
                    try:
                        header = packet[0].split('#')[1]
                        if header.startswith('BAT'):
                            self.bat = [float(packet[1]), float(packet[2]), float(packet[3])]
                        elif header.startswith('GYRO'):
                            self.gyro = [float(packet[1]), float(packet[2]), float(packet[3])]
                        elif header.startswith('ODO'):
                            self.odo = [float(packet[1]), float(packet[2])]
                        elif header.startswith('POSE'):
                            self.pose = [float(packet[1]), float(packet[2]), float(packet[3])]
                        elif header.startswith('VW'):
                            self.vw = [float(packet[1]), float(packet[2])]
                    except:
                        pass

class ComplementaryFilter():
    def __init__(self):
        self.theta = 0.
        self.pre_theta = 0.
        self.wheel_ang = 0.
        self.filter_coef = 2.5
        self.gyro_bias = 0.
        self.count_for_gyro_bias = 110

    def gyro_calibration(self, gyro):
        self.count_for_gyro_bias -= 1
        if self.count_for_gyro_bias > 100:
            return "Prepare for gyro_calibration"
        self.gyro_bias += gyro
        if self.count_for_gyro_bias == 1:
            self.gyro_bias /= 100
            self.get_logger().info('Complete : Gyro calibration')
            return "gyro_calibration OK"
        return "During gyro_calibration"

    def calc_filter(self, gyro, d_time):
        if self.count_for_gyro_bias != 1:
            tmp = self.gyro_calibration(gyro)
            return 0
        gyro -= self.gyro_bias
        self.pre_theta = self.theta
        temp = -1/self.filter_coef * (-self.wheel_ang + self.pre_theta) + gyro
        self.theta = self.pre_theta + temp*d_time
        return self.theta

class OdomPose():
    x = 0.0
    y = 0.0
    theta = 0.0
    timestamp = 0
    pre_timestamp = 0

class OdomVel():
    x = 0.0
    y = 0.0
    w = 0.0

class Joint():
    joint_name = ['wheel_left_joint', 'wheel_right_joint']
    joint_pos = [0.0, 0.0]
    joint_vel = [0.0, 0.0]

"""
Converts euler roll, pitch, yaw to quaternion (w in last place)
quat = [x, y, z, w]
Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
"""
def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr
    return q