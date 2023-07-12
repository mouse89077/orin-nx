#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""Subscribe IMU Magnetometer data and calculate boat's heading direction

Notes:
    Heading: -180 (to West) ~ 180 (to East) (deg), Magnetic North is 0 deg
"""

import math
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticFieldm Imu
from std_msgs.msg import Float64


class HeadingCalculator(Node):
    def __init__(self):
        super().__init__('heading_calculator')

        namespace_OS = '/OS'
        self.mag_sub = self.create_subscription(Imu, namespace_OS + '/imu', self.imu_callback, 1)
        self.heading_angle_pub = self.create_publisher(Float64, namespace_OS + '/heading', 1)

        self.heading_timer = self.create_timer(0.1, self.pub_heading_angle)

        self.heading = np.zeros(10)

        self.msg_received = False

    def imu_callback(self, msg):
        self.msg_received = True

        self.orientation = msg.orientation
        self.angular_vel = msg.angular_vel
        self.linear_acc = msg.linear_accelaration

        self.orientation_cov = msg.orientation_covariance
        self.angular_vel_cov= msg.angular_velocity_covariance
        self.linear_acc_cov = msg.linear_accelaration_covariance

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topics_status)

    def check_topics_status(self):
        if not self.msg_received:
            self.get_logger().info('No topic mag_received')
        else:
            self.get_logger().info('All topics received')

    def cal_heading(self):
        x = self.orientation.x
        y = self.orientation.y
        z = self.orientation.z
        w = self.orientation.w

        R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w], \
        [2*x*y + 2*z*w, 1- 2*x**2 - 2*z**2, 2*y*z - 2*x*w],  \
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 -2*x**2 - 2*y**2]])

        heading = np.atan2(R[1, 0], R[0, 0])
        heading_d = np.rad2deg(heading)
        self.heading = np.append(self.heading, heading_d)
        self.heading = self.heading[1:]

    def pub_heading_angle(self):
        if self.msg_received == True:
            self.cal_heading()
            heading_angle = Float64()
            heading_angle.data = self.heading[-1]
            self.heading_angle_pub.publish(heading_angle)
        else:
            return

def main(args=None):
    rclpy.init(args=args)
    heading_calculator = HeadingCalculator()
    heading_calculator.wait_for_topics()
    rclpy.spin(heading_calculator)
    heading_calculator.destroy_node()
    rclpy.shutodown()

if __name__ == "__main__":
    main()
