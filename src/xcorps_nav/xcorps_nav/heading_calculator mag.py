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
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64


class HeadingCalculator(Node):
    def __init__(self):
        super().__init__('heading_calculator')

        namespace_OS = '/OS'
        self.mag_sub = self.create_subscription(MagneticField, namespace_OS + '/mag', self.mag_callback, 1)
        self.heading_angle_pub = self.create_publisher(Float64, namespace_OS + '/heading', 1)

        self.heading_timer = self.create_timer(0.1, self.pub_heading_angle)

        self.magnetic_x = 0.0
        self.magnetic_y = 0.0
        self.magnetic_z = 0.0
        self.last_heading_angle = 0.0

        self.msg_received = False

    def mag_callback(self, msg):
        self.msg_received = True
        self.magnetic_x = msg.magnetic_field.x
        self.magnetic_y = msg.magnetic_field.y
        self.magnetic_z = msg.magnetic_field.z

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topics_status)

    def check_topics_status(self):
        if not self.msg_received:
            self.get_logger().info('No topic mag_received')
        else:
            self.get_logger().info('All topics received')

    def getRotationMatrix(self, R, I, gravity, geomagnetic):
        Ax = gravity[0]
        Ay = gravity[1]
        Az = gravity[2]

        normsqA = Ax * Ax + Ay * Ay + Az * Az
        g = 9.81
        freeFallGravitySquared = 0.01 * g * g

        if normsqA < freeFallGravitySquared:
            # gravity less than 10% of normal value
            return False

        Ex = geomagnetic[0]
        Ey = geomagnetic[1]
        Ez = geomagnetic[2]
        Hx = Ey * Az - Ez * Ay
        Hy = Ez * Ax - Ex * Az
        Hz = Ex * Ay - Ey * Ax
        normH = math.sqrt(Hx * Hx + Hy * Hy + Hz * Hz)

        if normH < 0.1:
            # device is close to free fall (or in space?), or close to
            # magnetic north pole. Typical values are  > 100.
            return False

        invH = 1.0 / normH
        Hx *= invH
        Hy *= invH
        Hz *= invH

        invA = 1.0 / math.sqrt(Ax * Ax + Ay * Ay + Az * Az)
        Ax *= invA
        Ay *= invA
        Az *= invA

        Mx = Ay * Hz - Az * Hy
        My = Az * Hx - Ax * Hz
        Mz = Ax * Hy - Ay * Hx
        if R is not None:
            if len(R) == 9:
                R[0] = Hx
                R[1] = Hy
                R[2] = Hz
                R[3] = Mx
                R[4] = My
                R[5] = Mz
                R[6] = Ax
                R[7] = Ay
                R[8] = Az
            elif len(R) == 16:
                R[0] = Hx
                R[1] = Hy
                R[2] = Hz
                R[3] = 0
                R[4] = Mx
                R[5] = My
                R[6] = Mz
                R[7] = 0
                R[8] = Ax
                R[9] = Ay
                R[10] = Az
                R[11] = 0
                R[12] = 0
                R[13] = 0
                R[14] = 0
                R[15] = 1

        if I is not None:
            # compute the inclination matrix by projecting the geomagnetic
            # vector onto the Z (gravity) and X (horizontal component
            # of geomagnetic vector) axes.
            invE = 1.0 / math.sqrt(Ex * Ex + Ey * Ey + Ez * Ez)
            c = (Ex * Mx + Ey * My + Ez * Mz) * invE
            s = (Ex * Ax + Ey * Ay + Ez * Az) * invEa

            if len(I) == 9:
                I[0] = 1
                I[1] = 0
                I[2] = 0
                I[3] = 0
                I[4] = c
                I[5] = s
                I[6] = 0
                I[7] = -s
                I[8] = c
            elif len(I) == 16:
                I[0] = 1
                I[1] = 0
                I[2] = 0
                I[4] = 0
                I[5] = c
                I[6] = s
                I[8] = 0
                I[9] = -s
                I[10] = c
                I[3] = I[7] = I[11] = I[12] = I[13] = I[14] = 0
                I[15] = 1

        return True

    def getOrientation(self, R, values):
        """
        4x4 (length=16) case:
            R[ 0]   R[ 1]   R[ 2]   0
            R[ 4]   R[ 5]   R[ 6]   0
            R[ 8]   R[ 9]   R[10]   0
            0       0       0    1

        3x3 (length=9) case:
            R[ 0]   R[ 1]   R[ 2]
            R[ 3]   R[ 4]   R[ 5]
            R[ 6]   R[ 7]   R[ 8]
        """
        if len(R) == 9:
            values[0] = math.atan2(R[1], R[4])
            values[1] = math.asin(-R[7])
            values[2] = math.atan2(-R[6], R[8])
        else:
            values[0] = math.atan2(R[1], R[5])
            values[1] = math.asin(-R[9])
            values[2] = math.atan2(-R[8], R[10])

        return values

    def calculate_heading_angle(self):
        gravity = [0, 0, -9.8]
        geomagnetic = [
            self.magnetic_x * 10**6,
            self.magnetic_y * 10**6,
            self.magnetic_z * 10**6,
        ]
        R = [0] * 9
        orientation = [0] * 3
        success = self.getRotationMatrix(R, None, gravity, geomagnetic)
        if success is True:
            self.getOrientation(R, orientation)
            heading_angle = orientation[0] * 180 / math.pi
            self.last_heading_angle = heading_angle
        else:
            heading_angle = self.last_heading_angle

        return heading_angle

    def pub_heading_angle(self):
        hdg = self.calculate_heading_angle()
        heading_angle = Float64()
        heading_angle.data = hdg
        self.heading_angle_pub.publish(heading_angle)

def main(args=None):
    rclpy.init(args=args)
    heading_calculator = HeadingCalculator()
    heading_calculator.wait_for_topics()
    rclpy.spin(heading_calculator)
    heading_calculator.destroy_node()
    rclpy.shutodown()

if __name__ == "__main__":
    main()
