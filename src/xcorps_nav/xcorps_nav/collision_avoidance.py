# Collision_Avoidance

import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.path as mpath
import math
from xcorps_nav.worstcase_vo import OwnShip, TargetShip, Cal_Shape, Cal_RV, Cal_RP, Cal_RAV, Cal_RAP, Cal_VO # to be added...

class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        default_params = {
            'dt' : 0.1,
            'Left_Bottom' : [37.4557583, 126.9517448],
            'Right_Bottom' : [37.4558121667, 126.9517401667],
            'Left_Top' : [37.4556311667, 126.9518098333],
            'Right_Top' : [37.4556311667, 126.9518098333],
            'origin' : [37.4557583, 126.9517448],
            'wp_set' : [37.4556311667, 126.9518098333],
            'tmin' : 20.0,
            'dmax' : 60.0,
            'TS_dim' : [0.432, 0.152],
            'OS_dim' : [0.914, 0.322],
            'ang_lim' : [-90., 90.], # deg
            'spd_lim' : [1.0, 1.2],
            'ref_spd' : [1.0],
            'namespace_OS' : '/OS',
            'namespace_TS1' : '/TS1',
            'namespace_TS2' : '/TS2',
            'namespace_TS3' : '/TS3',
            'namespace_TS4' : '/TS4',
        }
        # bring parameters
        self.dt = self.declare_parameter('dt', default_params['dt']).value
        self.Left_Bottom = self.declare_parameter('Left_Bottom', default_params['Left_Bottom']).value
        self.Right_Bottom = self.declare_parameter('Right_Bottom', default_params['Right_Bottom']).value
        self.Left_Top = self.declare_parameter('Left_Top', default_params['Left_Top']).value
        self.Right_Top = self.declare_parameter('Right_Top', default_params['Right_Top']).value
        self.origin = self.declare_parameter('origin', default_params['origin']).value
        self.wp_set = self.declare_parameter('wp_set', default_params['wp_set']).value
        self.tmin = self.declare_parameter('tmin', default_params['tmin']).value
        self.dmax = self.declare_parameter('dmax', default_params['dmax']).value
        self.TS_dim = self.declare_parameter('TS_dim', default_params['TS_dim']).value
        self.OS_dim = self.declare_parameter('OS_dim', default_params['OS_dim']).value
        self.ang_lim = self.declare_parameter('ang_lim', default_params['ang_lim']).value
        self.spd_lim = self.declare_parameter('spd_lim', default_params['spd_lim']).value
        self.ref_spd = self.declare_parameter('ref_spd', default_params['ref_spd']).value
        self.namespace_OS = self.declare_parameter('namespace_OS', default_params['namespace_OS']).value
        self.namespace_TS1 = self.declare_parameter('namespace_TS1', 	default_params['namespace_TS1']).value
        self.namespace_TS2 = self.declare_parameter('namespace_TS2', default_params['namespace_TS2']).value
        self.namespace_TS3 = self.declare_parameter('namespace_TS3', default_params['namespace_TS3']).value
        self.namespace_TS4 = self.declare_parameter('namespace_TS4', default_params['namespace_TS4']).value


        # initialize
        self.OS = OwnShip(self.OS_dim)
        self.TS1 = TargetShip(self.TS_dim)
        self.TS2 = TargetShip(self.TS_dim)
        self.TS3 = TargetShip(self.TS_dim)
        self.TS4 = TargetShip(self.TS_dim)

        # Subscriber
        # OS
        self.OS_enu_pos_sub = self.create_subscription(
            Point, self.namespace_OS + '/enu_pos', self.OS_enu_pos_callback, 1
        )
        self.OS_spd_sub = self.create_subscription(
            String, self.namespace_OS + '/gps/spd', self.OS_spd_callback, 1
        )
        self.OS_heading_sub = self.create_subscription(
            Float64, self.namespace_OS + '/heading', self.OS_heading_callback, 1
        )
        # TS1
        self.TS1_enu_pos_sub = self.create_subscription(
            Point, self.namespace_TS1 + '/enu_pos', self.TS1_enu_pos_callback, 1
        )
        self.TS1_spd_sub = self.create_subscription(
            String, self.namespace_TS1 + '/gps/spd', self.TS1_spd_callback, 1
        )
        self.TS1_heading_sub = self.create_subscription(
            Float64, self.namespace_TS1 + '/heading', self.TS1_heading_callback, 1
        )
        # TS2
        self.TS2_enu_pos_sub = self.create_subscription(
            Point, self.namespace_TS2 + '/enu_pos', self.TS2_enu_pos_callback, 1
        )
        self.TS2_spd_sub = self.create_subscription(
            String, self.namespace_TS2 + '/gps/spd', self.TS2_spd_callback, 1
        )
        self.TS2_heading_sub = self.create_subscription(
            Float64, self.namespace_TS2 + '/heading', self.TS2_heading_callback, 1
        )
        # TS3
        self.TS3_enu_pos_sub = self.create_subscription(
            Point, self.namespace_TS3 + '/enu_pos', self.TS3_enu_pos_callback, 1
        )
        self.TS3_spd_sub = self.create_subscription(
            String, self.namespace_TS3 + '/gps/spd', self.TS3_spd_callback, 1
        )
        self.TS3_heading_sub = self.create_subscription(
            Float64, self.namespace_TS3 + '/heading', self.TS3_heading_callback, 1
        )
        # TS4
        self.TS4_enu_pos_sub = self.create_subscription(
            Point, self.namespace_TS4 + '/enu_pos', self.TS4_enu_pos_callback, 1
        )
        self.TS4_spd_sub = self.create_subscription(
            String, self.namespace_TS4 + '/gps/spd', self.TS4_spd_callback, 1
        )
        self.TS4_heading_sub = self.create_subscription(
            Float64, self.namespace_TS4 + '/heading', self.TS4_heading_callback, 1
        )

        # Publisher
        self.des_heading_pub_ = self.create_publisher(Float64, self.namespace_OS + '/des/heading', 1)
        self.des_spd_pub_ = self.create_publisher(Float64, self.namespace_OS + '/des/spd', 1)
        self.des_timer = self.create_timer(0.1, self.des_pub)

        self.OS_enu_pos_received = False
        self.OS_heading_received = False
        self.OS_spd_received = False
        self.TS1_enu_pos_received = False
        self.TS1_heading_received = False
        self.TS1_spd_received = False
        self.TS2_enu_pos_received = False
        self.TS2_heading_received = False
        self.TS2_spd_received = False
        self.TS3_enu_pos_received = False
        self.TS3_heading_received = False
        self.TS3_spd_received = False
        self.TS4_enu_pos_received = False
        self.TS4_heading_received = False
        self.TS4_spd_received = False

    # check_
    def wait_for_topics(self):
        self.OS_timer = self.create_timer(1.0, self.OS_check_topics_status)
        self.TS_timer = self.create_timer(0.1, self.TS_check_topics_status)

    def OS_check_topics_status(self):
        if not self.OS_enu_pos_received:
            self.get_logger().info('No_OS_enu_pos_received')
        if not self.OS_heading_received:
            self.get_logger().info('No_OS_heading_received')
        if not self.OS_spd_received:
            self.get_logger().info('No_OS_spd_received')
        if (
            self.OS_enu_pos_received
            and self.OS_heading_received
            and self.OS_spd_received
        ):
            self.get_logger().info('All topics received')
        else:
            self.get_logger().info('Waiting for topics to be published')
    def TS_check_topics_status(self):
        if (
            self.TS1_enu_pos_received
            and self.TS1_heading_received
            and self.TS1_spd_received
        ):
            self.TS1.identification_status = True
        else:
            self.TS1.identification_status = False
        if (
            self.TS2_enu_pos_received
            and self.TS2_heading_received
            and self.TS2_spd_received
        ):
            self.TS2.identification_status = True
        else:
            self.TS2.identification_status = False
        if (
            self.TS3_enu_pos_received
            and self.TS3_heading_received
            and self.TS3_spd_received
        ):
            self.TS3.identification_status = True
        else:
            self.TS3.identification_status = False
        if (
            self.TS4_enu_pos_received
            and self.TS4_heading_received
            and self.TS4_spd_received
        ):
            self.TS_4.identification_status = True
        else:
            self.TS4.identification_status = False


    # callback
    # OS
    def OS_enu_pos_callback(self, msg):
        self.OS_enu_pos_received = True
        self.OS.enu_pos = np.array([msg.x, msg.y])
    def OS_heading_callback(self, msg): # deg
        self.OS_heading_received = True
        self.OS.heading = msg.data
    def OS_spd_callback(self, msg):
        self.OS_spd_received = True
        self.OS.spd = float(msg.data)
    # TS1
    def TS1_enu_pos_callback(self, msg):
        self.TS1_enu_pos_received = True
        self.TS1.enu_pos = np.array([msg.x, msg.y])
    def TS1_heading_callback(self, msg): # deg
        self.TS1_heading_received = True
        self.TS1.heading = msg.data
    def TS1_spd_callback(self, msg):
        self.TS1_spd_received = True
        self.TS1.spd = float(msg.data)
    # TS2
    def TS2_enu_pos_callback(self, msg):
        self.TS2_enu_pos_received = True
        self.TS2.enu_pos = np.array([msg.x, msg.y])
    def TS2_heading_callback(self, msg): # deg
        self.TS2_heading_received = True
        self.TS2.heading = msg.data
    def TS2_spd_callback(self, msg):
        self.TS2_spd_received = True
        self.TS2.spd = float(msg.data)
    # TS3
    def TS3_enu_pos_callback(self, msg):
        self.TS3_enu_pos_received = True
        self.TS3.enu_pos = np.array([msg.x, msg.y])
    def TS3_heading_callback(self, msg): # deg
        self.TS3_heading_received = True
        self.TS3.heading = msg.data
    def TS3_spd_callback(self, msg):
        self.TS3_spd_received = True
        self.TS3.spd = float(msg.data)
    # TS4
    def TS4_enu_pos_callback(self, msg):
        self.TS4_enu_pos_received = True
        self.TS4.enu_pos = np.array([msg.x, msg.y])
    def TS4_heading_callback(self, msg): # deg
        self.TS4_heading_received = True
        self.TS4.heading = msg.data
    def TS4_spd_callback(self, msg):
        self.TS4_spd_received = True
        self.TS4.spd = float(msg.data)

    # Publisher
    def des_pub(self):
        if (
            self.OS_enu_pos_received
            and self.OS_heading_received
            and self.OS_spd_received
        ):
            self.OS.shape = Cal_Shape(self.OS)
            self.OS.RAV = Cal_RV(self.spd_lim, self.ang_lim, self.OS.heading)
            self.OS.RAP = Cal_RP(self.OS.RAV, self.OS.enu_pos)

            if self.TS1.identification_status == True:
                self.TS1.shape = Cal_Shape(TS1)
                self.TS1.VO = Cal_VO(self.OS, self.TS1)
                self.RAV = Cal_RAV(self.RAP, self.TS1.VO)
                self.RAP = Cal_RAP(self.RAV, self.OS.enu_pos, self.tmin)
            if self.TS2.identification_status == True:
                self.TS2.VO = Cal_VO(self.OS, self.TS2)
                self.RAV = Cal_RAV(self.RAP, self.TS1.VO)
                self.RAP = Cal_RAP(self.RAV, self.OS.enu_pos, self.tmin)
            if self.TS3.identification_status == True:
                self.TS3.VO = Cal_VO(self.OS, self.TS3)
                self.RAV = Cal_RAV(self.RAP, self.TS1.VO)
                self.RAP = Cal_RAP(self.RAV, self.OS_enu_pos, self.tmin)
            if self.TS4.identification_status == True:
                self.TS4.VO = Cal_VO(self.OS, self.TS4)
                self.RAV = Cal_RAV(self.RAP, self.TS1.VO)
                self.RAP = Cal_RAP(self.RAV, self.OS.enu_pos, self.tmin)

            self.des_heading, self.des_spd = \
                Cal_Des(self.RAV, self.RAP, self.ref_heading, self.ref_spd)

            des_heading_ = Float64()
            des_heading_.data = self.des_heading

            des_spd_ = Float64()
            des_spd_.data = self.des_spd

            self.des_heading_pub_.publish(des_heading_)
            self.des_spd_pub_.publish(des_spd_)
        else:
            return


def main(args=None):
    rclpy.init(args=args)
    collision_avoidance = CollisionAvoidance()
    collision_avoidance.wait_for_topics()
    rclpy.spin(collision_avoidance)
    collision_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
