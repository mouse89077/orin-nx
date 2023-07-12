import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

FILEPATH_pos = os.path.join(os.path.expanduser('~'), 'pygpsclient', 'data_pos.txt')
FILEPATH_spd = os.path.join(os.path.expanduser('~'), 'pygpsclient', 'data_spd.txt')

lat = String()
lon = String()
spd = String()

class XcorpsGPSRTK(Node):
    def __init__(self):
        super().__init__('xcorps_gps_rtk')
        self.namespace = '/OS'
        self.lat_publisher_ = self.create_publisher(String, self.namespace + '/gps/lat', 10)
        self.lon_publisher_ = self.create_publisher(String, self.namespace + '/gps/lon', 10)
        self.spd_publisher_ = self.create_publisher(String, self.namespace + '/gps/spd', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Read data from files
        try:
            with open(FILEPATH_pos, 'r') as f1:
                data_pos = f1.read()
        except FileNotFoundError:
            self.get_logger().warning(f"File not found: {FILEPATH_pos}")
            return
        
        cache1 = data_pos.split('/')
        if len(cache1) > 1:
            lat.data = cache1[0].strip()
            lon.data = cache1[1].strip()
            self.lat_publisher_.publish(lat)
            self.lon_publisher_.publish(lon)
            # self.get_logger().info('Latitude is: "%s"' % lat.data)
            # self.get_logger().info('Longitude is: "%s"' % lon.data)
        else:
            self.get_logger().info('No gps lat, lon serial')

        try:
            with open(FILEPATH_spd, 'r') as f2:
                data_spd = f2.read()
        except FileNotFoundError:
            self.get_logger().warning(f"File not found: {FILEPATH_spd}")
            return
        cache2 = data_spd.split('/')
        spd.data = cache2[0].strip()
        self.spd_publisher_.publish(spd)
        # self.get_logger().info('Speed is: "%s"' % spd.data)


def main(args=None):
    rclpy.init(args=args)

    xcorps_gps_rtk = XcorpsGPSRTK()

    rclpy.spin(xcorps_gps_rtk)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    xcorps_gps_rtk.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
