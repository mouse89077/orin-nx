import rclpy
from rclpy.node import Node
from ublox_msgs.msg import PosVelCov
import serial
import struct


class UbloxPublisher(Node):

    def __init__(self):
        super().__init__('ublox_publisher')
        self.publisher_ = self.create_publisher(PosVelCov, 'ublox_gnss', 10)
        timer_period = 0.4  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('serial_device', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        self.serial_device = self.get_parameter('serial_device').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.ser = serial.Serial(self.serial_device, self.baud_rate)

    def timer_callback(self):
        msg = PosVelCov()
        position_read = False
        velocity_read = False
        covariance_read = False
        
        #self.get_logger().info('Entered callback')

        while (not(position_read and velocity_read and covariance_read)):
            while (self.ser.read() != b'\xb5'): #start seq 0
                pass
            if (self.ser.read() != b'\x62'): # start seq 1
                continue
            msg.header.stamp = self.get_clock().now().to_msg()
            if (self.ser.read() != b'\x01'):
                continue
            message_id = self.ser.read()
            if (message_id == b'\x02'): # POS
                self.ser.read(2) # Discard length
                msg.itow = int.from_bytes(self.ser.read(4), byteorder='little', signed=False)
                msg.lon = int.from_bytes(self.ser.read(4), byteorder='little', signed=True) / 1e7
                msg.lat = int.from_bytes(self.ser.read(4), byteorder='little', signed=True) / 1e7
                msg.height = int.from_bytes(self.ser.read(4), byteorder='little', signed=True) / 1e3
                msg.hmsl = int.from_bytes(self.ser.read(4), byteorder='little', signed=True) / 1e3
                msg.h_acc = int.from_bytes(self.ser.read(4), byteorder='little', signed=False) / 1e3
                msg.v_acc = int.from_bytes(self.ser.read(4), byteorder='little', signed=False) / 1e3

                position_read = True

            elif (message_id == b'\x12'): # VELNED
                self.ser.read(2) # Discard length
                self.ser.read(4) # Discard iTOW
                msg.vel_n = int.from_bytes(self.ser.read(4), byteorder='little', signed=True) / 100
                msg.vel_e = int.from_bytes(self.ser.read(4), byteorder='little', signed=True) / 100
                msg.vel_d = int.from_bytes(self.ser.read(4), byteorder='little', signed=True) / 100
                msg.speed = int.from_bytes(self.ser.read(4), byteorder='little', signed=False) / 100
                msg.g_speed = int.from_bytes(self.ser.read(4), byteorder='little', signed=False) / 100
                msg.heading = int.from_bytes(self.ser.read(4), byteorder='little', signed=True) / 1e5
                msg.s_acc = int.from_bytes(self.ser.read(4), byteorder='little', signed=False) / 100

                velocity_read = True

            elif (message_id == b'\x36'): # COV
                self.ser.read(2) # Discard length
                self.ser.read(4) # Discard iTOW
                self.ser.read(12)
                msg.pos_cov_nn = struct.unpack('<f', self.ser.read(4))[0]
                msg.pos_cov_ne = struct.unpack('<f', self.ser.read(4))[0]
                msg.pos_cov_nd = struct.unpack('<f', self.ser.read(4))[0]
                msg.pos_cov_ee = struct.unpack('<f', self.ser.read(4))[0]
                msg.pos_cov_ed = struct.unpack('<f', self.ser.read(4))[0]
                msg.pos_cov_dd = struct.unpack('<f', self.ser.read(4))[0]

                msg.vel_cov_nn = struct.unpack('<f', self.ser.read(4))[0]
                msg.vel_cov_ne = struct.unpack('<f', self.ser.read(4))[0]
                msg.vel_cov_nd = struct.unpack('<f', self.ser.read(4))[0]
                msg.vel_cov_ee = struct.unpack('<f', self.ser.read(4))[0]
                msg.vel_cov_ed = struct.unpack('<f', self.ser.read(4))[0]
                msg.vel_cov_dd = struct.unpack('<f', self.ser.read(4))[0]

                covariance_read = True
        
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: GNSS')
        

def main(args=None):
    rclpy.init(args=args)

    ublox_publisher = UbloxPublisher()

    rclpy.spin(ublox_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ublox_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
