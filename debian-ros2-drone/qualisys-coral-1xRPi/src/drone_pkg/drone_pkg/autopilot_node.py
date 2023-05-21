import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
import socket

# When simulating the flight controller firmware through SITL,
# MAVLink connection is transmitted over UDP instead of UART / USB

# Quickly creates a socket to get programmatically get ahold of current IP-address:
def get_ip_address():
    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("192.168.1.1", 80))
    return s.getsockname()[0]

class Autopilot(Node):
    def __init__(self):
        super().__init__('autopilot')
        self.get_logger().info('Node started')
        # Subscribes to yaw, thrust and pitch values being computed from the PIDs in "Controller node":
        self.subscription = self.create_subscription(Int32MultiArray, 'yaw_thrust_pitch', self.listener_callback, 10)
        self.serial0_udp = 'udpin:' + get_ip_address() + ':14550'
        # Tries to establish MAVLink connection on udpin:[IP_ADDRESS]:14550 and waits for heartbeat:
        self.the_connection = mavutil.mavlink_connection(self.serial0_udp)  
        self.the_connection.wait_heartbeat()
        self.get_logger().info('Heartbeat from: %s' % self.the_connection.target_system)

    def listener_callback(self, msg_in):
        self.the_connection.mav.manual_control_send(    self.the_connection.target_system,  # Established after heartbeat
                                                        msg_in.data[2],                     # Established after heartbeat
                                                        0,                                  # Roll value (static)
                                                        msg_in.data[1],                     # Thrust value
                                                        msg_in.data[0],                     # Yaw value
                                                        0)                                  # Bitfield corresponding to extra
                                                                                            # buttons , not needed and can be
                                                                                            # set to 0 for this purpose

def main(args=None):
    rclpy.init(args=args)
    autopilot = Autopilot()
    rclpy.spin(autopilot)
    autopilot.destroy_node()
    rclpy.shutdown()


if (__name__ == "__main__"):
    main()