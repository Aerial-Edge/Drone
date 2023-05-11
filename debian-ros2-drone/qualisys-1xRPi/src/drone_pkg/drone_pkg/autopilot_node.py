import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
import socket


def get_ip_address():
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.connect(("192.168.1.1", 80))
	return s.getsockname()[0]

class Autopilot(Node):
	def __init__(self):
		super().__init__('autopilot')
		self.get_logger().info('Node started')
		self.subscription = self.create_subscription(Int32MultiArray, 'yaw_thrust_pitch', self.listener_callback, 10)
		self.serial0_udp = 'udpin:' + get_ip_address() + ':14550'
		self.the_connection = mavutil.mavlink_connection(self.serial0_udp)
		self.the_connection.wait_heartbeat()
		self.get_logger().info('Heartbeat from: %s' % self.the_connection.target_system)

	def listener_callback(self, msg_in):
		self.the_connection.mav.manual_control_send(self.the_connection.target_system, msg_in.data[2], 0, msg_in.data[1], msg_in.data[0], 0)
		#self.get_logger().info('Callback function executed')

def main(args=None):
	rclpy.init(args=args)
	autopilot = Autopilot()
	rclpy.spin(autopilot)
	autopilot.destroy_node()
	rclpy.shutdown()


if (__name__ == "__main__"):
	main()
