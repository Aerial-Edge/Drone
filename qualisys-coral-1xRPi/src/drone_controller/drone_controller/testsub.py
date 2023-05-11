import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray



class TestNode(Node):
    def __init__(self):
        super().__init__('testnode')
        self.subscription = self.create_subscription(Int32MultiArray, 'yaw_thrust_pitch', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Recieved array: %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()