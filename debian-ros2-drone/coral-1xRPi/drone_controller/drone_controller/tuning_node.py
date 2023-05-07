import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .object_follower import ObjectFollower
from .basic_pid import PID
from pynput import keyboard


def press_callb(key):
    #print(key.char)
    print("hello")

class Tuner(Node):
    def __init__(self):
        super().__init__('tuner')
        self.subscription = self.create_subscription(Int32MultiArray, 'object_pos_and_distance', self.listener_callback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, 'yaw_thrust_pitch', 10)
        self.object_follower = ObjectFollower()

    def tune(self, pid : PID):

        p_input = input("Type p-gain float value (current value: {:.3f}):\n ".format(pid.get_kp()))
        i_input = input("Type i-gain float value (current value: {:.3f}): ".format(pid.get_ki()))
        d_input = input("Type d-gain float value (current value: {:.3f}): ".format(pid.get_kd()))
        pid.tune(float(p_input), float(i_input), float(d_input))

    def press_callback(self, key):
        if hasattr(key, 'char'):
            if key.char == "y":
                self.tune(self.object_follower.yaw_pid)
            elif key.char == 't':
                self.tune(self.object_follower.thrust_pid)
            elif key.char == 'p':
                self.tune(self.object_follower.pitch_pid)
            




    def listener_callback(self, msg_in):


        self.object_follower(x=msg_in.data[0], y=msg_in.data[1], distance=msg_in.data[2])
        msg_out = Int32MultiArray()
        msg_out.data = [self.object_follower.yaw_out, self.object_follower.thrust_out, self.object_follower.pitch_out ]
        self.publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    tuner = Tuner()
    listener = keyboard.Listener(on_press=tuner.press_callback, on_release=None)
    listener.start()
    rclpy.spin(tuner)
    tuner.destroy_node()
    rclpy.shutdown()
    listener.stop()
    

if (__name__ == "__main__"):
    main()

