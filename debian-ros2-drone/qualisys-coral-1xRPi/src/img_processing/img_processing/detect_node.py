import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import time
from picamera2 import Picamera2



#model ='../../share/img_processing/models/tennis_edgetpu.tflite'
#model ='/home/gruppe6/models/edl0_1k_edgetpu.tflite'
model = '/home/ros/ros2_ws/src/img_processing/models/edl0_edgetpu.tflite'
tpu_interpreter = tflite.Interpreter(model, experimental_delegates=[
    tflite.load_delegate('libedgetpu.so.1.0')])
cpu_interpreter = tflite.Interpreter(model)
threshold = 0.95


class Detect(Node):
    def __init__(self):
        super().__init__('detect')
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)
        self.subscription
        self.publisher = self.create_publisher(Int32MultiArray, 'object_pos_and_distance', 10)
        self.br = CvBridge()
        self.interpreter = tpu_interpreter
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.is_first_message = True
        self.period_timer_start = time.time()
        self.period_timer_end = time.time()
        self.fps = 0
        self.focal_length = 847 # pi camera 2
        self.ball_real_diameter = 6.5 # cm
        

    def constrain_detection(self, val, frame_dim, cutoff):
        return val > cutoff and val < (frame_dim - cutoff)

    def listener_callback(self, data):
        self.period_timer_end = time.time()
        self.timer_period = self.period_timer_end - self.period_timer_start
        self.fps = 1 / self.timer_period
        self.period_timer_start = time.time()
        self.get_logger().info('Recieving video frame, current FPS: {:.2f}'.format(self.fps))
        current_frame = self.br.imgmsg_to_cv2(data)

        if (self.is_first_message):
            self.raw_height, self.raw_width, _ = current_frame.shape
            self.is_first_message = False
            print(self.output_details)
        
        
        input_data = np.expand_dims(current_frame, axis=0)

        #input_tensor = self.interpreter.tensor(self.input_details[0]['index'])
        #output_box = self.interpreter.tensor(self.output_details[1]['index'])
        #output_scores = self.interpreter.tensor(self.output_details[0]['index'])
        #input_tensor()[:] = input_data

        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        #boxes = output_box()[0]
        boxes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[3]['index'])[0]
        #scores = output_scores()[0]
        scores = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        num_detections = self.interpreter.get_tensor(self.output_details[2]['index'])[0]



        # for i in range(len(scores)):
        #     if ((scores[i][0] > threshold) and (scores[i][0] <= 1.0)):
        #         print(scores[i][0])

        msg = Int32MultiArray()

        if (len(scores) != 0):
            if ((scores[0] > threshold) and (scores[0] <= 1.0)):
                x1, x2, y1, y2 = int(boxes[0][1] * self.raw_width) , int(boxes[0][3] * self.raw_width), int(boxes[0][0] * self.raw_height), int(boxes[0][2] * self.raw_height)
                w, h = x2 - x1, y2 - y1
                cx, cy = (int(x1 + 0.5*w),int(y1+0.5*h))
                box_diagonal_length = int(np.sqrt(w**2 + h**2))
                if (self.constrain_detection(cx, self.raw_width, 30) and self.constrain_detection(cy, self.raw_height, 30)):
                    dist = int((self.ball_real_diameter * self.focal_length) / w) # distance in cm
                    msg.data = [cx, cy, dist]
                    self.publisher.publish(msg)
                else:
                    msg.data = [-1, -1, -1]
                    self.publisher.publish(msg)
            else:
                msg.data = [-1, -1, -1]
                self.publisher.publish(msg)


                







def main(args=None):
    rclpy.init(args=args)
    detect = Detect()
    rclpy.spin(detect)
    detect.destroy_node()
    rclpy.shutdown()
    

if (__name__ == "__main__"):
    main()
