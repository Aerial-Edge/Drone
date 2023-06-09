#!/usr/bin/etv python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # import the Image message type
import cv2
from cv_bridge import CvBridge
from picamera2 import Picamera2
from libcamera import controls

class CameraCapture(Node): # inherits from Node
    def __init__(self): # constructor
        super().__init__('camera_capture') # call the constructor of the parent class
        self.publisher_ = self.create_publisher(Image, 'image_data', 10) # create a publisher
        self.timer = self.create_timer(1/30, self.publish_image_data) # create a timer
        self.opencv_video = Picamera2() # open the video capture device
        self.opencv_video.configure(self.opencv_video.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
        self.opencv_video.set_controls({"AwbEnable": True})
        self.opencv_video.start() 
        self.bridge = CvBridge() # create a bridge between OpenCV and ROS

    def publish_image_data(self): # callback function
        frame = self.opencv_video.capture_array() # read a frame from the video capture device
        #if not success:
       #     self.get_logger().info('Failed to read frame from camera')
       #     return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8') # convert the image to a ROS message
        

        self.publisher_.publish(msg) # publish the message

def main(args=None): # args is a list of strings
    rclpy.init(args=args) # initialize the ROS client library

    node = CameraCapture() # create a node

    rclpy.spin(node) # wait for messages
    node.destroy_node() # destroy the node explicitly

    rclpy.shutdown() # shutdown the ROS client library

if __name__ == '__main__': # if this file is run as a script
    main() # run the main function