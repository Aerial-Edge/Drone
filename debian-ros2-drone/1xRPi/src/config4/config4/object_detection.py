#!/usr/bin/etv python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
import cvzone
from cvzone.ColorModule import ColorFinder
import numpy as np
import time


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.subscription = self.create_subscription(Image, 'image_data', self.process_image, 1) 
        self.bridge = CvBridge() # create a bridge between OpenCV and ROS
        self.distance_publisher = self.create_publisher(Float64, 'distance', 1) 
        self.position_publisher = self.create_publisher(Float64, 'position', 1) 
    def process_image(self, msg): # callback function main processing function
        start_time = time.time()

        distance = 0.0 # create a message
        position = 0.0 # create a message
        
        myColorFinder = ColorFinder(False) # create a color finder object
        hsvVals = {'hmin': 45, 'smin': 67, 'vmin': 21, 'hmax': 99, 'smax': 155, 'vmax': 211}

        opencv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # convert the ROS message to an OpenCV image

        imgColor, mask = myColorFinder.update(opencv_image, hsvVals)
        imgContours, contours = cvzone.findContours(opencv_image, mask)

        circular_contours = [cnt for cnt in contours if is_circle(cnt['cnt'])]

        if circular_contours:
            cnt = circular_contours[0]
            data = cnt['center'][0], h - cnt['center'][1], int(cnt['area'])

            f = 474
            W = 6.5
            w = np.sqrt(cnt['area'] / np.pi) * 2
            d = (W * f) / w
            self.get_logger().info('Distance: %f' % d)

        processing_time = time.time() - start_time
        self.get_logger().info('Processing time: %f' % processing_time)

        self.publish_object_distance(distance) # Publish the distance
        self.publish_object_position(position) # Publish the position

        def is_circle(cnt, threshold=0.65):
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                return False
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            return circularity >= threshold
    
    def publish_object_distance(self, distance): # callback function
        msg = Float64() # create a message
        msg.data = distance # fill in the message
        self.distance_publisher.publish(msg) # publish the message

    def publish_object_position(self, position): # callback function
        msg = Float64() # create a message
        msg.data = position # fill in the message
        self.position_publisher.publish(msg) # publish the message

def main(args=None): # args is a list of strings
    rclpy.init(args=args) # initialize the ROS client library

    node = ObjectDetection() # create a node

    rclpy.spin(node) #  wait for messages
    node.destroy_node() # destroy the node explicitly

    rclpy.shutdown() # shutdown the ROS client library

if __name__ == '__main__':
    main()
