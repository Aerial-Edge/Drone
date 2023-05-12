#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from pymavlink import mavutil
from pymavlink_msgs.msg import DronePose
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np

class FollowAlgorithm(Node):
    def __init__(self): 
        super().__init__('follow_algorithm') 

        self.create_subscription(Int32MultiArray, 'object_pos_and_distance', self.position_and_distance_callback, 10)
        
        self.create_subscription(DronePose, 'drone_pose', self.qualisys_callback, 10)
        self.distance = 0.0
        self.fig = plt.axes(projection='3d')
        arrsize = 1000
        self.drone_x_arr = np.zeros(arrsize)
        self.drone_y_arr = np.zeros(arrsize)
        self.drone_z_arr = np.zeros(arrsize)
        self.qualisys_x_arr = np.zeros(arrsize)
        self.qualisys_y_arr = np.zeros(arrsize)
        self.qualisys_z_arr = np.zeros(arrsize)
        self.counter = 0
        
        
    def qualisys_callback(self, msg: DronePose):

        
        

        # Constants
        object_x = 0.0  # meters
        object_y = 0.0  # meters
        # Constants
        object_x = 0.0  # meters
        object_y = 0.0  # meters
        camera_angle = 45  # degrees
        
        # Variables
        yaw = msg.yaw.data # radians -pi to pi
        true_x = msg.pos.x # x position in meters from qualisys
        true_y = msg.pos.y # y position in meters from qualisys
        true_z = msg.pos.z
        distance_to_object = self.distance# meters


        # Calculate the drone's position
        drone_x, drone_y, drone_z = self.find_drone_position(yaw, distance_to_object, object_x, object_y, 0, camera_angle)
        
        self.get_logger().info(f"Estimated position {drone_x} {drone_y} ")
        self.get_logger().info(f"True position {true_x} {true_y} ")
        self.get_logger().info(f"Estimated position {drone_x} {drone_y} ")
        self.get_logger().info(f"True position {true_x} {true_y} ")

        self.plotter((drone_x, drone_y, drone_z), (true_x, true_y, true_z))


    def find_drone_position(self, yaw, distance_to_object, object_x, object_y, object_z, camera_angle):
    
        
        # Calculate drone position
        horizontal_distance = distance_to_object * math.cos(math.radians(camera_angle))
        altitude = distance_to_object * math.sin(math.radians(camera_angle))
        #drone_x = object_x + horizontal_distance * math.cos(yaw)
        #drone_y = object_y + horizontal_distance * math.sin(yaw)
        drone_x = object_x + distance_to_object * math.cos(yaw)
        drone_y = object_y + distance_to_object * math.sin(yaw)

        drone_z = 0

        return drone_x, drone_y, drone_z

    def plotter(self, drone_pos, qualisys_pos):


        if (self.counter < len(self.drone_x_arr)):
            # Scatter plots
            x_drone = drone_pos[0]
            y_drone = np.cos(drone_pos[1])
            z_drone = drone_pos[2]
            x_qualisys = qualisys_pos[0]
            y_qualisys = np.cos(qualisys_pos[1])
            z_qualisys = qualisys_pos[2]
            ndx = self.counter
            self.counter += 1

            self.drone_x_arr[ndx] = x_drone
            self.drone_y_arr[ndx] = y_drone
            self.drone_z_arr[ndx] = z_drone
            self.qualisys_x_arr[ndx] = x_qualisys
            self.qualisys_y_arr[ndx] = y_qualisys
            self.qualisys_z_arr[ndx] = z_qualisys
            #self.fig.scatter3D(x_drone, y_drone, z_drone, c=z_drone, cmap='Greens');
            #self.fig.scatter3D(x_qualisys, y_qualisys, z_qualisys, c=z_qualisys, cmap='Blues');
            #self.fig.Scatter(x_qualisys, y_qualisys, cmap='Greens')
            #self.fig.Scatter(x_drone, y_drone, cmap='Blues')
        else:
            self.save_plot()

    def save_plot(self):
        fig = plt.axes(projection='3d')
        #fig.scatter3D(self.drone_x_arr, self.drone_y_arr, self.drone_z_arr, c=self.qualisys_z_arr, cmap='Greens');
        #fig.scatter3D(self.qualisys_x_arr, self.qualisys_y_arr, self.qualisys_z_arr, c=self.qualisys_z_arr, cmap='Blues');
        self.fig.Scatter(qualisys_x_arr, qualisys_y_arr, cmap='Greens')
        self.fig.Scatter(drone_x_arr, drone_y_arr, cmap='Blues')
        plt.savefig("/home/ros/log/plot.png")


    def position_and_distance_callback(self, msg: Int32MultiArray):

        self.distance = msg.data[2]
        self.get_logger().info(f"Distance to object {self.distance} ")
       

    


def main(args=None):
    rclpy.init(args=args)
    follow_algorithm = FollowAlgorithm()
    rclpy.spin(follow_algorithm)
    follow_algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
