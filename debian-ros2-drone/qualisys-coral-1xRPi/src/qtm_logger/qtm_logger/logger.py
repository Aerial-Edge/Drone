#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink_msgs.msg import DronePose
from datetime import datetime
import os.path
import sys
import time
import atexit

time_prev = time.time()

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(DronePose, 'drone_pose', self.callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Initiating logging')
        now = datetime.now()
        current_time = now.strftime("%m%d%y%H%M")
        path = str(os.path.expanduser('~')+'/Logs/logs/')
        if(len(sys.argv)>1):
            filename = sys.argv[1]
            filename = filename.replace(' ', '_')
            self.file = open(path + 'log' + current_time + '_' + filename + '.csv','w')
        else:
            self.file = open(path + 'log' + current_time + '.csv','w')
        self.file.write('time,x,y,z,yaw,xvel,yvel,zvel,xacc,yacc,zacc\n')

    def callback(self, data):
        time_now = time.time() - time_prev
        data_str = str(time_now) + ","+ str(data.pos.x) + ","+ str(data.pos.y) + ","+ str(data.pos.z)+ "," + str(data.yaw.data) + ',' + str(data.vel.x) + ',' + str(data.vel.y) + ',' + str(data.vel.z) + ',' + str(data.accel.x) + ',' + str(data.accel.y) + ',' + str(data.accel.z) + '\n'
        self.file.write(data_str)
        self.file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    atexit.register(exit_handler, node)
    now = datetime.now()
    current_time = now.strftime("%m%d%y%H%M")
    path = str(os.path.expanduser('~')+'/Logs/logs/')

    if(len(sys.argv)>1):
        filename = sys.argv[1]
        filename = filename.replace(' ', '_')
        file = open(path + 'log' + current_time + '_' + filename + '.csv','w')
    else:
        file = open(path + 'log' + current_time + '.csv','w')

    file.write('time,x,y,z,yaw,xvel,yvel,zvel,xacc,yacc,zacc\n')

    try:
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #node.get_logger().info('Closing log file')
        node.destroy_node()
        rclpy.shutdown()

def exit_handler(node : Listener):
    if hasattr(node, 'file'):
        node.file.close()
        node.get_logger().info('Closing log file')


if __name__ == '__main__':
    main()
