#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import asyncio
import xml.etree.ElementTree as ET
import qtm
import numpy as np
from pymavlink_msgs.msg import DronePose
#from .utils.util import angle_diff, calculate_yaw
from .util import angle_diff, calculate_yaw
import sys

###GLOBAL VARIABLES###
q_freq =0 # frekvens fra som hentes fra qualisys
prev_vel=None 

###GAZEBO GLOBAL VARIABLES###
newPose = False

###QUALISYS GLOBAL VARIABLES###
non_published_packages=0
prev_msg=None
qualisys_ip='192.168.1.103' # ip of the Qualisys machine

if prev_msg is None:
    prev_msg = DronePose()
    prev_msg.pos.x = 0.0
    prev_msg.pos.y = 0.0
    prev_msg.pos.z = 0.0
    prev_msg.vel.x = 0.0
    prev_msg.vel.y = 0.0
    prev_msg.vel.z = 0.0
    prev_msg.yaw.data = 0.0

if prev_vel is None:
    prev_vel = [0.0, 0.0, 0.0] 
#######################################################################

#######################################################################
class UdpProtocol:
    """
    A callable asyncio Datagram Protocol implementation.
    For robotics programming purpose, I need this protocol does last-come-first-serve.
    Reference:
        https://docs.python.org/3/library/asyncio-protocol.html#datagram-protocols
        https://docs.python.org/3/library/asyncio-eventloop.html#asyncio.loop.create_datagram_endpoint    
        https://stackoverflow.com/questions/46140556/proper-way-to-clear-an-asyncio-queue-in-python3
    """
    def __init__(self):
        # initialize the queue
        self.packets = asyncio.Queue()

    def connection_made(self, transport):
        print("connection made")

    def datagram_received(self, data, addr):
        """
        Receive datagram from the UDP channel.
        """
        # clear the current queue and the accumulated data
        self.packets._queue.clear()

        self.packets.put_nowait(data)

    def connection_lost(self, transport):
        print("connection lost")

    def error_received(self, exc):
        pass

    async def recvfrom(self):
        # get the data from the queue
        return await self.packets.get()
###############################################################################

class Qualisys_node(Node):
    """
    This code is borrowed from Hilde Marie Moholt, and modified to fit ros2 by Aerial Edge.
    
    """
    def __init__(self):
        super().__init__('qualisys_node')

        self.pub = self.create_publisher(DronePose, 'drone_pose', 10)

    def talker(self, data):
        self.pub.publish(data)

    def create_msg(self,x,y,z,v_x,v_y,v_z,a_x,a_y,a_z,yaw,v_yaw,freq,full_msg):
    #Function to create the new message and store the old one so we can use it for calculations, it will not publis if full_msg is false.
    
        global prev_msg
        positionalData = DronePose()
        positionalData.freq=int(freq)
        positionalData.pos.x = x
        positionalData.pos.y = y
        positionalData.pos.z = z
        positionalData.vel.x = v_x
        positionalData.vel.y = v_y
        positionalData.vel.z = v_z
        positionalData.accel.x = a_x
        positionalData.accel.y = a_y
        positionalData.accel.z = a_z
        positionalData.yaw.data = yaw
        positionalData.yaw_vel.data = v_yaw

        published=False
        prev_msg = positionalData #save the old position
        try:
            if full_msg:
                self.talker(positionalData)
                published=True
        finally:
            return published, True #the last arg is if we saved the position.
        
    def calculate_vel_a(self,position,freq, yaw):
        global prev_msg #we will use the last message to calculate velocity and rotation
        global prev_vel
        full_msg=True # flag to see if we can publish the message or not.
        #self.get_logger().info(f"yaw: {yaw}")

        if (np.isnan(position).any()) or (position[0]==None) or yaw==None:
            return False,False
        pos_m = [position[0]/1000,position[1]/1000,position[2]/1000] #convert to meters
        if prev_msg != None: #if we sent a message before we have positional data
            vel_x=(pos_m[0]-prev_msg.pos.x)*freq
            vel_y=(pos_m[1]-prev_msg.pos.y)*freq
            vel_z=(pos_m[2]-prev_msg.pos.z)*freq
            if prev_msg.vel.x != None: #test if we have velocity in the last package so we can calculate acceleration
                #since we have prev vel, we can also calculate avg speed over two
                vel_f=[(vel_x+prev_vel[0])*0.5 , (vel_y+prev_vel[1])*0.5, (vel_z+prev_vel[2])*0.5] #filtering positions

                #calculating acc with avg speeds
                a_x=(vel_f[0]-prev_msg.vel.x)*freq
                a_y=(vel_f[1]-prev_msg.vel.y)*freq
                a_z=(vel_f[2]-prev_msg.vel.z)*freq

                prev_vel=[vel_x,vel_y,vel_z] #setting prev vel to the real previous velocity

                vel_x=vel_f[0]
                vel_y=vel_f[1]
                vel_z=vel_f[2]
                #self.get_logger().info(f"vel_x: {vel_x}, vel_y: {vel_y}, vel_z: {vel_z}")
            else:
                a_x=None
                a_y=None
                a_z=None
                full_msg=False
                prev_vel=[vel_x,vel_y,vel_z]
            if (prev_msg.yaw.data != None):
                v_yaw=angle_diff(yaw,prev_msg.yaw.data)*freq
            else:
                v_yaw=None
                full_msg=False
        else: #we do not have any previous data, but we can store our current data
            full_msg=False
            vel_x=None
            vel_y=None
            vel_z=None
            a_x=None
            a_y=None
            a_z=None
            v_yaw=None
            #Now call function to publish data (last argument is if it is valid or not)
        return self.create_msg(pos_m[0],pos_m[1],pos_m[2],vel_x,vel_y,vel_z,a_x,a_y,a_z,yaw,v_yaw,freq,full_msg)
    
    def calc(self,position,rot,freq):
        global non_published_packages
        non_published_packages+=1 #the package has not been published yet, so one is added
        yaw=-calculate_yaw(rot) # pid controllers wants the yaw to point the other direction, therefore it is multiplied with -1
        real_freq= freq/non_published_packages #calculate the frequency
        published, saved_pos=self.calculate_vel_a(position,real_freq,yaw)
        if saved_pos:
            non_published_packages=0 
        
        return
    
    def create_body_index(self, xml_string):
        """ Extract a name to index dictionary from 6-DOF settings xml """
        xml = ET.fromstring(xml_string)

        body_to_index = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            body_to_index[body.text.strip()] = index

        return body_to_index

    def get_freq(self, xml_string): #ok i made this one -Oscar
        #function to get frequency of data packets 
        xml = ET.fromstring(xml_string)
        return int(xml[0][0].text)

    async def qtmMain(self):
        global qualisys_ip
        global q_freq
        self.get_logger().info("Selecting streaming method")
        flag_realtime = 1

        self.get_logger().info("Declaring mocap server IP address")
        IP_server = qualisys_ip # ip of the Qualisys machine

        # Connect to qtm
        self.get_logger().info("Connecting to Qualisys realtime server")
        

        # Set a timeout for the connection attempt
        timeout = 20  # Change this value to the desired timeout in seconds
        try:
            connection = await asyncio.wait_for(qtm.connect(IP_server, version='1.10'), timeout)
        except asyncio.TimeoutError:
            self.get_logger().info("Connection attempt timed out")
            raise KeyboardInterrupt

        if connection is None:
            self.get_logger().info("Failed to connect")
            raise KeyboardInterrupt
        else:
            self.get_logger().info("Connected to Qualisys server")
        
        # Take control of qtm
        async with qtm.TakeControl(connection, "password"):
            await connection.new()

        # Get 6-DOF settings from QTM
        xml_string = await connection.get_parameters(parameters=["6d", "general"]) 

        # IP for listening data
        HOST = IP_server
        # Port for listening data
        PORT = 22222
        server_address_udp = (HOST, PORT)

        # Create a UDP socket for data streaming
        loop = asyncio.get_running_loop()
        transport, protocol = await loop.create_datagram_endpoint(
            UdpProtocol, local_addr=None, remote_addr=server_address_udp)

        # parser for mocap rigid bodies indexing
        body_index = self.create_body_index(xml_string)
        
        self.get_logger().info("got bodies")
        q_freq=self.get_freq(xml_string)
        self.get_logger().info(f"Camera_frequence is: {q_freq}")
        self.get_logger().info(f"Available bodies: {body_index}")

        #name of the body we want to select, defined in qualisys
        wanted_body = 'Drone'
        if (len(sys.argv)>2):
            wanted_body=sys.argv[2]
        self.get_logger().info(f"Getting data for body: {wanted_body}")

        def on_packet(packet): #This function will be called whenever a new packet arrives
            global non_published_packages
            if not rclpy.ok(): #if roscore is not running, an Interrupt Exception will be raised
                raise KeyboardInterrupt()
            bodies = packet.get_6d()[1]
            
            if wanted_body is not None and wanted_body in body_index:
                # Extract one specific body
                wanted_index = body_index[wanted_body]
                position, rotation = bodies[wanted_index]
                if not  np.isnan(position).any():
                    rotation_np = np.asarray(rotation.matrix, dtype=np.float64).reshape(3, 3)   #convert to 3x3 matrix
                    self.calc(position,rotation_np,q_freq)
                msg = np.asarray((position.x/1000.0, position.y/1000.0, position.z/1000.0) + rotation.matrix, dtype=np.float64).tobytes() 
                transport.sendto(msg, server_address_udp) #requests a new packet with info from the server
            else:
                # Print all bodies, vil bare aktiveres om den ikke finner Drone
                for position, rotation in bodies:
                    self.get_logger().info("There is no such a rigid body! Print all bodies.")
                    print("Pos: {} - Rot: {}".format(position, rotation))

        # Start streaming frames
        self.get_logger().info("starting stream")
        await connection.stream_frames(components=["6d"], on_packet=on_packet) #commando that tells Qualisys to start streaming data, is run only once

def main(args=None):
    rclpy.init(args=args)
    qtm_node = Qualisys_node()
    
    loop = asyncio.get_event_loop()
    qtm_task = loop.create_task(qtm_node.qtmMain())  # Starts an asyncio loop running the Qualisys main program

    try:
        flag = 'qualisys'  # If the node is not launched with arguments, the node will assume it is receiving qualisys data

        if(len(sys.argv) > 1):
            flag = sys.argv[1]

        qtm_node.get_logger().info("Launching publisher")
        qtm_node.get_logger().info(f"initiating node with flag {flag}")
        if flag == 'qualisys':
            qtm_node.get_logger().info('Starting Qualisys Loop')
            asyncio.ensure_future(qtm_task)
            loop.run_forever()  # This loop will continue until it is called to stop
    except KeyboardInterrupt:
        pass
    finally:
        qtm_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    

if __name__ == '__main__':
    main()
