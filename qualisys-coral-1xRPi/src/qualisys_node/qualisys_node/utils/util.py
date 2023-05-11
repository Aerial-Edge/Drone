import time
import numpy as np
from pymavlink_msgs.msg import DronePose
import qtm

def calculatePosition2(positions, current_yaw):
    return 2

start_time_ns = time.time_ns()
start_time = time.time()
time_prev = 0
prev_vel = np.zeros(3)
yaw = 0
data_failure_prev=True

def calculatePosition(positions, current_yaw):
    global time_prev
    global prev_pos
    global prev_vel
    global yaw
    global has_initialized
    global data_failure_prev
    data_failure = False


    qualisys_yaw=current_yaw


    time_now = time.time() - start_time
    time_dif = time_now - time_prev
    time_dif_hz = 1/(time_dif)


    q_pos=positions
  

    if (np.sum(np.isnan(q_pos))>0):
        data_failure=True
        data_failure_prev=True
        return DronePose()
    elif data_failure_prev:
        prev_pos=positions

    data_failure_prev=False

    vel = (q_pos - prev_pos)*time_dif_hz
    acc = (vel-prev_vel)*time_dif_hz

    diff = qualisys_yaw - yaw
    if(diff > np.pi):
        diff -= 2*np.pi
    elif(diff < -np.pi):
        diff += 2*np.pi
    yaw_vel = diff/time_dif
    positionalData = DronePose()
    positionalData.pos.x = q_pos[0]
    positionalData.pos.y = q_pos[1]
    positionalData.pos.z = q_pos[2]
    positionalData.vel.x = vel[0]
    positionalData.vel.y = vel[1]
    positionalData.vel.z = vel[2]
    positionalData.accel.x = acc[0]
    positionalData.accel.y = acc[1]
    positionalData.accel.z = acc[2]
    positionalData.yaw.data = qualisys_yaw
    positionalData.yawVel.data = yaw_vel

    time_prev = time_now
    prev_pos= q_pos
    yaw = qualisys_yaw
    prev_vel = vel

    #rospy.loginfo(yaw)

    if not data_failure:
        has_initialized = True
    
    return positionalData

######################
def angle_diff(angle_1, angle_2):
    diff=angle_1-angle_2
    while (diff > np.pi):
        diff -= 2*np.pi
    while (diff < -np.pi):
        diff += 2*np.pi
    return diff

def calculate_yaw(rot_matrix):
    """Function to calculate yaw. it returns None if we have a failure at any point."""
    if np.isnan(rot_matrix).any():
        return None
    pitch = np.arcsin(rot_matrix[0,2])               #formula to calculate pitch
    vinkel=np.clip(rot_matrix[0,0]/np.cos(pitch),-1,1)
    current_yaw = np.arccos(vinkel)   #formula to calculate yaw # <---- Kaster feil np.cos(pitch) kan bli 0 
    if np.isnan(current_yaw):
        return None
    if rot_matrix[1,0] < 0: #checks if yaw value should be positive or negative, see https://home.hvl.no/ansatte/gste/ftp/MarinLab_files/Manualer_utstyr/QTM%20Marine%20manual%2020081211.pdf p. 55 for calcuations
        current_yaw *= -1
    #rospy.loginfo('calculated yaw')
    return current_yaw

"""Function is not used in qualisys, but is used to test qualisys"""    
def rotation_matrix(yaw,pitch,roll):
    return np.matrix([[np.cos(yaw)*np.cos(pitch), (np.cos(yaw)*np.sin(pitch)*np.sin(roll))-(np.sin(yaw)*np.cos(roll)), (np.cos(yaw)*np.sin(pitch)*np.cos(roll))+(np.sin(yaw)*np.sin(roll))],
    [np.sin(yaw)*np.cos(pitch), (np.sin(yaw)*np.sin(pitch)*np.sin(roll))+(np.cos(yaw)*np.cos(roll)), (np.sin(yaw)*np.sin(pitch)*np.cos(roll))-(np.cos(yaw)*np.sin(roll))],
    [-np.sin(pitch),np.cos(pitch)*np.sin(roll),np.cos(pitch)*np.cos(roll)]])