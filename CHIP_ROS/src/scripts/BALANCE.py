#!/usr/bin/env python

'''
This node is both a publisher and a subscriber. It is basically a large
PID controller that takes IMU input to make the platform stable. It does
this in two ways, if the platform is falling in one direction or the other
it will correct by moving in the opposite direction and if it senses the
platform is not sitting level it will correct by positioning the legs as such
'''

import rospy
from std_msgs.msg import Int16, Float64MultiArray
from sensor_msgs.msg import Imu
import math
import numpy as np
from simple_pid import PID
from DEFAULTS import DEFAULTS
from CONTROL_MODES import CONTROL_MODE

'''
DEFINE SOME GLOBAL VARIABLES
'''
DESIRED = DEFAULTS.DESIRED_RPY #FOR NOW we're keeping all angles as 0, roll-pitch-yaw
LIMIT = 10.0
ROLL_PID = PID(Kp=25.0, Ki=0.0, Kd=0.0, setpoint=DESIRED[0], output_limits=(-LIMIT/2, LIMIT/2))
PITCH_PID = PID(Kp=25.0, Ki=0.0, Kd=0.0, setpoint=DESIRED[1], output_limits=(-LIMIT, LIMIT))

'''
DEFIINE VARIABLES TO PUBLISH
'''
SET_TO_PUB = [] #NO DEFAULT PUB SET FOR THE IMU NODE
ORI = [] #roll, pitch, yaw
CURRENT_SETPOINT = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] #default setpoint is the HOME position until otherwise noted  we should change to current foot pose
MODE = CONTROL_MODE.STAND_SIT

'''
creating publisher up here so on callback we can REPUBLISH
'''
ORI_PUB = rospy.Publisher('ORIENTATION', Float64MultiArray, queue_size=0) #default queue size is 0
if(DEFAULTS.IMU_ON):
    SET_PUB = rospy.Publisher('CMDS', Float64MultiArray, queue_size=0) #default queue size is 0


#____________________________________________________________________________________________________________________________________________________________________
# THESE METHODS ARE HELPER METHODS
#____________________________________________________________________________________________________________________________________________________________________

"""Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1,
                     np.sign(sinp) * np.pi / 2,
                     np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]

"""Bounds the value input between lower and upper, known limits of the system, think of them as virtual hardstops"""
def bound(lower, upper, value):
    if (value>=upper):
        return upper
    if (value<=lower):
        return lower
    return value

#____________________________________________________________________________________________________________________________________________________________________
# HERE'S THE ACTUAL ROS NODE METHODS AND ETC.
#____________________________________________________________________________________________________________________________________________________________________

#we can update the DESIRED roll pitch and yaw easily like this later!
def setpoint_callback(data):
    global CURRENT_SETPOINT
    #get the current setpoint and set it to that
    CURRENT_SETPOINT = data.data

#we can update the DESIRED roll pitch and yaw easily like this later!
def mode_callback(data):
    global MODE
    #get the current setpoint and set it to that
    MODE = data.data

'''
gets the imu data, feeds it to the method and calaucaltes the euler angles, and uses that to set the position of the robot legs to compensate. 
'''
def callback(data):
    global ORI

    #get the quaternion
    w = data.orientation.w
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z

    #get the roll/pitch/yaw angles
    ORI = quaternion_to_euler(w, x, y, z)

    if(MODE == CONTROL_MODE.WALK):
        #define the current set to publish
        global SET_TO_PUB
        SET_TO_PUB = list(CURRENT_SETPOINT) #make it a mutable list
        #define the current roll and pitch
        ROLL = ORI[0]
        PITCH = ORI[1]

        #execute the PID - ptch changes the shoulder and roll changes the hinge
        ROLL_OUTPUT = ROLL_PID(ROLL)
        PITCH_OUTPUT = PITCH_PID(PITCH)

        #set the commands to the legs WE WILL CHANGE THIS TO ONLY THE LEGS
        #THAT ARE ON THE GROUND AT A LATER POINT.
        SET_TO_PUB[0] = PITCH_OUTPUT+CURRENT_SETPOINT[0]
        SET_TO_PUB[1] = ROLL_OUTPUT+CURRENT_SETPOINT[1]
        SET_TO_PUB[3] = PITCH_OUTPUT+CURRENT_SETPOINT[3]
        SET_TO_PUB[4] = -ROLL_OUTPUT+CURRENT_SETPOINT[4]
        SET_TO_PUB[6] = PITCH_OUTPUT+CURRENT_SETPOINT[6]
        SET_TO_PUB[7] = ROLL_OUTPUT+CURRENT_SETPOINT[7]
        SET_TO_PUB[9] = PITCH_OUTPUT+CURRENT_SETPOINT[9]
        SET_TO_PUB[10] = -ROLL_OUTPUT+CURRENT_SETPOINT[10]

        #publishing happens at the end of this callback
    else:
        global SET_TO_PUB
        SET_TO_PUB = CURRENT_SETPOINT
    
    #actually publish the setpoint
    if(DEFAULTS.IMU_ON):
        SET_PUB.publish(Float64MultiArray(data=SET_TO_PUB))
    ORI_PUB.publish(Float64MultiArray(data=ORI))

'''
This does the actual subscribing and calling the callback
'''
def publisher_subscriber(): 
    #initilize the node
    rospy.init_node("STABILIZER")

    #read the SETPOINT_TOPIC to set the current setpoint
    rospy.Subscriber("CMDS_RAW", Float64MultiArray, setpoint_callback)
    rospy.Subscriber("CONTROL_MODE", Int16, mode_callback)
    #read the topic we're subscribing to and execute CALLBACK
    rospy.Subscriber("/mavros/imu/data", Imu, callback)

    #keeps node alive
    rospy.spin()


'''
not specific to ROS, this is the MAIN method, run this
'''
if __name__ == '__main__':
    try:
        publisher_subscriber()
    except rospy.ROSInterruptException:
        pass