#!/usr/bin/env python

'''
This node is both a publisher and a subscriber. It is basically a large
PID controller that takes IMU input to make the platform stable. It does
this in two ways, if the platform is falling in one direction or the other
it will correct by moving in the opposite direction and if it senses the
platform is not sitting level it will correct by positioning the legs as such
'''

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import Imu
import math
import numpy as np
from simple_pid import PID
from DEFAULTS import DEFAULTS

'''
DEFINE SOME GLOBAL VARIABLES
'''
DESIRED = DEFAULTS.DESIRED_RPY #FOR NOW we're keeping all angles as 0, roll-pitch-yaw
#here's the output limit variable
OUTPUT_LIMIT_LOWER = DEFAULTS.Y_LIMIT_LOWER
OUTPUT_LIMIT_UPPER = DEFAULTS.Y_LIMIT_UPPER
#we wont bound the PID controller, we'll just bound the actual output
ROLL_PID = PID(Kp=0.1, Ki=0.0, Kd=0.0, setpoint=DESIRED[0])
PITCH_PID = PID(Kp=0.1, Ki=0.0, Kd=0.0, setpoint=DESIRED[0])

'''
DEFIINE VARIABLES TO PUBLISH
'''
SET_TO_PUB = [] #NO DEFAULT PUB SET FOR THE IMU NODE
ORI = [] #roll, pitch, yaw
CURRENT_SETPOINT = DEFAULTS.SIT #default setpoint is the HOME position until otherwise noted  we should change to current foot pose

'''
creating publisher up here so on callback we can REPUBLISH
'''
SET_PUB = rospy.Publisher('SETPOINT', Float64MultiArray, queue_size=0) #default queue size is 0
ORI_PUB = rospy.Publisher('ORIENTATION', Float64MultiArray, queue_size=0) #default queue size is 0

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

'''
gets the imu data, feeds it to the method and calaucaltes the euler angles, and uses that to set the position of the robot legs to compensate. 
'''
def callback(data):
    global SET_TO_PUB
    global ORI

    #get the quaternion
    w = data.orientation.w
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z

    #get the roll/pitch/yaw angles
    ORI = quaternion_to_euler(w, x, y, z)
    ROLL = ORI[0]
    PITCH = ORI[1]

    #x is ROLL, y is PITCH, z is YAW, DIRECTIONS SHOULD FOLLOW STANDARD PID CONVENTION
    ROLL_OUTPUT = ROLL_PID(ROLL)
    PITCH_OUTPUT = PITCH_PID(PITCH)
    '''
    IF WE WANT TO CHANGE THE SETPOINT:
    global ROLL_PID
    ROLL_PID = PID(Kp=0.01, Ki=0.0, Kd=0.0, output_limits=(0.15,0.55), setpoint=NEW_SETPOINT)
    And THEN CALL IT, this will create a new object and reset the controller to the new setpoint and so on. 
    '''
    CMD_FLY = bound(OUTPUT_LIMIT_LOWER, OUTPUT_LIMIT_UPPER, CURRENT_SETPOINT[1]+ROLL_OUTPUT+PITCH_OUTPUT)
    CMD_FRY = bound(OUTPUT_LIMIT_LOWER, OUTPUT_LIMIT_UPPER, CURRENT_SETPOINT[4]-ROLL_OUTPUT+PITCH_OUTPUT)
    CMD_BLY = bound(OUTPUT_LIMIT_LOWER, OUTPUT_LIMIT_UPPER, CURRENT_SETPOINT[7]+ROLL_OUTPUT-PITCH_OUTPUT)
    CMD_BRY = bound(OUTPUT_LIMIT_LOWER, OUTPUT_LIMIT_UPPER, CURRENT_SETPOINT[10]-ROLL_OUTPUT-PITCH_OUTPUT)

    #set the new array
    SET_TO_PUB = [CURRENT_SETPOINT[0], CMD_FLY, CURRENT_SETPOINT[2], CURRENT_SETPOINT[3], CMD_FRY, CURRENT_SETPOINT[5], CURRENT_SETPOINT[6], CMD_BLY, CURRENT_SETPOINT[8], CURRENT_SETPOINT[9], CMD_BRY, CURRENT_SETPOINT[11]]

    #actually publish the setpoint
    SET_PUB.publish(Float64MultiArray(data=SET_TO_PUB))
    ORI_PUB.publish(Float64MultiArray(data=ORI))

'''
This does the actual subscribing and calling the callback
'''
def publisher_subscriber(): 
    #initilize the node
    rospy.init_node("STABILIZER")

    #read the SETPOINT_TOPIC to set the current setpoint
    rospy.Subscriber("SETPOINT_UNSTABLE", Float64MultiArray, setpoint_callback)
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