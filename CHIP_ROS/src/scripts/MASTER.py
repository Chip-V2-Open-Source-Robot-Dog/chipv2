#!/usr/bin/env python

'''
This node is both a publisher and a subscriber. It is the most
complicated node in the ROS system, it takes in joystick input,
the model, and messages from MAVROS and sends commands to the 
platform.
'''

import rospy
from std_msgs.msg import Float64, Float64MultiArray, Bool, Int16
from sensor_msgs.msg import Joy
import math
from CONTROL_MODES import CONTROL_MODE
from DEFAULTS import DEFAULTS

'''
DEFINE SOME GLOBAL VARIABLES
'''
#SETTING SOME CONSERVATIVE LIMITS FOR THE XYZ AXIS (in meters)
Y_LIMIT_LOWER = DEFAULTS.Y_LIMIT_LOWER
Y_LIMIT_UPPER = DEFAULTS.Y_LIMIT_UPPER
X_LIMIT_LOWER = DEFAULTS.X_LIMIT_LOWER
X_LIMIT_UPPER = DEFAULTS.X_LIMIT_UPPER
Z_LIMIT_LOWER = DEFAULTS.Z_LIMIT_LOWER
Z_LIMIT_UPPER = DEFAULTS.Z_LIMIT_UPPER

'''
DEIFNE GLOBAL PLATFORM CONTROL MODE, DEFAULTS TO STAND_SIT
'''
MODE = CONTROL_MODE.STAND_SIT

'''
DEFIINE VARIABLES TO PUBLISH
'''
SET_TO_PUB = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] #THIS IS THE DEFAULT "HOME" we should change this to the current foot pose
ENABLE = False

'''
creating publisher up here so on callback we can REPUBLISH
'''
#SET_PUB = rospy.Publisher('SETPOINT_UNSTABLE', Float64MultiArray, queue_size=0) #default queue size is 0
if(DEFAULTS.IMU_ON):
    SET_PUB = rospy.Publisher('CMDS_RAW', Float64MultiArray, queue_size=0) #default queue size is 0
else:
    SET_PUB = rospy.Publisher('CMDS', Float64MultiArray, queue_size=0)
#above quickly sets if we use IMU
MODE_PUB = rospy.Publisher('CONTROL_MODE', Int16, queue_size=0)

#____________________________________________________________________________________________________________________________________________________________________
# THESE METHODS ARE HELPER METHODS
#____________________________________________________________________________________________________________________________________________________________________

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


def joy_callback(data):
    #GET THE VALUES FROM THE JOYSTICK
    #1.0 is PRESSED
    #AXIS are all on a scale of -1.0 to 1.0
    RX = data.axes[2]
    RY = data.axes[3]
    LX = data.axes[0]
    LY = data.axes[1]
    RT = data.axes[4]
    LT = data.axes[5]
    POVX = data.axes[6]
    POVY = data.axes[7]
    LB = data.buttons[6]
    RB = data.buttons[7]
    A = data.buttons[0]
    B = data.buttons[1]
    X = data.buttons[3]
    Y = data.buttons[4]

    SET_MAG = 5.0

    if (LB == 1.0):
        global SET_TO_PUB
        SET_TO_PUB[2] = SET_MAG

    #actually publish the setpoint
    SET_PUB.publish(Float64MultiArray(data=SET_TO_PUB))


'''
This does the actual subscribing and calling the callback
'''
def publisher_subscriber(): 
    #initilize the node
    rospy.init_node("CHIP_MASTER")
    #read the topic we're subscribing to and execute CALLBACK
    rospy.Subscriber("joy", Joy, joy_callback)

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