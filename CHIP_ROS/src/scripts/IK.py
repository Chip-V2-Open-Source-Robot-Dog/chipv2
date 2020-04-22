#!/usr/bin/env python

'''
This node is both a publisher and a subscriber. It takes in the xyz
commands, and takes the leg model and converts the xyz commmands to
motor commands themselves. 
'''

import rospy
from std_msgs.msg import Float64MultiArray
import math
from CONTROL_MODES import CONTROL_MODE
from DEFAULTS import DEFAULTS
from LEG_MODEL import MODEL as legModel

'''
DEFINE SOME GLOBAL VARIABLES
'''

'''
DEIFNE GLOBAL PLATFORM CONTROL MODE, DEFAULTS TO STAND_SIT
'''
MODE = CONTROL_MODE.STAND_SIT

'''
DEFIINE VARIABLES TO PUBLISH
'''
SET_TO_PUB = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] #THIS IS THE DEFAULT "HOME" we should change this to the current foot pose

'''
creating publisher up here so on callback we can REPUBLISH
'''
#SET_PUB = rospy.Publisher('SETPOINT_UNSTABLE', Float64MultiArray, queue_size=0) #default queue size is 0
if(DEFAULTS.IMU_ON):
    SET_PUB = rospy.Publisher('CMDS_RAW', Float64MultiArray, queue_size=0) #default queue size is 0
else:
    SET_PUB = rospy.Publisher('CMDS', Float64MultiArray, queue_size=0)

#____________________________________________________________________________________________________________________________________________________________________
# THESE METHODS ARE HELPER METHODS
#____________________________________________________________________________________________________________________________________________________________________

"""Bounds the value input between lower and upper, known limits of the system, think of them as virtual hardstops"""
def clip(value, lower, upper):
    if (value>=upper):
        return upper
    if (value<=lower):
        return lower
    return value

"""actual inverse kinematic model/formula"""
def inverseKinematics(xD, yD, zD):
    l1 = legModel.L3
    l2 = legModel.L6

    rSquared = xD*xD+yD*yD

    #setup limits for rsquared and z
    rSquared = clip(rSquared, (l1-l2)*(l1-l2), (l1+l2)*(l1+l2))
    zD = clip(zD, -0.1, 0.1)

    print(rSquared)
    print(zD)

    phi = math.atan2(yD, xD)
    kneeTheta = math.acos((rSquared-l1*l1-l2*l2)/(-2.0*l1*l2))
    alpha = math.acos((l2*l2-l1*l1-rSquared)/(-2.0*l1*math.sqrt(rSquared)))
    shoulderTheta = phi-alpha
    hingeTheta = math.asin(zD/yD)

    return [shoulderTheta, hingeTheta, kneeTheta]

"""calcualte the commands to send"""
def calculateCMDS(thetas):
    return [thetas[0] * 100.0/(2*math.pi), thetas[1] * 100.0/(2*math.pi), thetas[2] * 100.0/(2*math.pi)]

#____________________________________________________________________________________________________________________________________________________________________
# HERE'S THE ACTUAL ROS NODE METHODS AND ETC.
#____________________________________________________________________________________________________________________________________________________________________

'''
This is the calabck function that does things.
'''
def callback(data):
    #recieve the XYZ data for a leg
    xFL = data.data[0]
    yFL = data.data[1]
    zFL = data.data[2]
    #perform inverse kinematics and get CMDS
    FLCMDS = calculateCMDS(inverseKinematics(xFL, yFL, zFL))

    #recieve the XYZ data for a leg
    xFR = data.data[3]
    yFR = data.data[4]
    zFR = data.data[5]
    #perform inverse kinematics and get CMDS
    FRCMDS = calculateCMDS(inverseKinematics(xFR, yFR, zFR))

    #recieve the XYZ data for a leg
    xBL = data.data[6]
    yBL = data.data[7]
    zBL = data.data[8]
    #perform inverse kinematics and get CMDS
    BLCMDS = calculateCMDS(inverseKinematics(xBL, yBL, zBL))

    #recieve the XYZ data for a leg
    xBR = data.data[9]
    yBR = data.data[10]
    zBR = data.data[11]
    #perform inverse kinematics and get CMDS
    BRCMDS = calculateCMDS(inverseKinematics(xBR, yBR, zBR))

    #populate the commands vector
    SET_TO_PUB = FLCMDS+FRCMDS+BLCMDS+BRCMDS
    #actually publish the setpoint
    SET_PUB.publish(Float64MultiArray(data=SET_TO_PUB))


'''
This does the actual subscribing and calling the callback
'''
def publisher_subscriber(): 
    #initilize the node
    rospy.init_node("IK_NODE")
    #read the topic we're subscribing to and execute CALLBACK
    rospy.Subscriber("XYZ", Float64MultiArray, callback)

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