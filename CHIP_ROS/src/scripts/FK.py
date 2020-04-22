#!/usr/bin/env python

'''
This node is both a publisher and a subscriber. It takes in the pos
from the platform, and takes the leg model and converts pos to
current XYZ foot positions.
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
SET_PUB = rospy.Publisher('XYZ_CURRENT', Float64MultiArray, queue_size=0) #default queue size is 0

#____________________________________________________________________________________________________________________________________________________________________
# THESE METHODS ARE HELPER METHODS
#____________________________________________________________________________________________________________________________________________________________________

"""calcualte the thetas from the positions"""
def calculateTHETAS(cmds):
    return [cmds[0] * (2*math.pi)/100.0, cmds[1] * (2*math.pi)/100.0, cmds[2] * (2*math.pi)/100.0]

"""actual inverse kinematic model/formula"""
def forwardsKinematics(thetas):
    #define some needed variables
    l1 = legModel.L3
    l2 = legModel.L6
    t1 = thetas[0]
    t2 = thetas[1]
    t3 = thetas[2]
    #caluclate X and Y
    xC = l1*math.cos(t1)-l2*math.cos(t3-t1)
    yC = l1*math.sin(t1)+l2*math.sin(t3-t1)
    #calculate Z
    zC = yC*math.sin(t2)
    #return
    return [xC, yC, zC]

#____________________________________________________________________________________________________________________________________________________________________
# HERE'S THE ACTUAL ROS NODE METHODS AND ETC.
#____________________________________________________________________________________________________________________________________________________________________

'''
This is the calabck function that does things.
'''
def callback(data):
    #recieve the XYZ data for a leg
    sFL = data.data[0]
    hFL = data.data[1]
    kFL = data.data[2]
    #perform inverse kinematics and get CMDS
    FL = forwardsKinematics(calculateTHETAS([sFL, hFL, kFL]))

    #recieve the XYZ data for a leg
    sFR = data.data[3]
    hFR = data.data[4]
    kFR = data.data[5]
    #perform inverse kinematics and get CMDS
    FR = forwardsKinematics(calculateTHETAS([sFR, hFR, kFR]))

    #recieve the XYZ data for a leg
    sBL = data.data[6]
    hBL = data.data[7]
    kBL = data.data[8]
    #perform inverse kinematics and get CMDS
    BL = forwardsKinematics(calculateTHETAS([sBL, hBL, kBL]))

    #recieve the XYZ data for a leg
    sBR = data.data[9]
    hBR = data.data[10]
    kBR = data.data[11]
    #perform inverse kinematics and get CMDS
    BR = forwardsKinematics(calculateTHETAS([sBR, hBR, kBR]))

    #populate the commands vector
    SET_TO_PUB = FL+FR+BL+BR
    #actually publish the setpoint
    SET_PUB.publish(Float64MultiArray(data=SET_TO_PUB))


'''
This does the actual subscribing and calling the callback
'''
def publisher_subscriber(): 
    #initilize the node
    rospy.init_node("FK_NODE")
    #read the topic we're subscribing to and execute CALLBACK
    rospy.Subscriber("POS", Float64MultiArray, callback)

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