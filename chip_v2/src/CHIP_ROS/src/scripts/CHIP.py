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
from TRAJECTORY import TRAJECTORY
from LEG_MODEL import MODEL as legModel
from CMA import CMA

'''
DEFINE SOME GLOBAL VARIABLES
'''

'''
DEIFNE GLOBAL PLATFORM CONTROL MODE, DEFAULTS TO STAND_SIT
'''
MODE = CONTROL_MODE.STAND_SIT
trajRunner = TRAJECTORY()
cmaCalc = CMA()
CURRENT_XYZ = []
POS = []
ORI = [0.0, 0.0, 0.0]

'''
DEFIINE VARIABLES TO PUBLISH
'''
LAST_PUB = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #THIS IS THE DEFAULT "HOME" we should change this to the current foot pose
STABLE = []

'''
creating publisher up here so on callback we can REPUBLISH
'''
#SET_PUB = rospy.Publisher('SETPOINT_UNSTABLE', Float64MultiArray, queue_size=0) #default queue size is 0
CMD_PUB = rospy.Publisher('CMDS', Float64MultiArray, queue_size=0) #default queue size is 0
MODE_PUB = rospy.Publisher('CONTROL_MODE', Int16, queue_size=0)

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

'''
This method takes in a xyz setpoint and outputs the CMDS
'''
def doIK(data):
    #recieve the XYZ data for a leg
    xFL = data[0]
    yFL = data[1]
    zFL = data[2]
    #perform inverse kinematics and get CMDS
    FLCMDS = calculateCMDS(inverseKinematics(xFL, yFL, zFL))

    #recieve the XYZ data for a leg
    xFR = data[3]
    yFR = data[4]
    zFR = data[5]
    #perform inverse kinematics and get CMDS
    FRCMDS = calculateCMDS(inverseKinematics(xFR, yFR, zFR))

    #recieve the XYZ data for a leg
    xBL = data[6]
    yBL = data[7]
    zBL = data[8]
    #perform inverse kinematics and get CMDS
    BLCMDS = calculateCMDS(inverseKinematics(xBL, yBL, zBL))

    #recieve the XYZ data for a leg
    xBR = data[9]
    yBR = data[10]
    zBR = data[11]
    #perform inverse kinematics and get CMDS
    BRCMDS = calculateCMDS(inverseKinematics(xBR, yBR, zBR))

    #populate the commands vector and return it
    return FLCMDS+FRCMDS+BLCMDS+BRCMDS

#____________________________________________________________________________________________________________________________________________________________________
# HERE'S THE ACTUAL ROS NODE METHODS AND ETC.
#____________________________________________________________________________________________________________________________________________________________________

#publishes the commands to the actuators to the NETWORK accounting for the zeroing-offsets. 
def publish_CMDS(CMDS):
    #add offsets and limit the outputs 
    ONE = clip(CMDS[0]+DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    TWO = clip(CMDS[1], -DEFAULTS.JL_L, DEFAULTS.JOINT_LIMIT)
    THREE = clip(CMDS[2]+DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    FOUR = clip(CMDS[3]+DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    FIVE = clip(CMDS[4], -DEFAULTS.JL_L, DEFAULTS.JOINT_LIMIT)
    SIX = clip(CMDS[5]+DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    SEVEN = clip(CMDS[6]+DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    EIGHT = clip(CMDS[7], -DEFAULTS.JL_L, DEFAULTS.JOINT_LIMIT)
    NINE = clip(CMDS[8]+DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    TEN = clip(CMDS[9]+DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    ELEVEN = clip(CMDS[10], -DEFAULTS.JL_L, DEFAULTS.JOINT_LIMIT)
    TWELVE = clip(CMDS[11]+DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    #populate and publish the vector
    CMDS_NEW = [ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE]
    CMD_PUB.publish(Float64MultiArray(data=CMDS_NEW))
    #update global variable
    global LAST_PUB
    LAST_PUB = CMDS

#handles getting the current actual platform RPY
def imu_callback(data):
    #set the current PITCH of the platform
    PITCH = data.data[1]
    vx = data.data[3]
    vy = data.data[4]
    #now do something with the pitch
    if (abs(PITCH) >= 0.12):
        #first let's set the control mode to stand-sit
        global MODE
        MODE = CONTROL_MODE.ERROR
        #then let's clear the trajectory and add some points
        COMMAND = [0.05, 0.2, 0.0, 0.05, 0.2, 0.0, 0.05, 0.2, 0.0, 0.05, 0.2, 0.0]
        trajRunner.clear(COMMAND)
        trajRunner.addWaypoint(doIK(COMMAND))
        trajRunner.addWaypoint([-8.0, 0.0, 0.0, -8.0, 0.0, 0.0, -8.0, 0.0, 0.0, -8.0, 0.0, 0.0])
        trajRunner.addWaypoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        trajRunner.addWaypoint(DEFAULTS.STARTING_CONFIG)
        #then it waits for human command to stand up again
    elif (abs(PITCH) >= 0.075 and not(MODE==CONTROL_MODE.ERROR)):
        #we DON'T NEED TO CHANGE THE CONTROL MODE this is NOT a fault condition! 
        #then let's clear the trajectory and publish the last stable command
        trajRunner.clear(STABLE)
    else:
        if(vx==0 and vy==0):
            global STABLE
            STABLE = LAST_PUB

    #now actually update the orientation variable
    global ORI
    ORI = data.data

#handles getting the current actual foot positions
def fp_callback(data):
    #set the current foot_pos
    global CURRENT_XYZ
    CURRENT_XYZ = data.data

    #tick the trajectory runner 
    ''' FOR NOW WE'RE ONLY USING TRAJECTORY so in ALL modes this happens'''
    publish_CMDS(trajRunner.tick(POS))

#handles getting the current actual foot positions IN THE USER COORD FRAME/not RAW FRAME
def pos_callback(data):
    #convert coordinate frames
    ONE = data.data[0]-DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET
    TWO = data.data[1]
    THREE = data.data[2]-DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET
    FOUR = data.data[3]-DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET
    FIVE = data.data[4]
    SIX = data.data[5]-DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET
    SEVEN = data.data[6]-DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET
    EIGHT = data.data[7]
    NINE = data.data[8]-DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET
    TEN = data.data[9]-DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET
    ELEVEN = data.data[10]
    TWELVE = data.data[11]-DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET
    #populate the vector
    POS_NEW = [ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE]
    #set the current foot_pos
    global POS
    POS = POS_NEW

#handles most of the main processes of the node
def joy_callback(data):
    #GET THE VALUES FROM THE JOYSTICK
    #1.0 is PRESSED
    #AXIS are all on a scale of -1.0 to 1.0
    RX = data.axes[2] #THIS WILL BE THE "STRAFE" OR THE SIDE/SIDE SPEED
    RY = data.axes[3] #THIS WILL BE THE "GATE" OR THE SPEED OF WALKING
    LX = data.axes[0] #THIS WILL BE THE ROTATIONAL VELOCITY, OR TURNING
    LY = data.axes[1]
    '''these axis will send change parameters'''
    RT = data.axes[4]
    LT = data.axes[5]
    '''these buttons will send specific commands'''
    POVX = data.axes[6]
    POVY = data.axes[7]
    LB = data.buttons[4]
    RB = data.buttons[5]
    '''these buttons will switch the platform into diff modes'''
    A = data.buttons[0]
    B = data.buttons[1]
    X = data.buttons[2]
    Y = data.buttons[3]

    if(A==1.0):
        trajRunner.clear(POS)
        global MODE
        MODE = CONTROL_MODE.WALK
    if(B==1.0):
        trajRunner.clear(POS)
        global MODE
        MODE = CONTROL_MODE.STAND_SIT
    if(X==1.0):
        trajRunner.clear(POS)
        global MODE
        MODE = CONTROL_MODE.DANCE
    if(Y==1.0):
        trajRunner.clear(POS)
        global MODE
        MODE = CONTROL_MODE.TEST

    #only if the mode is stand_sit mode
    if(MODE==CONTROL_MODE.STAND_SIT):
        #then publish stand/sit commands
        if(LB==1.0 and len(trajRunner.waypoints)==0):
            trajRunner.addWaypoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            trajRunner.addWaypoint([0.0, 0.0, 8.0, 0.0, 0.0, 8.0, 0.0, 0.0, 8.0, 0.0, 0.0, 8.0])
            trajRunner.addWaypoint([-9, 0.0, 10.0, -9, 0.0, 10.0, -9, 0.0, 10.0, -9, 0.0, 10.0])
            COMMAND = [0.02, 0.47, 0.0, 0.02, 0.47, 0.0, 0.02, 0.48, 0.0, 0.02, 0.48, 0.0]
            trajRunner.addWaypoint(doIK(COMMAND))
            COMMAND = [0.05, 0.5, 0.0, 0.05, 0.5, 0.0, 0.05, 0.5, 0.0, 0.05, 0.5, 0.0]
            trajRunner.addWaypoint(doIK(COMMAND))
        if(RB==1.0 and len(trajRunner.waypoints)==0):
            COMMAND = [0.02, 0.47, 0.0, 0.02, 0.47, 0.0, 0.02, 0.47, 0.0, 0.02, 0.47, 0.0]
            trajRunner.addWaypoint(doIK(COMMAND))
            COMMAND = [0.05, 0.2, 0.0, 0.05, 0.2, 0.0, 0.05, 0.2, 0.0, 0.05, 0.2, 0.0]
            trajRunner.addWaypoint(doIK(COMMAND))
            trajRunner.addWaypoint([-8.0, 0.0, 0.0, -8.0, 0.0, 0.0, -8.0, 0.0, 0.0, -8.0, 0.0, 0.0])
            trajRunner.addWaypoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            trajRunner.addWaypoint(DEFAULTS.STARTING_CONFIG)
    
    if(MODE==CONTROL_MODE.WALK):
        #then we're going to add the walking points
        #to the trajectory
        if(LB==1.0 and len(trajRunner.waypoints)==0):
            #test walking here
            GATE = 0.1
            COMMAND = doIK([0.05-0.07, 0.5, 0.0, 0.05-0.07, 0.5, 0.0, 0.05-0.07, 0.5, 0.0, 0.05-0.07, 0.5, 0.0])
            trajRunner.addWaypoint(COMMAND)
            COMMAND[2] = COMMAND[2]-15.0
            COMMAND[11] = COMMAND[11]-2.0
            trajRunner.addWaypoint(COMMAND)
        if(RB==1.0):
            #if i clear the trajectory "abort", send it to the last stable position
            trajRunner.clear(STABLE)

    if(MODE==CONTROL_MODE.DANCE):
        #will be written later
        pass

    if(MODE==CONTROL_MODE.TEST):
        #insert any TEST code here - mode is ONLY for testing
        pass

    #actually publish the setpoint
    MODE_PUB.publish(Int16(data=MODE))


'''
This does the actual subscribing and calling the callback
'''
def publisher_subscriber(): 
    #initilize the node
    rospy.init_node("CHIP_MASTER")
    #read the topic we're subscribing to and execute CALLBACK
    rospy.Subscriber("XYZ_CURRENT", Float64MultiArray, fp_callback)
    rospy.Subscriber("POS", Float64MultiArray, pos_callback)
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.Subscriber("ORIENTATION", Float64MultiArray, imu_callback)

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