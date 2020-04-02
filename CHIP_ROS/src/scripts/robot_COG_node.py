#!/usr/bin/env python

'''
This node is both a publisher and a subscriber. It takes in data from
the communicator_node about the THETAS and the foot positions and outputs 
the CoG of the robot and the legs that are LOADED
'''
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from component import component
from robot import robot
import math
from DEFAULTS import DEFAULTS

'''
DEFINING SOME CONSTANTS
'''
PUB_RATE = 60 #60hz

'''
DEFINING STATIC COMPONENTS OF THE ROBOT & the ORIGIN as the IMU LOCATION
ALL UNITS ARE m, AND kgs
'''
ORIGIN = DEFAULTS.PLATFORM_ORIGIN
FRAME_MC = component(0.3075, 0.1525, 1.3596, ORIGIN)
FL_SHOULDER = component(0.0254, 0.0697, 0.76, ORIGIN)
BL_SHOULDER = component(0.585, 0.0697, 0.76, ORIGIN)
FR_SHOULDER = component(0.0254, 0.2353, 0.76, ORIGIN)
BR_SHOULDER = component(0.585, 0.2353, 0.76, ORIGIN)
PC_BUTTON = component(0.51, 0.1525, 0.8391459, ORIGIN)
ROBORIO = component(0.12, 0.16, 0.33, ORIGIN)
JETSON = component(0.29, 0.15, 0.33, ORIGIN)
IMU = component(0.29, 0.15, 0.1, ORIGIN)

'''
DEFINING QUANTITIES WE WILL NEED LATER
'''
R = 10.81665383/100.0
THETA0 = 0.9827937232

'''
DEFIINE VARIABLES TO PUBLISH
'''
CoG = []

'''
creating publisher up here so on callback we can REPUBLISH
'''
COG_PUB = rospy.Publisher('CoG', Float64MultiArray, queue_size=0) #default queue size is 0


#____________________________________________________________________________________________________________________________________________________________________
# HERE'S THE ACTUAL ROS NODE METHODS AND ETC.
#____________________________________________________________________________________________________________________________________________________________________


'''
define the callback function
NOTE: THIS IS SUBSCRIBING TO THETAS
'''
def callback(data):
    global CoG
    global COG_PUB

    #let's get the THETA_ONE's
    FLT1 = data.data[0]
    FRT1 = data.data[3]
    BLT1 = data.data[6]
    BRT1 = data.data[9]

    #CALCUALTE POSITIONS
    FLX = R*math.cos(FLT1+THETA0)+0.0254
    FLZ = -0.07
    BLX = R*math.cos(BLT1+THETA0)+0.585
    BLZ = -0.07
    FRX = R*math.cos(FRT1+THETA0)+0.0254
    FRZ = 0.375
    BRX = R*math.cos(BRT1+THETA0)+0.585
    BRZ = 0.375
    
    #CREATE NEW OBJECTS
    FLL = component(FLX, FLZ, 1.52, ORIGIN)
    FRL = component(FRX, FRZ, 1.52, ORIGIN)
    BLL = component(BLX, BLZ, 1.52, ORIGIN)
    BRL = component(BRX, BRZ, 1.52, ORIGIN)

    #CREATE ROBOT
    ROBOT_MODEL = [FRAME_MC, FL_SHOULDER, BL_SHOULDER, FR_SHOULDER, BR_SHOULDER, PC_BUTTON, ROBORIO, JETSON, IMU, FLL, FRL, BLL, BRL]
    this_robot = robot(ROBOT_MODEL)
    CoG = this_robot.findCoG()
    COG_PUB.publish(Float64MultiArray(data=CoG))


'''
This does the actual subscribing and calling the callback
'''
def publisher_subscriber(): 
    #initilize the node
    rospy.init_node("ROBOT_COG")
    #read the topic we're subscribing to and execute CALLBACK
    rospy.Subscriber("THETAS", Float64MultiArray, callback)

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