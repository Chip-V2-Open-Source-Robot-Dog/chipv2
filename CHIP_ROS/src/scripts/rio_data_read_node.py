#!/usr/bin/env python

'''
This node is both a publisher and a subscriber that will eventually 
read information from the ROS system and send it to the RIO as 
well as read information from the RIO and send it to the jetson. 

PUBLISHED TOPICS:
FOOT_POSITIONS - Float64[] ARRAY OF FOOT POSITIONS XYZ
THETAS = Float64[] ARRAY OF THETAS FOR EACH LEG
LEG_CURRENTS = Float64[] ARRAY OF TOTAL CURRENT IN EACH LEG
DEAFULT_VALUE Float64 that tells the system what values means 
there is no communication with RIO

MIGHT WANT TO THINK ABOUT SPLITITNG THIS INTO TWO NODES, A PUBLISHER AND A SUBSCRIBER
'''

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from networktables import NetworkTables as nt
from DEFAULTS import DEFAULTS

'''
SOME INITIALIZATION CODE
'''
#start network tables server
nt.initialize(server=DEFAULTS.RIO_IP)
#initialize tables
readout=nt.getTable("rio_readout")

'''
DEFINE SOME CLASS VARIABLES
'''
FP_IN = []
THETAS_IN = []
CURRENTS_IN = []

'''
DEFINING SOME CONSTANTS
'''
DEFAULT_VAL = DEFAULTS.NT_DEFAULT_VAL
PUB_RATE = 60 #60hz

#____________________________________________________________________________________________________________________________________________________________________
# THESE METHODS ARE NON-ROS ROBORIO SPECIFIC FUNCTIIONS FOR COMMUNICATIONS
#____________________________________________________________________________________________________________________________________________________________________

'''
This method gets the information from the RIO and organizes it in the arrays/nested arrays
'''
def read_rio():
    #GET GLOBAL VARS
    global FP_IN
    global THETAS_IN
    global CURRENTS_IN

    '''
    FOOT POSITIONS GETTING
    '''
    #get the readouts from the RIO
    #FL_P = [readout.getNumber("fL_x", DEFAULT_VAL), readout.getNumber("fL_y", DEFAULT_VAL), readout.getNumber("fL_z", DEFAULT_VAL)]
    #FR_P = [readout.getNumber("fR_x", DEFAULT_VAL), readout.getNumber("fR_y", DEFAULT_VAL), readout.getNumber("fR_z", DEFAULT_VAL)]
    #BL_P = [readout.getNumber("bL_x", DEFAULT_VAL), readout.getNumber("bL_y", DEFAULT_VAL), readout.getNumber("bL_z", DEFAULT_VAL)]
    #BR_P = [readout.getNumber("bR_x", DEFAULT_VAL), readout.getNumber("bR_y", DEFAULT_VAL), readout.getNumber("bR_z", DEFAULT_VAL)]
    #put them in order of LEG, FL, FR, BL, BR
    FP_IN = [readout.getNumber("fL_x", DEFAULT_VAL), readout.getNumber("fL_y", DEFAULT_VAL), readout.getNumber("fL_z", DEFAULT_VAL), readout.getNumber("fR_x", DEFAULT_VAL), readout.getNumber("fR_y", DEFAULT_VAL), readout.getNumber("fR_z", DEFAULT_VAL), readout.getNumber("bL_x", DEFAULT_VAL), readout.getNumber("bL_y", DEFAULT_VAL), readout.getNumber("bL_z", DEFAULT_VAL), readout.getNumber("bR_x", DEFAULT_VAL), readout.getNumber("bR_y", DEFAULT_VAL), readout.getNumber("bR_z", DEFAULT_VAL)]

    '''
    GETTING THETAS
    '''
    #read in oder of shoulder, hinge, knee
    #FL_T = [readout.getNumber("fL_shoulderTheta", DEFAULT_VAL), readout.getNumber("fL_hingeTheta", DEFAULT_VAL), readout.getNumber("fL_kneeTheta", DEFAULT_VAL)]
    #FR_T = [readout.getNumber("fR_shoulderTheta", DEFAULT_VAL), readout.getNumber("fR_hingeTheta", DEFAULT_VAL), readout.getNumber("fR_kneeTheta", DEFAULT_VAL)]
    #BL_T = [readout.getNumber("bL_shoulderTheta", DEFAULT_VAL), readout.getNumber("bL_hingeTheta", DEFAULT_VAL), readout.getNumber("bL_kneeTheta", DEFAULT_VAL)]
    #BR_T = [readout.getNumber("bR_shoulderTheta", DEFAULT_VAL), readout.getNumber("bR_hingeTheta", DEFAULT_VAL), readout.getNumber("bR_kneeTheta", DEFAULT_VAL)]
    #put them in order of LEG, FL, FR, BL, BR
    THETAS_IN = [readout.getNumber("fL_shoulderTheta", DEFAULT_VAL), readout.getNumber("fL_hingeTheta", DEFAULT_VAL), readout.getNumber("fL_kneeTheta", DEFAULT_VAL), readout.getNumber("fR_shoulderTheta", DEFAULT_VAL), readout.getNumber("fR_hingeTheta", DEFAULT_VAL), readout.getNumber("fR_kneeTheta", DEFAULT_VAL), readout.getNumber("bL_shoulderTheta", DEFAULT_VAL), readout.getNumber("bL_hingeTheta", DEFAULT_VAL), readout.getNumber("bL_kneeTheta", DEFAULT_VAL), readout.getNumber("bR_shoulderTheta", DEFAULT_VAL), readout.getNumber("bR_hingeTheta", DEFAULT_VAL), readout.getNumber("bR_kneeTheta", DEFAULT_VAL)]

    '''
    GETTING CURRENTS
    '''
    #just get and add because it's total currents
    CURRENTS_IN = [readout.getNumber("fL_current", DEFAULT_VAL), readout.getNumber("fR_current", DEFAULT_VAL), readout.getNumber("bL_current", DEFAULT_VAL), readout.getNumber("bR_current", DEFAULT_VAL)]


#____________________________________________________________________________________________________________________________________________________________________
# HERE'S THE ACTUAL ROS NODE METHODS AND ETC.
#____________________________________________________________________________________________________________________________________________________________________

'''
This is a ROS SPECIFIC node that publishes the data at a certain rate
'''
def publisher():
    #CREATE THE PUBLISHERS
    POS_PUBLISHER = rospy.Publisher('FOOT_POSITIONS', Float64MultiArray, queue_size=0) #default queue size is 0
    THETAS_PUBLISHER = rospy.Publisher('THETAS', Float64MultiArray, queue_size=0) #default queue size is 0
    CURRENTS_PUBLISHER = rospy.Publisher('LEG_CURRENTS', Float64MultiArray, queue_size=0) #default queue size is 0

    #initilize the node
    rospy.init_node("RIO_READER")
    #set the publishing and reading rate
    rate = rospy.Rate(PUB_RATE) #in Hz

    while not rospy.is_shutdown():
        #NOW WE READ DATA FROM THE RIO
        read_rio()
        #WRITE DATA NOW THAT read_rio has modified it
        POS_PUBLISHER.publish(Float64MultiArray(data=FP_IN))
        THETAS_PUBLISHER.publish(Float64MultiArray(data=THETAS_IN))
        CURRENTS_PUBLISHER.publish(Float64MultiArray(data=CURRENTS_IN))
        #sleep for the pub rate
        rate.sleep()

'''
not specific to ROS, this is the MAIN method, run this
'''
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

