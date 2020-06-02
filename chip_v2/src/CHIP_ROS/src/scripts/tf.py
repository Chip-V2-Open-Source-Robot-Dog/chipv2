#!/usr/bin/env python

'''
This node is both a publisher and a subscriber. It basically takes
IMU input and publishes the orientation and the IMU tf.
'''

import rospy
from std_msgs.msg import Int16, Float64MultiArray
from sensor_msgs.msg import Imu
import math
import numpy as np
from simple_pid import PID
from DEFAULTS import DEFAULTS
from CONTROL_MODES import CONTROL_MODE
#vizualization imports
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

'''
DEFINE SOME GLOBAL VARIABLES
'''
DESIRED = DEFAULTS.DESIRED_RPY #FOR NOW we're keeping all angles as 0, roll-pitch-yaw

'''
DEFIINE VARIABLES TO PUBLISH
'''
ORI = [] #roll, pitch, yaw
QUART = []
MODE = CONTROL_MODE.STAND_SIT

'''
creating publisher up here so on callback we can REPUBLISH
'''
br = tf2_ros.TransformBroadcaster()
cube_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
marker_publisher = rospy.Publisher('vtext_marker', Marker, queue_size=5)
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
def mode_callback(data):
    global MODE
    #get the current setpoint and set it to that
    MODE = data.data

'''
gets the imu data, feeds it to the method and calaucaltes the euler angles, and uses that to set the position of the robot legs to compensate. 
'''
def callback(data):
    global ORI
    global QUART

    #get the quaternion
    w = data.orientation.w
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    vx = round(data.angular_velocity.x, 2)
    vy = round(data.angular_velocity.y, 2)
    vz = round(data.angular_velocity.z, 2)

    #generate the text marker for the IMU data
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=1,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.15, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.1, 0.1, 0.1),
                header=Header(frame_id='imu_frame'),
                color=ColorRGBA(255,255,255,1.0),
                text = "BACK OF ROBOT")
    marker_publisher.publish(marker)

    #generate the visualization marker for the IMU data
    cube = Marker(
                type=Marker.CUBE,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.0, 0.0, -0.1), Quaternion(x, y, z, w)),
                scale=Vector3(0.1, 0.1, 0.1),
                header=Header(frame_id='base_link_frd'),
                color=ColorRGBA(255,0,255,0.6))
    cube_publisher.publish(cube)

    #generate the TF visualization...
    t = geometry_msgs.msg.TransformStamped()
    #populate the geo_msg vector
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link_frd"
    t.child_frame_id = "imu_frame"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = -0.1
    t.transform.rotation.x = x
    t.transform.rotation.y = y
    t.transform.rotation.z = z
    t.transform.rotation.w = w
    #send the transform 
    br.sendTransform(t)

    #get the roll/pitch/yaw angles
    ORI = quaternion_to_euler(w, x, y, z)+[vx, vy, vz]
    ORI_PUB.publish(Float64MultiArray(data=ORI))

'''
This does the actual subscribing and calling the callback
'''
def publisher_subscriber(): 
    #initilize the node
    rospy.init_node("IMU_DATA_PUBLISHER")

    #read the SETPOINT_TOPIC to set the current setpoint
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