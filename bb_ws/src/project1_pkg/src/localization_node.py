#!/usr/bin/env python

import rospy
from math import degrees
import tf
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import random

## Global Variables
hdg_pub = None
pos_pub = None


def odom_callback(data):
    global odom
    odom = data
    send_current_heading()
    send_current_position()

def send_current_heading():
    # returns degrees (0 "north" to 360, CW)
    orientation = odom.pose.pose.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    yaw_rads = tf.transformations.euler_from_quaternion(quaternion)[2]

    cur_hdg = Float32()
    cur_hdg.data = degrees(yaw_rads) + 180
    hdg_pub.publish(cur_hdg)


def send_current_position():
    position = odom.pose.pose.position
    cur_pos = Point()
    cur_pos.x = position.x
    cur_pos.y = position.y
    pos_pub.publish(cur_pos)


def main():
    global hdg_pub, pos_pub
    # initialize node
    rospy.init_node('localization_node', anonymous=True)

    # subscribe to odom
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    # publish current heading and position
    hdg_pub = rospy.Publisher("/bb/hdg", Float32, queue_size=1)
    pos_pub = rospy.Publisher("/bb/pos", Point, queue_size=1)

    # cycle through callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
