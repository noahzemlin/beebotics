#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import Twist, Vector3
import random

# set up the publisher
command_pub = None
# left sensor side
left_side = 10
# right sensor side
right_side = 10
# tolerance for how close symettric is
tolerance = 1 # Not sure what to set

velocity = Vector3(0,0,0)
turn_val = Vector3(0,0,0)

def get_laser_val(laser_cmd):
    global left_side, right_side
    # this is for sure wrong
    left_side = laser_cmd.ranges[0]
    right_side = laser_cmd.ranges[len(laser_cmd.ranges)-1]

def random_turn():
    rand_degree = random.randint(-15,15)
    return rand_degree

def send_command(timer_event):
    global velocity, turn_val
    # Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    if(left_side - right_side <= tolerance):
        turn_val = Vector3(0,0,3)
    # Avoid asymmetric obstacles within 1ft in front of the robot.
    elif(left_side > right_side):
        turn_val = Vector3(0,0,-3) 
    elif(right_side > left_side):
        turn_val = Vector3(0,0,3)
    # Turn randomly (uniformly sampled within +-15) after every 1ft of forward movement.
    turn_val = Vector3(0, 0, random_turn())
    # Drive forward.
    if turn_val == 0:
        velocity = Vector3(3,0,0) #What do i set velocity to for forward?

    w2g_cmd = Twist()
    w2g_cmd.linear = velocity
    w2g_cmd.angular = turn_val
    command_pub.publish(w2g_cmd)

def main():
    global command_pub
    rospy.init_node('planning_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, get_laser_val, queue_size=1)
    command_pub = rospy.Publisher("/bb/where2go", Twist, queue_size=1)

    # Set up a timer to update robot's drive state at 10 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), send_command)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

	
