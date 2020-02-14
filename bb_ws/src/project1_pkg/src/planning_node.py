#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_geometry.laser_geometry as lg
import geometry_msgs
import random

# set up the publisher
command_pub = None
# left sensor side
left_side = None
# right sensor side
right_side = None
# tolerance for how close symettric is
tolerance = None # Not sure what to set

veloicty = None
turn_val = None

def get_laser_val(laser_cmd):
    # this is for sure wrong
    left_side = ranges[0]
    right_side = ranges[-1]

def random_turn(self):
	rand_degree = randint(-15,15)
    return rand_degree

def send_command(timer_event):
    w2g_cmd = Twist()
    w2g_cmd.linear = velocity
    w2g_cmd.angular = turn_val

def main():
    rospy.init_node('planning_node', anonymous=True)
	rospy.Subscriber("/scan", LaserScan, get_laser_val, queue_sizes=1)
	command_pub = rospy.Publisher("/bb/where2go", Twist, queue_size=1)
    # Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    if(left_side - right_side <= tolerance):
        turn_val = 180
    # Avoid asymmetric obstacles within 1ft in front of the robot.
    elif(left_side > right_side):
        turn_val = 15 
    elif(right_side > left_side):
        turn_val = 15
    # Turn randomly (uniformly sampled within ±15°) after every 1ft of forward movement.
    turn_val = random_turn()
    # Drive forward.
    if(turn_val == 0)
        velocity = 100 #What do i set velocity to for forward?

    # Set up a timer to update robot's drive state at 10 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), send_command)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

	
