#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import Twist, Vector3
import random

# set up the publishers
command_pub = None
# left and right LaserScan sensor values
left_side = 10
right_side = 10
# error used to check if two dists are appx equal
tolerance = 0.05 # meters
# dist to get in symmetric case before "escaping"
escape_dist = 0.3048 # 1 ft in meters
# make range in which to escape
escape_range = (escape_dist - tolerance, escape_dist + tolerance)

def get_laser_val(laser_cmd):
    global left_side, right_side
    # this is for sure wrong
    left_side = laser_cmd.ranges[0]
    right_side = laser_cmd.ranges[len(laser_cmd.ranges)-1]

def random_turn():
    rand_degree = random.randint(-15,15)
    return rand_degree

def publish_w2g_cmd(velocity, turn_val):
    w2g_cmd = Twist()
    w2g_cmd.linear = velocity
    w2g_cmd.angular = turn_val
    command_pub.publish(w2g_cmd)

def send_command(timer_event):
    # create some useful boolean values
    left_in_range = left_side > escape_range[0] and left_side < escape_range[1] 
    right_in_range = right_side > escape_range[0] and right_side < escape_range[1] 

    # escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    if left_in_range and right_in_range:
        # both sensors see something about 1 ft away, so escape (fixed action)
        turn_val = Vector3(0,0,3)
        velocity = Vector3(0,0,0)
        publish_w2g_cmd(velocity, turn_val)
        #TODO may need to adjust this sleep time to get correct turn amount
        rospy.sleep(2) #wait for turn to take place, then stop turning
        turn_val = Vector3(0,0,0)
        publish_w2g_cmd(velocity, turn_val)
    # avoid asymmetric obstacles within 1ft in front of the robot.
    elif left_in_range:
        # only left sensor sees obstacle, so avoid by turning right (reflex)
        turn_val = Vector3(0,0,-3)
        velocity = Vector3(0,0,0)
        publish_w2g_cmd(velocity, turn_val)
    elif right_in_range:
        # only right sensor sees obstacle, so avoid by turning left (reflex)
        turn_val = Vector3(0,0,3)
        velocity = Vector3(0,0,0)
        publish_w2g_cmd(velocity, turn_val)
    else: 
        # no obstacles within range, so drive forward 1ft
        turn_val = Vector3(0,0,0)
        velocity = Vector3(3,0,0)
        publish_w2g_cmd(velocity, turn_val)
        #TODO may need to adjust this sleep time to make robot go forward ~1ft
        rospy.sleep(2) #wait for movement to take place, then stop and turn randomly
        velocity = Vector3(0,0,0)
        # Turn randomly (uniformly sampled within +-15) after every 1ft of forward movement.
        turn_val = Vector3(0, 0, random_turn())
        publish_w2g_cmd(velocity, turn_val)
        #TODO may need to change this logic or the scaling factor to get a better +-15 degree turn spread
        rospy.sleep(abs(random_turn)/5)

def main():
    global command_pub
    # initialize node
    rospy.init_node('planning_node', anonymous=True)
    # subscribe to laser scan data
    rospy.Subscriber("/scan", LaserScan, get_laser_val, queue_size=1)
    # publish drive command
    command_pub = rospy.Publisher("/bb/where2go", Twist, queue_size=1)
    # Set up a timer to update robot's drive state at 10 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), send_command)
    # cycle through callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

	
