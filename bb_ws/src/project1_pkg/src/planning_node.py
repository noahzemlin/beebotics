#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import Odometry
import random

# set up the publishers
command_pub = None

# left and right LaserScan sensor values
left_side = 10
right_side = 10

# last odom update
odom = Odometry()

# error used to check if two dists are appx equal
tolerance = 0.05 # meters

# dist to get in symmetric case before "escaping"
escape_dist = 0.3048 # 1 ft in meters

# max differnece in headings to consider same heading (degrees)
angle_diff_threshold = 0.5

# make range in which to escape
escape_range = (escape_dist - tolerance, escape_dist + tolerance)

# targets
target_heading = 0
from_position = Vector3()

def get_laser_val(laser_cmd):
    global left_side, right_side
    # this is for sure wrong
    right_side = laser_cmd.ranges[0]
    left_side = laser_cmd.ranges[len(laser_cmd.ranges)-1]

def random_turn():
    rand_degree = random.randint(-15,15)
    return rand_degree

def odom_callback(data):
    global odom
    odom = data

# https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def publish_w2g_cmd(velocity, turn_val):
    w2g_cmd = Twist()
    w2g_cmd.linear = velocity
    w2g_cmd.angular = turn_val
    command_pub.publish(w2g_cmd)

# returns degrees (0 "north" to 360, CW)
def get_current_heading():
    odomn = odom.pose.pose.orientation
    rads = quaternion_to_euler(odomn.x, odomn.y, odomn.z, odomn.w)[0]
    return math.degrees(rads) + 180

# https://github.com/SoonerRobotics/igvc_software_2020/blob/master/igvc_ws/src/igvc_nav/src/igvc_nav_node.py
def get_angle_diff(angle1, angle2):
    delta = angle1 - angle2
    delta = (delta + 180) % 360 - 180
    return delta

def send_command(timer_event):
    global target_heading, from_position

    p_error = (get_current_heading() - target_heading) * -0.07
    if abs(p_error) > 0.1:
        p_error = math.copysign(0.1, p_error)
    turn_val = Vector3(0,0,p_error) # shitty p controller
    velocity = Vector3(0,0,0)

    #print("left_side: " + str(left_side) + ", right_side: " + str(right_side))

    #print("curr: " +str(get_current_heading()) + ", targ: " + str(target_heading) + " diff: " + str(get_angle_diff(get_current_heading(), target_heading)))


    if abs(get_angle_diff(get_current_heading(), target_heading)) > angle_diff_threshold:
        publish_w2g_cmd(velocity, turn_val)
        from_position = odom.pose.pose.position
        return

    # create some useful boolean values
    left_in_range = math.isnan(left_side)
    right_in_range = math.isnan(right_side)

    velocity = Vector3(0,0,0)
    turn_val = Vector3(0,0,0)
    # escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    if left_in_range and right_in_range:
        print("caught in corner")
        # both sensors see something about 1 ft away, so escape (fixed action)
        target_heading = (get_current_heading() + 180) % 360
    # avoid asymmetric obstacles within 1ft in front of the robot.
    elif left_in_range:
        #print("left too close")
        # only left sensor sees obstacle, so avoid by turning right (reflex)
        target_heading = (get_current_heading() + 30) % 360
    elif right_in_range:
        #print("right too close")
        # only right sensor sees obstacle, so avoid by turning left (reflex)
        target_heading = (get_current_heading() - 30) % 360
    else: 
        # no obstacles within range, so drive forward 1ft
        velocity = Vector3(0.3,0,0)

        cur_pos = odom.pose.pose.position
        distance = (cur_pos.x - from_position.x)**2 + (cur_pos.y - from_position.y)**2

        if distance >= escape_dist**2:
            velocity = Vector3(0,0,0)
            target_heading = (get_current_heading() + random.randint(-15, 15)) % 360

    publish_w2g_cmd(velocity, turn_val)

def main():
    global command_pub
    # initialize node
    rospy.init_node('planning_node', anonymous=True)
    # subscribe to laser scan data
    rospy.Subscriber("/scan", LaserScan, get_laser_val, queue_size=1)
    # subscribe to odom
    rospy.Subscriber("/odom", Odometry, odom_callback)
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

	
