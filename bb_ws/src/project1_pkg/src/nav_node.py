#!/usr/bin/env python

import rospy
from math import isnan, isinf, copysign
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import random

# -- Configuration variables --

# Feet to meters conversion
ft_to_m = 0.3048  # 1 ft in meters

# Min feet from obstacle until we avoid (feet)
min_distance = 2.5

# Max difference in headings to consider same heading (degrees)
angle_diff_threshold = 3

# Heading P-Controller proportional gain
heading_kp = 0.07

# Forward speed (m/s)
forward_speed = 0.5

# -- Define global variables --

# set up the publisher
command_pub = None

# left and right LaserScan sensor values
left_side = 10
right_side = 10

# robot's current and desired heading
current_heading = 0
target_heading = 0

# from planning_node (both Float32)
plan_hdg = 0
plan_speed = 0


def nan_to_inf(val):
    if isnan(val):
        return float('Inf')
    else:
        return val


def get_laser_val(laser_cmd):
    global left_side, right_side
    middle_index = int(len(laser_cmd.ranges)/2)

    # Find the smallest value for each side.
    right_side = min(laser_cmd.ranges[0:middle_index], key=nan_to_inf)
    left_side = min(laser_cmd.ranges[middle_index:len(
        laser_cmd.ranges)-1], key=nan_to_inf)

    # If the whole scan is NaN, just say we are 0 distance so we turn
    if isinf(right_side):
        right_side = 0

    if isinf(left_side):
        left_side = 0


def random_turn():
    rand_degree = random.randint(-15, 15)
    return rand_degree


def publish_w2g_cmd(linear_velocity, angular_velocity):
    w2g_cmd = Twist()
    w2g_cmd.linear = linear_velocity
    w2g_cmd.angular = angular_velocity
    command_pub.publish(w2g_cmd)


def receive_heading(hdg):
    global current_heading
    current_hading = hdg.data


def get_angle_diff(angle1, angle2):
    # from github.com/SoonerRobotics/igvc_software_2020/blob/master/igvc_ws/src/igvc_nav/src/igvc_nav_node.py
    delta = angle1 - angle2
    delta = (delta + 180) % 360 - 180
    return delta


def get_planned_path(plan_twist):
    # receive info from planning_node
    global plan_hdg, plan_speed
    plan_hdg = plan_twist.angular.z
    plan_speed = plan_twist.linear.x

def send_command(timer_event):
    global target_heading

    # Scuffed PID controller, just the P term
    p_error = get_angle_diff(current_heading, target_heading) * heading_kp
    if abs(p_error) > 1.25:  # Cap the p_error from turning the robot too fast
        p_error = copysign(1.25, p_error)

    # Define velocities for Twist
    angular_velocity = Vector3(0, 0, -p_error)
    linear_velocity = Vector3(0, 0, 0)

    # Don't allow movement unless we are facing the way we want
    if abs(get_angle_diff(current_heading, target_heading)) > angle_diff_threshold:
        publish_w2g_cmd(linear_velocity, angular_velocity)
        return

    # Test if we are triggering the sensors by if they are NaN or less than min_distance
    left_in_range = isnan(left_side) or left_side <= min_distance * ft_to_m
    right_in_range = isnan(right_side) or right_side <= min_distance * ft_to_m

    # escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    if left_in_range and right_in_range:
        # both sensors see something about 1 ft away, so escape (fixed action)
        # TODO maybe change this behavior?
        target_heading = (current_heading + 180) % 360
    # avoid asymmetric obstacles within 1ft in front of the robot.
    elif left_in_range:
        # only left sensor sees obstacle, so avoid by turning right 20 degrees (reflex)
        target_heading = (current_heading - 20) % 360
    elif right_in_range:
        # only right sensor sees obstacle, so avoid by turning left 20 degrees (reflex)
        target_heading = (current_heading + 20) % 360
    else:
        # Follow the planned path
        target_heading = plan_hdg
        linear_velocity = Vector3(plan_speed, 0, 0)

    publish_w2g_cmd(linear_velocity, Vector3(0, 0, 0))


def main():
    global command_pub
    # initialize node
    rospy.init_node('nav_node', anonymous=True)

    # subscribe to laser scan data
    rospy.Subscriber("/scan", LaserScan, get_laser_val, queue_size=1)
    # subscribe to robot's current heading
    rospy.Subscriber("/bb/hdg", Float32, receive_heading)
    # subscribe to command from path planner
    rospy.Subscriber("/bb/path_cmd", Twist, get_planned_path, queue_size=1)

    # publish drive command
    command_pub = rospy.Publisher("/bb/where2go", Twist, queue_size=1)
    
    # Set up a timer to update robot's drive state at 10 Hz
    rospy.Timer(rospy.Duration(secs=0.05), send_command)
    # cycle through callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
