#!/usr/bin/env python

import rospy
import pickle
import os
import numpy as np
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry, OccupancyGrid

# -- Configuration variables --

# How long we let keyboard send zero-magnitude Twists (seconds)
keyboard_timeout = 1

# Proprtional speed gain
speed_kp = 0.05

# -- Define global variables --

# Reference to mobile_base velocity publisher
command_pub = None

# Last non-zero Keyboard message
last_keyboard_time = 0

# The most recent keyboard command
key_cmd = Twist()

# The most recent bumper state
bumped = False

# The most recent nav command
mav_cmd = Twist()

# Last odom update
odom = Odometry()

# Control speed forward
control_speed = 0

nav_cmd = None


def odom_callback(data):
    global odom
    odom = data


def get_key_cmd(key_msg):
    global key_cmd, last_keyboard_time
    key_cmd = key_msg

    if not is_zero_twist(key_cmd):
        last_keyboard_time = rospy.get_time()


def get_bumper_state(bumper_state):
    global bumped
    bumped = bumped or bumper_state.state == 1


def get_nav_cmd(nav_msg):
    global nav_cmd
    nav_cmd = nav_msg


def is_zero_twist(twisty):
    return (twisty.linear.x == twisty.linear.y == twisty.linear.z == twisty.angular.x == twisty.angular.y == twisty.angular.z == 0)


def send_command(timer_event):
    global control_speed

    if bumped:
        # Bumper has been triggered, so halt by sending the zero twist
        halt_cmd = Twist()
        command_pub.publish(halt_cmd)
        return

    if rospy.get_time() - last_keyboard_time < keyboard_timeout:
        # Keyboard command is being received and should be passed through
        command_pub.publish(key_cmd)
    elif nav_cmd is not None:
        # There is no keyboard command being received, use our reactive command

        # Apply proportional speed gain
        control_speed += (nav_cmd.linear.x - odom.twist.twist.linear.x) * speed_kp
        nav_cmd.linear.x = control_speed

        command_pub.publish(nav_cmd)

first_map = True

def main():
    global command_pub, last_keyboard_time

    # initialize node
    rospy.init_node('control_node')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    last_keyboard_time = rospy.get_time()

    # Subscribers
    # let keyboard input take over
    rospy.Subscriber("/bb/keyboard_input", Twist, get_key_cmd)
    # halt if bumper is triggered
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, get_bumper_state)
    # otherwise do command from planning
    rospy.Subscriber("/bb/where2go", Twist, get_nav_cmd)
    # subscribe to odometry
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # Set up a timer to update robot's drive state at 20 Hz
    rospy.Timer(rospy.Duration(secs=0.05), send_command)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
