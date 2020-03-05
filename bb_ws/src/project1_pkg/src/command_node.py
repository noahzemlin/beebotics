#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry

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

# The most recent reactive command
reactive_cmd = Twist()

# Last odom update
odom = Odometry()

# Control speed forward
control_speed = 0


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


def get_reactive_cmd(reactive_msg):
    global reactive_cmd
    reactive_cmd = reactive_msg


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
    else:
        # There is no keyboard command being received, use our reactive command

        # Apply proportional speed gain
        control_speed += (reactive_cmd.linear.x - odom.twist.twist.linear.x) * speed_kp
        reactive_cmd.linear.x = control_speed

        command_pub.publish(reactive_cmd)


def main():
    global command_pub

    # initialize node
    rospy.init_node('command_node')

    # set up publisher
    command_pub = rospy.Publisher(
        "/mobile_base/commands/velocity", Twist, queue_size=1)

    # set up subscribers

    # let keyboard input take over
    rospy.Subscriber("/bb/keyboard_input", Twist, get_key_cmd)

    # halt if bumper is triggered
    rospy.Subscriber("/mobile_base/events/bumper",
                     BumperEvent, get_bumper_state)

    # otherwise do command from planning
    rospy.Subscriber("/bb/where2go", Twist, get_reactive_cmd)

    # Subscribe to odometry
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # Set up a timer to update robot's drive state at 20 Hz
    rospy.Timer(rospy.Duration(secs=0.05), send_command)

    # cycle through callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
