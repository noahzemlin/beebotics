#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from kobuki_msgs.msg import BumperEvent


# set up the publisher
command_pub = None
# keep track of the most recent keyboard command (Twist)
key_cmd = Twist()
# keep track of most recent bumper state (bool)
bumped = False
# keep track of most recent planned command (Twist)
plan_cmd = Twist()

def get_key_cmd(key_msg):
    global key_cmd
    key_cmd = key_msg

def get_bumper_state(bumper_state):
    global bumped
    bumped = bumper_state

def get_planned_cmd(planned_cmd):
    global plan_cmd
    plan_cmd = planned_cmd

def is_zero_twist(twisty):
    return (twisty.linear.x == twisty.linear.y == twisty.linear.z == twisty.angular.x == twisty.angular.y == twisty.angular.z == 0)

def send_command(timer_event):
    # get local time at every tick to use for escape to go 1 ft
    local_time = time.time() - start_time

    if bumped:
        # bumper has been triggered, so halt
        halt_cmd = Twist()
        halt_cmd.linear = Vector3(0,0,0)
        halt_cmd.angular = Vector3(0,0,0)
        command_pub.publish(halt_cmd)
        return

    if is_zero_twist(key_cmd):
        # there is no keyboard command being received
        command_pub.publish(plan_cmd)
    else:
        # keyboard command is being received and should be passed through
        command_pub.publish(key_cmd)

def main():
    global command_pub
    # initialize node
    rospy.init_node('navigation_node', anonymous=True)

    # set up publisher
    command_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    # set up subscribers
    # let keyboard input take over
    keyboard_sub = rospy.Subscriber("/bb/keyboard_input", Twist, get_key_cmd)
    # halt if bumper is triggered
    bumper_sub = rospy.Subscriber("/mobilebase/events/bumper", BumperEvent, get_bumper_state)
    # otherwise do command from planning
    planning_sub = rospy.Subscriber("/bb/where2go", Twist, get_planned_cmd)

    # Set up a timer to update robot's drive state at 10 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), send_command)

    # cycle through callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass