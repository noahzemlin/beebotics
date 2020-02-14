#!/usr/bin/env python

import rospy
import geometry_msgs, kobuki_msgs
from std_msgs.msg import String

# set up the publisher
command_pub = None
# keep track of the most recent keyboard command (Twist)
key_cmd = None
# keep track of most recent bumper state (bool)
bumped = False
# keep track of most recent planned command (Twist)
plan_cmd = None

def get_key_cmd(key_msg):
    global key_cmd
    key_cmd = key_msg

def get_bumper_state(bumper_state):
    global bumped
    bumped = bumper_state

def get_planned_cmd(planned_cmd)
    global plan_cmd
    plan_cmd = planned_cmd

def send_command(timer_event):
    if bumped:
        # bumper has been triggered, so halt
        halt_cmd = Twist()
        halt_cmd.linear = 0
        halt_cmd.angular = 0
        command_pub.publish(halt_cmd)
    if key_cmd = 0:
        # there is no keyboard command being received
        command_pub.publish(planned_cmd)
    else:
        # keyboard command is being received and should be passed through
        command_pub.publish(key_cmd)

def main():
    rospy.init_node('navigation_node', anonymous=True)

    # set up publisher
    command_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    # set up subscribers
    # let keyboard input take over
    keyboard_sub = rospy.Subscriber("/turtlebot_teleop_keyboard", Twist, get_key_cmd, queue_size=1)
    # halt if bumper trigger
    bumper_sub = rospy.Subscriber("/mobilebase/events/bumper", BumperEvent, get_bumper_state, queue_size=1)
    # do event from planning
    planning_sub = rospy.Subscriber("/bb/where2go", Twist, get_planned_cmd, queue_size=1)

    # Set up a timer to update robot's drive state at 10 Hz
    update_timer = rospy.Timer(rospy.Duration(secs=0.1), send_command)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass