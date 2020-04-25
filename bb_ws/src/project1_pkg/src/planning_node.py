#!/usr/bin/env python

import rospy
from math import degrees, atan2
from pure_pursuit import PurePursuit
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist, Vector3, Point

## Global Variables
command_pub = None

# navigation path
pp = PurePursuit()

# robot's current position and heading
pos = None
current_heading = None


def get_angle_diff(angle1, angle2):
    # from github.com/SoonerRobotics/igvc_software_2020/blob/master/igvc_ws/src/igvc_nav/src/igvc_nav_node.py
    delta = angle1 - angle2
    delta = (delta + 180) % 360 - 180
    return delta


def receive_heading(hdg):
    global current_heading
    current_heading = hdg.data

def receive_position(position):
    global pos
    pos = [position.x, position.y]


def receive_path(point_cloud):
    # set path for pure pursuit
    global pp
    pp.set_points(point_cloud.points)


# based on github.com/SoonerRobotics/nrc_software/blob/master/nrc_ws/src/nrc_nav/src/nrc_drive_pp.py
# PID ripped out and turning handled by nav_node, so just sends target_heading and speed
def generate_command(timer_event):
    global current_heading, target_heading

    if pos is None or current_heading is None or pp is None:
        # wait until sensors/localization bring in data
        # wait until path is generated and received
        return

    # take a snapshot of the current position so it doesn't change while this function is running
    cur_pos = (pos[0], pos[1])

    # declare the look-ahead point
    lookahead = None
    # start with a search radius of 0.4 meters
    radius = 0.4

    # look until finding the path at the increasing radius or hitting 2 meters
    while lookahead is None and radius <= 2: 
        lookahead = pp.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
        radius *= 1.25

    # make sure we actually found the path
    if lookahead is not None:
        heading_to_la = -degrees(atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0]))
        if heading_to_la <= 0:
            heading_to_la += 360

        delta = heading_to_la - current_heading
        delta = (delta + 180) % 360 - 180

        # set command velocity (drive power)
        # TODO this probably needs to be extremely adjusted
        cmd_speed = 0.5 * (1 - abs(delta / 180))**5

        path_cmd = Twist()
        path_cmd.linear = Vector3(cmd_speed, 0, 0)
        path_cmd.angular = Vector3(0, 0, heading_to_la)
        command_pub.publish(path_cmd)


def main():
    global command_pub
    # initialize node
    rospy.init_node('planning_node', anonymous=True)

    # subscribe to set of points that make path from GA or A*
    rospy.Subscriber("/bb/path", PointCloud, receive_path, queue_size=1)
    # subscribe to robot's current heading and position
    rospy.Subscriber("/bb/hdg", Float32, receive_heading, queue_size=1)
    rospy.Subscriber("/bb/pos", Point, receive_position, queue_size=1)

    # publish command to follow path
    command_pub = rospy.Publisher("/bb/path_cmd", Twist, queue_size=1)
    
    # timer to update the command at 20 Hz
    rospy.Timer(rospy.Duration(secs=0.05), generate_command)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
