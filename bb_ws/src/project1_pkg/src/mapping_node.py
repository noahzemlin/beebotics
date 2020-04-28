#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
from ga.gasearch import GASearch
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Point

# Global Variables
path_pub = None

# set True for GA or False for A*
RUN_GA = True

# robot's current position
cur_pos = None

# 2D array occupancy grid, 0 = empty and anything else (should be set to 1) is occupied
space = None

# Reorientation factor to account for map not aligning with robot initial heading
# (scale, origin_x, origin_y, origin_theta)
reorient_vector = (1, 0, 0, 0)


def receive_cur_pos(position):
    global cur_pos
    cur_pos = (position.x, position.y)


def map_update(ogrid):
    global space, reorient_vector

    info = ogrid.info
    space = np.reshape(ogrid.data, (info.height, info.width))

    space[space != 0] = 1

    pos = (info.origin.position.x, info.origin.position.y)
    orientation = info.origin.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    yaw_rads = tf.transformations.euler_from_quaternion(quaternion)[2]

    reorient_vector = (info.resolution, pos[0], pos[1], yaw_rads)


def world_point_to_grid(pt):
    pt_x = (pt[0] - reorient_vector[1]) / reorient_vector[0]
    pt_y = (pt[1] - reorient_vector[2]) / reorient_vector[0]

    x = pt_x * math.cos(-reorient_vector[3]) - \
        pt_y * math.sin(-reorient_vector[3])
    y = pt_y * math.cos(-reorient_vector[3]) + \
        pt_x * math.sin(-reorient_vector[3])
    return (int(x), int(y))


def grid_point_to_world(pt):
    x = pt[0] * math.cos(reorient_vector[3]) - pt[1] * \
        math.sin(reorient_vector[3])
    y = pt[1] * math.cos(reorient_vector[3]) + pt[0] * \
        math.sin(reorient_vector[3])

    pt_x = x * reorient_vector[0] + reorient_vector[1]
    pt_y = y * reorient_vector[0] + reorient_vector[2]
    return (pt_x, pt_y)


def run_ga():
    if space is None or cur_pos is None:
        return
    # run the genetic algorithm
    # TODO put all this in a loop to run repeatedly and keep sending the most updated path
    searchy = GASearch(space, population_size=200)

    # path should start at the robot's current position
    best_path = searchy.search(world_point_to_grid(cur_pos), (400, 300), iters=200)

    # best_path is a list of points forming the path. send to planning_node.
    pathcloud = PointCloud()
    pathcloud.header.stamp = rospy.Time.now()
    pathcloud.points = []
    for path_pt in best_path.pts:
        point = Point32()
        point.x, point.y = grid_point_to_world(path_pt)
        point.z = 0.1 # to put above ground in RViz
        pathcloud.points.append(point)
    path_pub.publish(pathcloud)


def run_astar():
    # run A* to create the path
    pass


def main():
    global path_pub
    # initialize node
    rospy.init_node('mapping_node', anonymous=True)

    # subscribe to the robot's current position (use as start of path)
    map_sub = rospy.Subscriber("/map", OccupancyGrid, map_update, queue_size=1)

    # publish command to follow path
    path_pub = rospy.Publisher("/bb/path", PointCloud, queue_size=1)

    # subscribe to the robot's current position (use as start of path)
    pos_sub = rospy.Subscriber("/bb/pos", Point, receive_cur_pos, queue_size=1)

    rate = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():
        # check whether we are using A* or GA to generate the path
        if RUN_GA:
            # run the GA to create the path
            run_ga()
        else:
            # run A* to create the path
            run_astar()

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
