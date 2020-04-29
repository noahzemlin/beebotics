#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
from ga.gasearch import GASearch
from aStar.aStarSearch import aStar
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

width = 0
height = 0


def receive_cur_pos(position):
    global cur_pos
    cur_pos = (position.x, position.y)


def map_update(ogrid):
    global space, reorient_vector, height, width

    info = ogrid.info
    space = np.reshape(ogrid.data, (info.height, info.width))
    space = np.transpose(space)

    width = info.width
    height = info.height

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

    print("reorient_vector", reorient_vector)


def world_point_to_grid(pt, scale = 1):
    pt_x = ((pt[0] - reorient_vector[1]) / reorient_vector[0])*scale
    pt_y = ((pt[1] - reorient_vector[2]) / reorient_vector[0])*scale

    return (int(pt_y), int(pt_x))


def grid_point_to_world(pt):
    pt = (height - pt[0], width - pt[1])

    pt_x = pt[0] * reorient_vector[0] + reorient_vector[1]
    pt_y = pt[1] * reorient_vector[0] + reorient_vector[2]

    return (pt_y, pt_x)


def run_ga():
    if space is None or cur_pos is None:
        return
    # run the genetic algorithm
    # TODO put all this in a loop to run repeatedly and keep sending the most updated path
    searchy = GASearch(space, population_size=200)

    # path should start at the robot's current position
    best_path = searchy.search((world_point_to_grid(cur_pos)), (400, 300), iters=200)

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
    if space is None or cur_pos is None:
        return

    # Coordinates for scale = 5
    # scale = 5 
    # start = [60, 55] 
    # goal = [60,95]

    # scaled down by 100 scale factor = 10
    scale = 10
    start = [30, 25] 
    goal = [30,47]

    # run A* to create the path
    search = aStar(space, start, goal)
    search.convertSpace(space, scale)
    path = search.grid_astar()
    best_path = search.convert(path)
    search.display(start, best_path)

    # best_path is a list of points forming the path. send to planning_node.
    pathcloud = PointCloud()
    pathcloud.header.stamp = rospy.Time.now()
    pathcloud.points = []

    # include start point
    point = Point32()
    point.x, point.y = grid_point_to_world(start, 100)
    point.z = 0.1 # to put above ground in RViz
    pathcloud.points.append(point)

    for path_pt in best_path:
        point = Point32()
        point.x, point.y = grid_point_to_world(path_pt, 100)
        point.z = 0.1 # to put above ground in RViz
        pathcloud.points.append(point)
    path_pub.publish(pathcloud)

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
