#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
import matplotlib.pyplot as plt
from ga.gasearch import GASearch
from aStar.aStarSearch import aStar
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Point

# Global Variables
path_pub = None

# set True for GA or False for A*
RUN_GA = True

# Enable debug Viz
DEBUG = True

# robot's current position
cur_pos = None

# 2D array occupancy grid, 0 = empty and anything else (should be set to 1) is occupied
space = None

# Reorientation factor to account for map not aligning with robot initial heading
# (scale, origin_x, origin_y, origin_theta)
reorient_vector = (1, 0, 0)

width = 0
height = 0

searchy = GASearch(population_size=500)


def receive_cur_pos(position):
    global cur_pos
    cur_pos = (position.x, position.y)


def map_update(ogrid):
    global space, reorient_vector, height, width

    info = ogrid.info
    space = np.reshape(ogrid.data, (info.height, info.width))
    space = np.transpose(space)
    space[space != 0] = 1

    width = info.width
    height = info.height

    reorient_vector = (
        info.resolution, info.origin.position.x, info.origin.position.y)

    searchy.set_world(space)

    print("reorient_vector", reorient_vector)


def world_point_to_grid(pt, scale = 1):
    pt_x = ((pt[0] - reorient_vector[1]) / reorient_vector[0])*scale
    pt_y = ((pt[1] - reorient_vector[2]) / reorient_vector[0])*scale

    return (int(pt_y), int(pt_x))


def grid_point_to_world(pt, scale = 1):
    pt_x = pt[1] / scale * reorient_vector[0] + reorient_vector[1]
    pt_y = pt[0] / scale * reorient_vector[0] + reorient_vector[2]

    return (pt_x, pt_y)


gen = 0


def run_ga():
    global gen
    if space is None or cur_pos is None:
        return

    # path should start at the robot's current position
    best_path = searchy.search(world_point_to_grid(cur_pos), (300, 400), iters=500, init_pop=(gen == 0))
    gen += 1

    if DEBUG:
        plt.clf()
        plt.ion()
        plt.imshow(searchy.world, interpolation='none')
        for i in range(0, len(best_path.pts) - 1):
            pt1 = [best_path.pts[i][0], best_path.pts[i + 1][0]]
            pt2 = [best_path.pts[i][1], best_path.pts[i + 1][1]]
            plt.plot(pt1, pt2, marker='o', color='green')
        plt.title("gen: " + str(gen * 500))
        plt.draw()
        plt.show()
        plt.pause(0.00001)

    # ff our path is complete
    if best_path.score < 1:
        # best_path is a list of points forming the path. send to planning_node.
        pathcloud = PointCloud()
        pathcloud.header.stamp = rospy.Time.now()
        pathcloud.points = []
        for path_pt in best_path.pts:
            point = Point32()
            point.x, point.y = grid_point_to_world(path_pt)
            pathcloud.points.append(point)
        path_pub.publish(pathcloud)


def run_astar():
    if space is None or cur_pos is None:
        return

    # Scaling
    scale = 5
    pos = world_point_to_grid(cur_pos)
    start = [pos[0]//scale, pos[1]//scale] 
    goal = [300//scale, 400//scale]

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
    point.x, point.y = grid_point_to_world(start, scale)
    pathcloud.points.append(point)

    for path_pt in best_path:
        point = Point32()
        point.x, point.y = grid_point_to_world(path_pt, scale)
        pathcloud.points.append(point)
    path_pub.publish(pathcloud)

def main():
    global path_pub
    # initialize node
    rospy.init_node('mapping_node', anonymous=True)

    # subscribe to the robot's current position (use as start of path)
    # use "/move_base/global_costmap/costmap" if slam works goodly
    map_sub = rospy.Subscriber("/map", OccupancyGrid, map_update, queue_size=1)

    # publish command to follow path
    path_pub = rospy.Publisher("/bb/path", PointCloud, queue_size=1)

    # subscribe to the robot's current position (use as start of path)
    pos_sub = rospy.Subscriber("/bb/pos", Point, receive_cur_pos, queue_size=1)

    rate = rospy.Rate(1)  # 1Hz, doesn't really matter since it'll take longer.
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
