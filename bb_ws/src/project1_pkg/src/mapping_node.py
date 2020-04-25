#!/usr/bin/env python

import rospy
from ga.gasearch import GASearch
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point

## Global Variables
path_pub = None

# set True for GA or False for A*
RUN_GA = True

# robot's current position
cur_pos = (0,0)

# 2D array occupancy grid, 0 = empty and anything else (should be set to 1) is occupied
space = None


def receive_cur_pos(position):
    global cur_pos
    cur_pos = (position.x, position.y)


#TODO make a function to import the map as a 2D array and transform to meters, save to global "space"


def run_ga():
    # run the genetic algorithm
    # TODO put all this in a loop to run repeatedly and keep sending the most updated path
    searchy = GASearch(space, population_size=200)
    # path should start at the robot's current position
    best_path = searchy.search(cur_pos, (9, 0), iters = 10000)
    # best_path is a list of points forming the path. send to planning_node.
    pathcloud = PointCloud()
    for i in range(len(best_path.pts))
        pathcloud.points.x = best_path.pts[i][0]
        pathcloud.points.x = best_path.pts[i][1]
    path_pub.publish(pathcloud)
    

def run_astar():
    # run A* to create the path


def main():
    global path_pub
    # initialize node
    rospy.init_node('mapping_node', anonymous=True)

    # check whether we are using A* or GA to generate the path
    if RUN_GA:
        # run the GA to create the path
        run_ga()
    else:
        # run A* to create the path
        run_astar()

    # publish command to follow path
    path_pub = rospy.Publisher("/bb/path", PointCloud, queue_size=1)

    # subscribe to the robot's current position (use as start of path)
    pos_sub = rospy.Subscriber("/bb/pos", Point, receive_cur_pos, queue_size=1)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
