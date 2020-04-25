#!/usr/bin/env python

import rospy
from ga.gasearch import GASearch
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point

## Global Variables
path_pub = None

# set True for GA or False for A*
RUN_GA = True

#TODO make a function to import the map as a 2D array and transform to meters


def run_ga():
    # run the genetic algorithm at every timer tick
    searchy = GASearch(space, population_size=200)
    best_path = searchy.search((0,12), (9, 0), iters = 10000)
    # TODO constantly run this and keep pushing the best_path as a PointCloud

def run_astar():
    # run A* to create the path


def main():
    global path_pub
    # initialize node
    rospy.init_node('planning_node', anonymous=True)

    # check whether we are using A* or GA to generate the path
    if RUN_GA:
        # timer to update the GA path at 1 Hz
        rospy.Timer(rospy.Duration(secs=1), run_ga)
    else:
        # run A* once to create the path
        run_astar()

    # publish command to follow path
    path_pub = rospy.Publisher("/bb/path", PointCloud, queue_size=1)
    
    
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
