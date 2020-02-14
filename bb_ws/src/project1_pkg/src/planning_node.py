#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_geometry.laser_geometry as lg
import geometry_msgs
import random

def main():
    rospy.init_node('planning_node', anonymous=True)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


def look():
	rospy.init_node('planning')
	rospy.Subscriber("looking", String, callback)

# Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
# Avoid asymmetric obstacles within 1ft in front of the robot.
# Turn randomly (uniformly sampled within ±15°) after every 1ft of forward movement.
# Drive forward.
def escape(self):
	rospy.Subscriber("/scan", LaserScan, send_command, queue_sizes=1)

def random_turn(self):
	rand_degree = randint(-15,15)
	
	rospy.Publisher("/bb/where2go", Twist, queue_size=1)

main():
	
