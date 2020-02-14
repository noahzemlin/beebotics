#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('navigation_node', anonymous=True)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass