#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def location_input():
    '''
    queries the user for an x and y value and publishes that as the new goal
    location for the turtlejectory node
    '''
    rospy.init_node('location_input', anonymous=True)
    pub = rospy.Publisher('~/goal', Point, queue_size=10)
    while not rospy.is_shutdown():
        loc = Point()
        print("INPUT: X & Y GOAL LOCATION")
        print("#"*50)
        loc.x = input("x: ")
        loc.y = input("y: ")
        print("#"*50)
        pub.publish(loc)

if __name__ == '__main__':
    try:
        location_input()
    except rospy.ROSInterruptException:
        pass
