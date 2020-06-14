#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

def location_input():
    '''
    queries the user for an x and y value and publishes that as the new goal
    location for the turtlejectory node
    '''
    rospy.init_node('location_input', anonymous=True)
    pub = rospy.Publisher('~/goal', Pose2D, queue_size=10)
    while not rospy.is_shutdown():
        pose = Pose2D()
        print("INPUT: X, Y & Theta GOAL LOCATION")
        print("#"*50)
        pose.x = input("x: ")
        pose.y = input("y: ")
        pose.theta = input("theta: ")
        print("#"*50)
        pub.publish(pose)

if __name__ == '__main__':
    try:
        location_input()
    except rospy.ROSInterruptException:
        pass
