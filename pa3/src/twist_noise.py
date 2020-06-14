#!/usr/bin/env python

import math
import numpy as np

#ROS IMPORTS
import rospy
from geometry_msgs.msg import Twist

class Turtlejectory:
    '''
    ROS node that handles trajectory planning

    Attributes:
        name(str): name of the bot node (used for ros)
        pose(float, float, float): pose of bot (x, y, theta)
        goal(float, float): goal location of bot
        kv(float): velocity gain
        kw(float): angular velocity gain
        pub_vel(rospy.Publisher): publishes geometry_msgs.msg.Twist messages
    '''
    def __init__(
        self,
        name="twist_noise"):
        '''
        '''
        # ros node creation with name provided
        rospy.init_node(name, anonymous=True)

        # publishers
        self.pub_vel = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        # subscribers
        rospy.Subscriber("/turtle1/cmd_vel_noiseless", Twist, self.add_noise)


    def add_noise(self, twist_msg):
        twist_msg.linear.x = twist_msg.linear.x + np.random.normal(0, 1)
        twist_msg.angular.z = twist_msg.angular.z + np.random.normal(0, 0.5);
        
        self.pub_vel.publish(twist_msg)

if __name__ == "__main__":
    tj = Turtlejectory(name="turtlejectory")
    rospy.spin()
