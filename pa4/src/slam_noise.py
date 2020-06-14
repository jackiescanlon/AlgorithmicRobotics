#!/usr/bin/env python

import math
import numpy as np

#ROS IMPORTS
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SlamNoise:
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
        self, is_control, is_laser):
        '''
        '''
        # ros node creation with name provided
        rospy.init_node('slam_noise')

        #params
        self.is_laser = False
        if (is_laser == 'true'):
            self.is_laser = True
            
        self.is_control = False
        if (is_control == 'true'):
            self.is_control = True

        # publishers
        self.pub_vel = rospy.Publisher("/robot0/cmd_vel", Twist, queue_size=10)
        self.pub_laser = rospy.Publisher("/robot0/scan", LaserScan, queue_size=10)

        # subscribers
        rospy.Subscriber("/robot0/input_vel", Twist, self.control_noise)
        rospy.Subscriber("/robot0/laser_0", LaserScan, self.lidar_noise)


    def control_noise(self, twist_msg):
        if (self.is_control):
            twist_msg.linear.x = twist_msg.linear.x + np.random.normal(0, 0.1)
            twist_msg.angular.z = twist_msg.angular.z + np.random.normal(0, 0.1)
        
        self.pub_vel.publish(twist_msg)
        
    def lidar_noise(self, laser_msg):
    
        if (self.is_laser):
            ranges = np.array(laser_msg.ranges)
            noise = np.random.normal(0, 0.1, (len(ranges),))
            ranges = ranges + noise
            laser_msg.ranges = ranges.tolist()
            self.pub_laser.publish(laser_msg)
        else:
            self.pub_laser.publish(laser_msg)

if __name__ == "__main__":
    sn = SlamNoise(sys.argv[1], sys.argv[2])
    rospy.spin()
