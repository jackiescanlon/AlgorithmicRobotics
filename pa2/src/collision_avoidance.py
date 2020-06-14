#!/usr/bin/env python

# collision_avoidance.py for PA2
# Jackie Scanlon
# 2/21/19

import rospy
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class CollisionNode(object):
    '''
    CollisionNode subscribes for LaserScan messages from lidar and publishes
    stop twist message upon detecting an obstical within the minimum threshhold

    Attributes:
        min_dist (float): min threshhold for publishing a stop command
        pub_vel (Publisher): publisher for publishing stop, go twist messages
    '''
    
    def __init__(self):
        rospy.init_node('collision_node')
        
        # Robot should stop when it is 1m away from wall.
        self.min_dist = 1.
        
        # Publisher(s) for velocity/twist
        self.obs_vel = rospy.Publisher("/r2d2/obs_vel", Twist, queue_size=10)
        
        # Subscriber(s) for the laser
        rospy.Subscriber('/r2d2/laser/scan', LaserScan, self.checkAhead)
        
        rospy.spin()
        
    def checkAhead(self, scan_msg):
        '''
        publishes a Twist message to stop the robot if the sensor detects an
        obstacle within self.min_dist from center based off of the scan_msg it
        recieves, continues moving the car otherwise.

        Args:
            scan_msg(LaserScan): message containing lidar data used to determine
                the distance away from the sensor
        '''
        # Build array of laser angles
        # Hint: look at the LaserScan message and numpy.linspace
        # Hint: consider change ranges into a numpy array
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)
        
        # Ranges not between range_max and range_min are invalid, remove them
        # Hint: use numpy's conditional indexing
        good_ind = (ranges < scan_msg.range_max) & (ranges > scan_msg.range_min)
        ranges = np.extract(good_ind, ranges)
        angles = np.extract(good_ind, angles)
        
        # If there are obstacles detected
        if not ranges.size == 0:
        
            # Turn the sensor ranges and thetas into x,y components
            # Hint: incorporate sensor offset
            # Hint: use np.sin and np.cos to take the sin and cos of entire np array
            # Shift the x value forward 0.27 to account for the placement of the laser 
            # on the robot
            x = np.multiply(ranges, np.cos(angles))+.27
            y = np.multiply(ranges, np.sin(angles))

            # Determine the distance from the robot
            # Hint: np.sqrt takes the square root of the entire np array
            dist = np.sqrt(np.square(x) + np.square(y))

            # Determine the min distance calculated and compare to self.min_dist
            # Hint: np.min finds the min of an np array
            # Hint: Publish a Twist to stop the robot if too close to obstacle
            # Hint: Publish a Twist message to move the robot again if the obstacle
            #           disapears
            current_min_dist = np.min(dist)
            if current_min_dist < self.min_dist:
                self.obs_vel.publish(Twist())
                
            else:
                tw = Twist()
                tw.linear.x = 1
                self.obs_vel.publish(tw)
        
        else:
            tw = Twist()
            tw.linear.x = 1
            self.obs_vel.publish(tw)
        
        
if __name__ == "__main__":
    CollisionNode()
