#!/usr/bin/env python

# Jackie Scanlon
# PA4 - Part 1 
# Task: Localize using dead reckoning
# 4/7/19


import rospy
import time
import math

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, Quaternion, Pose2D
from tf.transformations import quaternion_from_euler


class DeadReckoningNode(object):
    '''
        DeadReckoningNode: Publishes the robot's current pose using
        dead reckoning based on the control inputs.
    '''

    def __init__(self):
        rospy.init_node('dead_reckoning_node')
        
        # Initial Pose
        self.x = 1.
        self.y = 2.
        self.theta = 0.
        
        # Twist currently being sent
        self.twist = Twist()

        # Publishers
        self.pose_pub = rospy.Publisher('/my_dead_reckoning', PoseStamped, queue_size=1)
        
        # Subscribers
        self.vel_sub = rospy.Subscriber('/robot0/input_vel', Twist, self.updateTwist)        
        
        # Pose updator is independent of twist updates
        self.updatePose()

        rospy.spin()
        
        
    def updateTwist(self,twist):
        ''' Updates the current twist'''

        self.twist = twist

        
    def updatePose(self):
        '''Updates the pose continuously.'''

        # For calculating change in time
        prev_t = rospy.get_time()

        # Continuously update
        while not rospy.is_shutdown():

            # Get velocities
            v = self.twist.linear.x
            w = self.twist.angular.z

            # Get time
            now = rospy.get_time()
            dt = now - prev_t
            prev_t = now

            # Change in x, theta
            dx = v*dt
            dtheta = w*dt

            # Set current pose
            self.x = self.x + math.cos(self.theta)*dx
            self.y = self.y + math.sin(self.theta)*dx
            self.theta = self.theta + dtheta
            
            # Get the quaternion from theta
            q = quaternion_from_euler(0, 0, self.theta)

            # Publish
            my_pose_stamped = PoseStamped()
            my_header = Header()
            my_header.frame_id = 'map_static'
            my_pose_stamped.header = my_header
            my_pose_stamped.pose = Pose(Point(self.x,self.y,0.), Quaternion(q[0], q[1], q[2], q[3]))
            self.pose_pub.publish(my_pose_stamped)

            # Sets our publishing rate
            time.sleep(.025)
              
if __name__ == "__main__":
    DeadReckoningNode()
