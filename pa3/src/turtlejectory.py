#!/usr/bin/env python

# Jackie Scanlon
# PA3
# 3/28/19

import math
import numpy

#ROS IMPORTS
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose
from std_msgs.msg import Float64

class Turtlejectory:
    '''
    ROS node that handles trajectory planning

    Attributes:
        name(str): name of the bot node (used for ros)
        pose(float, float, float): pose of bot (x, y, theta)
        goal(float, float): goal location of bot
        pub_vel(rospy.Publisher): publishes geometry_msgs.msg.Twist messages
        direction(float): direction we are moving (forward or backward)
    '''
    def __init__(
        self,
        name="turtlejectory"):
        '''
        '''
        # ros node creation with name provided
        rospy.init_node(name, anonymous=True)

        # attribute initialization
        self.name = name
        self.pose = None # don't know location
        self.goal = None # don't have a goal in mind
        self.direction = 0 # Currently not moving
        
        # publishers
        self.pub_vel = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        # subscribers
        rospy.Subscriber("/turtle1/pose", Pose, self.on_location_change)
        rospy.Subscriber("~/goal", Pose2D, self.on_goal_change)

    def min_angle_diff(self, theta_1, theta_2):
        '''
        There's more than one angle to consider, this method finds the smallest

        Args:
            theta_1(float): angle theta where 0 radians is on same axis as theta_2
            theta_2(float): angle theta where 0 radians is on same axis as theta_1

        Returns:
            the shortest angle between two thetas in +/- radians
        '''
        delta_theta = theta_1 - theta_2
        tau = 2*math.pi
        return min(delta_theta, delta_theta + tau, delta_theta - tau, key=abs)

    def on_goal_change(self, new_goal):
        '''
        callback invoked when publisher changes goal location

        Args:
            new_goal(Pose2D): new goal location (z direction is ignored)
        '''
        if self.goal == (new_goal.x, new_goal.y, new_goal.theta):
            return 
        self.goal = (new_goal.x, new_goal.y, new_goal.theta)
        self.publish_twist() # re-adjust twist message

    def on_location_change(self, pose):
        '''
        callback invoked when publisher puts out new location of robot

        Args:
            pose(Pose): the location of the robot with theta from 0 to 2*pi, z direction ignored;
        '''
        # convert from 0 -- 2pi to -pi -- pi
        theta = pose.theta
        if theta > math.pi:
            theta -= 2*math.pi
        self.pose = (pose.x, pose.y, theta)

        # if there is no goal, change that
        if self.goal is None:
            self.goal = (pose.x, pose.y, theta)

        self.publish_twist()

    def publish_twist(self):
        '''
        publish a Twist message based on the state of the bot and the
        goal; only the x velocity and z angular velocity are
        used; other values set to zero; doesn't move if goal is within
        a certain tolerance
        '''
        vel_msg = Twist()
        
        # Set gains
        c = .25
        kd = 1
        ka = 7
        kb = -4
        
        x = self.pose[0]
        y = self.pose[1]
        theta = self.pose[2]
        xg = self.goal[0]
        yg = self.goal[1]
        thetag = self.goal[2]
        
        # Caculate rho to see if we are in tolerance of goal
        rho = math.sqrt((y - yg)**2 + (x - xg)**2 + c*(theta-thetag)**2)

        # If we are not within tolerance, calculate the control values
        if rho > 0.01: 

        	# Get a
            angle_diff = math.atan2(yg - y, xg - x)
            a = self.min_angle_diff(angle_diff, theta)

            # Get b    
            b = -theta - a +thetag

            # Get d
            d = math.sqrt((yg - y)**2 + (xg - x)**2)

            # Calculate v and w. Don't attempt to drive backwards
            vel_msg.linear.x = kd*d
            vel_msg.angular.z = ka*a+kb*b

        # Publish the message (will be 0 if within tolerance)
        self.pub_vel.publish(vel_msg)

if __name__ == "__main__":
    tj = Turtlejectory(name="turtlejectory")
    rospy.spin()
