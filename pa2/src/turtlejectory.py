#!/usr/bin/env python

# turtlejectory.py for PA2
# Jackie Scanlon
# 2/21/19

import math

#ROS IMPORTS
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from std_msgs.msg import Float64

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
        name="turtlejectory"):
        '''
        '''
        # ros node creation with name provided
        rospy.init_node(name, anonymous=True)

        # attribute initialization
        self.name = name
        self.pose = None # don't know location
        self.goal = None # don't have a goal in mind
        self.kv = 1.0
        self.kw = 2.0

        # publishers
        self.pub_vel = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        # subscribers
        rospy.Subscriber("/turtle1/pose", Pose, self.on_location_change)
        rospy.Subscriber("~/goal", Point, self.on_goal_change)

    def min_angle_diff(self, theta_1, theta_2):
        '''
        There's more than one angle to consider, this method finds the smallest

        Args:
            theta_1(float): angle theta where 0 radians is on same axis as
                theta_2
            theta_2(float): angle theta where 0 radians is on same axis as
                theta_1

        Returns:
            the shortest angle between two thetas in +/- radians
        '''
        delta_theta = theta_1 - theta_2
        tau = 2*math.pi
        return min(delta_theta, delta_theta + tau, delta_theta - tau, key=abs)

    def on_goal_change(self, new_goal):
        '''
        callback invoked when publisher changes goal location

        new_goal(Point): new goal location (z direction is ignored)
        '''
        self.goal = (new_goal.x, new_goal.y)
        self.publish_twist() # re-adjust twist message

    def on_location_change(self, pose):
        '''
        callback invoked when publisher puts out new location of robot

        pose(Pose): the location of the robot with theta from 0 to 2*pi,
            z direction ignored;
        '''

        # if there is no goal, make current location the goal
        if self.goal is None:
            self.goal = (pose.x, pose.y)

        # convert from 0 -- 2pi to -pi -- pi
        theta = pose.theta
        if theta > math.pi:
            theta -= 2*math.pi
        self.pose = (pose.x, pose.y, theta)

        self.publish_twist()

    def publish_twist(self):
        '''
        publish a Twist message based on the state of the bot and the
        goal; only the x velocity and z angular velocity are
        used; other values set to zero; doesn't move if goal is within
        a certain tolerance
        '''
        vel_msg = Twist()
        x, y, theta = self.pose
        
        # Get the changes in the x and y direction
        # Hint: self.goal[0] is goal loc x, self.goal[1] is goal loc y
        
        # Get error
        e_x = self.goal[0] - x
        e_y = self.goal[1] - y
        # Calculate distance and use it to set linear velocity
        d = math.sqrt(e_x**2 + e_y**2)
        velocity = self.kv*d
        
        # Check if speed is within a tolerance of 0.01, if so halt
        if abs(velocity) <= .01:
            velocity = 0
            
        # Calculate the goal angle and get the minimum angle difference
        # Hint: use self.min_angle_diff
        goal_theta = math.atan2(e_y, e_x)
        b = self.min_angle_diff(goal_theta, theta)
        heading = self.kw*b
        
        # Check if heading is within a tolerance of 0.01, if so halt
        if abs(heading) <= .01:
            heading = 0
            
        # Set the velocities
        vel_msg.linear.x = velocity
        vel_msg.angular.z = heading
        
        # Publish velocity message
        self.pub_vel.publish(vel_msg)
        

if __name__ == "__main__":
    tj = Turtlejectory(name="turtlejectory")
    rospy.spin()
