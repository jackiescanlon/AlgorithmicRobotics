#!/usr/bin/env python

# Jackie Scanlon
# PA4 - Part 2
# Task: Execute path
# 4/8/19


import rospy
import time
import math

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Pose2D
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool


class PathFollowerNode(object):
    '''
        PathFollowerNode: Given a path, sends a series of twists to the robot
        until the robot has achieved the goal location.
    '''

    def __init__(self):
        rospy.init_node('path_follower_node')

        self.current_pose = None
        self.current_goal = None
        self.final_goal = None
        self.theta = 0
        self.const_vel = 0.5
        self.rho = .2

        # Keeps track of whether the robot is moving or not
        self.moving = False

        # Gains for executing twists
        self.kw = 5.0
        self.kd = 1
        self.ka = 7
        self.kb = -4

        # Publishers
        self.twist_pub = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.plan_sub = rospy.Subscriber('/path', Path, self.executePath)
        self.goal_sub = rospy.Subscriber('goal', Pose2D, self.setGoalTheta)
        self.current_pose_sub = rospy.Subscriber('/robot0/odom', Odometry, self.setCurrentPose)
        self.emergency_stop = rospy.Subscriber('/stop', Bool, self.stop)

        rospy.spin()

    def stop(self, bool_msg):
        ''' Stop the car when we see a victim.'''

        self.moving = False
        self.current_goal = self.current_pose
        self.final_goal = self.current_pose


    def setCurrentPose(self, odom_msg):
        ''' Sets the current pose based on odometery messages.'''

        self.current_pose = self.poseToArray(odom_msg.pose)
                
        # convert from 0 -- 2pi to -pi -- pi
        theta = self.current_pose[2]
        if theta > math.pi:
            theta -= 2*math.pi
        self.current_pose[2] = theta

        # if there is no goal, change that
        if self.current_goal is None:
            self.current_goal = list(self.current_pose)
            self.final_goal = list(self.current_pose)

        # If there is a goal
        else:

            # If its the final goal, care about orientation
            if self.current_goal == self.final_goal:
                self.executeLastTwist()

            # If its not the final goal, we don't care about orientation
            else:
                self.executeTwist()


    def executePath(self, path):
        ''' Feeds path points to twist executer.'''

        rospy.loginfo('Executing path.')

        # Set the final goal as the last pose in the pose array
        self.final_goal = self.poseToArray(path.poses[len(path.poses)-1])
        self.final_goal[2] = self.theta

        # Go through the intermediate poses
        for i in range(0, len(path.poses)):
            pose = path.poses[i]

            # Set as the current goal and execute
            self.current_goal = self.poseToArray(pose)
            self.executeTwist()

            # Wait until we are done with the goal to proceed
            while self.moving:
                pass


        #self.current_goal = self.poseToArray(

        # Last pose - care about orientation now
        self.current_goal = self.final_goal
        self.executeLastTwist()


    def setGoalTheta(self, pose2d):
        ''' Since the path doesn't come with a theta,
            we need to set the final theta of the robot.'''

        self.theta = pose2d.theta

    def poseToArray(self, pose):
        ''' Converts a Pose (ROS message type) to a list.'''

        x = pose.pose.position.x
        y = pose.pose.position.y
        
        # Pull the quaternion for conversion
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w)
            
        # Get theta from the quaternion
        (roll,pitch,theta) = euler_from_quaternion(quaternion) 

        return [x, y, theta]


    def executeTwist(self):
        ''' Performs twists to intermediate path points.'''

        # Twist object
        vel_msg = Twist()

        # If we have already reached the final goal, stop moving
        if self.current_pose == self.final_goal:
            self.moving = False
            # vel_msg is already 0s so we don't need to do anything

        # If we are still working on the path
        else:
            x = self.current_pose[0]
            y = self.current_pose[1]
            theta = self.current_pose[2]

            # Get error
            e_x = self.current_goal[0] - x
            e_y = self.current_goal[1] - y

            # Calculate distance and use it check how close we are to goal
            d = math.sqrt(e_x**2 + e_y**2)

            # If we are close enough, move onto next point
            #if d < .05:
            if d < self.rho:
                self.moving = False
                omega = 0

            # Otherwise, set heading appropriately
            else:

                self.moving = True

                # We don't care about orientation so just pick a theta that's easy
                goal_theta = math.atan2(e_y, e_x)
                b = self.min_angle_diff(goal_theta, theta)
                omega = self.kw*b
                
            # Set constant velocity
            vel_msg.linear.x = self.const_vel
            vel_msg.angular.z = omega
        
        # Publish velocity message
        self.twist_pub.publish(vel_msg)


    
    def executeLastTwist(self):
        '''Executes a twist for the last goal -
            i.e., we need to care about orientation now.'''

        vel_msg = Twist()
        
        x = self.current_pose[0]
        y = self.current_pose[1]
        theta = self.current_pose[2]

        # If we are not within tolerance, calculate the control values
        # Get error
        e_x = self.current_goal[0] - x
        e_y = self.current_goal[1] - y

        # Calculate distance and use it check how close we are to goal
        d = math.sqrt(e_x**2 + e_y**2)

        # If we are close enough, move onto next point
        #if d < .05:
        if d < self.rho:
            self.moving = False
            omega = 0

        # Otherwise, set heading appropriately
        else:

            self.moving = True

            # We don't care about orientation so just pick a theta that's easy
            goal_theta = math.atan2(e_y, e_x)
            b = self.min_angle_diff(goal_theta, theta)
            omega = self.kw*b
            vel_msg.linear.x = self.kd*d
            vel_msg.angular.z = omega


        # Publish the message (will be 0 if within tolerance)
        self.twist_pub.publish(vel_msg)

   


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


if __name__ == "__main__":
    PathFollowerNode()
    
