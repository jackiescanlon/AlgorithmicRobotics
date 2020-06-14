#!/usr/bin/env python

import numpy as np

import sys
import rospy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry

class VictimsNode(object):

    def __init__(self, is_noise):
        rospy.init_node('victims_node')

        self.is_noise = False
        if (is_noise == 'true'):
            self.is_noise = True

        # victims will publish their pose when the robot's
        # true position is within this threshold distance
        self.threshold_0 = 3.0
        self.threshold_1 = 1.5
        self.threshold_2 = 0.5
        self.threshold_3 = 0.1
        
        
        
        # Array of victim positions [x, y, z]
        # Each row is a victim
        # A victim's ID is the index of its row
        # i.e the victim 0 is at [4.0, 1.0, 0.0]
        self.true_poses = np.array([[4.0, 1.0, 0.0],
                                    [2.0, 8.0, 0.0],
                                    [11.0, 3.0, 0.0],
                                    [14.0, 4.5, 0.0],
                                    [14.0, 14.0, 0.0],
                                    [9.0, 10.5, 0.0],
                                    [3.0, 14.0, 0.0]])

        # Publishes poses
        self.pub_victims = rospy.Publisher("/victims", PoseArray, queue_size=10)

        # Subs to robot odometry
        rospy.Subscriber("/robot0/odom", Odometry, self.odom_callback)
        self.pub_true_victims = rospy.Publisher('/true_victims', PoseArray, queue_size=10)


    def publishTrue(self):
        # Publishes the true location of landmarks so we can see them
        # on RVIZ
        landmark_arr = PoseArray()
        
        for i in range(0, len(self.true_poses)):
            landmark_arr.poses.append(Pose())
            landmark_arr.poses[len(landmark_arr.poses)-1].position.x = self.true_poses[i, 0]
            landmark_arr.poses[len(landmark_arr.poses)-1].position.y = self.true_poses[i, 1]
            landmark_arr.poses[len(landmark_arr.poses)-1].orientation.w = i

        landmark_arr.header.stamp = rospy.Time.now()
        landmark_arr.header.frame_id = '/map'
        self.pub_true_victims.publish(landmark_arr)


    def odom_callback(self, odom_msg):

        self.publishTrue()
    
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        robot_pose = np.array([[x, y, z]])
        
        dists = np.sqrt(np.sum((self.true_poses - robot_pose)**2, axis=1))
        
        victim_arr = PoseArray()
        mean = 0
        std = 0
        for i in range(0, len(dists)):
            mean = 0
            std = 0
            
            if (dists[i] <= self.threshold_0):
                std = dists[i]/2.
                if (dists[i] <= self.threshold_3):
                    mean = 0
                    std = 0.0001
                elif (dists[i] <= self.threshold_2):
                    mean = 0
                    std = dists[i]/10.
                elif (dists[i] <= self.threshold_1):
                    mean = 0
                    std = dists[i]/5.
                if (not self.is_noise):
                    mean = 0
                    std = 0.0001
                
                victim_arr.poses.append(Pose())
                victim_arr.poses[len(victim_arr.poses)-1].position.x = self.true_poses[i, 0] + np.random.normal(mean, std)
                victim_arr.poses[len(victim_arr.poses)-1].position.y = self.true_poses[i, 1] + np.random.normal(mean, std)
                
        victim_arr.header.stamp = rospy.Time.now()
        victim_arr.header.frame_id = '/map'
        self.pub_victims.publish(victim_arr)
        

if __name__ == "__main__":
    d = VictimsNode(sys.argv[1])
    rospy.spin()
