#!/usr/bin/env python

import numpy as np

import sys
import rospy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry

class LandmarksNode(object):

    def __init__(self, is_noise):
        rospy.init_node('landmarks_node')

        self.is_noise = False
        if (is_noise == 'true'):
            self.is_noise = True

        # Landmarks will publish their pose when the robot's
        # true position is within this threshold distance
        self.threshold = 3.0
        
        
        # Array of landmark positions [x, y, z]
        # Each row is a landmark
        # A landmark's ID is the index of its row
        # i.e the landmark 0 is at [7.0, 2.0, 0.0]
        self.true_poses = np.array([[4.0, 2.0, 0.0],
                                    [3.0, 8.0, 0.0],
                                    [11.0, 2.0, 0.0],
                                    [12.5, 6.5, 0.0],
                                    [13.0, 11.0, 0.0],
                                    [9.0, 8.5, 0.0],
                                    [6.0, 13.0, 0.0]])

        # Publishes poses
        self.pub_landmarks = rospy.Publisher("/landmarks", PoseArray, queue_size=10)
        self.pub_true_landmarks = rospy.Publisher('/true_landmarks', PoseArray, queue_size=10)
        # Subs to robot odometry
        rospy.Subscriber("/robot0/odom", Odometry, self.odom_callback)

        

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
        self.pub_true_landmarks.publish(landmark_arr)
        
    def odom_callback(self, odom_msg):
    
        self.publishTrue()

        landmark_arr = PoseArray()

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        robot_pose = np.array([[x, y, z]])
        
        dists = np.sqrt(np.sum((self.true_poses - robot_pose)**2, axis=1))
        
        
        for i in range(0, len(dists)):
            
            if (dists[i] <= self.threshold and not self.is_noise):
                landmark_arr.poses.append(Pose())
                landmark_arr.poses[len(landmark_arr.poses)-1].position.x = self.true_poses[i, 0]
                landmark_arr.poses[len(landmark_arr.poses)-1].position.y = self.true_poses[i, 1]
                landmark_arr.poses[len(landmark_arr.poses)-1].orientation.w = i
            elif (dists[i] <= self.threshold and self.is_noise):
                landmark_arr.poses.append(Pose())
                landmark_arr.poses[len(landmark_arr.poses)-1].position.x = self.true_poses[i, 0] + np.random.normal(0, 0.1)
                landmark_arr.poses[len(landmark_arr.poses)-1].position.y = self.true_poses[i, 1] + np.random.normal(0, 0.1)
                landmark_arr.poses[len(landmark_arr.poses)-1].orientation.w = i
                
        landmark_arr.header.stamp = rospy.Time.now()
        landmark_arr.header.frame_id = '/map'
        self.pub_landmarks.publish(landmark_arr)
        

if __name__ == "__main__":
    d = LandmarksNode(sys.argv[1])
    rospy.spin()
