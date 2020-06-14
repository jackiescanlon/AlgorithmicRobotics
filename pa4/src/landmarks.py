#!/usr/bin/env python

import numpy as np
import tf

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
        
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        robot_pose = np.array([[x, y, z]])
        
        dists = np.sqrt(np.sum((self.true_poses - robot_pose)**2, axis=1))
        
        landmark_arr = PoseArray()
        for i in range(0, len(dists)):
            
            if (dists[i] <= self.threshold and not self.is_noise):
                landmark_arr.poses.append(Pose())
                r_and_beta = self.get_r_and_beta(odom_msg, np.copy(self.true_poses[i,:]))
                landmark_arr.poses[len(landmark_arr.poses)-1].position.x = r_and_beta[0] #self.true_poses[i, 0]
                landmark_arr.poses[len(landmark_arr.poses)-1].position.y = r_and_beta[1] #self.true_poses[i, 1]
                landmark_arr.poses[len(landmark_arr.poses)-1].orientation.w = i
            elif (dists[i] <= self.threshold and self.is_noise):
                landmark_arr.poses.append(Pose())
                r_and_beta = self.get_r_and_beta(odom_msg, np.copy(self.true_poses[i,:] + np.random.normal(0, 0.1, size=(3,))))
                landmark_arr.poses[len(landmark_arr.poses)-1].position.x = r_and_beta[0] #self.true_poses[i, 0] + np.random.normal(0, 0.1)
                landmark_arr.poses[len(landmark_arr.poses)-1].position.y = r_and_beta[1] #self.true_poses[i, 1] + np.random.normal(0, 0.1)
                landmark_arr.poses[len(landmark_arr.poses)-1].orientation.w = i
                
        landmark_arr.header.stamp = rospy.Time.now()
        landmark_arr.header.frame_id = '/map'
        self.pub_landmarks.publish(landmark_arr)
        
        
    def get_r_and_beta(self, odom_msg, l_pose):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        
        (theta_x, theta_y, theta_z) = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
            
        theta = theta_z
            
        T_robot = np.linalg.inv(np.array([[np.cos(theta), -1*np.sin(theta), x],
                                     [np.sin(theta),  np.cos(theta),   y],
                                     [0.0,                 0.0,                  1]]))
        l_pose[2] = 1  
        l_robot_frame = np.matmul(T_robot, l_pose.reshape((3,1)))
        #print l_robot_frame
        
        l_robot_frame[2] = 0
        r = np.linalg.norm(l_robot_frame)
        
        beta = np.arctan2(l_robot_frame[1], l_robot_frame[0])
        #print (r, beta)
        
        return (r, beta[0])

if __name__ == "__main__":
    d = LandmarksNode(sys.argv[1])
    rospy.spin()
