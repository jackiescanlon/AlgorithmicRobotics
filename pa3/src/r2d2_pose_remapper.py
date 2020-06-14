#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import tf

class PoseNode(object):
    '''
    remaps quaternion pose from /groundtruth to cartesian x,y coordinates
    '''
    def __init__(self):
        rospy.init_node('pose_node')

        # Publishers
        self.pose_pub = rospy.Publisher('/r2d2/pose', Pose, queue_size=1)

        # Subscribers
        rospy.Subscriber('/groundtruth', Odometry, self.poseChange)

    def poseChange(self, odom_msg):
        pose = Pose()
        pose.x = odom_msg.pose.pose.position.x
        pose.y = odom_msg.pose.pose.position.y

        (theta_x, theta_y, theta_z) = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
        pose.theta = theta_z

        self.pose_pub.publish(pose)

if __name__ == "__main__":
    PoseNode()
    rospy.spin()
