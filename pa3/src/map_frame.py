#!/usr/bin/env python
import rospy
import tf

from nav_msgs.msg import Odometry

def handle_r2d2_pose(odom_msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((odom_msg.pose.pose.position.x,
                      odom_msg.pose.pose.position.y, 
                      odom_msg.pose.pose.position.z),
                     (odom_msg.pose.pose.orientation.x,
                      odom_msg.pose.pose.orientation.y,
                      odom_msg.pose.pose.orientation.z,
                      odom_msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     'base_link',
                     'map')
                     
if __name__ == '__main__':
    rospy.init_node('map_tf_broadcast')
    rospy.Subscriber('/groundtruth', Odometry, handle_r2d2_pose)
    rospy.spin() 
