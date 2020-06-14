#!/usr/bin/env python

# Ros python library
import rospy

# Establish this program as a ROS node.
rospy.init_node('hello_ros')

# Send some output as a log message.
rospy.loginfo('Hello, ROS!')
