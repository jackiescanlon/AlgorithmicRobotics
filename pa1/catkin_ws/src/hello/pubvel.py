#!usr/bin/env python

''' This program publishes randomly-generated velocity
messages for turtlesim. '''

import rospy
from geometry_msgs.msg import Twist
import random

# Initialize the ROS system and become a node.
rospy.init_node('publish_velocity')

# Create a publisher object
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1000)

# Loop at 2Hz until the node is shut down.
r = rospy.Rate(2)
while not rospy.is_shutdown():
    # Create and fill the message. The other fields
    # will default to 0.
    msg = Twist()
    msg.linear.x = 1
    msg.angular.z = 1

    # Publish the message.
    pub.publish(msg)

    # Send a message to rosout with the details
    rospy.loginfo("Sending random velocity command: "
                  "linear = %.2f "
                  "angular = %.2f", msg.linear.x, msg.angular.z)
    
    # Wait until it's time for another iteration.
    r.sleep()
