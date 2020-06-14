#!usr/bin/env python

''' This program subscribes to turtle1/pose and shows its
messages on the screen.
'''

import rospy
from turtlesim.msg import Pose

def poseMessageReceived(msg):
    ''' A callback function. Executed each time a 
        new pose message arrives. '''
        
    rospy.loginfo("position=(,%.2f, %.2f) "
                  "direction=%.2f", msg.x, msg.y, msg.theta)

# Initialize the ROS system and become a node.             
rospy.init_node('subscribe_to_pose')

# Create a subscriber object.
rospy.Subscriber('turtle1/pose', Pose, poseMessageReceived)

# Let ROS take over.
rospy.spin()
