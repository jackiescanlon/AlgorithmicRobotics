#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class VelRegNode(object):

    def __init__(self):
        rospy.init_node('vel_reg_node')
        
        self.is_blocked = False
        
        # Publishers
        self.vel_pub = rospy.Publisher('/r2d2/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/r2d2/path_vel', Twist, self.path_velocity)
        rospy.Subscriber('/r2d2/obs_vel', Twist, self.obs_velocity)
        
        rospy.spin()
        
    def obs_velocity(self, twist_msg):
        
        if twist_msg.linear.x == 0:
            self.is_blocked = True
            self.vel_pub.publish(twist_msg)
            
        else:
            self.is_blocked = False   
        
    def path_velocity(self, twist_msg):
        
        if twist_msg.linear.x >= 1.:
            twist_msg.linear.x = 0.8
            
        if not self.is_blocked:
            self.vel_pub.publish(twist_msg)
        
if __name__ == "__main__":
    VelRegNode()
