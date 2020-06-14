#!/usr/bin/env python

# drive.py for PA2
# Jackie Scanlon
# 2/21/19

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DriverNode(object):
    '''
    DriverNode subscribes to a user defined Twist topic and converts that twist
    message into right and left wheel velocities (that it publishes to R2D2)

    Attributes:
        pub_left_wheel (Publisher): publisher for the left wheel
        pub_right_wheel (Publisher): publisher for the right wheel
    '''
        
    def __init__(self):
        rospy.init_node('driver_node')
        
        # Publishers for wheels
        self.pub_right_wheel = rospy.Publisher("/r2d2/right_velocity_controller/command", Float64, queue_size=10)
        self.pub_left_wheel = rospy.Publisher("/r2d2/left_velocity_controller/command", Float64, queue_size=10)
        
        # Subscriber for Twist commands
        rospy.Subscriber('/r2d2/cmd_vel', Twist, self.unicycle_drive)
        
        rospy.spin()
        
    def unicycle_drive(self, twist_msg):
        '''
        converts a twist_msg to wheel velocity commands to make R2D2 move

        Args:
            twist_msg(Twist): twist message for speed and rotational velocity
        '''
        
        R = 0.2 # Wheel radius
        L = 0.5 # Distance between wheels
        
        v = twist_msg.linear.x
        w = twist_msg.angular.z
        
        # Determine wheel velocities
        right_wheel_msg = v + w*L/2
        left_wheel_msg = v - w*L/2
        
        # Publish wheel velocities
        self.pub_right_wheel.publish(right_wheel_msg)
        self.pub_left_wheel.publish(left_wheel_msg)
        
        pass

        
if __name__ == "__main__":
    DriverNode()
