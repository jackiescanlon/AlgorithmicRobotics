#!/usr/bin/env python

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
        # Hint: use rostopic info to get message type
        self.pub_left_wheel = rospy.Publisher(
            "/r2d2/left_velocity_controller/command",
            Float64,
            queue_size=10)

        self.pub_right_wheel = rospy.Publisher(
            "/r2d2/right_velocity_controller/command",
            Float64,
            queue_size=10)

        # Subscriber for Twist commands
        rospy.Subscriber("/r2d2/cmd_vel", Twist, self.on_velocity_change)

    def on_velocity_change(self, twist_msg):
        '''
        converts a twist_msg to wheel velocity commands to make R2D2 move

        Args:
            twist_msg(Twist): twist message for speed and rotational velocity
        '''

        R = 0.2 # Wheel radius
        L = 0.5 # Distance between wheels

        speed = 2.0*twist_msg.linear.x
        omega = twist_msg.angular.z

        # Make right and left wheel messages
        right_wheel = Float64()
        left_wheel = Float64()

        # populate wheel velocity data
        right_wheel.data = (speed + (L*(omega)))/(2.0*R)
        left_wheel.data = (speed - (L*(omega)))/(2.0*R)

        # Publish wheel velocities
        self.pub_left_wheel.publish(left_wheel)
        self.pub_right_wheel.publish(right_wheel)


if __name__ == "__main__":
    d = DriverNode()
    rospy.spin()
