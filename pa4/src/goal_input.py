#!/usr/bin/env python
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import Bool
import tf


class GoalInput:
    '''
    queries the user for an x and y value and publishes that as the new goal
    location; it listens for pose information to announce when the goal has
    been met; the goal is meant for a 2-D platform

    Attributes:
        robot_pose(Point): the pose current pose of the robot
    '''
    def __init__(self):
        rospy.init_node("GoalInput")

        self.robot_loc = Pose2D()

        # Publsihers
        self.goal_pub = rospy.Publisher("/goal", Pose2D, queue_size=10)
        self.goal_reached_pub = rospy.Publisher("/goal_reached", Bool, queue_size=10)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.on_pose_change)

    def on_pose_change(self, odometry):
        self.robot_loc.x = odometry.pose.pose.position.x
        self.robot_loc.y = odometry.pose.pose.position.y
        self.robot_loc.theta =  tf.transformations.euler_from_quaternion([odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w])[2]

    def spin(self):
        while not rospy.is_shutdown():
            loc = Pose2D()
            print("INPUT: X, Y and Theta GOAL POSE")
            print("#"*50)
            loc.x = input("x: ")
            loc.y = input("y: ")
            loc.theta = input("theta: ")
            self.goal_pub.publish(loc)
            print("PUBLISHING...")
            print("#"*50)
            print("PUBLISHED! AWAITING GOAL")
            cut_off = 60
            while not rospy.is_shutdown():
                rospy.sleep(0.5) #sleeps for 1 second
                if not self.robot_loc:
                    continue # robot_pose is None, check pose is working
                diff_x = abs(self.robot_loc.x - loc.x)
                diff_y = abs(self.robot_loc.y - loc.y)
                diff_theta = abs(self.robot_loc.theta - loc.theta)
                if max(diff_y, diff_x, diff_theta) <= 0.5:
                    print("GOAL REACHED!")
                    b = Bool()
                    b.data = True
                    self.goal_reached_pub.publish(b)
                    break
                '''if cut_off <= 0: # past 30 seconds, didn't make it too goal
                    print("FALURE TO REACH GOAL!")
                    b = Bool()
                    b.data = False
                    self.goal_reached_pub.publish(b)
                    break
                '''
                cut_off -= 1

if __name__ == '__main__':
    g = GoalInput()
    try:
        g.spin()
    except rospy.ROSInterruptException:
        pass
