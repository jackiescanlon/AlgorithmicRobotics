#!/usr/bin/env python

from std_msgs.msg import Bool
from std_msgs.msg import Float64
import math
import rospy

class P2ScoreNode(object):
    """
    Scoring node for part two of the assignment

    Attributes:
        count(int): counts the number of points reached
    """
    def __init__(self):
        rospy.init_node('p2_score_node')

        #parameters
        self.score = 0

        # Publishers
        self.score_pub = rospy.Publisher("/p2/score", Float64, queue_size=10)

        # Subscribers
        rospy.Subscriber("/goal_reached", Bool, self.on_goal_reached)

    def on_goal_reached(self, b):
        """
        Callback invoked when the goal for the goal_input node is either met,
        or was failed to be achieved in the alloted time

        Args:
            b(Bool): whether the goal pose was achieved or not (increments
            self.score if it was acheived)
        """
        goal_reached = b.data
        if goal_reached:
            self.score += 1

        current_score = Float64()
        current_score.data = self.score
        self.score_pub.publish(current_score)



if __name__ == "__main__":
    d = P2ScoreNode()
    rospy.spin()
