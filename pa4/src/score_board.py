#!/usr/bin/env python

from std_msgs.msg import Float64
import math
import rospy

class ScoreBoard:
    '''
    Listens for scores being published and neatly displays them here

    Attributes:
        p1_demo1_score(float): score for part 1 demo 1
        p1_demo2_score(float): score for part 1 demo 2
        p2_score(float): score for part 2
        p3_demo1_score(float): score for part 3 demo 1
    '''

    def __init__(self):
        rospy.init_node("ScoreBoard")

        #field variables
        self.p1_demo1_score = None
        self.p1_demo2_score = None
        self.p2_score = None
        self.p3_demo1_score = None

        # Subscribers
        rospy.Subscriber(
            "/p1_demo1/score",
            Float64,
            self.on_p1_demo1_score_change)

        rospy.Subscriber(
            "/p1_demo2/score",
            Float64,
            self.on_p1_demo2_score_change)

        rospy.Subscriber("/p2/score", Float64, self.on_p2_score_change)

        rospy.Subscriber(
            "/p3_demo1/score",
            Float64,
            self.on_p3_demo1_score_change)

    def on_p1_demo1_score_change(self, new_score):
        """
        when the score for project 2 is changed

        Args:
            new_score(Float): next score for part
        """
        self.p1_demo1_score = new_score.data

    def on_p1_demo2_score_change(self, new_score):
        """
        when the score for project 2 is changed

        Args:
            new_score(Float): next score for part
        """
        self.p1_demo2_score = new_score.data

    def on_p2_score_change(self, new_score):
        """
        when the score for project 2 is changed

        Args:
            new_score(Float): next score for part
        """
        self.p2_score = new_score.data

    def on_p3_demo1_score_change(self, new_score):
        """
        when the score for project 2 is changed

        Args:
            new_score(Float): next score for part
        """
        self.p3_demo1_score= new_score.data

    def spin(self):
        while not rospy.is_shutdown():
            raw_input("PRESS ENTER TO VIEW CURRENT SCORE")
            print("#"*50)
            if self.p1_demo1_score is not None:
                print("PART 1, DEMO 1: " + str(self.p1_demo1_score))
            if self.p1_demo2_score is not None:
                print("PART 1, DEMO 2: " + str(self.p1_demo2_score))
            if self.p2_score is not None:
                print("PART 2: " + str(self.p2_score))
            if self.p3_demo1_score is not None:
                print("PART 3, DEMO 1: " + str(self.p3_demo1_score))
            print("#"*50)

if __name__ == '__main__':
    sb = ScoreBoard()
    try:
        sb.spin()
    except rospy.ROSInterruptException:
        pass
