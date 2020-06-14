#!/usr/bin/env python

import numpy as np

import sys
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid, MapMetaData

class P1ScoreNode(object):

    def __init__(self):
        rospy.init_node('p1_score_node')
        
        self.answer_loaded = False
        
        self.max_points_demo1 = 12.5
        self.max_points_demo2 = 12.5
        
        self.main_map = None
        
        # Publishes Scores
        self.pub_demo1 = rospy.Publisher("/p1_demo1/score", Float64, queue_size=10)
        self.pub_demo2 = rospy.Publisher("/p1_demo2/score", Float64, queue_size=10)

        # Subs to answers
        rospy.Subscriber("/map", OccupancyGrid, self.load_answer)
        rospy.Subscriber("/p1_demo1/answer", OccupancyGrid, self.score_demo1)
        rospy.Subscriber("/p1_demo2/answer", OccupancyGrid, self.score_demo2)


    def load_answer(self, map_msg):
        self.main_map = np.array(map_msg.data)
        self.answer_loaded = True

    def score_demo1(self, answer):
        score = Float64()
        score.data = -1
        
        if not self.answer_loaded:
            self.pub_demo1.publish(score)
            return
        
        answer_map = np.array(answer.data)
        num_cells = self.main_map.shape[0]
        
        diff = np.absolute(self.main_map - answer_map)
        errors = diff[diff > 0]
        num_errors = errors.shape[0]
        points = self.max_points_demo1 * (1 - num_errors/float(num_cells))
        score.data = points
        self.pub_demo1.publish(score)
        
    def score_demo2(self, answer):
        score = Float64()
        score.data = -1
        
        if not self.answer_loaded:
            self.pub_demo2.publish(score)
            return
        
        answer_map = np.array(answer.data)
        num_cells = self.main_map.shape[0]
        
        diff = np.absolute(self.main_map - answer_map)
        errors = diff[diff > 0]
        num_errors = errors.shape[0]
        points = self.max_points_demo2 * (1 - num_errors/float(num_cells))
        score.data = points
        self.pub_demo2.publish(score)
        
        

if __name__ == "__main__":
    d = P1ScoreNode()
    rospy.spin()
