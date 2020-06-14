#!/usr/bin/env python

import numpy as np

import sys
import rospy
from std_msgs.msg import Float64

class P3ScoreNode(object):

    def __init__(self):
        rospy.init_node('p3_score_node')
        
        self.is_ready = False
        
        self.max_points_demo1 = 25.0 
        self.num_victims = 7.0
        self.points_per_victim = self.max_points_demo1 / self.num_victims
        
        self.dist_threshold = 0.1
        self.points_per_dist = 0.5
        
        self.points_per_copy = 2
        
        self.true_poses = np.array([[4.0, 1.0, 0.0],
                                    [2.0, 8.0, 0.0],
                                    [11.0, 3.0, 0.0],
                                    [14.0, 4.5, 0.0],
                                    [14.0, 14.0, 0.0],
                                    [9.0, 10.5, 0.0],
                                    [3.0, 14.0, 0.0]])
        
        
        try:
            self.answer = np.loadtxt('victim_report.csv', delimiter=',')
            self.is_ready = True
        except IOError:
            pass
        self.pub_demo1 = rospy.Publisher("/p3_demo1/score", Float64, queue_size=10)
        rospy.Subscriber("/p3_demo1/answer", Float64, self.score_demo1)
        
        #while True:
        #    self.answer = np.loadtxt('victim_report.csv', delimiter=',')
        #    self.score_demo1(answer)
        

    def score_demo1(self, msg):
        if not is_ready:
            return
    
        score = Float64()
        score.data = 0
        
        closest_victims = []
        
        for i in range(0, self.answer.shape[0]):
            curr = self.answer[i, :]
            dist = np.sqrt(np.sum((curr - self.true_poses)**2, axis=1))
            idx = np.argmin(dist)
            closest_victims.append(idx)
            
            min_dist = np.min(dist)
            penalty = -1 * self.points_per_dist * (min_dist / self.dist_threshold)
            curr_points = self.points_per_victim - penalty
            if curr_points < 0:
                curr_points = 0
            score.data = score.data + curr_points
            
        counted = []
        for victim in closest_victims:
            if victim not in counted:
                counted.append(victim)
                total = closest_victims.count(victim)
                score.data = score.data - (total * self.points_per_copy)
        
        if score.data < 0:
            score.data = 0
        
        self.pub_demo1.publish(score)
        
        
        

if __name__ == "__main__":
    d = P3ScoreNode()
    rospy.spin()
