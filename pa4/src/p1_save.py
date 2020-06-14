#!/usr/bin/env python

import numpy as np

import sys
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid, MapMetaData

class P1SaveNode(object):

    def __init__(self):
        rospy.init_node('p1_save_node')
        
        self.answer_loaded = False
        
        self.main_map = None
        
        # Subs to answers
        rospy.Subscriber("/p1_demo1/answer", OccupancyGrid, self.save_demo1)
        rospy.Subscriber("/p1_demo2/answer", OccupancyGrid, self.save_demo2)


    def load_answer(self, map_msg):
        self.main_map = np.array(map_msg.data)
        self.answer_loaded = True

    def save_demo1(self, answer):
        f = open('demo1_map.meta', 'w')
        f.write('Resolution: ' + str(answer.info.resolution) + '\n')
        f.write('Width: ' + str(answer.info.width) + '\n')
        f.write('Height: ' + str(answer.info.height) + '\n')
        f.close()
        
        data = np.array(answer.data).reshape((answer.info.height, answer.info.width))
        
        np.save('demo1_map.dat', data)
        print 'Done Saving'
        #scipy.misc.imsave('outfile.jpg', data)
        
    def save_demo2(self, answer):
        f = open('demo2_map.meta')
        f.write('Resolution: ' + str(answer.info.resolution) + '\n')
        f.write('Width: ' + str(answer.info.width) + '\n')
        f.write('Height: ' + str(answer.info.height) + '\n')
        f.close()
        
        data = np.array(answer.data)
        np.save('demo2_map.dat', data)
        print 'Done Saving'
        
        

if __name__ == "__main__":
    d = P1SaveNode()
    rospy.spin()
