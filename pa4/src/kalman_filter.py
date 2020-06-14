#!/usr/bin/env python

# Jackie Scanlon
# PA4 - Part 1 
# Task: Localize using Kalman Filter
# 4/11/19


import rospy
import time
import numpy as np
import math

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, PoseArray, PoseStamped, Point, Quaternion
import message_filters
from tf.transformations import quaternion_from_euler


class KalmanFilterNode(object):
    '''
        KalmanFilterNode: Publishes the robot's current pose using
        kalman filtering.
    '''

    def __init__(self):
        rospy.init_node('kalman_filter_node')

        # Current twist
        self.twist = Twist()

        # Array of r and beta's for landmarks
        self.landmarks = {}

        # True landmark locations
        self.true_landmarks = {}

        # Dictionary for q: keys are landmark id, entries are list with xi and yi
        # key -1 is for vehicle pose
        self.q = {}
        self.q[-1] = [1., 2., 0.]

        # Order that the landmarks are stored in q and P
        self.q_order = []

        # Covariance of state
        self.P = np.zeros((3,3))

        # Covariance matricies for noise
        self.V_hat = np.array([[.1**2, 0.], [0., .1**2]])
        self.W_hat = np.array([[.1**2, 0.], [0., .1**2]])

        # To compute dt
        self.dt = 0.
        self.time = rospy.get_time()

        # Publishers
        self.pose_pub = rospy.Publisher('/my_kalman_filter', PoseStamped, queue_size=1)
        
        # Subscribers
        self.vel_sub = rospy.Subscriber('/robot0/input_vel', Twist, self.updateTwist)        
        self.landmark_sub = rospy.Subscriber('/landmarks', PoseArray, self.kalmanFilter)
        self.true_landmark_sub = rospy.Subscriber('/true_landmarks', PoseArray, self.updateTrueLandmarks)
        rospy.spin()
        

    def updateTwist(self, twist_msg):
        ''' Update self.twist '''

        self.twist = twist_msg


    def kalmanFilter(self, landmarks_msg):
        ''' Updates the current pose based on a new landmark reading.'''

        # Wait for true landmarks to be published
        while len(self.true_landmarks) == 0:
            time.sleep(.5)

        new_landmarks = self.landmarksToArray(landmarks_msg)
    
        # If we've lost landmarks, reduce size of x and P
        if self.landmarks.viewkeys() != new_landmarks.viewkeys():
            self.reduceSize(new_landmarks)

        # Update our landmark array (sensor input)
        self.updateLandmarks(new_landmarks)

        # Perform prediction
        self.prediction()

        # If there are landmarks left, perform correction
        if len(self.landmarks) > 0:
            self.correction()

        # If there are new landmarks, add them
        if self.landmarks.viewkeys() != new_landmarks.viewkeys():
            self.expand(new_landmarks)

        # Publish the pose
        my_pose_stamped = PoseStamped()
        my_header = Header()
        my_header.frame_id = 'map_static'
        my_header.stamp = rospy.Time.now()
        my_pose_stamped.header = my_header
        c = quaternion_from_euler(0, 0, self.q[-1][2])
        my_pose_stamped.pose = Pose(Point(self.q[-1][0],self.q[-1][1],0.), Quaternion(c[0], c[1], c[2], c[3]))

        self.pose_pub.publish(my_pose_stamped)


    def updateLandmarks(self, new_landmarks):
        ''' Updates self.landmarks based on the new message '''

        for key, noisy_pose in self.landmarks.items():
            self.landmarks[key] = list(new_landmarks[key])


    def updateTrueLandmarks(self, pose_array_msg):
        ''' Reads in the true position of the landmarks.'''

        for pose in pose_array_msg.poses:
            self.true_landmarks[pose.orientation.w] = [pose.position.x, pose.position.y]

    def expand(self, new_landmarks):
        ''' Expands q and P with the addition of new landmarks. '''

        # Vehicle pose
        xv = self.q[-1][0]
        yv = self.q[-1][1]
        theta_v = self.q[-1][2]

        # Number of landmarks
        N = len(self.landmarks)

        if N == 1:
            # don't add the new landmark if we already have 1
            # BUG FIX (in actuality, should be able to handle multiple landmarks)
            return

        # Go through the new landmarks
        for land_id, noisy_pose in new_landmarks.items():

            # If the landmark isn't already in our list, add it
            if land_id not in self.q:
                rospy.loginfo('Adding landmark ' + str(land_id))
                self.q_order.append(land_id)

                # Read in r and beta
                r = noisy_pose[0]
                b = noisy_pose[1]

                # g function to update state
                self.q[land_id] = [xv + r*math.cos(theta_v + b),
                                   yv + r*math.sin(theta_v + b)]

               # Update p
                G_z = np.array([[math.cos(theta_v+b), -r*math.sin(theta_v+b)],
                               [math.sin(theta_v+b), r*math.cos(theta_v+b)]])

                new_P = np.matmul(np.matmul(G_z, self.W_hat), G_z.T)

                P_mat = np.vstack((self.P, np.zeros((2, 3+2*N))))

                new_cols = np.zeros((3+2*N, 2))
                new_cols =np.vstack((new_cols, new_P))

                self.P = np.hstack((P_mat, new_cols))

                # Change the stored landmark value
                self.landmarks[land_id] = noisy_pose


    def matrixToQ(self, q_matrix):
        ''' Converts q in matrix form back to dictionary form. '''

        # Number of landmarks
        N = len(self.landmarks)

        # Restore the state of the robot
        self.q[-1] = [q_matrix[0,0], q_matrix[1,0], q_matrix[2,0]]

        # Which landmark are we on
        j = 0

        # Restore the state of the landmarks
        for q_id in self.q_order:
            self.q[q_id] = [q_matrix[2*j+3,0], q_matrix[2*j+4,0]]
            j = j + 1

    def qToMatrix(self):
        ''' Converts q, in its dictionary form, into a matrix. '''

        N = len(self.landmarks)

        q_matrix = np.zeros((3+2*N, 1))

        # Set the robot state
        pose = self.q[-1]
        q_matrix[0,0] = pose[0]
        q_matrix[1,0] = pose[1]
        q_matrix[2,0] = pose[2]

        j = 0

        # Set the landmark state
        for q_id in self.q_order:

            pose = self.q[q_id]
            q_matrix[2*j+3,0] = pose[0]
            q_matrix[2*j+4,0] = pose[1]

            j = j + 1

        return q_matrix


    def correction(self):
        ''' Performs correction step of EKF. '''

        tau = 2*math.pi

        # Vehicle state
        xv = self.q[-1][0]
        yv = self.q[-1][1]
        theta_v = self.q[-1][2]
        N = len(self.landmarks)

        # h and its jacobians
        h = np.zeros((2*N,1))
        H_x = np.zeros((2*N, 3+2*N))
        H_w = np.zeros((2*N, 2))

        # Sensor reading
        z = np.zeros((2*N, 1))

        # Innovation
        v = np.zeros((2*N,1)) 

        j = 0

        # Update h, Hx, Hw, z, v (innovation)
        for land_id in self.q_order:

            noisy_pose = self.landmarks[land_id]
            xi = self.true_landmarks[land_id][0]
            yi = self.true_landmarks[land_id][1]

            
            d = math.sqrt((yi - yv)**2 + (xi - xv)**2)

            h[2*j, 0] = d
            delta = math.atan2((yi-yv),(xi-xv)) - theta_v
            h[2*j+1, 0] = min(delta, delta + tau, delta - tau, key=abs)

            H_x[2*j, 0] = -(xi-xv)/d
            H_x[2*j, 1] = -(yi-yv)/d
            H_x[2*j+1, 0] =  (yi-yv)/d**2
            H_x[2*j+1, 1] = -(xi-xv)/d**2 
            H_x[2*j+1, 2] = -1.

            H_x[2*j, 2*j+3] = (xi-xv)/d
            H_x[2*j, 2*j+4] = (yi-yv)/d
            H_x[2*j+1, 2*j+3] = -(yi-yv)/d**2
            H_x[2*j+1, 2*j+4] = (xi-xv)/d**2

            H_w[2*j,0] = 1.
            H_w[2*j+1, 1] = 1.

            z[2*j, 0] = noisy_pose[0]
            z[2*j+1, 0] = noisy_pose[1]
            
            v[2*j, 0] = z[2*j, 0] - h[2*j, 0]
            delta = z[2*j+1, 0] - h[2*j+1, 0]
            v[2*j+1, 0] = min(delta, delta + tau, delta - tau, key=abs)

            j = j+1

        S = np.matmul(np.matmul(H_x,self.P),H_x.T) + np.matmul(np.matmul(H_w,self.W_hat),H_w.T)
        K = np.matmul(np.matmul(self.P,H_x.T),np.linalg.pinv(S))

        # self.q is a dictionary, so have to update carefully
        q_matrix = self.qToMatrix()
        q_matrix = q_matrix + np.matmul(K,v)
        self.matrixToQ(q_matrix)

        # Update P
        self.P = np.matmul(np.identity(3+2*N)-np.matmul(K,H_x), self.P)


    def prediction(self):
        ''' Performs prediction using new twist information. '''

        # change in time
        current_time = rospy.get_time()
        self.dt = current_time - self.time
        self.time = current_time

        # Control input
        u = [self.dt * self.twist.linear.x, self.dt * self.twist.angular.z]

        # Calculate f --> update q
        xv = self.q[-1][0]
        yv = self.q[-1][1]
        theta_v = self.q[-1][2]
        N = len(self.landmarks)

        self.q[-1][0] = xv + u[0]*math.cos(theta_v)
        self.q[-1][1] = yv + u[0]*math.sin(theta_v)
        self.q[-1][2] = theta_v + u[1]

        # Landmark positions don't change in prediction step.

        # Calculate F_x (want this to be a matrix)
        F_x = np.identity(3+2*N, dtype=float)

        F_x[0:3,0:3] = np.matrix([[1, 0, -u[0]*math.sin(theta_v)], 
                                  [0, 1, u[0]*math.cos(theta_v)],
                                  [0, 0, 1]])

        # Calculate F_v 
        F_v = np.zeros((3+2*N, 2))

        F_v[0,0] = math.cos(theta_v)
        F_v[1,0] = math.sin(theta_v)
        F_v[2,1] = 1

        # Update P
        self.P = np.matmul(np.matmul(F_x, self.P),F_x.T) + np.matmul(np.matmul(F_v, self.V_hat), F_v.T)


    def reduceSize(self, new_landmarks):
        '''Used to shrink the size of x and P.'''

        # Loop through current landmarks list 

        j = 0
        for key in self.q_order:

            

            if key not in new_landmarks:
                rospy.loginfo('Removing landmark ' + str(key))

                # Delete the old values
                del self.q[key]
                del self.landmarks[key]
                self.q_order.remove(key)
                self.P = np.delete(self.P, [2*j, 2*j+1], 0)
                self.P = np.delete(self.P, [2*j, 2*j+1], 1)
            j = j + 1


    def landmarksToArray(self, msg):
        ''' Convert the landmarks PoseArray into a dictionary.'''

        landmarks = {}
        for pose in msg.poses:
            landmarks[int(pose.orientation.w)] = [pose.position.x, pose.position.y]

        return landmarks


if __name__ == "__main__":
    KalmanFilterNode()
