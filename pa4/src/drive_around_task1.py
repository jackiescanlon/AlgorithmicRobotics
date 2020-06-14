#!/usr/bin/env python

# Jackie Scanlon
# PA4 - Part 3
# Task: Use PRM to plan a path to 10 random goal points.
# 4/8/19


import rospy
import time
import numpy as np
import math
import random

from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData, Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose2D, Point, Point32, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import operator


class driveAroundNode(object):
    '''
        driveAroundNode: Plans a path and publishes that path given a map for 10 goals
    '''

    def __init__(self):
        rospy.init_node('drive_around_node')

        # Initial pose as the start location, and goal (stored as strings)
        self.start = None
        self.goal = None 

        # 10 random goals
        self.num_goals = 10

        # How accuarately do we need to achieve the goals
        self.rho = 12 #In cells, not in m

        # List of goals we've already achieved
        self.goals = []

        # How far away do the goals have to be from eachother (in cells)
        self.far_goal = 200

        # Map
        self.map = None
        self.map_meta_data = MapMetaData()

        # Robot radii -NOT diameter. Adaded in buffer
        self.robot_width = .45 + .1
        self.robot_height = .3 + .1

        # List of robot points for checking for valid points
        self.robot_points = []

        # Gain for how much to weigh orientation
        #self.c = .1

        # How many verticies do we want to randomly generate
        #self.K = 7000 # Simple rooms
        #self.K = 13000 # Sparse obstacles
        self.K = 10000 # Hospital sections

        # How far away can we connect points (in cells)
        self.max_rad = 30
        self.far = 10

        # graph is a dictionary
        # each key is the pose (string form)
        # each entry contains a list of keys that it has edges with
        self.g = {}
        self.blank_graph = {}

        # Path object
        self.path = Path()

        # For generating points
        self.first = True

        # For drawing the prm in RViz
        self.prm_points = PointCloud()

        # Publishers
        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)
        self.prm_pub = rospy.Publisher('/prm', PointCloud, queue_size=1)
        self.start_goal_pub = rospy.Publisher('/start_goal', Odometry, queue_size=1)
        
        # Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.setMap)
        self.pose_sub = rospy.Subscriber('/robot0/odom', Odometry, self.setInitialPose)
        #self.goal_sub = rospy.Subscriber('/goal', Pose2D, self.createPlan) 
        
        
        rospy.spin()
        

    def setMap(self,occupancy_grid):
        '''Sets the map variable and starts working on the c-space. '''

        # Wait for 2 seconds because ROS needs a rest
        time.sleep(2)

        # Set our meta data
        self.map_meta_data = occupancy_grid.info

        # Convert the occupancy grid to a numpy array
        self.map = np.asarray(occupancy_grid.data).reshape((self.map_meta_data.height, -1))

        # Get the points the robot occupies (in robot frame, in cells not meters)
        robot_w = int(math.ceil(self.robot_width/self.map_meta_data.resolution))
        robot_h = int(math.ceil(self.robot_height/self.map_meta_data.resolution))

        for i in range(-robot_w, robot_w+1):
            for j in [-robot_h, robot_h]:
                self.robot_points.append([i,j])

        for j in range(-robot_h+1, robot_h):
            for i in [-robot_w, robot_w]:
                self.robot_points.append([i,j])

        self.createPlan()
        

    def setInitialPose(self,odom_msg):
        '''Sets the initial pose of the robot for path planning.'''
        #rospy.loginfo('Got odom_msg')

        # Wait for map to be published
        while self.map is None:
            rospy.loginfo('Waiting to receive occupancy grid.')
            time.sleep(2)

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        
        # Pull the quaternion from the odom_msg
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)
            
        # Get theta from the quaternion
        (roll,pitch,theta) = euler_from_quaternion(quaternion)   

        # Convert to map frame
        self.start = self.poseToString(self.toMap(Pose2D(x,y,theta)))
        #rospy.loginfo('Got a pose and set start pose.')
        # For testing 
        #if self.first:
        #    goal = Pose2D()
        #    goal.x = 8.f
        #    goal.y = 11.
        #    goal.theta = 0.
        #    self.createPlan(goal)
        #    self.first = False

        # if its the first time, generate points
        if self.first:
            rospy.loginfo('Generating points.')
            start_time = time.time()

            actual_k = 0
            for k in range(0,self.K):
                # Get random new point
                q_rand = self.getRandomPose()

                if self.isFeasible(q_rand) and self.isFar(q_rand):
                    self.blank_graph[q_rand] = []
                    actual_k += 1

            rospy.loginfo('Placed ' + str(actual_k) + ' points.')


            rospy.loginfo('Points generated in ' + str(time.time() - start_time))

            self.first = False


    def createPlan(self):
        '''Publishes an array of poses as the plan to be executed, for each goal.'''
        rospy.loginfo('Waiting points to be generated.')
        while self.first:
            time.sleep(1)

        i = 0
        while i < self.num_goals: 

            # Generate goals
            self.goal = self.createGoal()
            rospy.loginfo('Headed to goal: ' + str(self.goal))

            # Draw the start and goal locations.
            self.drawStartGoal()

            while self.first:
                time.sleep(1)

            # Add the intial and final poses to the graph
            self.g = self.blank_graph.copy()
            self.g[self.start] = []
            self.g[self.goal] = []

            # Draw the points in the prm
            self.drawPRM()

            # Find edges
            self.findEdges()
            
            # Get a path
            path_list = self.getPath()

            if path_list is not None:
                self.drawPath(path_list)
                i += 1

                while(self.getDistance(self.start, self.goal) > self.rho):
                    time.sleep(1)

                rospy.loginfo('Achieved goal')
                self.goals.append(self.goal)

            else:
                rospy.loginfo('No path exists.')
                self.goal = self.start

        rospy.loginfo('Done searching map.')


    def isFar(self, q_rand):
        # Checks if a point is past a certain radius of other points.

        for point in self.blank_graph:
            if self.getDistance(q_rand, point) < self.far:
                return False

        return True

        

    def createGoal(self):
        # Generates a bunch of random goals to try


        q_rand = self.getRandomPose()

        # keep generating until we have a goal that is feasible 
        # and is far enough away from the goals we already hit
        while(not self.isFeasible(q_rand) or not self.isFarGoal(q_rand)):
            q_rand = self.getRandomPose()

        return q_rand

    def isFarGoal(self, q_rand):

        # Same as isFar, but for goals instead of all points
        for point in self.goals:
            if self.getDistance(q_rand, point) < self.far_goal:
                return False
        return True
        

    def drawStartGoal(self):
        ''' Draws the start and goal in RVIZ'''

        # Save as a Odometry message
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'map'

        # Publish the start
        s = self.stringToPose(self.start)
        odom_msg.pose.pose.position = Point(s[1]*self.map_meta_data.resolution, s[0]*self.map_meta_data.resolution, 0)
        q = quaternion_from_euler(0,0,s[2])
        odom_msg.pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        self.start_goal_pub.publish(odom_msg)

        # Publish the goal
        g = self.stringToPose(self.goal)
        odom_msg.pose.pose.position = Point(g[1]*self.map_meta_data.resolution, g[0]*self.map_meta_data.resolution, 0)
        q = quaternion_from_euler(0,0,g[2])
        odom_msg.pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        self.start_goal_pub.publish(odom_msg)


    def drawPath(self, path_list):
        ''' Draws the path that we found.'''

        path_poses = []

        # for each point in the list, convert it appropriately
        for point in path_list:
            pose = self.stringToPose(point)
            p = PoseStamped()

            p.pose.position.x = pose[1]*self.map_meta_data.resolution
            p.pose.position.y = pose[0]*self.map_meta_data.resolution
            p.header.frame_id = 'map'

            path_poses.append(p)

        # Publish it
        self.path.poses = path_poses
        self.path.header.frame_id = 'map'
        self.path_pub.publish(self.path)


    def drawPRM(self):
        '''Draws the randomly generated prm points in RViz.'''

        data = []

        # Go through each vertex in the graph
        for vertex in self.g:

            # Get the list form
            v = self.stringToPose(vertex)

            # Transpose and convert to world frame
            v = Point32(v[1]*self.map_meta_data.resolution, v[0]*self.map_meta_data.resolution, 0)
            
            # Add the array to the cells
            data.append(v)

        # Publish it
        self.prm_points.points = data
        self.prm_points.header.frame_id = 'map'
        self.prm_pub.publish(self.prm_points)


    def getPath(self):
        ''' Gets the fastest path using Dijkstra's Algorithm.'''

        # List of unexplored verticies
        Q = []

        # Distance metric - dictionary
        dist = {}
        #dist_in_Q = {}

        # Previous vertex - dictionary
        prev = {}

        # Initialize
        for v in self.g:
            dist[v] = float("inf")
            #dist_in_Q[v] = float("inf")
            prev[v] = None
            Q.append(v)

        dist[self.start] = 0
        #dist_in_Q[self.start] = 0

        # While there are still nodes left to explore
        while Q:

            # sort dist

            sort_dist = sorted(dist.items(), key=operator.itemgetter(1))

            # Get the vertex in Q with the min dist
            for i in range(0,len(sort_dist)):
                 u = sort_dist[i][0]
                 if u in Q:
                    break

            Q.remove(u)
            #del dist_in_Q[u]

            # If we found the goal, quit here
            if u == self.goal:
                break

            # Go through each neighbor of u that is in Q and calculate dist
            for v in self.g[u]:
                if v in Q:
                    alt = dist[u] + self.getDistance(u, v)
                    if alt < dist[v]:
                        dist[v] = alt
                        prev[v] = u

            del dist[u]

        # Search is done. Compile path goal to start     
        path_list = []

        # Start with the goal
        u = self.goal

        if prev[u] is not None or u == self.start:

            # If we still have previous values to go through
            while u is not None:

                # Add the value to the beginning of path_list
                path_list.insert(0, u)
                u = prev[u]

            # Return the path
            return path_list

        # If no path was found
        return None


    def getDistance(self, u, v):
        ''' Gets the distance between 2 nodes, including theta.'''

        u = self.stringToPose(u)
        v = self.stringToPose(v)
        #dist = math.sqrt((u[0] - v[0])**2 + (u[1] - v[1])**2 + self.c*(u[2] - v[2])**2)
        dist = math.sqrt((u[0] - v[0])**2 + (u[1] - v[1])**2)

        return dist

    def isFeasible(self,pose):
        ''' Figures out if a given pose is feasible, 
        including theta. '''

        pose_array = self.stringToPose(pose)
        x = pose_array[0]
        y = pose_array[1]
        theta = pose_array[2]

        # Rotation matrix 
        rot = np.matrix([[math.cos(theta), -1*math.sin(theta)],[math.sin(theta), math.cos(theta)]])

        # Get all the points within the robot using a transform
        for point in self.robot_points:
            new_point = np.matmul(rot, np.array([point[0],point[1]]))
            k = int(new_point[0,0] + x)
            m = int(new_point[0,1] + y)

            # Check if the point is valid
            if k not in range(0,self.map_meta_data.height):
                return False
            if m not in range(0, self.map_meta_data.width):
                return False
            if self.map[k,m] != 0:
                return False

        return True


    def getNearest(self, pose):
        ''' Get all points within a certain radius'''

        connect = []

        # Go through each vertex in the graph
        for p in self.g:

            point = self.stringToPose(p)
            d = math.sqrt((pose[0]-point[0])**2 + (pose[1]-point[1])**2)

            # Add the point as a neighbor
            if d < self.max_rad:
                connect.append(p)

        return connect


    def findEdges(self):
        ''' Finds all edges between feasible points.'''

        for vertex in self.g:
            self.g[vertex] = self.getNearest(self.stringToPose(vertex))


    def poseToString(self,pose):
        ''' Turns a pose array into a string for dictionary key. '''

        return str(pose[0]) + ' ' + str(pose[1]) + ' ' + str(pose[2])


    def stringToPose(self,s):
        ''' Turns a string from dicitionary key into a pose. '''

        s = s.split(' ')
        return [int(s[0]), int(s[1]), float(s[2])]


    def toMap(self, pose):
        ''' Convert a pose in the world frame into the map frame, cell array '''

        x = int((pose.x - self.map_meta_data.origin.position.x)/self.map_meta_data.resolution)
        y = int((pose.y - self.map_meta_data.origin.position.y)/self.map_meta_data.resolution)
        return [y,x, pose.theta]



    def getRandomPose(self):
        ''' Gets a random pose for map generation'''

        x = random.randrange(0,self.map_meta_data.height)
        y = random.randrange(0, self.map_meta_data.width)
        #theta = random.random()*2*math.pi - math.pi
        theta = round(random.random())*math.pi/2.0

        return self.poseToString([x, y, theta])


if __name__ == "__main__":
    driveAroundNode()
