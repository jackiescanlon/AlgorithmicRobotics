#!/usr/bin/env python

# Jackie Scanlon
# PA3
# 3/28/19

import rospy
import tf
import numpy as np
import math

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Float64
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
import message_filters
from tf.transformations import euler_from_quaternion


class MappingNode(object):
    '''
        MappingNode: Publishes an Occupancy grid. The grid is generated
        from lidar laser scans.
    '''

    def __init__(self):
        rospy.init_node('mapping_node')
        
        # Mapping Constants
        self.origin = Pose()
        self.origin.position.x = -25.
        self.origin.position.y = -25.
        self.origin.orientation.w = 1.
        self.map_info = MapMetaData()
        self.map_info.map_load_time = rospy.Time.now()
        self.map_info.resolution = .1
        self.map_info.height = 1000
        self.map_info.width = 1000
        self.map_info.origin = self.origin
        
        # Sensor Model
        self.alpha = 0.9
        self.beta = 0.7
        
        # Transform(s)
        # sensor to robot
        self.T_sensor = np.array([[np.cos(0), -1*np.sin(0), 0.27],
                                 [np.sin(0),  np.cos(0),    0.0],
                                 [0.0,        0.0,          1.0]])

        # robot to world
        self.T_robot = np.array([[np.cos(0), -1*np.sin(0), 0.0],
                                 [np.sin(0), np.cos(0), 0.0],
                                 [0.0, 0.0, 1]])
                                 
        # Occupancy grid variable (array)
        
        # occ_grid is a numpy matrix
        # Prefill with 0.5 for unknown occupancy
        self.occ_grid = np.repeat(np.matrix(np.repeat(.5, self.map_info.width)), self.map_info.height, axis=0)
        
        # ros_occ_grid is a ROS OccupancyGrid instance
        self.ros_occ_grid = OccupancyGrid()
        self.ros_occ_grid.info = self.map_info
        
        # Publishers
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        
        # Subscribers
        # pose_sub and scan_sub are synchronized.
        # when the updateMap callback is executed it receives a message from both.
        pose_sub = message_filters.Subscriber('/groundtruth', Odometry)
        scan_sub = message_filters.Subscriber('/r2d2/laser/scan', LaserScan)
        ts = message_filters.TimeSynchronizer([pose_sub, scan_sub], 10)
        ts.registerCallback(self.updateMap)
        
        
        rospy.spin()
        
    def laserArray(self, scan_msg):
        # Build array of laser angles
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)
        # Ranges not between range_max and range_min are invalid, remove them
        obs_ind =(ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
        
        # For the remaining readings, interpret as free space
        free_ind = ranges >= scan_msg.range_max
        
        # Get the obstacle readings
        obs_ranges = np.extract(obs_ind, ranges)
        obs_angles = np.extract(obs_ind, angles)
        
        # Get the free space readings and set a free space up to maximum laser range / 2 (for safety)
        free_angles = np.extract(free_ind, angles)
        free_ranges = np.repeat(scan_msg.range_max, len(free_angles))
        
        # Turn the sensor ranges and thetas into x,y components
        x_obs_array = np.multiply(obs_ranges, np.cos(obs_angles))
        y_obs_array = np.multiply(obs_ranges, np.sin(obs_angles))
        x_free_array = np.multiply(free_ranges, np.cos(free_angles))
        y_free_array = np.multiply(free_ranges, np.sin(free_angles))
        
        # Stack into a 2xn array
        laser_obs_array = np.vstack((x_obs_array, y_obs_array))
        laser_free_array = np.vstack((x_free_array, y_free_array))
        
        # Combine into a list
        laser_array = [laser_obs_array, laser_free_array]
        
        # Iterate through each col of the laser arrays by transposing them
        # (numpy iterates through rows)
        for laser in laser_array:
            for p in laser.T:
            
                # Transform to world frame                
                pos = np.matmul(self.T_robot, np.matmul(self.T_sensor, np.array([p[0], p[1], 1])))
                
                # Convert the metric grid into a cell grid (int values)
                # Save in the same array
                p[0] = math.floor((pos[0] - self.origin.position.x)/self.map_info.resolution)
                p[1] = math.floor((pos[1] - self.origin.position.y)/self.map_info.resolution)
        
        # Convert to int array
        laser_array = [laser_array[0].astype(int), laser_array[1].astype(int)]
        
        # Send back
        return laser_array
        
    def lineSearch(self, laser_array, x0, y0):
        # Source: https://en.wikipedia.org/wiki/Bresenham's_line_algorithm
                
        # Contains a list of lines that start with x0,y0 and contain free space
        grid_obs_list = [] # terminates in an object
        grid_free_list = [] # terminates in free space

        # Keeps track of which laser/grid we are on
        laser_num = 0

        for laser in laser_array:
            
            # Set grid_list according to which laser it is
            if laser_num == 0:
                grid_list = grid_obs_list
            else:
                grid_list = grid_free_list
                
            line_counter = 0
            # Transpose to iterate through cols
            for p in laser.T:

                x1 = p[0]
                y1 = p[1]
                deltax = float(x1 - x0)
                deltay = float(y1 - y0)

                if abs(deltay) < abs(deltax):
                    if x0 > x1:
                        line,counter = self.plotLineLow(x1, y1, x0, y0)
                        line = np.fliplr(line)
                    else:
                        line,counter = self.plotLineLow(x0, y0, x1, y1)
                
                else:
                    if y0 > y1:
                        line,counter = self.plotLineHigh(x1, y1, x0, y0)
                        line = np.fliplr(line)
                    else:
                        line,counter = self.plotLineHigh(x0, y0, x1, y1)

                # Cut off x0 and y0
                line = line[:,1:counter].astype(int)

                # Add the line to the list of lines update
                if len(line) != 0:
                    grid_list.append(line)
                    
            # Switch to second laser/grid
            laser_num = 1
                    
        # Combine the lists
        grid_list = [grid_obs_list, grid_free_list]
        
        # Send back the lists of lines
        return grid_list

    def plotLineHigh(self, x0, y0, x1, y1):
                   
        dx = x1 - x0
        dy = y1 - y0
        xi = 1
        if dx < 0:
            xi = -1
            dx = -dx

        D = 2*dx - dy
        x = x0

        counter = 0
        # Set up empty 2xn matrix for the points to fill
        line = np.zeros((2, max(abs(x1-x0)+1, abs(y1-y0)+1)), dtype=int)
         
        for y in range(y0, y1+1):
            line[0,counter] = int(x)
            line[1, counter] = int(y)
            counter+=1

            if D > 0:
               x = x + xi
               D = D - 2*dy
            D = D + 2*dx

        return line[:,0:counter], counter



    def plotLineLow(self, x0,y0, x1,y1):

        dx = x1 - x0
        dy = y1 - y0
        yi = 1
        if dy < 0:
            yi = -1
            dy = -dy

        D = 2*dy - dx
        y = y0

        counter = 0
        # Set up empty 2xn matrix for the points to fill
        line = np.zeros((2, max(abs(x1-x0)+1, abs(y1-y0)+1)), dtype=int)

        for x in range(x0, x1+1):
            line[0,counter] = int(x)
            line[1, counter] = int(y)
            counter += 1
            if D > 0:
                y = y + yi
                D = D - 2*dx

            D = D + 2*dy

        return line[:,0:counter], counter


        
    def updateProbabilities(self, grid_list, x0, y0):
        
        # Keep track of which grid we are on
        grid_num = 0
        
        for grid in grid_list:
        # Iterate through each line
            for line in grid:

                # if line contains more than just x0 y0
                if np.size(line, 0) == 2 and np.size(line, 1) > 1: 

                    # interate through each cell
                    for i in range(0, np.size(line,1)):
                    
                        # Get grid cell
                        x = line[0, i]
                        y = line[1, i]
                        
                        # Only do if we are in the correct range for the map
                        if x < 0 or x >= self.map_info.height or y < 0 or y >= self.map_info.width:
                            pass
                        else:
                            z = self.occ_grid[y,x]
                            
                            if z != 0: # If 0, no way to be not 0 anymore
                            
                                # If we are on the list with obstacles terminating the lines
                                if grid_num == 0:
                                    # Empty grid cell - set probability
                                    if i in range(0, np.size(line, 1)-1):
                                        self.occ_grid[y,x] = (1-self.alpha)*z/(self.beta*(1-z) + z*(1-self.alpha))
                                        
                                    # Occupied grid cell - set probability
                                    else:
                                        self.occ_grid[y,x] = self.alpha*z/(self.alpha*z + (1-self.beta)*(1-z))
                                        
                                # If we are on the list with empty space only
                                else:
                                    self.occ_grid[y,x] = (1-self.alpha)*z/(self.beta*(1-z) + z*(1-self.alpha))
            
            # After doing obstacle grid, switch to free space grid
            grid_num = 1
            
        # At the end, set x0, y0 value to 0
        self.occ_grid[y0, x0] = 0
        
    def publishOccupancyGrid(self):
    
        # Convert the grid to the right shape
        # Convert to 100 scale
        pre_occ_grid = (100*self.occ_grid).astype(int)
        self.ros_occ_grid.data = np.ravel(pre_occ_grid.reshape(1,-1)).tolist()
        
        # Publish the grid
        self.map_pub.publish(self.ros_occ_grid)
        
    def updateMap(self, odom_msg, scan_msg):
        ''' 
            Parameters:
                odom_msg - nav_msgs/Odometry - robot pose in world frame
                scan_msg - sensor_msgs/LaserScan - lidar reading
        '''
        
        # Determine the homogenous transform for the robot position.
        # This will be used for computing points in the world frame.
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
                                 
        # Get x0 and y0, which is the robot's location in the world frame
        # Use it from the sensor because that it where we will start our lines
        # in the line algorithm at
        x0 = x #+.27
        y0 = y
        
        # Convert laser origin from metric to cells
        x0 = int((x0 - self.origin.position.x)/self.map_info.resolution)
        y0 = int((y0 - self.origin.position.y)/self.map_info.resolution)
        
        # Transform from robot to world
        self.T_robot = np.array([[np.cos(theta), -1*np.sin(theta), x],
                                 [np.sin(theta),  np.cos(theta), y],
                                 [0.0,        0.0,          1.]])
        
        # Build the laser angle array and transform to world frame
        laser_array = self.laserArray(scan_msg)
        
        # Run the line search
        grid_list = self.lineSearch(laser_array, x0, y0)
        
        # Update occupancy probabilities
        self.updateProbabilities(grid_list, x0, y0)
        
        # Publish the occupancy grid. Note: values should be 0 to 100
        self.publishOccupancyGrid()
              
if __name__ == "__main__":
    MappingNode()