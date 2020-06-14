# rrt.py backup

	def isFeasible(self,pose_array,r):
        ''' Figures out if a given pose (in array form) is feasible
        given the specified radius that the robot needs. '''

        x = pose_array[0]
        y = pose_array[1]
        theta = pose_array[2]

        # Cycle through all values within the map. Check occupancy grid value
        # and if its within the borders of the map
        for k in range(x-r, x+r+1):
            if k < 0 or k > self.map_meta_data.width:
                return False
            for m in range(y-r, y+r+1):
                if m < 0 or m > self.map_meta_data.height:
                    return False
                if self.map[k,m] != 0:
                    return False 

        return True


    def drawEdges(self):
        ''' draws the edges in the graph. '''
        
        # Go through each point in the graph
        for point in self.g:

            # Holds 
            path = Path()
            path_poses = []
            for neighbor in self.g[point]:
                pose = self.stringToPose(neighbor)
                p = PoseStamped()

                p.pose.position.x = pose[1]*self.map_meta_data.resolution
                p.pose.position.y = pose[0]*self.map_meta_data.resolution
                p.header.frame_id = 'map'

            path_poses.append(p)
            path.poses = path_poses
            path.header.frame_id = 'map'
            self.edge_pub.publish(path)


        def getNearest(self, pose):
        ''' Get the nearest vertex to the given pose. '''

        # Start with the first vertex
        nearest = self.stringToPose(self.g.keys()[0])

        # Set minimum distance to beat
        min_dist = math.sqrt((pose[0]-nearest[0])**2 + (pose[1]-nearest[1])**2)

        # Go through each vertex in the graph
        for p in g:

            point = self.stringToPose(p)
            d = math.sqrt((pose[0]-point[0])**2 + (pose[1]-point[1])**2)

            # If there is a closer point, make it the nearest point
            if d < min_dist:
                min_dist = d
                nearest = point

        return nearest

    def getNearest2(self, pose, g):
        ''' Get nearest point within a certain radius,
            and make sure the edge is feasible.'''

        # Maximum radius away we wll look (in cells)
        max_rad = 50

        # Start with the first vertex
        nearest = self.stringToPose(g.keys()[0])

        # Set minimum distance to beat
        min_dist = math.sqrt((pose[0]-nearest[0])**2 + (pose[1]-nearest[1])**2)

        # Go through each vertex in the graph
        for p in g:

            point = self.stringToPose(p)
            d = math.sqrt((pose[0]-point[0])**2 + (pose[1]-point[1])**2)

            # If there is a closer point, make it the nearest point
            if d < min_dist:
                min_dist = d
                nearest = point

        # Check if nearest is not within the max radius
        if min_dist > max_rad:
            return None

        # Now check that the line is feasible


        # NOT CURRENTLY IN USE
    def getCSpace(self):
        '''
        turns the Occupancy Grid into a C-Space.
        '''

        print('Starting to make C-Space')
        # Start with what we currently have
        self.c_space = self.map

        height = self.map_meta_data.height
        width = self.map_meta_data.width
        res = self.map_meta_data.resolution

        # Get the radius we are going to impose
        r = int(math.ceil(max(self.robot_width, self.robot_height)/res))

        # Process center of the map first
        for i in range(r, width-r+1):
            print('Processing row ' + str(i))
            for j in range(r,height-r+1):
                if self.map[str(i) + ' ' + str(j)] == 100: # or greater than some threshold
                    # Black out cells to left and right of this one
                    for k in range(i-r, i+r+1):
                        for m in range(j-r, j+r+1):
                            if self.c_space[str(m) + ' ' + str(k)] != 100:
                                self.c_space[str(m) + ' ' + str(k)] = 100
        '''
        # Process center of the map first
        for i in range(0, r):
            print('Processing row ' + str(i))
            for j in range(r,height-r+1):
                if self.map[str(i) + ' ' + str(j)] == 100: # or greater than some threshold
                    # Black out cells to left and right of this one
                    for k in range(i-r, i+r+1):
                        for m in range(j-r, j+r+1):
                            self.c_space[str(m) + ' ' + str(k)] = 100
                            # If the cell is within bounds of the map
                            if k in range(0,width) and m in range(0, height):
        '''
        print('Finished processing, publishing now')
        # Publish the map (for testing)
        occ_grid = OccupancyGrid()
        occ_grid.map_info = map_meta_data

        temp = list(self.c_space)

        occ_grid.ros_occ_grid.data = np.asarray(temp)
        self.map_pub.publish(occ_grid)