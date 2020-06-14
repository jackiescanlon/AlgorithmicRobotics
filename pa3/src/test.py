import numpy as np
import matplotlib.pyplot as plt

def plotLineHigh(x0, y0, x1, y1):
               
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
     
    for y in range(y0, y1 + 1):
        line[0,counter] = int(x)
        line[1, counter] = int(y)
        counter+=1

        if D > 0:
           x = x + xi
           D = D - 2*dy
        D = D + 2*dx

    return line[:,0:counter],counter



def plotLineLow(x0,y0, x1,y1):

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

    for x in range(x0, x1 + 1):
        line[0,counter] = int(x)
        line[1, counter] = int(y)
        counter += 1
        if D > 0:
            y = y + yi
            D = D - 2*dx

        D = D + 2*dy

    return line[:,0:counter],counter

if __name__== "__main__":
    x0 = 0
    y0 = 0
    laser_array = [np.array([[100, 100, -100, -100],
           [100, -100, 100, -100]]), np.array([[],[]])]


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
            print('p: ' + str(p[0]) + '   ' + str(p[1]))
            x1 = p[0]
            y1 = p[1]
            deltax = float(x1 - x0)
            deltay = float(y1 - y0)

            if abs(deltay) < abs(deltax):
                if x0 > x1:
                    line,counter = plotLineLow(x1, y1, x0, y0)
                    line = np.fliplr(line)
                else:
                    line,counter = plotLineLow(x0, y0, x1, y1)
            
            else:
                if y0 > y1:
                    line,counter = plotLineHigh(x1, y1, x0, y0)
                    line = np.fliplr(line)
                else:
                    line,counter = plotLineHigh(x0, y0, x1, y1)

            print('line before chopping: \n' + str(line))
            line = line[:,1:].astype(int)
            print('line after chopping x0 and y0: \n' + str(line))

            '''
            dir_iter=1 # direction of iteration
            if deltax < 0:
                dir_iter = -1
                    
            if deltax !=0:
                deltaerr = abs(deltay/deltax)
                error = float(0)
                y = float(y0)

                # Set up empty 2xn matrix for the points to fill
                line = np.zeros((2, max(abs(p[0]-x0)+1, abs(p[1]-y0)+1)), dtype=int)
                
                # counts number of points in the line
                counter = 0
                
                # Go through each point and add to list
                for x in range(x0, p[0]+dir_iter, dir_iter):
                    line[0,counter] = int(x)
                    line[1, counter] = int(y)
                    counter+=1
                    
                    error = error + deltaerr
                    if error >= .5:
                        y = y + np.sign(deltay)*1
                        error = error - 1
                
                # Get rid of x0, y0 since the probability of occupancy is 0
                # Get rid of extra 0s on the end of the matrix
                line = line[:,1:counter].astype(int)
                line_counter = line_counter + 1
            
            # If it is a vertical line
            elif deltay!=0:
                y_cells = range(y0+1, p[1]+1, dir_iter)
                x_cells = np.zeros((1, np.size(y_cells)))
                line = np.vstack((x_cells, y_cells)).astype(int)
            '''

            # Add the line to the list of lines update
            if len(line) != 0:
                grid_list.append(line)
                
        # Switch to second laser/grid
        laser_num = 1
                
    # Combine the lists
    grid_list = [grid_obs_list, grid_free_list]



'''
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
        #print('p in laser ' + str(line_counter) + ':\n' + str(p))
        #print('x0 and y0: \n' + str(x0) + '   ' + str(y0) + '\n')
        
        deltax = float(p[0] - x0)
        deltay = float(p[1] - y0)
        #print('p ' + str(p))
        #print('detax ' + str(deltax))
        #print('deltay ' + str(deltay))
        dir_iter=1 # direction of iteration
        if deltax < 0:
            dir_iter = -1
                
        if deltax !=0:
            deltaerr = abs(deltay/deltax)
            #print('deltay: ' + str(deltay) + '\ndeltax: ' + str(deltax) + '\ndeltaerr: ' + str(deltaerr))
            error = 0
            y = y0

            # Set up empty 2xn matrix for the points to fill
            line = np.zeros((2, max(abs(p[0]-x0)+1, abs(p[1]-y0)+1)), dtype=int)
            
            # counts number of points in the line
            counter = 0
            
            # Go through each point and add to list
            for x in range(x0, p[0]+dir_iter, dir_iter):
                line[0,counter] = x
                line[1, counter] = y
                counter+=1
                
                error = error + deltaerr
                #print('error: ' + str(error))
                if error >= .5:
                    y = y + np.sign(deltay)*1
                    error = error - 1
            
            # Get rid of x0, y0 since the probability of occupancy is 0
            # Get rid of extra 0s on the end of the matrix
            line = line[:,1:counter].astype(int)
            #print('non-vertical line # ' + str(line_counter) + '\n' + str(line))
            line_counter = line_counter + 1
        # If it is a vertical line
        elif deltay!=0:
            y_cells = range(y0+1, p[1]+1, dir_iter)
            x_cells = np.zeros((1, np.size(y_cells)))
            line = np.vstack((x_cells, y_cells)).astype(int)
            
        # Add the line to the list of lines update
        # if line != None and len(line) != 0: 
        if len(line) != 0:
            grid_list.append(line)
            
    # Switch to second laser/grid
    laser_num = 1
            
# Combine the lists
grid_list = [grid_obs_list, grid_free_list]
'''
# plot
grid = grid_list[0]
for line in grid:
    plt.plot(line[0,:], line[1,:], 'ro')
    pass

grid = grid_list[1]
for line in grid:
    plt.plot(line[0,:], line[1,:], 'bo')
    pass

# plot the lines
laser = laser_array[0]
for p in laser.T:
    plt.plot([x0, p[0]], [y0, p[1]], 'y-')
    print(str(p[0]) + '   ' + str(p[1]))

laser = laser_array[1]
for p in laser.T:
    plt.plot([x0, p[0]], [y0, p[1]], 'g-')
    print(str(p[0]) + '   ' + str(p[1]))

#plt.set_xticks(range(200,1, 550))
#plt.set_yticks(range(240, 1, 420))
#plt.grid(b = True, which='both',color='k', linestyle='-', linewidth=1)  
#plt.grid(True, which='both')
plt.show()
    
#print('grid_list: \n' + str(grid_list))

# Send back the lists of lines
#print('grid_list: \n' + str(grid_list))
