"""These classes are designed to support a simplified version of the
RoboRally game.  Robots are programmed with cards, then the robots
take turns executing their cards.

Author: Nathan Sprague and Jackie Scanlon
Version: 2
"""

import collections

import itertools

class Robot(object):
    """Robots have a position and a heading.  The x and y coordinates are
    integer grid positions. The heading is in degrees with four
    possible values:

    0 - Pointing East, toward increasing x
    90 - Poiting North, toward increasing y
    180 - Pointing West, toward decreasing x
    270 - Poiting South, toward decreasing y

    Robots also have set of "registers" (represented as a deque) that
    store a sequence of control cards.

    """
    def __init__(self, name, x, y, heading=90):
        """ Instantiate a Robot """
        self.name = name
        self.x = x
        self.y = y
        self.heading = heading
        self.registers = collections.deque()

    def add_card(self, card):
        """Add a new card to the instructions for this robot.  Cards will be
        executed in the order they are added.

        """
        self.registers.appendleft(card)

    def __str__(self):
        """String representation of the robot"""
        return "<Robot name:{}, x:{}, y:{}, heading:{}>".format(self.name,
                                                                self.x,
                                                                self.y,
                                                                self.heading)

        
class Card(object):
    """A card defining a single robot action. Priority determines
    which robot moves first on a turn (higher priority wins)

    Possible movement commands are:

    'F' - Forward
    'L' - Turn left 90 degrees
    'R' - Turn Right 90 degrees

    The 'F' command must include an integer distance indicating how
    many steps to move.  The distance value will be ignored for the
    'L' and 'R' commands.

    """
    
    def __init__(self, priority, command, distance=0):
        """ Instantiate a Card. """
        self.command = command
        self.priority = priority
        self.distance = distance
        
    def __str__(self):
        """ String representation of a Card."""
        if self.command == "F":
            result = "<Card priority: {} command:{}, distance:{}>"
            return result.format(self.priority, self.command, self.distance)
        else:
            result = "<Card priority: {} command:{}>"
            return result.format(self.priority, self.command)
			
class RoboRally(object):
    """This class handles the movement and interaction logic for the
    RoboRally game.
    """
    
    def __init__(self):
        """ Instantiate a game instance with no robots. """
        # The robots are stored in a dictionary.
        self.robots = {}
        self.max_x = 0
        self.min_x = 0
        self.max_y = 0
        self.min_y = 0
        
        # The remaining 4 items hold the points of the robots that are at the edges of the board
        # (farthest right, farthest up, farthest left, farthest down, respectively)
		
    def add_robot(self, robot):
        """ Add a robot to the game. """
        
        # Each robot key will be a string with its x value, space and y value.
        self.robots[str(robot.x) + " " +  str(robot.y)] = robot
		
    def push_robot(self, prev_robot, prev_location, direction):
        """ prev_robot is the initial robot that performs the push.
        prev_location is the previous location of prev_robot (before card was performed).
        direction is the direction the robot is moving in ('+x', '-x', '+y', '-y').
        """
        
        # At the end of pushing the robots, we will need to 
        # re-key them in the dictionary. So keep track of what
        # was moved and where they moved from.
        moved_robots = []
        moved_locations = []
        
        # Add the initial pushing robot to the list
        moved_robots.append(prev_robot)
        moved_locations.append(prev_location)
        
        # If we are moving in the +x direction
        if direction == '+x':
        
            # Loop through the spaces to the right of the robot
            # to see if there are any robots to get pushed
            # Break statement will exit loop
            while True:
            
                # Get the next space to the right
                pushed_robot = self.robots.get(str(prev_location + 1) + " " + str(prev_robot.y))
                
                # If there is no robot there, move right prev_location
                if pushed_robot == None:
                    prev_location += 1
                    
                # If we have finished searching the area the robot moved across, exit the loop
                elif pushed_robot.x > prev_robot.x:
                    break
                    
                # If it got pushed, its now one right the current robot
                else:
                        # Set prev_location to this pushed robot's x location
                        prev_location = pushed_robot.x
                        
                        # Push the robot (change x location) to be 1 right of prev_robot
                        pushed_robot.x = prev_robot.x + 1
                        
                        # Add it to the list of robots that were pushed
                        moved_robots.append(pushed_robot)
                        # Add its previous location to the list of previous locations
                        moved_locations.append(prev_location)
                        
                        # Check and reset self.max_x if necessary
                        if pushed_robot.x > self.max_x:
                            self.max_x = pushed_robot.x
                            
                        # Set the previous robot to be the robot that just got pushed
                        prev_robot = pushed_robot
                        
                # If we have reached the farthest robot, stop searching
                if prev_location > self.max_x:
                    break
                    
            # Re-key the robots that have moved
            while len(moved_robots) != 0:
                moved_robot = moved_robots.pop()
                moved_location = moved_locations.pop()
                del self.robots[str(moved_location) + " " + str(moved_robot.y)]
                self.robots[str(moved_robot.x) + " " + str(moved_robot.y)] = moved_robot
                

        # If we are moving in the -x direction
        if direction == '-x':
            
            # Loop through the spaces left of the robot
            # to see if there are any robots to get pushed.
            # Break statement will exit loop.
            while True:
            
                # Get the next space to the left
                pushed_robot = self.robots.get(str(prev_location - 1) + " " + str(prev_robot.y))
                
                # If there is no robot there, move left prev_location
                if pushed_robot == None:
                    prev_location -= 1
                    
                # If we have finished searching the area the robot moved across, exit the loop
                elif pushed_robot.x < prev_robot.x:
                    break
                    
                # If it got pushed, its now one left of the current robot
                else:
                        # Set prev_location to this pushed robot's x location
                        prev_location = pushed_robot.x
                        
                        # Push the robot (change x location) to be 1 left of prev_robot
                        pushed_robot.x = prev_robot.x - 1
                        # Add it to the list of robots that were pushed
                        moved_robots.append(pushed_robot)
                        # Add its previous location to the list of previous locations
                        moved_locations.append(prev_location)
                        
                        # Check and reset self.min_x if necessary
                        if pushed_robot.x < self.min_x:
                            self.min_x = pushed_robot.x
                            
                        # Set the previous robot to be the robot that just got pushed
                        prev_robot = pushed_robot
                        
                # If we have reached the farthest robot, stop searching
                if prev_location < self.min_x:
                    break
                    
            # Re-key the robots that have moved
            while len(moved_robots) != 0:
                moved_robot = moved_robots.pop()
                prev_location = moved_locations.pop()
                del self.robots[str(prev_location) + " " + str(moved_robot.y)]
                self.robots[str(moved_robot.x) + " " + str(moved_robot.y)] = moved_robot
                
        # If we are moving in the +y direction
        if direction == '+y':

            # Loop through the spaces above of the robot
            # to see if there are any robots to get pushed.
            # Break statement will exit loop.
            while True:
            
                # Get the next space above
                pushed_robot = self.robots.get(str(prev_robot.x) + " " + str(prev_location + 1))
                
                # If there is no robot there, move up prev_location
                if pushed_robot == None:
                    prev_location += 1
                    
                # If we have finished searching the area the robot moved across, exit the loop
                elif pushed_robot.y > prev_robot.y:
                    break
                    
                # If it got pushed, its now one above the current robot
                else:
                        # Set prev_location to this pushed robot's y location
                        prev_location = pushed_robot.y
                        
                        # Push the robot (change y location) to be 1 above prev_robot
                        pushed_robot.y = prev_robot.y + 1
                        # Add it to the list of robots that were pushed
                        moved_robots.append(pushed_robot)
                        # Add its previous location to the list of previous locations
                        moved_locations.append(prev_location)
                        
                        # Check and reset self.max_y if necessary
                        if pushed_robot.y > self.max_y:
                            self.max_y = pushed_robot.y
                            
                        # Set the previous robot to be the robot that just got pushed
                        prev_robot = pushed_robot
                        
                # If we have reached the farthest robot, stop searching
                if prev_location > self.max_y:
                    break
            
            # Re-key the robots that have moved
            while len(moved_robots) != 0:
                moved_robot = moved_robots.pop()
                prev_location = moved_locations.pop()
                del self.robots[str(moved_robot.x) + " " + str(prev_location)]
                self.robots[str(moved_robot.x) + " " + str(moved_robot.y)] = moved_robot
                    
        # If we are moving in the -y direction
        if direction == '-y':
        
            # Loop through the spaces below the robot
            # to see if there are any robots to get pushed.
            # Break statement will exit loop.
            while True:
            
                # Get the next space below
                pushed_robot = self.robots.get(str(prev_robot.x) + " " + str(prev_location - 1))
                
                # If there is no robot there, move down prev_location
                if pushed_robot == None:
                    prev_location -= 1
                    
                # If we have finished searching the area the robot moved across, exit the loop
                elif pushed_robot.y < prev_robot.y:
                    break
                    
                # If it got pushed, its now one below the current robot
                else:
                        # Set prev_location to this pushed robot's y location
                        prev_location = pushed_robot.y
                        
                        # Push the robot (change y location) to be 1 below prev_robot
                        pushed_robot.x = prev_robot.y - 1
                        # Add it to the list of robots that were pushed
                        moved_robots.append(pushed_robot)
                        # Add its previous location to the list of previous locations
                        moved_locations.append(prev_location)
                        
                        # Check and reset self.min_y if necessary
                        if pushed_robot.y < self.min_y:
                            self.min_y = pushed_robot.y
                            
                        # Set the previous robot to be the robot that just got pushed
                        prev_robot = pushed_robot
                        
                # If we have reached the farthest robot, stop searching
                if prev_location < self.min_y:
                    break
                    
            # Re-key the robots that have moved
            while len(moved_robots) != 0:
                moved_robot = moved_robots.pop()
                prev_location = moved_locations.pop()
                del self.robots[str(moved_robot.x) + " " + str(prev_location)]
                self.robots[str(moved_robot.x) + " " + str(moved_robot.y)] = moved_robot
            
    def run(self):
        """Play all robot cards.  After this method executes all Robots will
        be in their correct final positions and all 'hands' will be
        empty.  Robots need not begin with the same number of cards.
        If a robot runs out of cards it will remain stationary in
        subsequent rounds (unless it is pushed by another robot).
        """
        
        # If there are no robots, exit
        if len(self.robots) == 0:
            print("There are no robots!")
            return
       
        # Loop through all cards (will break when there are no cards left)
        while True: 

            # cards_left_robots is a list that will contain robots with cards left
            cards_left_robots = []
            
            # First indicates its the first robot to be checked
            first = True
            
            # Find the robots with cards left
            for robot_key in self.robots:
            
                # Get the robot from the dictionary
                robot = self.robots[robot_key]
                length = len(robot.registers)
                
                # If its the first robot, set the max and min values
                if first:
                    self.min_x = robot.x
                    self.max_x = robot.x
                    self.min_y = robot.y
                    self.max_y = robot.y
                    first = False
                    
                # If its not the first robot, check if it is further than current min/max
                else:
                    if robot.x > self.max_x:
                        self.max_x = robot.x
                    elif robot.x < self.min_x:
                        self.min_x = robot.x
                    if robot.y > self.max_y:
                        self.max_y > robot.y
                    elif robot.y < self.min_y:
                        self.min_y = robot.y
                        
                # If there is at least one card left, add that robot to cards_left_robots
                if length != 0:
                    cards_left_robots.append(robot)
                
            # If there are no robots with cards left, exit the loop and game
            if len(cards_left_robots) == 0:
                break
                
            # Sort the robots with cards left according to priority of the next card
            cards_left_robots = sorted(
                cards_left_robots, 
                key=lambda robot: robot.registers[-1].priority, 
                reverse=True)
            
            # Move each robot one at a time
            for robot in cards_left_robots:

                # Remove the next card
                card = robot.registers.pop()
                
                # Turn left, correcting for 0 --> 270 degrees
                if card.command == 'L':
                    robot.heading += 90
                    if robot.heading == 360:
                        robot.heading = 0

                # Turn right, correcting for 270 --> 0 degrees
                elif card.command == 'R':
                    robot.heading -= 90
                    if robot.heading == -90:
                        robot.heading = 270

                # Move in the direction of the heading
                # Check for new edge robots 
                # Push other robots accordingly
                else:
                    if robot.heading == 0:
                        prev_location = robot.x
                        robot.x += card.distance
                        if robot.x > self.max_x:
                            self.max_x = robot.x
                        self.push_robot(robot, prev_location, '+x')
                                                         
                    elif robot.heading == 90:
                        prev_location = robot.y
                        robot.y += card.distance
                        if robot.y > self.max_y:
                            self.max_y = robot.y
                        self.push_robot(robot, prev_location, '+y')
                                
                    elif robot.heading == 180:
                        prev_location = robot.x
                        robot.x -= card.distance
                        if robot.x < self.min_x:
                            self.min_x = robot.x
                        self.push_robot(robot, prev_location, '-x')
                    else:
                        prev_location = robot.y
                        robot.y -= card.distance
                        if robot.y < self.min_y:
                            self.min_y = robot.y
                        self.push_robot(robot, prev_location, '-y')
                        
