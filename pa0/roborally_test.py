""" Unit tests for roborally.py

Author: Nathan Sprague
Version: 8/28/17
"""

import unittest
import time

from roborally import Robot, Card, RoboRally

class RoboRallyTests(unittest.TestCase):

    def setUp(self):
        """Call before every test case."""
        self.robot1 = Robot("Alice", 10, 11, 0)
        self.robot2 = Robot("Bob", 11, 11, 0)
        self.robot3 = Robot("Bender", 12, 11, 0)
        self.rally = RoboRally()

    def test_move_one_robot_east_1(self):
        self.robot1.heading = 0
        self.robot1.add_card(Card(10, "F", 1))
        self.rally.add_robot(self.robot1)
        self.rally.run()

        self.assertEqual(self.robot1.x, 11)
        self.assertEqual(self.robot1.y, 11)
        self.assertEqual(self.robot1.heading, 0)

    def test_move_one_robot_north_2(self):
        self.robot1.heading = 90
        self.robot1.add_card(Card(10, "F", 2))
        self.rally.add_robot(self.robot1)
        self.rally.run()

        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 13)
        self.assertEqual(self.robot1.heading, 90)

    def test_move_one_robot_west_3(self):
        self.robot1.heading = 180
        self.robot1.add_card(Card(10, "F", 3))
        self.rally.add_robot(self.robot1)
        self.rally.run()

        self.assertEqual(self.robot1.x, 7)
        self.assertEqual(self.robot1.y, 11)
        self.assertEqual(self.robot1.heading, 180)

    def test_move_one_robot_south_4(self):
        self.robot1.heading = 270
        self.robot1.add_card(Card(10, "F", 4))
        self.rally.add_robot(self.robot1)
        self.rally.run()

        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 7)
        self.assertEqual(self.robot1.heading, 270)

    def test_turn_one_robot_left_once(self):
        self.robot1.heading = 0
        self.robot1.add_card(Card(10, "L"))
        self.rally.add_robot(self.robot1)
        self.rally.run()
        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 11)
        self.assertEqual(self.robot1.heading, 90)

    def test_turn_one_robot_three_lefts(self):
        self.robot1.heading = 270
        self.robot1.add_card(Card(10, "L"))
        self.robot1.add_card(Card(10, "L"))
        self.robot1.add_card(Card(10, "L"))
        self.rally.add_robot(self.robot1)
        self.rally.run()
        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 11)
        self.assertEqual(self.robot1.heading, 180)

    def test_turn_one_robot_four_lefts(self):
        self.robot1.heading = 0
        self.robot1.add_card(Card(10, "L"))
        self.robot1.add_card(Card(10, "L"))
        self.robot1.add_card(Card(10, "L"))
        self.robot1.add_card(Card(10, "L"))
        self.rally.add_robot(self.robot1)
        self.rally.run()
        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 11)
        self.assertEqual(self.robot1.heading, 0)

    def test_turn_one_robot_right_once(self):
        self.robot1.heading = 90
        self.robot1.add_card(Card(10, "R"))
        self.rally.add_robot(self.robot1)
        self.rally.run()
        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 11)
        self.assertEqual(self.robot1.heading, 0)

    def test_turn_one_robot_three_rights(self):
        self.robot1.heading = 90
        self.robot1.add_card(Card(10, "R"))
        self.robot1.add_card(Card(10, "R"))
        self.robot1.add_card(Card(10, "R"))
        self.rally.add_robot(self.robot1)
        self.rally.run()
        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 11)
        self.assertEqual(self.robot1.heading, 180)

    def test_turn_one_robot_four_rights(self):
        self.robot1.heading = 0
        self.robot1.add_card(Card(10, "R"))
        self.robot1.add_card(Card(10, "R"))
        self.robot1.add_card(Card(10, "R"))
        self.robot1.add_card(Card(10, "R"))
        self.rally.add_robot(self.robot1)
        self.rally.run()
        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 11)
        self.assertEqual(self.robot1.heading, 0)

    def test_one_robot_several_cards(self):
        self.robot1.add_card(Card(10, "F", 2)) # moves to 12, 11
        self.robot1.add_card(Card(10, "R")) # points south
        self.robot1.add_card(Card(10, "F", 11)) # moves to 12, 0
        self.robot1.add_card(Card(10, "R")) # points west
        self.robot1.add_card(Card(10, "F", 12)) # moves to 0, 0
        self.rally.add_robot(self.robot1)
        self.rally.run()
        self.assertEqual(self.robot1.x, 0)
        self.assertEqual(self.robot1.y, 0)
        self.assertEqual(self.robot1.heading, 180)
        
    def test_multiple_robots_no_collissions(self):

        self.robot1.add_card(Card(10, "L"))
        self.robot1.add_card(Card(10, "F", 2))
        self.robot2.add_card(Card(10, "L"))
        self.robot2.add_card(Card(10, "F", 3))
        self.robot3.add_card(Card(10, "L"))
        self.robot3.add_card(Card(10, "F", 4))
        self.rally.add_robot(self.robot1)
        self.rally.add_robot(self.robot2)
        self.rally.add_robot(self.robot3)
        self.rally.run()

        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 13)
        self.assertEqual(self.robot1.heading, 90)

        self.assertEqual(self.robot2.x, 11)
        self.assertEqual(self.robot2.y, 14)
        self.assertEqual(self.robot2.heading, 90)

        self.assertEqual(self.robot3.x, 12)
        self.assertEqual(self.robot3.y, 15)
        self.assertEqual(self.robot3.heading, 90)
        
    def test_multiple_robots_no_collissions_unequal_cards(self):

        self.robot1.add_card(Card(10, "L"))
        self.robot1.add_card(Card(10, "F", 2))
        self.robot2.add_card(Card(10, "L"))
        self.robot2.add_card(Card(10, "F", 3))
        self.robot3.add_card(Card(10, "L"))
        self.robot3.add_card(Card(10, "F", 4))
        self.robot3.add_card(Card(10, "F", 1))
        self.robot3.add_card(Card(10, "F", 1))
        self.rally.add_robot(self.robot1)
        self.rally.add_robot(self.robot2)
        self.rally.add_robot(self.robot3)
        self.rally.run()

        self.assertEqual(self.robot1.x, 10)
        self.assertEqual(self.robot1.y, 13)
        self.assertEqual(self.robot1.heading, 90)

        self.assertEqual(self.robot2.x, 11)
        self.assertEqual(self.robot2.y, 14)
        self.assertEqual(self.robot2.heading, 90)

        self.assertEqual(self.robot3.x, 12)
        self.assertEqual(self.robot3.y, 17)
        self.assertEqual(self.robot3.heading, 90)

    def test_single_collision_one_push_regardless_priority(self):

        self.robot2.add_card(Card(10, "F", 1)) 
        self.robot3.add_card(Card(10, "L")) 
        self.rally.add_robot(self.robot2)
        self.rally.add_robot(self.robot3)
        self.rally.run()

        self.assertEqual(self.robot2.x, 12)
        self.assertEqual(self.robot2.y, 11)
        self.assertEqual(self.robot2.heading, 0)

        self.assertEqual(self.robot3.x, 13)
        self.assertEqual(self.robot3.y, 11)
        self.assertEqual(self.robot3.heading, 90)


    def test_single_collision_three_push_regardless_priority(self):

        self.robot2.add_card(Card(10, "F", 3))
        self.robot3.add_card(Card(10, "L"))
        self.rally.add_robot(self.robot2)
        self.rally.add_robot(self.robot3)
        self.rally.run()

        self.assertEqual(self.robot2.x, 14)
        self.assertEqual(self.robot2.y, 11)
        self.assertEqual(self.robot2.heading, 0)

        self.assertEqual(self.robot3.x, 15)
        self.assertEqual(self.robot3.y, 11)
        self.assertEqual(self.robot3.heading, 90)
        
    def test_priority_respected_v1(self):

        self.robot2.add_card(Card(10, "F", 3))
        self.robot3.add_card(Card(100, "F", 3)) # should move before pushed
        self.rally.add_robot(self.robot2)
        self.rally.add_robot(self.robot3)
        self.rally.run()

        self.assertEqual(self.robot2.x, 14)
        self.assertEqual(self.robot2.y, 11)
        self.assertEqual(self.robot2.heading, 0)

        self.assertEqual(self.robot3.x, 15)
        self.assertEqual(self.robot3.y, 11)
        self.assertEqual(self.robot3.heading, 0)

    def test_priority_respected_v2(self):

        self.robot2.add_card(Card(100, "F", 3)) 
        self.robot3.add_card(Card(10, "F", 3)) # pushed before move
        self.rally.add_robot(self.robot2)
        self.rally.add_robot(self.robot3)
        self.rally.run()

        self.assertEqual(self.robot2.x, 14)
        self.assertEqual(self.robot2.y, 11)
        self.assertEqual(self.robot2.heading, 0)

        self.assertEqual(self.robot3.x, 18)
        self.assertEqual(self.robot3.y, 11)
        self.assertEqual(self.robot3.heading, 0)

    def test_multiple_robots_pushed(self):

        self.robot1.add_card(Card(100, "F", 3))
        self.rally.add_robot(self.robot1)
        self.rally.add_robot(self.robot2)
        self.rally.add_robot(self.robot3)
        self.rally.run()

        self.assertEqual(self.robot1.x, 13)
        self.assertEqual(self.robot1.y, 11)
        self.assertEqual(self.robot1.heading, 0)

        self.assertEqual(self.robot2.x, 14)
        self.assertEqual(self.robot2.y, 11)
        self.assertEqual(self.robot2.heading, 0)

        self.assertEqual(self.robot3.x, 15)
        self.assertEqual(self.robot3.y, 11)
        self.assertEqual(self.robot3.heading, 0)

    def test_efficiency_many_robots(self):
        
        for i in xrange(10000):
            robot = Robot(str(i), i, 0, 0)
            robot.add_card(Card(10, "R"))
            robot.add_card(Card(10, "F", 10))
            self.rally.add_robot(robot)
        start = time.time()
        self.rally.run()
        total = time.time() - start
        self.assertLess(total, 1.5)
        
    def test_efficiency_many_cards(self):
        for i in xrange(10000):
            self.robot1.add_card(Card(10, "R"))
            self.robot2.add_card(Card(10, "R"))
    
        self.rally.add_robot(self.robot1)
        self.rally.add_robot(self.robot2)
        start = time.time()
        self.rally.run()
        total = time.time() - start
        self.assertLess(total, 1.5)
            

if __name__ == "__main__":
    unittest.main() #
