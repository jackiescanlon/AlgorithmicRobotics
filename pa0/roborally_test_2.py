""" my tests for roborally.py"""

import unittest, time
from roborally import Robot, Card, RoboRally

class RoboRallyTests(unittest.TestCase):

    def setUp(self):
        """Call before every test case."""
        # robot: name, x, y, heading
        # card: priority, type, distance
        self.robot1 = Robot("Alice", 10, 11, 0)
        self.robot2 = Robot("Bob", 11, 11, 0)
        self.robot3 = Robot("Bender", 12, 11, 0)
        self.rally = RoboRally()

    def test_multiple_robots_pushed_v2(self):

        # Robot 2 will move forward 3, pushing Robot 3
        # Then Robot 1 will move forward 4, pushing Robots 2 and 3
        self.robot2.add_card(Card(100, "F", 3))
        self.robot1.add_card(Card(10, "F", 4))
        self.rally.add_robot(self.robot1)
        self.rally.add_robot(self.robot2)
        self.rally.add_robot(self.robot3)
        self.rally.run()

        self.assertEqual(self.robot1.x, 14)
        self.assertEqual(self.robot2.x, 15)
        self.assertEqual(self.robot3.x, 16)
    
if __name__ == "__main__":
    unittest.main() #
