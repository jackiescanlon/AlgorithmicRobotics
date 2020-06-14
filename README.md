# Projects from Algorithmic Robotics

Spring 2019 - Professor Trinkle

[VIDEO]()

Learned algorithms and techniques used in the field of mobile robotics for perception, planning, and control. The topics include: representation of position and orientation, trajectory generation, ROS (Robot Operating System), motion control, obstacle avoidance, the “bug” algorithms, simultaneous localization and mapping (SLAM), and basic motion planning.

[Course webpage](https://www.cs.rpi.edu/~trink/Courses/AlgorithmicRobotics/spring2019/)

### Programming Assignments

* "pa" = Programming Assignment
* Each folder has a PDF explaining assignment
* *indicates assignment is shown in video
* **Indicates to use keyboard to control robot manually: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot0/input_vel`

| folder | assignment summary | how to run |
| - | - | - |
| pa4 | Autonomous search and rescue using SLAM, motion planning algorithms | *Part 1: localization using dead reckoning, Kalman filters. Static and dynamic occupancy grids.<br>`roslaunch pa4_student section_1_demo_1.launch` (noiseless)<br>`roslaunch pa4_student section_1_demo_2.launch` (with noisy controls and lidar) |
| | | *Part 2: Motion planning using Probabalistic Roadmap method, Dijkstra's algorithm<br>`roslaunch pa4_student navigation.launch map:=<simple_rooms, hospital_section, sparse_obstacles> x:=<start x location> y:=<start y location> theta:=<start theta>`| 
| | | *Part 3: Search and Rescue (combine localization and motion planning to find victims in a disaster)<br>`roslaunch pa4_student search_rescue_task1.launch`<br>`roslaunch pa4_student search_rescue_task2.launch`<br>`roslaunch pa4_student search_rescue_demo.launch` |
| pa3 | occupancy grid construction w/ Bresenham's line algorithm, Gaussian noise, pose-to-pose control | Part 1: Pose-to-Pose motion control<br>`roslaunch pa3_student turtlejectory.launch`<br>Optional: add noise to with argument `use_noise:=true`|
| | | *Part 2: Creating occupancy grid<br>`roslaunch pa3_student r2d2_mapping.launch`<br>Optional: add noise to with argument `use_noise:=true` |
| pa2 | bike model and diff drive | `roslaunch pa2_student turtlejectory.launch` |
| pa1 | Intro to ROS | `roscore`<br>`rosrun turtlesim turtlesim_node`<br>`python pubvel.py` |
| pa0 | RoboRally - code up the board game [RoboRally](https://en.wikipedia.org/wiki/RoboRally) | Python only (no ROS or GUI)<br>`python roborally_test.py`<br>`python roborally_test_2.py` |




