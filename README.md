# Robotics-Projects
=====================
### Project 1: Udacity Ball chasing robot
#### Description
This project involves creating a robot that can chase a ball autonomously. The robot will be equipped with
a camera, a LiDAR sensor, and a computer vision system. The computer vision system will detect the
ball and send the coordinates to the motor controller, which will then move the robot to chase the ball

### Project 2: Localize and Navigate robot
#### Description
This project involves creating a robot that can navigate through a known map of home world and localize itself. The robot will have Adaptive Monte Carlo localization package to locate itselt in a known map. With the help of gmapping-ROS package it can autonomously navigate itself in a known map when given a 2D point.

### Project 3: Robot Mapping and Map Creation
#### Description
This project involves creating a robot that can map its environment and create a map of the space. The
robot will use the R-TAB ROS package to create a 3D map of the space and the ros-teleop package through which we can move the robot
around in the map.

### Project 4: Home Service Robot
#### Description
This project involves creating a robot that can perform home service tasks such as object pick-up, drop-off and navigation. The robot
uses gmapping package for SLAM and AMCL for localization and navigation. The path-planning is based Djikstra's algorithm. Two ROS nodes are created one to add markers in the map visualization and another to command the robot to pick-up objects and drop them off at the desired coordinates in the known environment.