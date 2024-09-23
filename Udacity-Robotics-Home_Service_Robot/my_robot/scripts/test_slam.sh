#!/bin/sh

# Deploy turtlebot3 in custom Gazebo world
xterm  -e  "roslaunch my_robot home_service_world.launch " &
sleep 50

# Launch gmapping with tuned parameters to create a 2D occupancy grid map
xterm  -e  "roslaunch my_robot gmapping.launch " &
sleep 5

# Launch rviz with saved configurations 
xterm  -e "roslaunch my_robot view_navigation.launch" &
sleep 5

# Launch teleop to teleoperate the robot and create a map 
xterm  -e  "rosrun teleop_twist_keyboard teleop_twist_keyboard.py " 
