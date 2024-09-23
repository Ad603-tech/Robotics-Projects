#!/bin/sh

# Deploy turtlebot3 in custom Gazebo world
xterm  -e  "roslaunch my_robot home_service_world.launch " &
sleep 15

# Launch amcl for localization
xterm -e "roslaunch my_robot amcl.launch " &
sleep 5

# Launch rviz with saved configurations 
xterm  -e "roslaunch my_robot view_navigation.launch" 
