#!/bin/sh

# Deploy turtlebot3 in custom Gazebo world
xterm  -e  " roslaunch my_robot home_service_world.launch " &
sleep 50

# Launch amcl for localization
xterm -e " roslaunch my_robot amcl.launch " &
sleep 5

# Launch rviz with saved configurations 
xterm  -e " roslaunch my_robot view_navigation.launch" &
sleep 10

# Launch pick_objects node to navigate to pick-up & drop-off zones  
xterm  -e " rosrun pick_objects pick_objects "
