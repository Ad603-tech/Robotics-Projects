# Home-service-robot

![Home_Service_Robot](https://github.com/Ad603-tech/Robotics-Projects/raw/main/Udacity-Robotics-Home_Service_Robot/Images/Home_Service_robot.gif)


## Packages used in the project:

* **slam-gmapping**: This package performs laser based SLAM, using it with turtlebot-teleop, the
                      robot can be moved around the room to generate a binary occupancy map which can be used
                      later for localization and navigation.
* **amcl**: This package performs Advanced Monte Carlo Localization, an algorithm that uses
            particle filters to locate our robot.
* **navigation**: This packages implements the Navigation Stack which allows us to send a
                  navigation goal for our robot, the underlying algorithm used for path planning is Dijkstra's
                  Algorithm.
* **rviz**: This package let us use Rviz, a visualization tool to see how well performs our robot.
* **pgm_map_creator**: This package is used only in simulated environments to quickly generate a
                        binary occupancy grid.
* **teleop_twist_keyboard** package is for teleoperating the robot around its environment.

## Packages created by me:-

* **pick_objects**: This package sends a navigation goal to our robot so it can pick, carry and place
                    objects around the room.
* **add_markers**: This package publishes markers that are displayed in Rviz, for evaluation
                    purposes.
* **my_robot**: This package contains custom robot description. I have used my own modeled robot
                for this project. Therefore, this package contains it’s urdf files, the gazebo world, amcl
                parameters, rviz configurations, generated map and all necessary scripts and launch files.


## Project Info: -

My own robot model is used.
The project consists of the following parts:
* Design an indoor environment with the Building Editor in Gazebo.
* Create a 2D occupancy grid map of the environment by teleoperating the robot and performing
  SLAM using the gmapping package (utilizing odometry and laser scan data).
* Use Adaptive Monte Carlo Localization (AMCL) to localize the robot inside the environment
  with the amcl package (by utilizing odometry and laser scan data).
* Test the navigation stack and send a goal for the robot to navigate to via the 2D Nav Goal
  button in Rviz.
* Write a pick_objects node that commands the robot to navigate to the desired pick-up and drop-
  off zones.
* Write an add_markers node that subscribes to the robot odometry and publishes markers to Rviz
  to simulate the object pick-up and drop-off.

![Home_Environment](https://github.com/Ad603-tech/Robotics-Projects/raw/main/Udacity-Robotics-Home_Service_Robot/Images/home.png)


## Directory Tree Structure:-
![Directory_structure](https://github.com/Ad603-tech/Robotics-Projects/raw/main/Udacity-Robotics-Home_Service_Robot/Images/directory_structure.png)
## Technologies
The project was developed on Ubuntu 20.04 LTS with:
* ROS Noetic
* Gazebo 11.11.0

## Dependencies
The following dependencies need to be installed:
```sh
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get install ros-noetic-teleop-twist-keyboard
$ sudo apt-get install ros-noetic-openslam-gmapping
$ sudo apt-get install ros-noetic-navigation
```

The scripts to run multiple packages utilize the xterm terminal which can be installed via:
```sh
$ sudo apt-get install xterm
```
## Installation
To run this project, you must have ROS and Gazebo installed.
* 1. Create a catkin_ws, if you do not already have one.
```sh
$ mkdir -p /catkin_ws/src/
$ cd catkin_ws/src/
$ catkin_init_workspace
```
* 2. Download the project in catkin_ws/src/ and move its contents to src/.
* 3. Build the packages.
```sh
$ cd /catkin_ws/
$ catkin_make
```
* 4. Source your environment.
```sh
$ source devel/setup.bash
```
* 5. Make the scripts executable.
```sh
$ cd src/my_robot/scripts
$ sudo chmod +x *.sh
```
## Part 1: SLAM
Optional: If you would like to test the SLAM algorithm and create a new map of the environment, run:
```sh
$ ./test_slam.sh
```
Teleoperate the robot through your keyboard to explore the environment and see the map appear in
Rviz. Once the house is fully explored, save the map with:
```sh
$ rosrun map_server map_saver -f ../map/home_service_map
```
![Home_Service_map](https://github.com/Ad603-tech/Robotics-Projects/raw/main/Udacity-Robotics-Home_Service_Robot/my_robot/map/home_service_map.pgm)

## Part 2: Navigation
Optional: If you would like to test the navigation algorithm, run:
```sh
$ ./test_navigation.sh
```
![Home_Service_rviz_navigation](https://github.com/Ad603-tech/Robotics-Projects/raw/main/Udacity-Robotics-Home_Service_Robot/Images/rviz.png)

Now you will be using the already generated map and localize with AMCL. Press the 2D Nav Goal
button in Rviz and click somewhere on the map to command the robot to navigate there.

## Part 3: Home Service
To simulate the full home service robot, run:
```sh
$ ./home_service.sh
```
The robot will be using the generated map and will localize itself with the acml package. It will
navigate to a virtual object (indicated by a green cube in Rviz), pick-up the object (the cube will
disappear to simulate the pick-up) and then the robot will navigate to a drop-off zone and deliver the
object (the cube will re-appear once the robot reaches the drop-off location).Figure 2: Robot going to the pick-up location
Figure 3: Robot heading to drop-off locationFigure 4: Robot reached the goal position