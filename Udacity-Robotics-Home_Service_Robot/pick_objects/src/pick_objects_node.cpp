#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

   int status = 1;
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting 5 sec for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach to pick objects
  goal.target_pose.pose.position.x = -3.86;
  goal.target_pose.pose.position.y = 3.04;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal to drop off objects");
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    status = 0;
    ROS_INFO("Ok. Pickup zone reached");

    ROS_INFO("Waiting 5 seconds (robot collecting object) ...");
    ros::Duration(5.0).sleep();
    
    goal.target_pose.pose.position.x = 4.36;
    goal.target_pose.pose.position.y = 3.42;
    goal.target_pose.pose.orientation.w = 1.0;
    
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("------");
    ROS_INFO("Sending Drop off zone data to robot");
    ac.sendGoal(goal);
    // Wait an infinite time for the results
    ac.waitForResult();
    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      status = 0;
      ROS_INFO("Ok. Drop off zone reached");
    }
    else
    {
      status = 2;
      ROS_INFO("ERROR: fail to reach Drop off zone");
    }
  }
  
  else
  {
    status = 1;
    ROS_INFO("ERROR: fail to reach Pickup zone");
  }
  ROS_INFO("Shutting down pick_objects node");
  return status;
}
