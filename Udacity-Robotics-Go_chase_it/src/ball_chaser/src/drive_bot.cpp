#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{
	auto linear_x = (float)req.linear_x;
	auto angular_z = (float)req.angular_z;
	
	ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);
	
	// create motor_command object of type geometry_msgs::Twist
	geometry_msgs::Twist motor_command;
	
	// set the wheel velocities and yaw orientation to the requested ones
	motor_command.linear.x = linear_x;
	motor_command.angular.z = angular_z;
	
	// publish the requested linear x and angular velocities to robot wheel jointss
	motor_command_publisher.publish(motor_command);
	
	// Return a response message
    	res.msg_feedback = "Move command is set to - linear_x: " + std::to_string(linear_x) + " , angular_z: " + std::to_string(angular_z);
    	ROS_INFO_STREAM(res.msg_feedback);

    return true;
}



int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
    ROS_INFO("Ready to send commands for moving the robot");

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
