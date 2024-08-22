#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{

  // Create a request
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // Call the command_robot service with request values given above
  if (!client.call(srv))
    ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image& img)
{
int white_pixel = 255;
int left_side = img.step /3;
int right_side = 2*img.step /3;

bool white_pixel_found = false;
int white_pixel_column = -1;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    for(int i=0; i<img.height * img.step; i+=3){
    if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
            white_pixel_found = true;
            white_pixel_column = i % img.step;
            break;
        }
    }
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    if(white_pixel_found){
    if (white_pixel_column < left_side) {
            drive_robot(0.5, 0.5); // Turn left
        } else if (white_pixel_column >= left_side && white_pixel_column < right_side) {
            drive_robot(0.5, 0.0); // Move forward
        } else {
            drive_robot(0.5, -0.5); // Turn right
        }
    } else {
        // Stop the robot when there's no white ball seen by the camera
        drive_robot(0.0, 0.0);
        }
}

int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}