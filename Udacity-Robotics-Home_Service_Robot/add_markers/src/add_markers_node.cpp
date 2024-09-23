#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

// Define pickup and drop-off positions
double pick_up_position[2] = {-3.86, 3.04};
double drop_off_position[2] = {4.36, 3.42};

// Store the current robot position
double current_pose[2] = {0, 0};

// Threshold for determining if the robot has reached the zone
double threshold = 0.5;

// Flag to check if the marker is hidden at pickup
bool picked_up = false;

// Function to calculate the distance between two points
double get_distance(double pos1[2], double pos2[2]) {
  return sqrt(pow(pos1[0] - pos2[0], 2) + pow(pos1[1] - pos2[1], 2));
}

// Callback function to update the robot's current position from odometry
void get_current_pose(const nav_msgs::Odometry::ConstPtr& msg) {
  current_pose[0] = msg->pose.pose.position.x;
  current_pose[1] = msg->pose.pose.position.y;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  // Publisher for the marker
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscriber for the robot's current pose (odometry)
  ros::Subscriber pose_sub = n.subscribe("/odom", 10, get_current_pose);

  // Create a marker
  visualization_msgs::Marker marker;

  // Set frame ID and timestamp
  marker.header.frame_id = "map";  
  marker.header.stamp = ros::Time::now();

  // Set namespace and id for the marker
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type to CYLINDER (you can change this if you like)
  marker.type = visualization_msgs::Marker::CYLINDER;
  

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set marker color (green)
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  // Ensure subscribers are present before publishing
  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN("Waiting for a subscriber to the marker topic...");
    sleep(1);
  }

  // Show marker at the pickup zone initially
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pick_up_position[0];
  marker.pose.position.y = pick_up_position[1];
  marker_pub.publish(marker);

  ROS_INFO("Marker placed at pickup zone.");

  // Main loop
  while (ros::ok()) {
    ros::spinOnce();  // Process incoming messages

    // Calculate the distance to the pickup and drop-off zones
    double dist_to_pickup = get_distance(current_pose, pick_up_position);
    double dist_to_dropoff = get_distance(current_pose, drop_off_position);

    // Check if the robot has reached the pickup zone
    if (!picked_up && dist_to_pickup < threshold) {
      ROS_INFO("Robot reached the pickup zone. Simulating pickup...");

      // Hide the marker (DELETE) and wait for 5 seconds to simulate pickup
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      ros::Duration(5.0).sleep();
      picked_up = true;  // Set the flag to indicate the pickup has happened

      ROS_INFO("Pickup completed. Heading to drop-off zone.");
    }

    // Check if the robot has reached the drop-off zone
    if (picked_up && dist_to_dropoff < threshold) {
      ROS_INFO("Robot reached the drop-off zone. Placing the marker...");

      // Show the marker at the drop-off zone
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = drop_off_position[0];
      marker.pose.position.y = drop_off_position[1];
      marker_pub.publish(marker);

      // End the process once the drop-off is completed
      break;
    }

    r.sleep();
  }

  ros::spinOnce();
  
  return 0;
}

