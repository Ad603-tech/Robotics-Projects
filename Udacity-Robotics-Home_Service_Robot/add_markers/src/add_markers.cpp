#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

// Global variable to store the current position of the robot
double current_pose[2] = {0.0, 0.0};

// Callback function to update the current robot pose from the odometry messages
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose[0] = msg->pose.pose.position.x;
  current_pose[1] = msg->pose.pose.position.y;

  ROS_INFO("Current Pose: x = %f, y = %f", current_pose[0], current_pose[1]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  // Publisher for the marker
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscriber for the robot's pose (using odometry data)
  ros::Subscriber pose_sub = n.subscribe("/odom", 10, poseCallback);

  // Define pick-up and drop-off positions
  double pick_up_position[2] = {-3.86, 3.04};
  double drop_off_position[2] = {4.36, 3.42};

  // Set the shape type to cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Create the marker
  visualization_msgs::Marker marker;

  // Set frame ID and timestamp
  marker.header.frame_id = "map";  // Make sure this frame exists in RViz
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type to a cube
  marker.type = shape;

  // Set the marker pose (initially at the pick-up zone)
  marker.pose.position.x = pick_up_position[0];
  marker.pose.position.y = pick_up_position[1];
  marker.pose.position.z = 0.0;  // flat on the ground

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color (green cube)
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;  // fully opaque

  marker.lifetime = ros::Duration();  // Marker never auto-expires

  // Ensure subscribers are present before publishing
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN("Waiting for a subscriber to the marker topic...");
    sleep(1);
  }

  // Step 1: Publish the marker at the pick-up zone
  marker.action = visualization_msgs::Marker::ADD;
  ROS_INFO("Publishing marker at pick-up zone");
  marker_pub.publish(marker);

  // Step 2: Pause for 5 seconds
  ros::Duration(5.0).sleep();

  // Step 3: Hide the marker (DELETE)
  marker.action = visualization_msgs::Marker::DELETE;
  ROS_INFO("Hiding marker...");
  marker_pub.publish(marker);

  // Step 4: Pause for 5 seconds
  ros::Duration(5.0).sleep();

  // Step 5: Publish the marker at the drop-off zone
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = drop_off_position[0];
  marker.pose.position.y = drop_off_position[1];
  ROS_INFO("Publishing marker at drop-off zone");
  marker_pub.publish(marker);

  // Keep spinning to process pose updates
  ros::spinOnce();
  
  return 0;
}

