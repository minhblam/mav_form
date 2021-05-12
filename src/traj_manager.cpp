// Trajectory Manager
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <control_functions.hpp>

// Basic collision avoidance here
#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

ros::Subscriber sub_wp;
geometry_msgs::PoseArray wppose_sub; //Original Waypoint Array

geometry_msgs::Pose poseshit;

void wp_cb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  wppose_sub = *msg;
}

void set_destination() //int n
{
  for (int n = 0; n < wppose_sub.poses.size(); n++)
  {
    posestamped.pose.position.x = wppose_sub.poses[n].position.x;
    posestamped.pose.position.y = wppose_sub.poses[n].position.y;
    posestamped.pose.position.z = wppose_sub.poses[n].position.z;
    // posestamped.pose.orientation.w = wp_posearray.poses[counter].orientation.w;
    // posestamped.pose.orientation.x = wp_posearray.poses[counter].orientation.x;
    // posestamped.pose.orientation.y = wp_posearray.poses[counter].orientation.y;
    // posestamped.pose.orientation.z = wp_posearray.poses[counter].orientation.z;
  }
  pose_pub.publish(posestamped);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nh");
  ros::NodeHandle nh;

  init_publisher_subscriber(nh, "/drone1");
  sub_wp = nh.subscribe<geometry_msgs::PoseArray>("/gnc/form", 10, wp_cb);
  ROS_INFO("Subbed burh");
  wait4connect();
  set_mode("GUIDED");
  takeoff(5);

  // boost::shared_ptr<geometry_msgs::PoseArray const> sharedarray;
  // geometry_msgs::PoseArray array;
  // sharedarray = ros::topic::waitForMessage<geometry_msgs::PoseArray>("/gnc/form", 10,wp_cb);
  // if(sharedarray != NULL){
  //   array = *sharedarray;
  // }

  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    set_destination();
    ros::spin();
    loop_rate.sleep();
  }
  return 0;
}
