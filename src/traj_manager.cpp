// Trajectory Manager
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <control_functions.hpp>

// Basic collision avoidance here
#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

ros::Subscriber sub_wp;
geometry_msgs::Pose wppose_sub;

ros::Publisher bool_pub;
std_msgs::Bool wp_update;

void wp_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
  wppose_sub = *msg;
}


void set_destination(geometry_msgs::PoseArray wp_pose) //int n
{
  posestamped.pose.position.x = wp_pose.pose.position.x;
  posestamped.pose.position.y = wp_pose.pose.position.y;
  posestamped.pose.position.z = wp_pose.pose.position.z;
  posestamped.pose.orientation.w = wp_pose.pose.orientation.w;
  posestamped.pose.orientation.x = wp_pose.pose.orientation.x;
  posestamped.pose.orientation.y = wp_pose.pose.orientation.y;
  posestamped.pose.orientation.z = wp_pose.pose.orientation.z;
  pose_pub.publish(posestamped);
}

int wp_reached ( geometry_msgs::Pose wppose_sub )
{
  something about checking distance between combined drone position and shared waypoint???;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nh");
  ros::NodeHandle nh;

  init_publisher_subscriber(nh, "/drone1");
  sub_wp = nh.subscribe<geometry_msgs::Pose>("/gnc/form", 10, wp_cb);
  bool_pub = nh.advertise<std_msgs::Bool>("/gnc/wpreach",10);
  wait4connect();
  set_mode("GUIDED");
  takeoff(5);

  bool wp_update.data = 0;

  if (wp_reached)
  {
    wp_update.data=1;
    bool_pub.publish(wp_update);
  }

  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    
    // set_destination();

    ros::spin();
    loop_rate.sleep();
  }
  return 0;
}
