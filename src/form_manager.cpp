// Navigation Manager
// Modifies given waypoints from the navigation manager and creates offset for any given number of drones to pass down to the tracjectory manager.

#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <control_functions.hpp>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
ros::Publisher wp_pub;
ros::Subscriber wp_sub;
geometry_msgs::PoseArray wp_subpose;   //Original Waypoint Array
geometry_msgs::PoseArray wp_posearray; //New Waypoint Array for Drones
geometry_msgs::Pose wp_pose;    //New Waypoint

void nav_wp(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  wp_subpose = *msg; //wp_subpose
}

void line_form(float x_off) //, int k
{ //Where off is offset dist and k is the number of drones
  // for (int i = 0; i<k-1 ;k++){ //For each drone

  //If statement to alternate sides or something NEED HERE
  for (int n = 0; n < wp_subpose.poses.size() - 1; n++)
  { //For each WP
    wp_posearray.header.frame_id = "shitcunt";
    wp_pose.position.x = wp_subpose.poses[n].position.x + x_off;
    wp_pose.position.y = wp_subpose.poses[n].position.y + x_off;
    wp_pose.position.z = wp_subpose.poses[n].position.z + x_off;

    // wp_pose.orientation.w = wp_subpose.poses[n].orientation.w;
    // wp_pose.orientation.x = wp_subpose.poses[n].orientation.x;
    // wp_pose.orientation.y = wp_subpose.poses[n].orientation.y;
    // wp_pose.orientation.z = wp_subpose.poses[n].orientation.z;
    //Ideally the below:
    wp_pose.orientation = wp_subpose.poses[n].orientation;
    wp_posearray.poses.push_back(wp_pose);
    // wp_posearray.header.seq.push_back(wp_pose.header.seq);
    // ROS_INFO("seq: %i  x: %f  y: %f  z: %f, ", wp_posearray.header.seq, wp_posearray.poses[n].position.x,wp_posearray.poses[n].position.y, wp_posearray.poses[n].position.z);
  }
  wp_pub.publish(wp_posearray);
}

int main(int argc, char **argv)
{
  std::string line;
  //Add ROScore stuff here
  ros::init(argc, argv, "gnc_node");
	// ros::NodeHandle gnc_node("~");
	ros::NodeHandle gnc_node;
	//Something to make this run once for ROScore(until the function is completed)
	
  wp_sub = gnc_node.subscribe<geometry_msgs::PoseArray>("/gnc/goal", 10, nav_wp);
  wp_pub = gnc_node.advertise<geometry_msgs::PoseArray>("/gnc/form", 10);


  ros::Rate loop_rate(0.1);
  while (ros::ok())
  {
    // line_form(2);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
