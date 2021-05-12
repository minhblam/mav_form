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
ros::Publisher wp_pub1;
ros::Publisher wp_pub2;

ros::Subscriber wp_sub;
geometry_msgs::PoseArray wp_subpose;    //Original Waypoint Array
geometry_msgs::PoseArray wp_posearray;  //New Waypoint Array for Drones
geometry_msgs::PoseArray wp_posearray1; //New Waypoint Array for Drones
geometry_msgs::PoseArray wp_posearray2; //New Waypoint Array for Drones
geometry_msgs::Pose wp_pose;            //New Waypoint
geometry_msgs::Pose wp_pose1;           //New Waypoint
geometry_msgs::Pose wp_pose2;           //New Waypoint

void nav_wp(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  wp_subpose = *msg; //wp_subpose
}

void line_form(float xoff) //, int k
{                //Where off is offset dist and k is the number of drones
  // for (int i = 0; i<k-1 ;k++){ //For each drone

  //If statement to alternate sides or something NEED HERE
  for (int n = 0; n < wp_subpose.poses.size(); n++)
  { //For each WP

    wp_pose.position.x = wp_subpose.poses[n].position.x;
    wp_pose.position.y = wp_subpose.poses[n].position.y;
    wp_pose.position.z = wp_subpose.poses[n].position.z;

    wp_pose.orientation.w = wp_subpose.poses[n].orientation.w;
    wp_pose.orientation.x = wp_subpose.poses[n].orientation.x;
    wp_pose.orientation.y = wp_subpose.poses[n].orientation.y;
    wp_pose.orientation.z = wp_subpose.poses[n].orientation.z;

    wp_posearray.poses.push_back(wp_pose);

    ///////////////////////////////////////////
    q_form yaw;
    std::vector<q_form> angle_in;
    yaw.w = wp_subpose.poses[n].orientation.w;
    yaw.x = wp_subpose.poses[n].orientation.x;
    yaw.y = wp_subpose.poses[n].orientation.y;
    yaw.z = wp_subpose.poses[n].orientation.z;

    // yaw.w = 1;
    // yaw.x = 1;
    // yaw.y = 1;
    // yaw.z = 1;  
    angle_in.push_back(yaw);  
    float heading = yaw_to_q(angle_in);

    float xf;
    float yf;
    float zf;
    float x;
    float y;
    float z;

    if (heading > -(M_PI / 2) && heading < (M_PI / 2))
    {
      x = sin(abs(heading)) * xoff;
      y = cos(abs(heading)) * xoff;
    }
    else
    {
      x = cos((abs(heading) - (M_PI / 2))) * xoff;
      y = sin((abs(heading) - (M_PI / 2))) * xoff;
    }

    if (heading > 0 && heading < M_PI)
    {
      xf = wp_subpose.poses[n].position.x - x;
    }
    else
    {
      xf = wp_subpose.poses[n].position.x + x;
    }

    //For y-axis sign, where first and fourth quadrant is negative
    if (heading > -(M_PI / 2) && heading < (M_PI / 2))
    {
      yf = wp_subpose.poses[n].position.y + y;
    }
    else
    {
      yf = wp_subpose.poses[n].position.y - y;
    }
    wp_pose1.position.x = xf;
    wp_pose1.position.y = yf;
    wp_pose1.position.z = zf;

    wp_pose1.orientation.w = wp_subpose.poses[n].orientation.w;
    wp_pose1.orientation.x = wp_subpose.poses[n].orientation.x;
    wp_pose1.orientation.y = wp_subpose.poses[n].orientation.y;
    wp_pose1.orientation.z = wp_subpose.poses[n].orientation.z;

    wp_posearray1.poses.push_back(wp_pose1);

    // ROS_INFO("x: %f",wp_subpose.poses[2].position.x);
  }
  wp_pub.publish(wp_posearray);
  wp_pub1.publish(wp_posearray1);
  // ROS_INFO("Last Form WP x:%f y:%f z:%f",wp_pose.position.x,wp_pose.position.y,wp_pose.position.z);
}

int main(int argc, char **argv)
{
  // std::string line;

  ros::init(argc, argv, "k");
  ros::NodeHandle k;

  wp_sub = k.subscribe<geometry_msgs::PoseArray>("/gnc/goal", 10, nav_wp);
  wp_pub = k.advertise<geometry_msgs::PoseArray>("/gnc/form/d1", 2);
  wp_pub1 = k.advertise<geometry_msgs::PoseArray>("/gnc/form/d2", 2);
  wp_pub2 = k.advertise<geometry_msgs::PoseArray>("/gnc/form/d3", 2);

  ros::Rate loop_rate(0.3);
  while (ros::ok())
  {
    line_form(3);
    ROS_INFO("published");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
