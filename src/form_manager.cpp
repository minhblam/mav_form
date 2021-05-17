// Navigation Manager
// Modifies given waypoints from the navigation manager and creates offset for any given number of drones to pass down to the tracjectory manager.

// #include <math.h>
// #include <vector>
// #include <iostream>
// #include <string>
#include <control_functions.hpp>

// #include <geometry_msgs/Pose.h>
ros::Publisher wp_pub;

ros::Subscriber wp_sub;


int drone_count = 3;


geometry_msgs::Pose wp_subpose;//New Waypoint from nav_manager
geometry_msgs::PoseStamped wp_pubpose;

void nav_wp(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  wp_subpose = *msg; //wp_subpose
}



void line_form(float xoff) //, int k
{                //Where off is offset dist and k is the number of drones
  std::vector<gnc_wppose> wp_mod;

  wp_mod.x = wp_subpose.position.x;
  wp_mod.y = wp_subpose.position.y;
  wp_mod.z = wp_subpose.position.z;
  wp_mod.qw = wp_subpose.orientation.w;
  wp_mod.qx = wp_subpose.orientation.x;
  wp_mod.qy = wp_subpose.orientation.y;
  wp_mod.qz = wp_subpose.orientation.z;
  wp_pubpose.header.frameid= 'd1';
  wp_pubpose.pose = wp_mod;
  wp_pub.publish(wp_pubpose);

  for (int n = 1; n < drone_count)
  {
    f
  }

}

int main(int argc, char **argv)
{
  // std::string line;

  ros::init(argc, argv, "k");
  ros::NodeHandle k;

  wp_sub = k.subscribe<geometry_msgs::Pose>("/gnc/goal", 10, nav_wp);
  wp_pub = k.advertise<geometry_msgs::PoseStamped>("/gnc/form", 2);

  ros::Rate loop_rate(0.3);
  while (ros::ok())
  {
    if (message_received){ //Create a function for this
      wp_pub.publish(wp_pose); //Will need to make this scalable
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
