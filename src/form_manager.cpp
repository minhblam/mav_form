// Navigation Manager
// Modifies given waypoints from the navigation manager and creates offset for any given number of drones to pass down to the tracjectory manager.

#include <math.h>
#include <vector>
#include <iostream>
// #include <string>
#include <control_functions.hpp>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
ros::Publisher wp_pub;
ros::Subscriber wp_sub;
geometry_msgs::PoseArray wp_subpose;   //Original Waypoint Array
geometry_msgs::PoseArray wp_posearray; //New Waypoint Array for Drones
geometry_msgs::PoseStamped wp_pose;    //New Waypoint

void nav_wp(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  wp_subpose = *msg; //wp_subpose
}

void line_form(float x_off, int k)
{ //Where off is offset dist and k is the number of drones
  // for (int i = 0; i<k-1 ;k++){ //For each drone

  //If statement to alternate sides or something NEED HERE
  for (int n = 0; n < wp_subpose.size() - 1; n++)
  { //For each WP
    wp_pose.header.seq = i;
    wp_pose.pose.position.x = wp_subpose[n].poses.position.x + 2;
    wp_pose.pose.position.y = wp_subpose[n].poses.position.y + 2;
    wp_pose.pose.position.z = wp_subpose[n].poses.position.z + 2;

    wp_pose.pose.orientation.w = wp_subpose[n].orientation.w;
    wp_pose.pose.orientation.x = wp_subpose[n].orientation.x;
    wp_pose.pose.orientation.y = wp_subpose[n].orientation.y;
    wp_pose.pose.orientation.z = wp_subpose[n].orientation.z;
    //Ideally the below:
    // wp_pose.pose.orientation = wp_subpose[n].orientation;
    wp_posearray.pushback(wp_pose);
  }

  wp_pub.publish(wp_posearray);
}
// This function generates offset waypoints for each drone. Obviously needs more loops right now.
}

// void vee_form(float x_off, float y_off, int k){ //Where off is offset dist and k is the number of drones
// // This function generates offset waypoints for each drone. Obviously needs more loops right now.
//   if (drone_number_odd(k)){ //Left
//     offset[k].x = heading*line; //Find the x coordinate in a line next to original WP
//     offset[k].y = heading*line; //Find the y coordinate in a line next to original WP
//     offset[k].z = heading*line; //Find the z coordinate in a line next to original WP
//   }else{                    //Right
//     offset[k].x = heading*line; //Find the x coordinate in a line next to original WP
//     offset[k].y = heading*line; //Find the y coordinate in a line next to original WP
//     offset[k].z = heading*line; //Find the z coordinate in a line next to original WP
//   }
// }

void run_form(std::string form)
{
  if (form == 'line')
  {
    line_form(2, 4) // Replace with subscribed values from nav_manager
  }
  if (form == 'vee')
  {
    vee_form(x, y, z, psi) // Replace with subscribed values from nav_manager
  }
}

int main(int argc, char **argv)
{
  //Add ROScore stuff here
  wp_sub = n.subscribe<geometry_msgs::PoseArray>("/gnc/goal", 10, nav_wp);
  wp_pub = n.advertise<geometry_msgs::PoseArray>("/gnc/form", 10);
  run_form('v');
  return 0;
}
