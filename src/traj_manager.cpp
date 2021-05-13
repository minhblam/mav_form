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
geometry_msgs::PoseArray wppose_sub;
ros::Subscriber sub_wp1;
geometry_msgs::PoseArray wppose_sub1;

void wp_cb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  wppose_sub = *msg;
}

void wp_cb1(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  wppose_sub1 = *msg;
}

void set_destination1(geometry_msgs::PoseArray array) //int n
{
  posestamped.pose.position.x = array.poses[n].position.x;
  posestamped.pose.position.y = array.poses[n].position.y;
  posestamped.pose.position.z = array.poses[n].position.z;
  // posestamped.pose.orientation.w = wp_posearray.poses[counter].orientation.w;
  // posestamped.pose.orientation.x = wp_posearray.poses[counter].orientation.x;
  // posestamped.pose.orientation.y = wp_posearray.poses[counter].orientation.y;
  // posestamped.pose.orientation.z = wp_posearray.poses[counter].orientation.z;
  pose_pub1.publish(posestamped);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nh");
  ros::NodeHandle nh;

  init_publisher_subscriber(nh, "/drone1");
  init_publisher_subscriber(nh, "/drone2");
  sub_wp = nh.subscribe<geometry_msgs::PoseArray>("/gnc/form/d1", 10, wp_cb);
  sub_wp1 = nh.subscribe<geometry_msgs::PoseArray>("/gnc/form/d2", 10, wp_cb1);
  wait4connect();
  set_mode("GUIDED");
  takeoff(5);

  std::vector<gnc_WP> wp_in;
  gnc_WP wp_list;
  wp_list.x = wppose_sub.poses[1].position.x;
  wp_list.y = 0;
  wp_list.z = 0;
  wp_in.push_back(wp_list);
  posestamped.pose.position.x=wp_list.x;
  posestamped.pose.position.y=wp_list.y;
  posestamped.pose.position.z=wp_list.z;

  pose_pub.publish(posestamped);

  // boost::shared_ptr<geometry_msgs::PoseArray const> sharedarray;
  // geometry_msgs::PoseArray array;
  // sharedarray = ros::topic::waitForMessage<geometry_msgs::PoseArray>("/gnc/form", 10,wp_cb);
  // if(sharedarray != NULL){
  //   array = *sharedarray;
  // }

  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    
    // set_destination();

    ros::spin();
    loop_rate.sleep();
  }
  return 0;
}
