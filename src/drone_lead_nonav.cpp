// Follower drones. Currently set to P stabilised. Velocity error control relative to lead drone.
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <gnc_functions.hpp>

#include <geometry_msgs/Point.h>
ros::Subscriber error_sub;
geometry_msgs::Point error_point;

int n = 1; //Waypoint Progression
std::vector<gnc_WP> wp_in;

void error_cb (const geometry_msgs::Point::ConstPtr &msg)
{
  error_point = *msg;
  // std::vector<gnc_error> error_in;
  // gnc_error error_group;
  // float sum_error;

  // error_group.x = error_point.x;
  // error_group.y = error_point.y;
  // error_group.z = error_point.z;
  // ros::NodeHandle gnc_node("~");
  // // The idea is to have the vector set to the last 3 values
  // if (error_in.size() < (ros_inumber(gnc_node)-1))
  // {
  //   error_in.push_back(error_group);
  // }else{
  //   error_in.erase(error_in.begin(), error_in.begin()+1);
  // }
  // float sum = 0;
  // for (int i = 0; i < (ros_inumber(gnc_node)-1);i++)
  // {
  //   sum = sum + error_in[i];
  // }
  // float avg_error = sum/error_in.size();
  // ROS_INFO("Sum error = %.2f",avg_error);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node("~");
  init_publisher_subscriber(gnc_node);
  ros::Duration(3.0).sleep();

  
  error_sub = gnc_node.subscribe<geometry_msgs::Point>("/gnc/pos_error",10, error_cb);
  // wp_sub = gnc_node.subscribe<geometry_msgs::Pose>("/gnc/goal", 10, nav_wp);
  // bool_pub = gnc_node.advertise<std_msgs::Bool>("/gnc/wpreach", 10);
 
  // bool navigate = 1; //This may be able to be used as a topic like nav_complete.publish(navigate) where std_msgs::Bool navigate as a global.
  wait4connect();
  set_mode("GUIDED");
  takeoff(3);
  ros::Rate loop_rate(3);
  ros::Duration(12.0).sleep();

  while (ros::ok())
  {
    cmd_twist.twist.angular.z = 0.1;
    twist_pub.publish(cmd_twist);
    ros::spinOnce(); //set to spinonce
    loop_rate.sleep();
  }

  // land();


  return 0;
}
