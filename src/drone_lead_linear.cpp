/*
  Lead Drone Node. Navigation is based on the leader. Navigation speed is limited by the positional formation error from the follower drones.
  Navigation is set as direct yaw to each waypoint.
*/
#include <gnc_functions.hpp>
#include <geometry_msgs/PoseArray.h>

ros::Subscriber error_sub;
geometry_msgs::Point error_point;
float avg_error = 0;

ros::Publisher wp_pub;
std_msgs::Bool wp_nav;


void error_cb (const geometry_msgs::Point::ConstPtr &msg)
{
  error_point = *msg;
  float upper_limit = 1;                            //Need to add this as parameter
  avg_error = (error_point.x + error_point.y)/2;    //Ignore vertical z error for now
  if (avg_error > upper_limit)
  {
    avg_error = upper_limit;
  }
  // ROS_INFO("Average position error %.2f",avg_error);
}

void move(float x, float y, float z)
{
  cmd_twist.twist.linear.x = x;
  cmd_twist.twist.linear.y = y;
  cmd_twist.twist.angular.z = z;
}


int main(int argc, char **argv)
{
  /* Establish Publisher and Subscribers
  */
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node("~");
  init_publisher_subscriber(gnc_node);
  error_sub = gnc_node.subscribe<geometry_msgs::Point>("/gnc/pos_error",10, error_cb);
  ROS_INFO("Subscribed to error");

  /* Drone Startup Procedure
  */
  wait4connect();
  set_mode("GUIDED");  ros::Duration(4.0).sleep();
  takeoff(3);          ros::Duration(5.0).sleep();

  /*Begin Control and Navigation
  */
  ros::Rate loop_rate(3);
  while (ros::ok())
  {
    move(0,0.5,0);
    twist_pub.publish(cmd_twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
