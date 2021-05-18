// Follower drones. Currently set to P stabilised. Velocity error control relative to lead drone.
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <gnc_functions.hpp>

gnc_error error_form(float xoff) //float yoff, float zoff
{
  float x;
  float y;
  float z;
  float xf;
  float yf;
  float zf;
  std::string ros_namespace;
  ros::NodeHandle gnc_node;
  if (!gnc_node.hasParam("namespace"))
  {
    ROS_INFO("Using default namespace");
  }else{
    gnc_node.getParam("namespace",ros_namespace);
    ROS_INFO("Using namespace %s",ros_namespace.c_str());
  }
  int namespace_int = atoi(ros_namespace.c_str()); //get drone number as integer

  if (lpsi < M_PI/2 && lpsi > -M_PI/2){
    x = cos(abs(lpsi))*(xoff*((namespace_int-1)/2));
    y = sin(abs(lpsi))*(xoff*((namespace_int-1)/2));
  }else{
    x = sin(abs(lpsi))*(xoff*((namespace_int-1)/2));
    y = cos(abs(lpsi))*(xoff*((namespace_int-1)/2));
  }

  if(namespace_int & 1) //If odd number
  {
    if (lpsi < M_PI/2 && lpsi > -M_PI/2)
    {
      xf = lead_pose.pose.pose.position.x - x;
    }else{
      xf = lead_pose.pose.pose.position.x + x;
    }
    if (lpsi > 0 && lpsi <M_PI)
    {
      yf = lead_pose.pose.pose.position.y - y;
    }else{
      yf = lead_pose.pose.pose.position.y + y;
    }
  }else{ //If even number
    if (lpsi < M_PI/2 && lpsi > -M_PI/2)
    {
      xf = lead_pose.pose.pose.position.x + x;
    }else{
      xf = lead_pose.pose.pose.position.x - x;
    }
    if (lpsi > 0 && lpsi <M_PI)
    {
      yf = lead_pose.pose.pose.position.y + y;
    }else{
      yf = lead_pose.pose.pose.position.y - y;
    }
  }

  gnc_error d_error;
  d_error.x = xf - d_pose.pose.pose.position.x;
  d_error.y = yf - d_pose.pose.pose.position.y;
  d_error.z = zf - d_pose.pose.pose.position.z;

  return d_error;
}

void set_heading()
{
  float yaw_error = psi - lpsi;
  cmd_twist.twist.angular.z = d_twist.twist.angular.z + yaw_error; //May need to change direction
}

void set_form(gnc_error d_error, float xoff)
{
  cmd_twist.twist.linear.x = d_twist.twist.linear.x + error_form(xoff).x;
  cmd_twist.twist.linear.x = d_twist.twist.linear.y + error_form(xoff).y;
  cmd_twist.twist.linear.x = d_twist.twist.linear.z + error_form(xoff).z;
  set_heading();
  twist_pub.publish(cmd_twist);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node("~");

  init_publisher_subscriber(gnc_node);
  init_leader_subscriber();

  wait4connect();

  set_mode("GUIDED");

  takeoff(3);

  bool wp_nav = 1; //Change this to 0 when navigation is complete

  ros::Rate loop_rate(2);
  while (ros::ok() && wp_nav)
  {
    set_form(error_form(2),2);
    ros::spin();
    loop_rate.sleep();
  }
  return 0;
}
