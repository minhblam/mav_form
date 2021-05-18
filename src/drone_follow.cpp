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
  std::string ros_number;
  ros::NodeHandle gnc_node;
  if (!gnc_node.hasParam("number"))
  {
    ROS_INFO("Drone No. fix failed");
  }else{
    gnc_node.getParam("number",ros_number);
    ROS_INFO("Drone No.%s fix successful",ros_number.c_str());
  }
  int namespace_int = stoi(ros_number.c_str()); //get drone number as integer

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
  ROS_INFO("Drone No. %s formation error (%f,%f,%f)",ros_number.c_str(),d_error.x,d_error.y,d_error.z )
  return d_error;
}

void set_form(gnc_error d_error, float xoff)
{
  cmd_twist.twist.linear.x = d_twist.twist.linear.x + error_form(xoff).x;
  cmd_twist.twist.linear.x = d_twist.twist.linear.y + error_form(xoff).y;
  cmd_twist.twist.linear.x = d_twist.twist.linear.z + error_form(xoff).z;

  float yaw_error = psi - lpsi;
  cmd_twist.twist.angular.z = d_twist.twist.angular.z + yaw_error; //May need to change direction

  twist_pub.publish(cmd_twist);
  ROS_INFO("Velocity inputs (%f,%f,%f)",cmd_twist.twist.linear.x,cmd_twist.twist.linear.y,cmd_twist.twist.linear.z)
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node("~");

  init_publisher_subscriber(gnc_node);
  init_leader_subscriber(gnc_node);

  wait4connect();

  set_mode("GUIDED");

  takeoff(3);

  bool wp_nav = 1; //Change this to 0 when navigation is complete

  float xoff = 2;
  ros::Rate loop_rate(5);
  while (ros::ok() && wp_nav)
  {
    set_form(error_form(xoff),xoff);
    ros::spin();
    loop_rate.sleep();
  }
  return 0;
}
