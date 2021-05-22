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
  gnc_error d_error;
  ros::NodeHandle gnc_node("~");
  int number_int = ros_inumber(gnc_node);
  float number_float = ros_fnumber(gnc_node);
  
  

  if (l_heading < M_PI/2 && l_heading > -M_PI/2){
    x = cos(abs(l_heading))*(xoff*((number_float-1)/2));
    y = sin(abs(l_heading))*(xoff*((number_float-1)/2));
  }else{
    x = sin(abs(l_heading))*(xoff*((number_float-1)/2));
    y = cos(abs(l_heading))*(xoff*((number_float-1)/2));
  }

  if(number_int & 1) //If odd number
  {
    // ROS_INFO("Drone %i is an odd numbered drone",namespace_int);
    if (l_heading < M_PI/2 && l_heading > -M_PI/2)
    {
      xf = lead_pose.pose.pose.position.x - x;
    }else{
      xf = lead_pose.pose.pose.position.x + x;
    }
    if (l_heading > 0 && l_heading <M_PI)
    {
      yf = lead_pose.pose.pose.position.y - y;
    }else{
      yf = lead_pose.pose.pose.position.y + y;
    }
  }else{ //If even number
  // ROS_INFO("Drone %i is an odd numbered drone",namespace_int);
    if (l_heading < M_PI/2 && l_heading > -M_PI/2)
    {
      xf = lead_pose.pose.pose.position.x + x;
    }else{
      xf = lead_pose.pose.pose.position.x - x;
    }
    if (l_heading > 0 && l_heading <M_PI)
    {
      yf = lead_pose.pose.pose.position.y + y;
    }else{
      yf = lead_pose.pose.pose.position.y - y;
    }
  }
  zf = lead_pose.pose.pose.position.z;

  d_error.x = xf - d_pose.pose.pose.position.x;
  d_error.y = yf - d_pose.pose.pose.position.y;
  d_error.z = zf - d_pose.pose.pose.position.z;
  
  if (ros_inumber(gnc_node) == 2)
  {
    // ROS_INFO("Heading Desired z:%f Desired x:%f y:%f z:%f   Actual x:%f y:%f z:%f",l_heading,xf,yf,zf,d_pose.pose.pose.position.x,d_pose.pose.pose.position.y,d_pose.pose.pose.position.z);
    ROS_INFO("OFFSET l_heading:%f xoff: %f n_f: %f is x_off: %f and y_off: %f    L_POS lx: %f ly: %f    DESIRED xf: %f yf: %f   D_POS x: %f y: %f   ERROR: xe: %f ye: %f",l_heading,xoff,number_float,x,y,lead_pose.pose.pose.position.x,lead_pose.pose.pose.position.y,xf,yf,d_pose.pose.pose.position.x,d_pose.pose.pose.position.y,d_error.x,d_error.y);
    // ROS_INFO("Drone No. %i formation error (%f,%f,%f)",ros_inumber(gnc_node),d_error.x,d_error.y,d_error.z );
  }
  // ROS_INFO("Leader Heading %f",l_heading);
  return d_error;
}

void set_form(gnc_error d_error, float xoff)
{
  ros::NodeHandle gnc_node("~");
  cmd_twist.twist.linear.x = d_twist.twist.linear.x + error_form(xoff).x*0.5;
  cmd_twist.twist.linear.y = d_twist.twist.linear.y + error_form(xoff).y*0.5;
  cmd_twist.twist.linear.z = d_twist.twist.linear.z + error_form(xoff).z*0.5;
  // ROS_INFO("Error x:%f y:%f z:%f",error_form(xoff).x,error_form(xoff).y,error_form(xoff).z);

  float yaw_error = d_heading - l_heading; //error in radians
  // cmd_twist.twist.angular.z = d_twist.twist.angular.z + yaw_error*0.05; //May need to change direction
  cmd_twist.twist.angular.z = 0.2; //positive is left
  if (ros_inumber(gnc_node)==2){
    ROS_INFO("Yaw Error: %f",yaw_error);
  }

  twist_pub.publish(cmd_twist);
  // ROS_INFO("Drone %i Velocity inputs (%f,%f,%f)",ros_number(gnc_node),cmd_twist.twist.linear.x,cmd_twist.twist.linear.y,cmd_twist.twist.linear.z);
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

  // bool wp_nav = 1; //Change this to 0 when navigation is complete

  float xoff = 1; //metres????
  ros::Rate loop_rate(2);
  while (ros::ok()) // && wp_nav
  {
    set_form(error_form(xoff),xoff);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
