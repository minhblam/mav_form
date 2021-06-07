// Follower drones. Currently set to P stabilised. Velocity error control relative to lead drone.
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <gnc_functions.hpp>

#include <geometry_msgs/Point.h>
ros::Publisher error_pub;
geometry_msgs::Point error_point;

gnc_error error_form(float xoff) //float yoff, float zoff
{
  //Verify if this is gazebo X or ROS X (ENU OR NED Frame)
  float x;
  float y;
  float z;
  float xf;
  float yf;
  float zf;
  float x_off;
  gnc_error d_error;
  ros::NodeHandle gnc_node("~");
  int number_int = ros_inumber(gnc_node);
  float number_float = ros_fnumber(gnc_node);
  float spawnx = spawn_offset("x",gnc_node);
  float spawny = spawn_offset("y",gnc_node);
  
  if(number_int & 1) //If odd number
  {
    x_off = xoff*(number_float-1)/2;
  }else{
    x_off = xoff*(number_float/2);
  }

  x = abs(cos(l_heading)*x_off) - spawny;
  y = abs(sin(l_heading)*x_off) + spawnx;


  if(number_int & 1) //If odd number
  {
    // ROS_INFO("Drone %i is an odd numbered drone",namespace_int);
    if (l_heading > -M_PI/2 && l_heading < M_PI/2)
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
  ROS_INFO("OFFSET head:%.3f space: %.1f Drone %.1f    x_off: %.2f and y_off: %.2f     L_POS lx: %.2f ly: %.2f     DES xf: %.2f yf: %.2f    D_POS x: %.2f y: %.2f    ERR: xe: %.2f ye: %.2f",l_heading,x_off,number_float,x,y,lead_pose.pose.pose.position.x,lead_pose.pose.pose.position.y,xf,yf,d_pose.pose.pose.position.x,d_pose.pose.pose.position.y,d_error.x,d_error.y);
  
  // if (ros_inumber(gnc_node) == 2)
  // {
  //   // ROS_INFO("Heading Desired z:%f Desired x:%f y:%f z:%f   Actual x:%f y:%f z:%f",l_heading,xf,yf,zf,d_pose.pose.pose.position.x,d_pose.pose.pose.position.y,d_pose.pose.pose.position.z);
  //   // 
  //   // ROS_INFO("Drone No. %i formation error (%f,%f,%f)",ros_inumber(gnc_node),d_error.x,d_error.y,d_error.z );
  // }
  // ROS_INFO("Leader Heading %f",l_heading);
  return d_error;
}


void set_form(float xoff)
{
  cmd_twist.twist.linear.x = d_twist.twist.linear.x + error_form(xoff).x*0.6;
  cmd_twist.twist.linear.y = d_twist.twist.linear.y + error_form(xoff).y*0.6;
  cmd_twist.twist.linear.z = d_twist.twist.linear.z + error_form(xoff).z*0.6;
  // ROS_INFO("Error x:%f y:%f z:%f",error_form(xoff).x,error_form(xoff).y,error_form(xoff).z);

  float yaw_error = d_heading - l_heading; //error in radians
  cmd_twist.twist.angular.z = d_twist.twist.angular.z - yaw_error*0.3; //May need to change direction
  // cmd_twist.twist.angular.z = 0.2; //positive is left

  twist_pub.publish(cmd_twist);
  // ROS_INFO("Drone %i Velocity inputs (%f,%f,%f)",ros_number(gnc_node),cmd_twist.twist.linear.x,cmd_twist.twist.linear.y,cmd_twist.twist.linear.z);
}

void publish_error (float xoff)
{
  error_point.x = error_form(xoff).x;
  error_point.y = error_form(xoff).y;
  error_point.z = error_form(xoff).z;
  error_pub.publish(error_point);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node("~");

  init_publisher_subscriber(gnc_node);
  init_leader_subscriber(gnc_node);

  float xoff = 2; //metres????
  float spawnx = spawn_offset("x",gnc_node);
  float spawny = spawn_offset("y",gnc_node);

  error_pub = gnc_node.advertise<geometry_msgs::Point>("/gnc/pos_error",10);

  wait4connect();

  set_mode("GUIDED");

  takeoff(3);
  ros::Duration(5.0).sleep();

  // bool wp_nav = 1; //Change this to 0 when navigation is complete

  ros::Rate loop_rate(3);
  while (ros::ok()) // && wp_nav
  {
    set_form(xoff);
    publish_error(xoff);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
