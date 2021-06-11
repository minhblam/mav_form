/*
  Follower Drone Node. Currently set to P stabilised. Velocity error control relative to lead drone.
  Manages direct control of drones.
*/
#include <gnc_functions.hpp>

#include <geometry_msgs/Point.h>
ros::Publisher error_pub;
geometry_msgs::Point error_point;

gnc_error error_form(float xoff, float spawnx, float spawny, int number)
{
  //ROS NED Frame
  float x;
  float y;
  float z;
  float xf;
  float yf;
  float zf;
  float x_off;
  gnc_error d_error;

  float number_float = number;
  if(number % 2==0) //If even
  {
    x_off = xoff*(number_float/2);
  }else{
    x_off = xoff*(number_float-1)/2;
  }

  x = abs(sin(l_heading)*x_off);
  y = abs(cos(l_heading)*x_off);
  // top left formation conflict WHY?? 3 moves to 2's position

  if(number % 2==0) //If even
  {
    if (l_heading > 0 && l_heading <M_PI)
    {
      xf = lead_pose.pose.pose.position.x + spawny - x;
    }else{
      xf = lead_pose.pose.pose.position.x + spawny + x;
    }
    if (l_heading <= M_PI/2 && l_heading >= -M_PI/2)
    {
      yf = lead_pose.pose.pose.position.y - spawnx + y;
    }else{
      yf = lead_pose.pose.pose.position.y - spawnx - y;
    }
  }else{ //else odd
    if (l_heading <= M_PI/2 && l_heading >= -M_PI/2)
    {
      xf = lead_pose.pose.pose.position.x + spawny + x;
    }else{
      xf = lead_pose.pose.pose.position.x + spawny - x;
    }
    if (l_heading > 0 && l_heading <M_PI)
    {
      yf = lead_pose.pose.pose.position.y - spawnx - y;
    }else{
      yf = lead_pose.pose.pose.position.y - spawnx + y;
    }
  }
  zf = lead_pose.pose.pose.position.z + 1; // Add numbers here to offset position

  d_error.x = xf - d_pose.pose.pose.position.x;
  d_error.y = yf - d_pose.pose.pose.position.y;
  d_error.z = zf - d_pose.pose.pose.position.z;
  
  // if (ros_inumber(gnc_node) == 3)
  // {
  //   ROS_INFO("OFFSET l_head:%.3f space: %.1f Drone %.1f    x_off: %.2f and y_off: %.2f     L_POS lx: %.2f ly: %.2f     DES xf: %.2f yf: %.2f    D_POS x: %.2f y: %.2f    ERR: xe: %.2f ye: %.2f",l_heading,x_off,number_float,x,y,lead_pose.pose.pose.position.x,lead_pose.pose.pose.position.y,xf,yf,d_pose.pose.pose.position.x,d_pose.pose.pose.position.y,d_error.x,d_error.y);
  //   ROS_INFO("Heading Desired z:%f Desired x:%f y:%f z:%f   Actual x:%f y:%f z:%f",l_heading,xf,yf,zf,d_pose.pose.pose.position.x,d_pose.pose.pose.position.y,d_pose.pose.pose.position.z);
  // }
  return d_error;
}


void set_form(float xoff, float spawnx, float spawny, int number)
{
  cmd_twist.twist.linear.x = error_form(xoff,spawnx,spawny,number).x*0.6;
  cmd_twist.twist.linear.y = error_form(xoff,spawnx,spawny,number).y*0.6;
  cmd_twist.twist.linear.z = error_form(xoff,spawnx,spawny,number).z*0.6;

  float yaw_error = -(d_heading - l_heading);
  cmd_twist.twist.angular.z = ::atan2(::sin(yaw_error),::cos(yaw_error)) * 0.6;

  twist_pub.publish(cmd_twist);
  // ROS_INFO("Drone %i Velocity inputs (%f,%f,%f)",ros_number(gnc_node),cmd_twist.twist.linear.x,cmd_twist.twist.linear.y,cmd_twist.twist.linear.z);
}

void publish_error (float xoff,float spawnx, float spawny, int number)
{
  error_point.x = error_form(xoff,spawnx,spawny,number).x;
  error_point.y = error_form(xoff,spawnx,spawny,number).y;
  error_point.z = error_form(xoff,spawnx,spawny,number).z;
  error_pub.publish(error_point);
}

int main(int argc, char **argv)
{
  /* Establish Publisher and Subscribers
  */
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node("~");
  init_publisher_subscriber(gnc_node);
  init_leader_subscriber(gnc_node);
  error_pub = gnc_node.advertise<geometry_msgs::Point>("/gnc/pos_error",10);

  /* Initiate Formation Parameters
  */
  int number = ros_inumber(gnc_node);
  float xoff = 2; //metres
  float spawnx = spawn_offset("x",gnc_node);
  float spawny = spawn_offset("y",gnc_node);

  /* Drone Startup Procedure
  */
  wait4connect();
  set_mode("GUIDED");  ros::Duration(4.0).sleep();
  takeoff(3);          ros::Duration(5.0).sleep();

  /*Begin Control and Navigation
  */
  ros::Rate loop_rate(3);
  while (ros::ok()) // && wp_nav
  {
    set_form(xoff,spawnx,spawny,number);
    publish_error(xoff,spawnx,spawny,number);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
