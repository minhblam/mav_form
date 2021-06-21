/*
  Follower Drone Node. Currently set to P stabilised. Velocity error control relative to lead drone.
  Manages direct control of drones.
*/
#include <gnc_functions.hpp>

#include <std_msgs/Float64.h>

#include <geometry_msgs/Point.h>
ros::Publisher error_pub;
geometry_msgs::Point error_point;

ros::Subscriber wp_sub;
std_msgs::Bool wp_nav;


/* PID Functions
*/
ros::Subscriber pidx_sub;
ros::Subscriber pidy_sub;
ros::Subscriber pidz_sub;

ros::Publisher pidx_pub;
ros::Publisher pidy_pub;
ros::Publisher pidz_pub;

ros::Publisher pidx_pos_pub;
ros::Publisher pidy_pos_pub;
ros::Publisher pidz_pos_pub;

std_msgs::Float64 pidx;
std_msgs::Float64 pidy;
std_msgs::Float64 pidz;
/* PID Functions End
*/

void pidx_cb (const std_msgs::Float64::ConstPtr &msg)
{
  pidx = *msg;
}

void pidy_cb (const std_msgs::Float64::ConstPtr &msg)
{
  pidy = *msg;
}

void pidz_cb (const std_msgs::Float64::ConstPtr &msg)
{
  pidz = *msg;
}

void wp_cb (const std_msgs::Bool::ConstPtr &msg)
{
  wp_nav = *msg;
}


void setpoint_form(float xoff, float offset_x_spawn, float offset_y_spawn, int number)
{
  //ROS NED Frame
  float x;
  float y;
  float z;
  float x_off;
  std_msgs::Float64 xf;
  std_msgs::Float64 yf;
  std_msgs::Float64 zf;
  std_msgs::Float64 d_pos_x;
  std_msgs::Float64 d_pos_y;
  std_msgs::Float64 d_pos_z;
  d_pos_x.data = d_pose.pose.pose.position.x;
  d_pos_y.data = d_pose.pose.pose.position.y;
  d_pos_z.data = d_pose.pose.pose.position.z;

  float number_float = number;
  if(number % 2==0) //If even
  {
    x_off = xoff*(number_float/2);
  }else{
    x_off = xoff*(number_float-1)/2;
  }

  x = abs(sin(l_heading)*x_off);
  y = abs(cos(l_heading)*x_off);

  if(number % 2==0) //If even
  {
    if (l_heading > 0 && l_heading <M_PI)
    {
      xf.data = lead_pose.pose.pose.position.x + offset_y_spawn - x;
    }else{
      xf.data = lead_pose.pose.pose.position.x + offset_y_spawn + x;
    }
    if (l_heading <= M_PI/2 && l_heading >= -M_PI/2)
    {
      yf.data = lead_pose.pose.pose.position.y - offset_x_spawn + y;
    }else{
      yf.data = lead_pose.pose.pose.position.y - offset_x_spawn - y;
    }
  }else{ //else odd
    if (l_heading <= M_PI/2 && l_heading >= -M_PI/2)
    {
      xf.data = lead_pose.pose.pose.position.x + offset_y_spawn + x;
    }else{
      xf.data = lead_pose.pose.pose.position.x + offset_y_spawn - x;
    }
    if (l_heading > 0 && l_heading <M_PI)
    {
      yf.data = lead_pose.pose.pose.position.y - offset_x_spawn - y;
    }else{
      yf.data = lead_pose.pose.pose.position.y - offset_x_spawn + y;
    }
  }

  zf.data = lead_pose.pose.pose.position.z;       // Add numbers here to offset position
  error_point.x = abs(xf.data - d_pos_x.data);
  error_point.y = abs(yf.data - d_pos_y.data);
  error_point.z = abs(zf.data - d_pos_z.data);

  //Publish Setpoint
  pidx_pub.publish(xf);
  pidy_pub.publish(yf);
  pidz_pub.publish(zf);
  //Publish State Position
  pidx_pos_pub.publish(d_pos_x);
  pidy_pos_pub.publish(d_pos_y);
  pidz_pos_pub.publish(d_pos_z);

  error_pub.publish(error_point);
  
  // if (ros_inumber(gnc_node) == 3)
  // {
  //   ROS_INFO("OFFSET l_head:%.3f space: %.1f Drone %.1f    x_off: %.2f and y_off: %.2f     L_POS lx: %.2f ly: %.2f     DES xf: %.2f yf: %.2f    D_POS x: %.2f y: %.2f    ERR: xe: %.2f ye: %.2f",l_heading,x_off,number_float,x,y,lead_pose.pose.pose.position.x,lead_pose.pose.pose.position.y,xf,yf,d_pose.pose.pose.position.x,d_pose.pose.pose.position.y,d_error.x,d_error.y);
  //   ROS_INFO("Heading Desired z:%f Desired x:%f y:%f z:%f   Actual x:%f y:%f z:%f",l_heading,xf,yf,zf,d_pose.pose.pose.position.x,d_pose.pose.pose.position.y,d_pose.pose.pose.position.z);
  // }
}

void set_position()
{
  cmd_twist.twist.linear.x = pidx.data;
  cmd_twist.twist.linear.y = pidy.data;
  cmd_twist.twist.linear.z = pidz.data;

  float yaw_error = -(d_heading - l_heading);
  cmd_twist.twist.angular.z = ::atan2(::sin(yaw_error),::cos(yaw_error)) * 0.6;

  //Publish Control Inputs
  twist_pub.publish(cmd_twist);
  // ROS_INFO("Drone Velocity inputs (%f,%f,%f)",cmd_twist.twist.linear.x,cmd_twist.twist.linear.y,cmd_twist.twist.linear.z);
}

int init_pid_follow(ros::NodeHandle controlnode)
{
  std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{
		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
	}
  // Subscribe to suggested control inputs
  pidx_sub = controlnode.subscribe<std_msgs::Float64>((ros_namespace + "/pidx/control").c_str(), 10, pidx_cb);
  pidy_sub = controlnode.subscribe<std_msgs::Float64>((ros_namespace + "/pidy/control").c_str(), 10, pidy_cb);
  pidz_sub = controlnode.subscribe<std_msgs::Float64>((ros_namespace + "/pidz/control").c_str(), 10, pidz_cb);

  //Publish desired position
  pidx_pub = controlnode.advertise<std_msgs::Float64>((ros_namespace + "/pidx/setpoint").c_str(), 10);
  pidy_pub = controlnode.advertise<std_msgs::Float64>((ros_namespace + "/pidy/setpoint").c_str(), 10);
  pidz_pub = controlnode.advertise<std_msgs::Float64>((ros_namespace + "/pidz/setpoint").c_str(), 10);

  //Publish Actual Position
  pidx_pos_pub = controlnode.advertise<std_msgs::Float64>((ros_namespace + "/pidx/state").c_str(), 10);
  pidy_pos_pub = controlnode.advertise<std_msgs::Float64>((ros_namespace + "/pidy/state").c_str(), 10);
  pidz_pos_pub = controlnode.advertise<std_msgs::Float64>((ros_namespace + "/pidz/state").c_str(), 10);
  return 0;
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
  wp_sub = gnc_node.subscribe<std_msgs::Bool>("/gnc/nav",10,wp_cb);
  wp_nav.data = 1;
  bool navigate = 1;

  /* Initiate Formation Parameters
  */
  int number = ros_number(gnc_node);
  float xoff = 2; //metres
  float offset_x_spawn = spawn_offset("x",gnc_node);
  float offset_y_spawn = spawn_offset("y",gnc_node);
  init_pid_follow(gnc_node);
  /* Drone Startup Procedure
  */
  wait4connect();
  // ros::Duration(20.0).sleep(); //May be required to ensure subscriber and publisher is ready
  set_mode("GUIDED");  ros::Duration(4.0).sleep();
  takeoff(3);          ros::Duration(5.0).sleep();

  /*Begin Control and Navigation
  */
  ros::Rate loop_rate(10);
  while (ros::ok() && navigate) // && wp_nav
  {
    if (wp_nav.data)
    {
      setpoint_form(xoff,offset_x_spawn,offset_y_spawn,number);
      set_position();
    }else{
      land();
      ros::Duration(3.0).sleep();
      navigate = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
