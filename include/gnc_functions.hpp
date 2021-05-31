#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>

#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <cmath>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <ros/ros.h>
#include <ros/duration.h>


//Messages
mavros_msgs::State state;
geometry_msgs::PoseStamped cmd_pose; //Pose Pub
nav_msgs::Odometry d_pose; //Pose Sub
geometry_msgs::TwistStamped cmd_twist; //Twist Pub
geometry_msgs::TwistStamped d_twist; //Twist Sub

nav_msgs::Odometry lead_pose;
geometry_msgs::TwistStamped lead_twist;


//Topics
ros::Subscriber state_sub; // Drone 2 State Feedback
ros::Publisher pose_pub;   // Drone 2 State Control Must be from main variable pose_pub.publish(value)
ros::Subscriber pose_sub;  // Must be from main variable
ros::Publisher twist_pub;
ros::Subscriber twist_sub;

ros::Subscriber lead_twist_sub;
ros::Subscriber lead_pose_sub;
//Services
ros::ServiceClient set_mode_client; //Enables String Input for setting GUIDED mode among others
ros::ServiceClient arming_client;   //Enables Built-in Arming of drone
ros::ServiceClient takeoff_client;  //Enables Built-in Takeoff
ros::ServiceClient command_client;  //Primarily for Set Speed Command
ros::ServiceClient land_client;

float psi = 0;
float d_heading;
float lpsi = 0;
float l_heading;

/*      Control Functions
*/
void pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  d_pose = *msg;
  float qw = d_pose.pose.pose.orientation.w;
  float qx = d_pose.pose.pose.orientation.x;
  float qy = d_pose.pose.pose.orientation.y;
  float qz = d_pose.pose.pose.orientation.z;
  float psi = atan2(2 * (qw * qz + qx * qy) , 1 - 2 * (qy * qy + qz * qz) );
  d_heading = psi;
  // ROS_INFO("Position x:%.3f y:%.3f z:%.3f",d_pose.pose.pose.position.x,d_pose.pose.pose.position.y,d_pose.pose.pose.position.z );
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  state = *msg;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  d_twist = *msg;
}

void lead_pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  lead_pose = *msg;
  float qw = lead_pose.pose.pose.orientation.w;
  float qx = lead_pose.pose.pose.orientation.x;
  float qy = lead_pose.pose.pose.orientation.y;
  float qz = lead_pose.pose.pose.orientation.z;
  float lpsi = atan2(2 * (qw * qz + qx * qy) , 1 - 2 * (qy * qy + qz * qz) );
  l_heading = lpsi;
  // ROS_INFO("Lead Heading %f", lpsi*(180/M_PI));
}
void lead_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  lead_twist = *msg;
}

/*      Pathing Functions
*/

struct gnc_WP
{
  float x;   // distance in x with respect to your reference frame
  float y;   // distance in y with respect to your reference frame
  float z;   // distance in z with respect to your reference frame
};

struct gnc_error
{
  float x;   // distance in x with respect to your reference frame
  float y;   // distance in y with respect to your reference frame
  float z;   // distance in z with respect to your reference frame
};

// struct q_form
// {
//   float w;
//   float x;
//   float y;
//   float z;
// };

struct gnc_wppose
{
  float x;   // distance in x with respect to your reference frame
  float y;   // distance in y with respect to your reference frame
  float z;   // distance in z with respect to your reference frame
  float qw;  // quaternion in w
  float qx;  // quaternion in x
  float qy;  // quaternion in y
  float qz;  // quaternion in z
};

/*      Preflight Functions
*/
int wait4connect()
{
  ROS_INFO("Waiting for FCU connection");
  // wait for FCU connection
  while (ros::ok() && !state.connected)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if (state.connected)
  {
    ROS_INFO("Connected to FCU");
    return 0;
  }
  else
  {
    ROS_INFO("Error connecting to drone");
    return -1;
  }
}

int set_speed(float speed__mps)
{
	mavros_msgs::CommandLong speed_cmd;
	speed_cmd.request.command = 178;
	speed_cmd.request.param1 = 1; // ground speed type 
	speed_cmd.request.param2 = speed__mps;
	speed_cmd.request.param3 = -1; // no throttle change
	speed_cmd.request.param4 = 0; // absolute speed
	ROS_INFO("setting speed to %f", speed__mps);
	if(command_client.call(speed_cmd))
	{
		ROS_INFO("change speed command succeeded %d", speed_cmd.response.success);
		return 0;
	}else{
		ROS_ERROR("change speed command failed %d", speed_cmd.response.success);
		ROS_ERROR("change speed result was %d ", speed_cmd.response.result);
		return -1;
	}
	ROS_INFO("change speed result was %d ", speed_cmd.response.result);
	return 0;
}

int takeoff(float takeoff_alt)
{

  ROS_INFO("Arming drone");
  mavros_msgs::CommandBool arm_request;
  arm_request.request.value = true;
  while (!state.armed && !arm_request.response.success && ros::ok())
  {
    ros::Duration(.1).sleep();
    arming_client.call(arm_request);
    pose_pub.publish(cmd_pose);
  }
  if (arm_request.response.success)
  {
    ROS_INFO("Arming Successful");
  }
  else
  {
    ROS_INFO("Arming failed with %d", arm_request.response.success);
    return -1;
  }

  //request takeoff
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = takeoff_alt;
  if (takeoff_client.call(srv_takeoff))
  {
    sleep(3);
    ROS_INFO("Takeoff Sent %d", srv_takeoff.response.success);
  }
  else
  {
    ROS_ERROR("Failed Takeoff");
    return -2;
  }
  sleep(2);
  return 0;
}

int set_mode(std::string mode)
{
  mavros_msgs::SetMode srv_setMode;
  srv_setMode.request.base_mode = 0;
  srv_setMode.request.custom_mode = mode;
  if (set_mode_client.call(srv_setMode))
  {
    ROS_INFO("SetMode send ok");
  }
  else
  {
    ROS_ERROR("Failed SetMode");
    return -1;
  }
}

int land()
{
  mavros_msgs::CommandTOL srv_land;
  if(land_client.call(srv_land) && srv_land.response.success)
  {
    ROS_INFO("land sent %d", srv_land.response.success);
    return 0;
  }else{
    ROS_ERROR("Landing failed");
    return -1;
  }
}


/*      Establish Connections
*/

int ros_inumber(ros::NodeHandle controlnode)
{
  int ros_number;
	if (!controlnode.hasParam("number"))
	{

		// ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("number", ros_number);
		// ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
  return ros_number;
}

float ros_fnumber(ros::NodeHandle controlnode)
{
  float ros_number;
	if (!controlnode.hasParam("number"))
	{

		// ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("number", ros_number);
		// ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
  return ros_number;
}


int init_leader_subscriber(ros::NodeHandle controlnode)
{
  lead_pose_sub = controlnode.subscribe<nav_msgs::Odometry>("/drone1/mavros/global_position/local", 10, lead_pose_cb);// add unique callback
  lead_twist_sub = controlnode.subscribe<geometry_msgs::TwistStamped>("/drone1/mavros/global_position/gp_vel", 10, lead_vel_cb);
  
  return 0;
}

int init_publisher_subscriber(ros::NodeHandle controlnode)
{
  std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{

		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
  //Waypoint Position Publishing
  pose_pub = controlnode.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local").c_str(), 10); // For Built in setpoint WP control
  //Position Subscribing
  pose_sub = controlnode.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);// add unique callback
  //Velocity Publishing
  twist_pub = controlnode.advertise<geometry_msgs::TwistStamped>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel").c_str(), 10);
  //Velocity Subscribing
  twist_sub = controlnode.subscribe<geometry_msgs::TwistStamped>((ros_namespace + "/mavros/global_position/gp_vel").c_str(), 10, vel_cb);
  //State Subscribing
  state_sub = controlnode.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 10, state_cb); //add nique callback
  //Set_Mode Client
  set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
  //Arming Client
  arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
  //Takeoff Client
  takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());

  land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());

  command_client = controlnode.serviceClient<mavros_msgs::CommandLong>((ros_namespace +"/mavros/cmd/command").c_str());

  return 0;
}
