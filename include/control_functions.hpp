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
geometry_msgs::PoseStamped posestamped; // Probably Redundant Master Message
nav_msgs::Odometry pose; //.pose.pose.orientation.xyzw and pose.pose.position.xyz
geometry_msgs::TwistStamped twist_yaw;

float f_heading;
float fpsi = 0;

//Topics
ros::Subscriber state_sub; // Drone 2 State Feedback
ros::Publisher pose_pub;   // Drone 2 State Control Must be from main variable pose_pub.publish(value)
ros::Subscriber pose_sub;  // Must be from main variable
ros::Publisher twist_pub;
//Services
ros::ServiceClient set_mode_client; //Enables String Input for setting GUIDED mode among others
ros::ServiceClient arming_client;   //Enables Built-in Arming of drone
ros::ServiceClient takeoff_client;  //Enables Built-in Takeoff
ros::ServiceClient command_client;  //Primarily for Set Speed Command


/*      Control Functions
*/
void pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  pose = *msg;
  float q0 = pose.pose.pose.orientation.w;
  float q1 = pose.pose.pose.orientation.x;
  float q2 = pose.pose.pose.orientation.y;
  float q3 = pose.pose.pose.orientation.z;
  float fpsi = atan2((2 * (q0 * q3 + q1 * q2)), (1 - 2 * (pow(q2, 2) + pow(q3, 2))));
  // ROS_INFO("Follower Heading %f", fpsi*(180/M_PI));
  f_heading = fpsi;
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  state = *msg;
}

/*      Pathing Functions
*/

struct gnc_WP
{
  float x;   ///< distance in x with respect to your reference frame
  float y;   ///< distance in y with respect to your reference frame
  float z;   ///< distance in z with respect to your reference frame
  float psi; ///< rotation about the third axis of your reference frame
};

struct q_form
{
  float w;
  float x;
  float y;
  float z;
};

std::vector<q_form> q_to_yaw(float yaw) // In radians
{
  std::vector<q_form> angle_in;
  q_form angle;
  //"Float yaw = yaw"
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  angle.w = qw;
  angle.x = qx;
  angle.y = qy;
  angle.z = qz;

  angle_in.push_back(angle);

  return angle_in;
  //usage is q_to_yaw(ANGLE)[0].w,x,y,z etc.
}

float yaw_to_q (std::vector<q_form> q)
{
  float psi = atan2((2*(q[0].w*q[0].z + q[0].x*q[0].y)), (1 - 2*(pow(q[0].y,2) + pow(q[0].z,2))) );
  return psi;
}

/*      Preflight Functions
*/
int wait4connect()
{
  ROS_INFO("Waiting for Follower FCU connection");
  // wait for FCU connection
  while (ros::ok() && !state.connected)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if (state.connected)
  {
    ROS_INFO("Connected to Follower FCU");
    return 0;
  }
  else
  {
    ROS_INFO("Error connecting to Follower drone");
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

  ROS_INFO("Arming Follower drone");
  mavros_msgs::CommandBool arm_request;
  arm_request.request.value = true;
  while (!state.armed && !arm_request.response.success && ros::ok())
  {
    ros::Duration(.1).sleep();
    arming_client.call(arm_request);
    pose_pub.publish(posestamped);
  }
  if (arm_request.response.success)
  {
    ROS_INFO("Follower Arming Successful");
  }
  else
  {
    ROS_INFO("Follower Arming failed with %d", arm_request.response.success);
    return -1;
  }

  //request takeoff
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = takeoff_alt;
  if (takeoff_client.call(srv_takeoff))
  {
    sleep(3);
    ROS_INFO("Follower Takeoff Sent %d", srv_takeoff.response.success);
  }
  else
  {
    ROS_ERROR("Follower Failed Takeoff");
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
    ROS_INFO("Follower SetMode send ok");
  }
  else
  {
    ROS_ERROR("Follower Failed SetMode");
    return -1;
  }
}

/*      Establish Connection
*/

int init_publisher_subscriber(ros::NodeHandle controlnode, std::string ros_namespace)
{
  std::string ros_namespace; // Modify this to cycle for each drone
  // if (!controlnode.hasParam("namespace"))
  // {

  //   ROS_INFO("using default namespace");
  // }
  // else
  // {
  //   controlnode.getParam("namespace", ros_namespace);
  //   ROS_INFO("using namespace %s", ros_namespace.c_str());
  // }

  //Waypoint Position Publishing
  pose_pub = controlnode.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local").c_str(), 10); // For Built in setpoint WP control
  //Velocity Publishing
  twist_pub = controlnode.advertise<geometry_msgs::TwistStamped>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel").c_str(), 10);
  //Position Subscribing
  pose_sub = controlnode.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);
  //State Subscribing
  state_sub = controlnode.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 10, state_cb);
  //Set_Mode Client
  set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
  //Arming Client
  arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
  //Takeoff Client
  takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());

  command_client = controlnode.serviceClient<mavros_msgs::CommandLong>((ros_namespace +"/mavros/cmd/command").c_str());
  // set_speed_client = n.serviceClient<mavros_msgs::ParamSet>("/drone2/mavros/param/set");
  return 0;
}
