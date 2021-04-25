#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>

//Topics
//Drone 2 State
mavros_msgs::State ls_state;
//Drone 2 Position Publishing
geometry_msgs::PoseStamped lp_pose;        // Probably Redundant Master Message
// geometry_msgs::TwistStamped lp_twist;
//Drone 2 Position Subscribing
nav_msgs::Odometry ls_pose;                //.pose.pose.orientation.xyzw and pose.pose.position.xyz

//Nodes
// ros::Publisher twist_pub;                //Twist Stamped Drone Velocity Feedback
ros::Subscriber state_sub;                // Drone 2 State Feedback
ros::Publisher pose_pub;                  // Drone 2 State Control Must be from main variable pose_pub.publish(value)
ros::Subscriber pose_sub;                  // Must be from main variable
//Drone 2 Service Clients
ros::ServiceClient set_mode_client;         //Enables String Input for setting GUIDED mode among others
ros::ServiceClient arming_client;           //Enables Built-in Arming of drone
ros::ServiceClient takeoff_client;          //Enables Built-in Takeoff
ros::ServiceClient wp_nav_param;        //Set maximum horizontal speed

float l_heading;
float lpsi = 0;
//Testing time boys
//mavros_msgs::SetMode srv_cmdlong;



/*      Control Functions
*/
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  ls_pose = *msg;
  float q0 = ls_pose.pose.pose.orientation.w;
  float q1 = ls_pose.pose.pose.orientation.x;
  float q2 = ls_pose.pose.pose.orientation.y;
  float q3 = ls_pose.pose.pose.orientation.z;
  float lpsi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  // ROS_INFO("Leader Heading %f", lpsi*(180/M_PI));
  l_heading = lpsi; //in radians
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  ls_state = *msg;
}

void set_heading(float heading) //quaternion Conversion For Yaw Rotations Input in degrees
{
  //ROS_INFO("Desired Heading %f ", local_desired_heading_g);
  float yaw = (heading)*(M_PI/180); //Convert input desired heading to radians
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

  lp_pose.pose.orientation.w = qw;
  lp_pose.pose.orientation.x = qx;
  lp_pose.pose.orientation.y = qy;
  lp_pose.pose.orientation.z = qz;
}

// void set_turn(float turn)
// {
//   // lp_twist.twist.angular.x = turn;
//   // lp_twist.twist.angular.y = turn;
//   lp_twist.twist.angular.z = turn;
//   twist_pub.publish(lp_twist);
// }
void set_destination(float x, float y, float z, float psi)
{
  set_heading(psi);
  float deg2rad = (M_PI/180);
  lp_pose.pose.position.x = x;
  lp_pose.pose.position.y = y;
  lp_pose.pose.position.z = z;

  pose_pub.publish(lp_pose);
}

/*      Preflight Functions
*/
int wait4connect()
{
  ROS_INFO("Waiting for Leader FCU connection");
  // wait for FCU connection
  while (ros::ok() && !ls_state.connected)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if(ls_state.connected)
  {
    ROS_INFO("Connected to Leader FCU");
    return 0;
  }
  else
  {
    ROS_INFO("Error connecting to Leader drone");
    return -1;
  }

}

int takeoff(float takeoff_alt)
{
  ROS_INFO("Arming Leader drone");
  mavros_msgs::CommandBool arm_request;
  arm_request.request.value = true;
  while (!ls_state.armed && !arm_request.response.success && ros::ok())
  {
    ros::Duration(.1).sleep();
    arming_client.call(arm_request);
    // pose_pub.publish(lp_pose);
  }
  if(arm_request.response.success)
  {
    ROS_INFO("Leader Arming Successful");
  }else{
    ROS_INFO("Leader Arming failed with %d", arm_request.response.success);
    return -1;
  }

  //request takeoff
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = takeoff_alt;
  if(takeoff_client.call(srv_takeoff)){
    sleep(3);
    ROS_INFO("Leader Takeoff Sent %d", srv_takeoff.response.success);
  }else{
    ROS_ERROR("Leader Failed Takeoff");
    return -2;
  }
  sleep(2);
  return 0;
}

int set_speed(int speed)
{
  mavros_msgs::ParamSet srv_setspeed;
  srv_setspeed.request.param_id = "WPNAV_SPEED";
  srv_setspeed.request.value.integer = speed;
  if(wp_nav_param.call(srv_setspeed))
  {
    ROS_INFO("Leader Speed Set");
  }
  else
  ROS_ERROR("Leader failed to set custom speed");
  return -1;
}
int set_speed_up(int speed)
{
  mavros_msgs::ParamSet srv_setspeed;
  srv_setspeed.request.param_id = "WPNAV_SPEED_UP";
  srv_setspeed.request.value.integer = speed;
  if(wp_nav_param.call(srv_setspeed))
  {
    ROS_INFO("Leader Up Speed Set");
  }
  else
  ROS_ERROR("Leader failed to set custom up speed");
  return -1;
}
int set_speed_dn(int speed)
{
  mavros_msgs::ParamSet srv_setspeed;
  srv_setspeed.request.param_id = "WPNAV_SPEED_DN";
  srv_setspeed.request.value.integer = speed;
  if(wp_nav_param.call(srv_setspeed))
  {
    ROS_INFO("Leader Down Speed Set");
  }
  else
  ROS_ERROR("Leader failed to set custom down speed");
  return -1;
}


int set_mode(std::string mode)
{
  mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = mode;
    if(set_mode_client.call(srv_setMode)){
      ROS_INFO("Leader SetMode send ok");
    }
    else
    {
      ROS_ERROR("Leader Failed SetMode");
      return -1;
    }
}



/*      End Preflight Functions
*/
int main(int argc, char **argv)
{
    ros::init (argc,argv,"gnc_node");
    ros::NodeHandle n;

    //Drone 1 Position Publishing
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/drone1/mavros/setpoint_position/local", 10);
    //Drone 1 Position Subscribing
    pose_sub = n.subscribe<nav_msgs::Odometry>("/drone1/mavros/global_position/local", 10,pose_cb);
    //Drone 1 State Subscribing
    state_sub = n.subscribe<mavros_msgs::State>("/drone1/mavros/state",10,state_cb);
    //Drone 1 Set_Mode Client
    set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/drone1/mavros/set_mode");
    //Drone 1 Arming Client
    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/drone1/mavros/cmd/arming");
    //Drone 1 Takeoff Client
    takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/drone1/mavros/cmd/takeoff");

    // set_speed_client = n.serviceClient<mavros_msgs::CommandLong>("/drone1/mavros/cmd/command");
    wp_nav_param = n.serviceClient<mavros_msgs::ParamSet>("/drone1/mavros/param/set");
    // twist_pub = n.advertise<geometry_msgs::TwistStamped>("/drone1/mavros/setpoint_attitude/cmd_vel",10);
    //default down speed of 150
    //default up speed of 250

    wait4connect();
    set_mode("GUIDED");
    takeoff(2);
    set_speed(40);
    set_speed_up(50);
    set_speed_dn(50);

    ros::Rate loop_rate(3);
    while (ros::ok())
    {
      // set_turn(0.5);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
