#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
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
mavros_msgs::State fs_state;
//Drone 2 Position Publishing
geometry_msgs::PoseStamped fp_pose;        // Probably Redundant Master Message
// geometry_msgs::Quaternion fp_quater;       // Follower Drone Orientation as quaternion float x,y,z,w
// geometry_msgs::Point fp_point;             // Follower Drone Position as cartesian float x,y,z
// geometry_msgs::Pose fp_correction;
//Drone 2 Position Subscribing
nav_msgs::Odometry fs_pose;                //.pose.pose.orientation.xyzw and pose.pose.position.xyz
                                           //.twist.twist.linear.xyz and twist.twist.angular.xyz
float f_heading;
float fpsi = 0;
//Drone 1 Position Subscribing
ros::Subscriber lead_pose_sub;
nav_msgs::Odometry ls_pose;
float l_heading;
float lpsi = 0;

//Nodes
ros::Subscriber state_sub;                // Drone 2 State Feedback
ros::Publisher pose_pub;                  // Drone 2 State Control Must be from main variable pose_pub.publish(value)
ros::Subscriber pose_sub;                  // Must be from main variable
//Drone 2 Service Clients
ros::ServiceClient set_mode_client;         //Enables String Input for setting GUIDED mode among others
ros::ServiceClient arming_client;           //Enables Built-in Arming of drone
ros::ServiceClient takeoff_client;          //Enables Built-in Takeoff

//Testing time boys
//mavros_msgs::SetMode srv_cmdlong;
ros::ServiceClient set_speed_client;



/*      Control Functions
*/
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  fs_pose = *msg;
  float q0 = fs_pose.pose.pose.orientation.w;
  float q1 = fs_pose.pose.pose.orientation.x;
  float q2 = fs_pose.pose.pose.orientation.y;
  float q3 = fs_pose.pose.pose.orientation.z;
  float fpsi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  // ROS_INFO("Follower Heading %f", fpsi*(180/M_PI));
  f_heading = fpsi;
}

void lead_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  ls_pose = *msg;
  float q0 = ls_pose.pose.pose.orientation.w;
  float q1 = ls_pose.pose.pose.orientation.x;
  float q2 = ls_pose.pose.pose.orientation.y;
  float q3 = ls_pose.pose.pose.orientation.z;
  float lpsi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  // ROS_INFO("Lead Heading %f", lpsi*(180/M_PI));
  l_heading = lpsi;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  fs_state = *msg;
}

void set_heading(float heading) //quaternion Conversion For Yaw Rotations
{
  //ROS_INFO("Desired Heading %f ", local_desired_heading_g);
  float yaw = heading;
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

  fp_pose.pose.orientation.w = qw;
  fp_pose.pose.orientation.x = qx;
  fp_pose.pose.orientation.y = qy;
  fp_pose.pose.orientation.z = qz;
}

void set_destination(float x, float y, float z, float psi)
{
  set_heading(psi);
  fp_pose.pose.position.x = x;
  fp_pose.pose.position.y = y+5;
  fp_pose.pose.position.z = z;

  pose_pub.publish(fp_pose);
}

void form_position(float xoff, float zoff) // Left aligned formation
{
  float xf;
  float yf;
  float zf;
  float x;
  float y;
  float z;
  // float deg2rad = (M_PI/180);
  if (l_heading > -(M_PI/2) && l_heading < (M_PI/2))
  {
    x = sin(abs( l_heading ))*xoff;
    y = cos(abs( l_heading ))*xoff;
  }
  else
  {
    x = cos( (abs(l_heading)-(M_PI/2)) )*xoff;
    y = sin( (abs(l_heading)-(M_PI/2)) )*xoff;
  }

  if (l_heading > 0 && l_heading < M_PI)
  {
    xf = ls_pose.pose.pose.position.x-x;
  }
  else
  {
    xf = ls_pose.pose.pose.position.x+x;
  }


  //For y-axis sign, where first and fourth quadrant is negative
  if (l_heading > -(M_PI/2) && l_heading < (M_PI/2))
  {
    yf = ls_pose.pose.pose.position.y+y;
  }
  else
  {
    yf = ls_pose.pose.pose.position.y-y;
  }

  zf = ls_pose.pose.pose.position.z+zoff; //Match Follower altitude to lead altitude
  //Run Movement API
  // ROS_INFO("Xoff %f", x);
  // ROS_INFO("Yoff %f", y);
  set_destination(xf,yf,zf,l_heading); //Altitude Matching currently disables
}

/*      Preflight Functions
*/
int wait4connect()
{
  ROS_INFO("Waiting for Follower FCU connection");
  // wait for FCU connection
  while (ros::ok() && !fs_state.connected)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if(fs_state.connected)
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

int takeoff(float takeoff_alt)
{

  ROS_INFO("Arming Follower drone");
  mavros_msgs::CommandBool arm_request;
  arm_request.request.value = true;
  while (!fs_state.armed && !arm_request.response.success && ros::ok())
  {
    ros::Duration(.1).sleep();
    arming_client.call(arm_request);
    // pose_pub.publish(fp_pose);
  }
  if(arm_request.response.success)
  {
    ROS_INFO("Follower Arming Successful");
  }else{
    ROS_INFO("Follower Arming failed with %d", arm_request.response.success);
    return -1;
  }

  //request takeoff
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = takeoff_alt;
  if(takeoff_client.call(srv_takeoff)){
    sleep(3);
    ROS_INFO("Follower Takeoff Sent %d", srv_takeoff.response.success);
  }else{
    ROS_ERROR("Follower Failed Takeoff");
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
  if(set_speed_client.call(srv_setspeed))
  {
    ROS_INFO("Follower Speed Set");
  }
  else
  ROS_ERROR("Follower Failed to set speed");
  return -1;
}

int set_mode(std::string mode)
{
  mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = mode;
    if(set_mode_client.call(srv_setMode)){
      ROS_INFO("Follower SetMode send ok");
    }
    else
    {
      ROS_ERROR("Follower Failed SetMode");
      return -1;
    }
}

/*      End Preflight Functions
*/
int main(int argc, char **argv)
{
    ros::init (argc,argv,"gnc_node");
    ros::NodeHandle n;
    //Drone 1 Position Subscribing
    lead_pose_sub = n.subscribe<nav_msgs::Odometry>("/drone1/mavros/global_position/local", 10,lead_pose_cb);

    //Drone 2 Position Publishing
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/drone2/mavros/setpoint_position/local", 10);
    //Drone 2 Position Subscribing
    pose_sub = n.subscribe<nav_msgs::Odometry>("/drone2/mavros/global_position/local", 10,pose_cb);
    //Drone 2 State Subscribing
    state_sub = n.subscribe<mavros_msgs::State>("/drone2/mavros/state",10,state_cb);
    //Drone 2 Set_Mode Client
    set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/drone2/mavros/set_mode");
    //Drone 2 Arming Client
    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/drone2/mavros/cmd/arming");
    //Drone 2 Takeoff Client
    takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/drone2/mavros/cmd/takeoff");

    // set_speed_client = n.serviceClient<mavros_msgs::CommandLong>("/drone1/mavros/cmd/command");
    set_speed_client = n.serviceClient<mavros_msgs::ParamSet>("/drone2/mavros/param/set");


    wait4connect();
    set_mode("GUIDED");
    takeoff(3);
    set_speed(1000);

    ros::Rate loop_rate(2);
    while (ros::ok())
    {
      form_position(2,0);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
