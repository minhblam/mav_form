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

// Trajectory Manager

// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.

//Topics
//Drone 2 State
mavros_msgs::State fs_state;
//Drone 2 Position Publishing
geometry_msgs::PoseStamped fp_pose;        // Probably Redundant Master Message
//Drone 2 Position Subscribing
nav_msgs::Odometry fs_pose;                //.pose.pose.orientation.xyzw and pose.pose.position.xyz

float f_heading;
float fpsi = 0;

//Nodes
ros::Subscriber state_sub;                // Drone 2 State Feedback
ros::Publisher pose_pub;                  // Drone 2 State Control Must be from main variable pose_pub.publish(value)
ros::Subscriber pose_sub;                  // Must be from main variable
//Drone 2 Service Clients
ros::ServiceClient set_mode_client;         //Enables String Input for setting GUIDED mode among others
ros::ServiceClient arming_client;           //Enables Built-in Arming of drone
ros::ServiceClient takeoff_client;          //Enables Built-in Takeoff



// NEW FUNCTIONS HERE
// Forwards
mavros_msgs::PositionTarget fp_for;
ros::Publisher twist_pub;
// Yaw
geometry_msgs::TwistStamped fp_yaw;
ros::Publisher tstamp_pub;

// void forward( float speed )
// {
//   fp_for.coordinate_frame = 8;
//   fp_for.velocity.y = speed; //y = forward, x = to the right
//   twist_pub.publish(fp_for);

// }

void turn() //float x, float y
{
  // float yaw_rate = atan2(y-fs_pose.pose.pose.position.y , x-fs_pose.pose.pose.position.x) * 2;
  fp_yaw.twist.angular.z = 1; // Inde grees?
  fp_yaw.twist.linear.x = 10;
  tstamp_pub.publish(fp_yaw);
}




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

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  fs_state = *msg;
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

// float d_to_WP (float x, float y)
// {
//   float dist = sqrt(pow(fs_pose.pose.pose.position.x - x,2) + pow(fs_pose.pose.pose.position.y - y,2));
//   return dist;
//   ROS_INFO("Distance to WP %f", dist);
// }

struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	// float psi; ///< rotation about the third axis of your reference frame
};

/*      End Preflight Functions
*/
int main(int argc, char **argv)
{
  
  std::vector<gnc_api_waypoint> waypointList;
  gnc_api_waypoint nextWayPoint;
  nextWayPoint.x = 0;
  nextWayPoint.y = 0;
  // nextWayPoint.z = 3;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 10;
  nextWayPoint.y = 0;
  // nextWayPoint.z = 3;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 17;
  nextWayPoint.y = 5;
  // nextWayPoint.z = 3;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 6;
  nextWayPoint.y = 5;
  // nextWayPoint.z = 3;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 20;
  nextWayPoint.y = 0;
  // nextWayPoint.z = 3;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 0;
  nextWayPoint.y = 0;
  // nextWayPoint.z = 3;
  waypointList.push_back(nextWayPoint);

  ros::init (argc,argv,"gnc_node");
  ros::NodeHandle n;
  //Drone 2 Position Publishing
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/drone1/mavros/setpoint_position/local", 10); // For Built in setpoint WP control

  //Drone 2 Velocity Publishing
  // tstamp_pub = n.advertise<geometry_msgs::TwistStamped>("/drone1/mavros/setpoint_attitude/cmd_vel", 10); //might be setpoint_velocity/cmd_vel
  tstamp_pub = n.advertise<geometry_msgs::TwistStamped>("/drone1/mavros/setpoint_velocity/cmd_vel", 10);
  twist_pub = n.advertise<mavros_msgs::PositionTarget>("drone1/mavros/setpoint_raw/local",10);

  //Drone 2 Position Subscribing
  pose_sub = n.subscribe<nav_msgs::Odometry>("/drone1/mavros/global_position/local", 10,pose_cb);
  //Drone 2 State Subscribing
  state_sub = n.subscribe<mavros_msgs::State>("/drone1/mavros/state",10,state_cb);
  //Drone 2 Set_Mode Client
  set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/drone1/mavros/set_mode");
  //Drone 2 Arming Client
  arming_client = n.serviceClient<mavros_msgs::CommandBool>("/drone1/mavros/cmd/arming");
  //Drone 2 Takeoff Client
  takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/drone1/mavros/cmd/takeoff");

  // set_speed_client = n.serviceClient<mavros_msgs::CommandLong>("/drone1/mavros/cmd/command");
  // set_speed_client = n.serviceClient<mavros_msgs::ParamSet>("/drone2/mavros/param/set");



  wait4connect();
  set_mode("GUIDED");
  takeoff(5);
  // forward(20);
  turn();
  

  int counter = 0;
  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    // turn(waypointList[counter].x,waypointList[counter].y);
    ros::spinOnce();
    loop_rate.sleep();
    // if (d_to_WP(nextWayPoint.x, nextWayPoint.y) < 0.3)
    // {
    //   forward(100);
    //   if (counter < waypointList.size())
    //   {
    //     counter++;
    //   }
    //   else
    //   {
    //     forward(10);
    //   }
    // }

  }
  return 0;
}
