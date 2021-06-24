/*
  Lead Drone Node. Navigation is based on the leader. Navigation speed is limited by the positional formation error from the follower drones.
  Navigation is set as direct yaw to each waypoint.
*/
#include <gnc_functions.hpp>
#include <geometry_msgs/PoseArray.h>

ros::Subscriber error_sub;
geometry_msgs::Point error_point;
float avg_error = 0;

ros::Publisher wp_pub;
std_msgs::Bool wp_nav;

ros::Subscriber wp_sub;
geometry_msgs::PoseArray wp_subpose;    //Original Waypoint Array
int n = 0; //Waypoint Progression

void error_cb (const geometry_msgs::Point::ConstPtr &msg)
{
  error_point = *msg;
  float upper_limit = 1;                            //Need to add this as parameter
  avg_error = (error_point.x + error_point.y)/2;    //Ignore vertical z error for now
  if (avg_error > upper_limit)
  {
    avg_error = upper_limit;
  }
  // ROS_INFO("Average position error %.2f",avg_error);
}

void nav_wp(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  wp_subpose = *msg;
  int size = wp_subpose.poses.size();
  ROS_INFO("Received %i Waypoints",size);
}

void forward (float vel_desired)
{
  float vy;
  float vx;
  float v_actual = vel_desired*(1 - avg_error*0.8);

  if (d_heading > -M_PI/2 && d_heading < M_PI/2) //from -1.57 to 1.57
    {
      vy = abs(sin(d_heading))*v_actual;
      vx = abs(cos(d_heading))*v_actual;
      cmd_twist.twist.linear.x = vx;
    }else{
      vy = abs(cos(d_heading-M_PI/2))*v_actual;
      vx = abs(sin(d_heading-M_PI/2))*v_actual;
      cmd_twist.twist.linear.x = -vx;
    }

    if (d_heading > 0 && d_heading <M_PI)
    {
      cmd_twist.twist.linear.y = vy;
    }else{
      cmd_twist.twist.linear.y = -vy;
    }
}

void move(float vel_desired)
{
  float des_angle;
  if (wp_subpose.poses[n].position.y > d_pose.pose.pose.position.y)
  {
    if (wp_subpose.poses[n].position.x > d_pose.pose.pose.position.x)
    {
      des_angle = atan2(abs(d_pose.pose.pose.position.y - wp_subpose.poses[n].position.y), abs(d_pose.pose.pose.position.x - wp_subpose.poses[n].position.x));
    }else{
      des_angle = atan2(abs(d_pose.pose.pose.position.x - wp_subpose.poses[n].position.x), abs(d_pose.pose.pose.position.y - wp_subpose.poses[n].position.y)) +M_PI/2;
    }
  }else{
    if (wp_subpose.poses[n].position.x > d_pose.pose.pose.position.x)
    {
      des_angle = -(atan2(abs(d_pose.pose.pose.position.y - wp_subpose.poses[n].position.y), abs(d_pose.pose.pose.position.x - wp_subpose.poses[n].position.x)));
    }else{
      des_angle = -(atan2(abs(d_pose.pose.pose.position.x - wp_subpose.poses[n].position.x), abs(d_pose.pose.pose.position.y - wp_subpose.poses[n].position.y)) +M_PI/2);
    }
  }

  float error = -(d_heading - des_angle);
  // ROS_INFO("Desired angle: %.2f",des_angle);
  float angle_e = ::atan2(::sin(error),::cos(error));

  forward(vel_desired);                                                   //Move forward function in body frame
  float yaw_limit = 0.3;                                                  //Need to add this as parameter
  cmd_twist.twist.angular.z = angle_e * 0.7 * (1 - (avg_error*0.8));          //Yaw Control positive is left

  if (angle_e*0.7*(1 - (avg_error*0.8)) > yaw_limit)
  {
    cmd_twist.twist.angular.z = yaw_limit;
  }else if ((angle_e*0.7*(1 - (avg_error*0.8))) < -yaw_limit)
  {
    cmd_twist.twist.angular.z = -yaw_limit;
  }

  cmd_twist.twist.linear.z = (wp_subpose.poses[n].position.z - d_pose.pose.pose.position.z) * 0.2;   //Vertical control
  twist_pub.publish(cmd_twist);
  // ROS_INFO("Yaw Command: %.2f",cmd_twist.twist.angular.z);
}

bool check_waypoint_reached(float pos_tolerance = 0.3)
{
  float deltaX = abs(wp_subpose.poses[n].position.x - d_pose.pose.pose.position.x);
  float deltaY = abs(wp_subpose.poses[n].position.y - d_pose.pose.pose.position.y);
  float deltaZ = 0; //abs(wp_in[n].z - d_pose.pose.pose.position.z);
  float dMag = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

  if (dMag < pos_tolerance)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


int main(int argc, char **argv)
{
  /* Establish Publisher and Subscribers
  */
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node("~");
  init_publisher_subscriber(gnc_node);
  wp_sub = gnc_node.subscribe<geometry_msgs::PoseArray>("/gnc/goal", 10, nav_wp);
  ROS_INFO("Subscribed to goal");
  error_sub = gnc_node.subscribe<geometry_msgs::Point>("/gnc/pos_error",10, error_cb);
  ROS_INFO("Subscribed to error");
  wp_pub = gnc_node.advertise<std_msgs::Bool>("/gnc/nav", 10);
  ROS_INFO("Subscribed to nav");
  // ros::Duration(10.0).sleep(); //May be required to ensure subscriber and publisher is ready
  
  /* Drone Startup Procedure
  */
  wait4connect();
  set_mode("GUIDED");  ros::Duration(4.0).sleep();
  takeoff(3);          ros::Duration(5.0).sleep();
  wp_nav.data = 1;
  wp_pub.publish(wp_nav);

  /*Begin Control and Navigation
  */
  int wp_size = wp_subpose.poses.size();
  ROS_INFO("Beginning Navigation to Waypoint %i (%.1f,%.1f,%.1f)",n,wp_subpose.poses[n].position.x,wp_subpose.poses[n].position.y,wp_subpose.poses[n].position.z);
  ros::Rate loop_rate(3);
  while (ros::ok() && wp_nav.data)
  {
    if (n < wp_size)
    {
      move(1); //Desired speed
      if (check_waypoint_reached(0.3))
      {
        n++;
        ROS_INFO("Waypoint Reached, moving to Waypoint %i (%.1f,%.1f,%.1f)",n,wp_subpose.poses[n].position.x,wp_subpose.poses[n].position.y,wp_subpose.poses[n].position.z);
      }
    }else{
      land();
      ros::Duration(3.0).sleep();
      wp_nav.data = 0;
      wp_pub.publish(wp_nav);
      ROS_INFO("All waypoints reached, shutting down...");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
