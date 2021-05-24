// Follower drones. Currently set to P stabilised. Velocity error control relative to lead drone.
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <gnc_functions.hpp>

// void nav_cb(const geometry_msgs::Pose::ConstPtr &msg)
// {
//   wp_subpose = *msg;
// }

int n = 1;
std::vector<gnc_WP> wp_in;

std::vector<gnc_WP> func_wplist() //Create a separate function for a list of waypoints
{
  // std::vector<gnc_WP> wp_in;
  gnc_WP wp_list;
  wp_list.x = 0; //update this to 0,1
  wp_list.y = 0;
  wp_list.z = 3;
  wp_in.push_back(wp_list);
  wp_list.x = 3; //1
  wp_list.y = 1;
  wp_list.z = 3;
  wp_in.push_back(wp_list);
  wp_list.x = 9; //2
  wp_list.y = 3;
  wp_list.z = 3;
  wp_in.push_back(wp_list);
  wp_list.x = 9; //3
  wp_list.y = 5;
  wp_list.z = 3;
  wp_in.push_back(wp_list);
  wp_list.x = 9; //4
  wp_list.y = 7;
  wp_list.z = 3;
  wp_in.push_back(wp_list);
  wp_list.x = 5; //5
  wp_list.y = 9;
  wp_list.z = 3;
  wp_in.push_back(wp_list);
  return wp_in;
}

void move(float v_des, float lookahead) // std::vector<gnc_WP> wp_in
{
  float phi_path = atan2(wp_in[n].y - wp_in[n - 1].y, wp_in[n].x - wp_in[n - 1].x); //Angle of path, prev WP to current WP
  ROS_INFO("phi_path is %f", phi_path);
  float phi_vt = atan2(d_pose.pose.pose.position.y = wp_in[n - 1].y, d_pose.pose.pose.position.x = wp_in[n - 1].x); //Angle of car to previous WP (math reference)

  float prevwp = sqrt(pow((d_pose.pose.pose.position.x - wp_in[n - 1].x), 2) + pow((d_pose.pose.pose.position.y - wp_in[n - 1].y), 2)); //Distance to Previous waypoint
  float trackwp = cos(phi_path - phi_vt) * prevwp;                                                                                      //Distance of WP track completed
  float lookdist = abs(sin(phi_path - phi_vt) * prevwp);                                                                                //Cross track error
  float lookwp = sqrt(pow(lookahead, 2) + pow(lookdist, 2));                                                                            //G
  float lookpath = trackwp + lookwp;

  //Set lookahead WP coordinates
  float yi = (sin(phi_path) * lookpath) + wp_in[n - 1].y;
  float xi = (cos(phi_path) * lookpath) + wp_in[n - 1].x;

  // float angle_to_wp = atan2(wp_in[n].y-d_pose.pose.position.y, wp_in[n].x-d_pose.pose.position.x);
  // float heading_error = angle_to_wp - psi;

  float turn_rate = atan2(yi - d_pose.pose.pose.position.y, xi - d_pose.pose.pose.position.x); //point at lookahead WP. Add gain here, check signs.

  cmd_twist.twist.angular.z = turn_rate;
  // cmd_twist.twist.angular.z = 0.2; //positive is left
  cmd_twist.twist.linear.x = cos(psi) * v_des; //X Axis control based on heading (To simulate moving forward)
  cmd_twist.twist.linear.y = sin(psi) * v_des; //Y Axis control based on heading (To simulate moving forward)

  cmd_twist.twist.linear.z = wp_in[n].z - d_pose.pose.pose.position.z * 0.2; //Axis control Needs PID Z

  ROS_INFO("V Input x:%f y:%f z:%f yaw:%f", cmd_twist.twist.linear.x, cmd_twist.twist.linear.y, cmd_twist.twist.linear.z, cmd_twist.twist.angular.z);
  twist_pub.publish(cmd_twist);
}

bool check_waypoint_reached(float pos_tolerance = 0.3) //const std::vector<gnc_WP> wp_in = {}
{
  //check for correct position
  float deltaX = abs(wp_in[n].x - d_pose.pose.pose.position.x);
  float deltaY = abs(wp_in[n].y - d_pose.pose.pose.position.y);
  float deltaZ = 0;
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

  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node("~");
  wp_in = func_wplist();
  ROS_INFO("First WP: %f,%f   Second WP: %f,%f", wp_in[0].x, wp_in[0].y, wp_in[1].x, wp_in[1].y);
  int wp_size;

  init_publisher_subscriber(gnc_node);
  // wp_sub = gnc_node.subscribe<geometry_msgs::Pose>("/gnc/goal", 10, nav_wp);
  // bool_pub = gnc_node.advertise<std_msgs::Bool>("/gnc/wpreach", 10);
  wp_size = wp_in.size();

  wait4connect();
  set_mode("GUIDED");
  takeoff(3);
  ROS_INFO("Started waypoint navigation with %i waypoints", wp_size);
  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    ROS_INFO("First Waypoint x:%f y:%f z:%f", wp_in[n].x, wp_in[n].y, wp_in[n].z);

    while (n < wp_size)
    {
      move(0.3, 0.5);
      if (check_waypoint_reached(0.3))
      {
        n++;
      }
    }
    if (n == wp_size)
    {
      land();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
