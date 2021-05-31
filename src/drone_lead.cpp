// Follower drones. Currently set to P stabilised. Velocity error control relative to lead drone.
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <gnc_functions.hpp>

// #include <geometry_msgs/Point.h>
// ros::Subscriber error_sub;
// geometry_msgs::Point error_point;

// void nav_cb(const geometry_msgs::Pose::ConstPtr &msg)
// {
//   wp_subpose = *msg;
// }

// void error_cb (const geometry_msgs::Point::ConstPtr &msg)
// {
//   error_point = *msg;
//   std::vector<gnc_error> error_in;
//   gnc_error error_group;
//   float sum_error;

//   error_group.x = error_point.x;
//   error_group.y = error_point.y;
//   error_group.z = error_point.z;
//   ros::NodeHandle gnc_node("~");
//   // The idea is to have the vector set to the last 3 values
//   if (error_in.size() < (ros_inumber(gnc_node)-1))
//   {
//     error_in.push_back(error_group);
//   }else{
//     error_in.erase(error_in.begin(), error_in.begin()+1);
//   }

//   //May need to initialise a value??
//   float sum; 
//   float sum1;
//   float sum2;
//   sum = 0;
//   for (int k = 0; k<error_in.size(); k++)
//   {
//     sum = sum + error_in[k].x;
//     sum1 = sum1 + error_in[k].y;
//     sum1 = sum1 + error_in[k].z;
//   }
//   float avg_error = (sum+sum1+sum2)/(error_in.size()*3);
//   ROS_INFO("Average formation of %f",avg_error);
//   //Something to average in x,y,z of error_in vector
//   //Do some kinda of loop to create an average error for both subscribed error messages
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

void forward (float v_des) //Needs to add error from formation to slow speed as required
{
  float vy;
  float vx;
  
  if (d_heading > -M_PI/2 && d_heading < M_PI/2) //from -1.57 to 1.57
    {
      vy = abs(sin(d_heading))*v_des;
      vx = abs(cos(d_heading))*v_des;
      cmd_twist.twist.linear.x = vx;
    }else{
      vy = abs(cos(d_heading-M_PI/2))*v_des;
      vx = abs(sin(d_heading-M_PI/2))*v_des;
      cmd_twist.twist.linear.x = -vx;
    }

    if (d_heading > 0 && d_heading <M_PI)
    {
      cmd_twist.twist.linear.y = vy;
    }else{
      cmd_twist.twist.linear.y = -vy;
    }

  ROS_INFO("vx:%.2f vy:%.2f x:%.2f y:%.2f heading: %.2f",vx,vy,cmd_twist.twist.linear.x,cmd_twist.twist.linear.y, d_heading );
}

void move(float v_des, float lookahead)
{
  float phi_vt = atan2(d_pose.pose.pose.position.y - wp_in[n - 1].y, d_pose.pose.pose.position.x - wp_in[n - 1].x);                     //Angle of car to previous WP (math reference)  
  float phi_path = atan2(wp_in[n].y - wp_in[n - 1].y, wp_in[n].x - wp_in[n - 1].x);                                                     //Angle of path, prev WP to current WP
  float phi_inner = phi_path - phi_vt;
  ROS_INFO("phi_inner : %.4f",phi_inner);

  float prev_wp_dist = sqrt(  abs(pow((d_pose.pose.pose.position.x - wp_in[n - 1].x), 2)) + abs(pow((d_pose.pose.pose.position.y - wp_in[n - 1].y), 2))   ); //C - Distance to Previous waypoint
  ROS_INFO("prev_wp_dist : %.4f   deltax: %.2f  deltay: %.2f",prev_wp_dist,d_pose.pose.pose.position.x , d_pose.pose.pose.position.y);

  float trackwp = cos(phi_inner) * prev_wp_dist;                                                                                      //B - Distance of WP track completed
  ROS_INFO("trackwp : %.4f",trackwp);

  float cte = abs(sin(phi_inner) * prev_wp_dist);                                                                                     //A - Cross track error
  ROS_INFO("cte : %.4f",cte);

  float forwardtrack;                                                                                 //G - Distance of completed track
  if (cte <= lookahead)
  {
    forwardtrack = sqrt( pow(lookahead,2) + pow(cte,2) );
  }else{
    forwardtrack = 0;
  }

   
  float lookpath = trackwp + forwardtrack;                                                                                                   //H - Distance from Previous Waypoint to lookahead waypoint
  ROS_INFO("forwardtrack : %.4f",forwardtrack); 

  float path_angle;
  if (wp_in[n].x < wp_in[n - 1].x)
  {
    path_angle = M_PI + atan2(wp_in[n].y - wp_in[n - 1].y, wp_in[n].x - wp_in[n - 1].x);
  }else{
    path_angle = atan2(wp_in[n].y - wp_in[n - 1].y, wp_in[n].x - wp_in[n - 1].x);
  }
  ROS_INFO("path_angle : %.4f",path_angle);
  //Set lookahead WP coordinates
  float yi = (sin(path_angle) * lookpath) + wp_in[n - 1].y;
  float xi = (cos(path_angle) * lookpath) + wp_in[n - 1].x;
  ROS_INFO("xi : %.2f yi : %.2f",xi,yi);

  float thetai = atan2(yi - d_pose.pose.pose.position.y , xi - d_pose.pose.pose.position.x);    //Angle to lookahead waypoint
  ROS_INFO("Angle to Lookahead: %.3f",thetai);
  float thetae = d_heading - thetai;                                                            //Error between current heading and heading required to intermediate waypoint
  ROS_INFO("Angle error: %.3f",thetae);
  //Do something to v_des to reduce speed if error gets too high by using form_e(error_point)

  cmd_twist.twist.angular.z = d_twist.twist.angular.z + thetae*0.3;

  // cmd_twist.twist.angular.z = 0.2; //positive is left
  forward(v_des);


  cmd_twist.twist.linear.z = d_twist.twist.linear.z + (wp_in[n].z - d_pose.pose.pose.position.z) * 0.5;

  ROS_INFO("Turn Rate: %.2f Velocity x:%f y:%f z:%f",cmd_twist.twist.angular.z, d_twist.twist.linear.x,d_twist.twist.linear.y,d_twist.twist.linear.z);
  // twist_pub.publish(cmd_twist);
}

bool check_waypoint_reached(float pos_tolerance = 0.3)
{
  //check for correct position
  float deltaX = abs(wp_in[n].x - d_pose.pose.pose.position.x);
  float deltaY = abs(wp_in[n].y - d_pose.pose.pose.position.y);
  float deltaZ = 0;
  float dMag = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

  if (dMag < pos_tolerance)
  {
    return 1;
    ROS_INFO("Waypoint Reached, moving to waypoint %i",n);
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
  init_publisher_subscriber(gnc_node);
  
  ROS_INFO("Position (%.2f,%.2f) Heading %.3f",d_pose.pose.pose.position.x, d_pose.pose.pose.position.y, d_heading);
  wp_in = func_wplist();
  ROS_INFO("First WP: %f,%f   Second WP: %f,%f", wp_in[0].x, wp_in[0].y, wp_in[1].x, wp_in[1].y);
  // int wp_size;
  // wp_size = wp_in.size();
  
  // error_sub = gnc_node.subscribe<geometry_msgs::Point>("/gnc/pos_error",10, error_cb);
  // wp_sub = gnc_node.subscribe<geometry_msgs::Pose>("/gnc/goal", 10, nav_wp);
  // bool_pub = gnc_node.advertise<std_msgs::Bool>("/gnc/wpreach", 10);
 

  wait4connect();
  set_mode("GUIDED");
  // takeoff(3);
  // ROS_INFO("Started waypoint navigation with %i waypoints", wp_size);
  ROS_INFO("First Waypoint x:%f y:%f z:%f", wp_in[n].x, wp_in[n].y, wp_in[n].z);
  move(0.5, 0.3);
  ros::Rate loop_rate(1);
  // ros::Duration(4.0).sleep();
  while (ros::ok())
  {
    

    // while (n < wp_size)
    // {
      
    //   if (check_waypoint_reached(0.3))
    //   {
    //     n++;
    //   }
    // }
    // if (n == wp_size)
    // {
    //   land();
    // }
    ros::spinOnce(); //set to spinonce
    loop_rate.sleep();
  }

  return 0;
}
