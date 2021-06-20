/*
  Lead Drone Node. Navigation is based on the leader. Navigation speed is limited by the positional formation error from the follower drones.
  Navigation is set as direct yaw to each waypoint.
*/
#include <gnc_functions.hpp>

#include <geometry_msgs/Point.h>
ros::Subscriber error_sub;
geometry_msgs::Point error_point;

int n = 0; //Waypoint Progression
std::vector<gnc_WP> wp_in;

ros::Publisher wp_pub;
std_msgs::Bool wp_nav;

// ros::Subscriber wplist_sub;
// geometry_msgs::Point wplist;

void error_cb (const geometry_msgs::Point::ConstPtr &msg)
{
  error_point = *msg;
}

// void wp_cb (const geometry_msgs::Point::ConstPtr &msg)
// {
//   wplist = *msg;
//   gnc_WP wp;
//   wp.x = wplist.x; //0
//   wp.y = wplist.y;
//   wp.z = wplist.z;
//   wp_in.push_back(wp);
//   int size = wp_in.size();
//   ROS_INFO("received waypoint %i at %f,%f",size,wp.x,wp.y);
// }

std::vector<gnc_WP> func_wplist() //Create a separate function for a list of waypoints
{
  gnc_WP wp_list;
  wp_list.x = 0; //0
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
}

void move(float v_des)
{
  float des_angle;
  if (wp_in[n].y > d_pose.pose.pose.position.y)
  {
    if (wp_in[n].x > d_pose.pose.pose.position.x)
    {
      des_angle = atan2(abs(d_pose.pose.pose.position.y - wp_in[n].y), abs(d_pose.pose.pose.position.x - wp_in[n].x));
    }else{
      des_angle = atan2(abs(d_pose.pose.pose.position.x - wp_in[n].x), abs(d_pose.pose.pose.position.y - wp_in[n].y)) +M_PI/2;
    }
  }else{
    if (wp_in[n].x > d_pose.pose.pose.position.x)
    {
      des_angle = -(atan2(abs(d_pose.pose.pose.position.y - wp_in[n].y), abs(d_pose.pose.pose.position.x - wp_in[n].x)));
    }else{
      des_angle = -(atan2(abs(d_pose.pose.pose.position.x - wp_in[n].x), abs(d_pose.pose.pose.position.y - wp_in[n].y)) +M_PI/2);
    }
  }

  float error = -(d_heading - des_angle);
  // ROS_INFO("Desired angle: %.2f",des_angle);
  float angle_e = ::atan2(::sin(error),::cos(error));
  //Do something to v_des to reduce speed if error gets too high by using form_e(error_point)
  cmd_twist.twist.angular.z = angle_e*0.7;                                      //Yaw Control positive is left
  forward(v_des);                                                               //Move forward function in body frame
  cmd_twist.twist.linear.z = (wp_in[n].z - d_pose.pose.pose.position.z) * 0.2;  //Vertical control
  twist_pub.publish(cmd_twist);
}

bool check_waypoint_reached(float pos_tolerance = 0.3)
{
  //check for correct position only in x and y (may add z)
  float deltaX = abs(wp_in[n].x - d_pose.pose.pose.position.x);
  float deltaY = abs(wp_in[n].y - d_pose.pose.pose.position.y);
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
  // wplist_sub = gnc_node.subscribe<geometry_msgs::Point>("/gnc/goal", 10, wp_cb);
  error_sub = gnc_node.subscribe<geometry_msgs::Point>("/gnc/pos_error",10, error_cb);
  wp_pub = gnc_node.advertise<std_msgs::Bool>("/gnc/nav", 10);
  // ros::Duration(10.0).sleep(); //May be required to ensure subscriber and publisher is ready
  
  /* Initiate Waypoint List
  */
  wp_in = func_wplist();
  int wp_size = wp_in.size();

  /* Drone Startup Procedure
  */
  wait4connect();
  set_mode("GUIDED");  ros::Duration(4.0).sleep();
  takeoff(3);          ros::Duration(5.0).sleep(); //Make this extra long just for follower to move to position
  wp_nav.data = 1; //This may be able to be used as a topic like nav_complete.publish(navigate) where std_msgs::Bool navigate as a global.
  wp_pub.publish(wp_nav);

  /*Begin Control and Navigation
  */
  ROS_INFO("%i waypoints, starting with Waypoint %i (x:%f y:%f z:%f)", wp_size, n,wp_in[n].x, wp_in[n].y, wp_in[n].z);
  

  ros::Rate loop_rate(3);
  while (ros::ok() && wp_nav.data)
  {
    if (n < wp_size)
    {
      move(0.3); //Desired speed
      if (check_waypoint_reached(0.3))
      {
        ROS_INFO("Waypoint Reached, moving to waypoint %i",n);
        n++;
      }
    }else{
      land();
      ros::Duration(3.0).sleep();
      wp_nav.data = 0;
      wp_pub.publish(wp_nav);
      ROS_INFO("All waypoints reached, shutting down...");
    }
    ros::spinOnce(); //set to spinonce
    loop_rate.sleep();
  }
  return 0;
}
