// Follower drones. Currently set to P stabilised. Velocity error control relative to lead drone.
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <gnc_functions.hpp>

#include <geometry_msgs/Point.h>
ros::Subscriber error_sub;
geometry_msgs::Point error_point;

// void nav_cb(const geometry_msgs::Pose::ConstPtr &msg)
// {
//   wp_subpose = *msg;
// }

void error_cb (const geometry_msgs::Point::ConstPtr &msg)
{
  error_point = *msg;
  std::vector<gnc_error> error_in;
  gnc_error error_group;
  float sum_error;

  error_group.x = error_point.x;
  error_group.y = error_point.y;
  error_group.z = error_point.z;
  ros::NodeHandle gnc_node("~");
  // The idea is to have the vector set to the last 3 values
  if (error_in.size() < (ros_inumber(gnc_node)-1))
  {
    error_in.push_back(error_group);
  }else{
    error_in.erase(error_in.begin(), error_in.begin()+1);
  }
  // float sum = 0;
  // for (int i = 0; i < (ros_inumber(gnc_node)-1);i++)
  // {
  //   sum = sum + error_in[i];
  // }
  // float avg_error = sum/error_in.size();
  // ROS_INFO("Sum error = %.2f",avg_error);
}

int n = 1; //Waypoint Progression
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
  //Drift may be partially compensated for by multiplying d_heading value???

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
  ROS_INFO("Desired angle: %.2f",des_angle);
  float angle_e = ::atan2(::sin(error),::cos(error));
  //Do something to v_des to reduce speed if error gets too high by using form_e(error_point)

  cmd_twist.twist.angular.z = angle_e*0.7;                                      //Yaw Control positive is left
  forward(v_des);                                                               //Move forward in body frame
  cmd_twist.twist.linear.z = (wp_in[n].z - d_pose.pose.pose.position.z) * 0.2;  //Vertical control
  // ROS_INFO("Turn:%.2f    Vx:%f    Vy:%f    Vz:%f",cmd_twist.twist.angular.z, d_twist.twist.linear.x,d_twist.twist.linear.y,d_twist.twist.linear.z);
  twist_pub.publish(cmd_twist);
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
    // ROS_INFO("Waypoint Reached, moving to waypoint %i",n);
  }
  else
  {
    return 0;
  }
}

void pos_print()
{
  float xd = d_pose.pose.pose.position.x;
  float yd = d_pose.pose.pose.position.y;
  float wpx = wp_in[n].x;
  float wpy = wp_in[n].y;
  float angle = d_heading;
  ROS_INFO("xd:%.3f yd:%.3f wpx:%.3f wpy::%.3f angle:%.3f",xd,yd,wpx,wpy,angle);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node("~");
  init_publisher_subscriber(gnc_node);
  ros::Duration(10.0).sleep();
  
  wp_in = func_wplist();
  // ROS_INFO("First WP: %f,%f   Second WP: %f,%f", wp_in[0].x, wp_in[0].y, wp_in[1].x, wp_in[1].y);
  int wp_size;
  wp_size = wp_in.size();
  
  error_sub = gnc_node.subscribe<geometry_msgs::Point>("/gnc/pos_error",10, error_cb);
  // wp_sub = gnc_node.subscribe<geometry_msgs::Pose>("/gnc/goal", 10, nav_wp);
  // bool_pub = gnc_node.advertise<std_msgs::Bool>("/gnc/wpreach", 10);
 
  bool navigate = 1;
  wait4connect();
  set_mode("GUIDED");
  // ROS_INFO("Position (%.2f,%.2f) Heading %.3f",d_pose.pose.pose.position.x, d_pose.pose.pose.position.y, d_heading);
  ros::Duration(4.0).sleep();
  takeoff(3);
  ROS_INFO("%i waypoints, starting with Waypoint %i (x:%f y:%f z:%f)", wp_size, n,wp_in[n].x, wp_in[n].y, wp_in[n].z);
  ROS_INFO("Position (%.2f,%.2f) Heading %.3f",d_pose.pose.pose.position.x, d_pose.pose.pose.position.y, d_heading);
  pos_print();
  ros::Rate loop_rate(3);
  ros::Duration(7.0).sleep();
  while (ros::ok() && navigate)
  {
    pos_print();
    // ROS_INFO("Turn Rate: %.2f Velocity x:%f y:%f z:%f",cmd_twist.twist.angular.z, d_twist.twist.linear.x,d_twist.twist.linear.y,d_twist.twist.linear.z);
    if (n < wp_size)
    {
      // ROS_INFO("Position (%.2f,%.2f) Heading %.3f",d_pose.pose.pose.position.x, d_pose.pose.pose.position.y, d_heading);
      move(0.3); //Desired speed


      // cmd_twist.twist.angular.z = 0.4;
      // forward(0.5);
      // twist_pub.publish(cmd_twist);


      if (check_waypoint_reached(0.3))
      {
        ROS_INFO("Waypoint Reached, moving to waypoint %i",n);
        n++;
      }
    }else{
      land();
      ros::Duration(3.0).sleep();
      navigate = 0;
    }
    ros::spinOnce(); //set to spinonce
    loop_rate.sleep();
  }

  return 0;
}
