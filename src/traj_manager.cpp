// Trajectory Manager
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <control_functions.hpp>

// Basic collision avoidance here


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
  init_publisher_subscriber(gnc_node, /drone1);
  init_publisher_subscriber(gnc_node, /drone2);
  init_publisher_subscriber(gnc_node, /drone3);

  wait4connect();
  set_mode("GUIDED");
  takeoff(5);



  ros::Rate loop_rate(2);
  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
