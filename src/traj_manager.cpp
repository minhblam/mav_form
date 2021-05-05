// Trajectory Manager
// Manages direct control of drones. This includes connection of telemetry and control of all drones and receives pathplanning commands from the formation manager.
#include <control_functions.hpp>

// Basic collision avoidance here


int main(int argc, char **argv)
{

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
