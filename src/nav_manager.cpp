#include <iostream>
#include <vector>


struct gnc_WP{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	// float psi; ///< rotation about the third axis of your reference frame
};

//new WP adds the base WP. formation creates the new offset WPs for drones to follow.
//The active WP is selected by nav manager and sent to form_control. form_control only requests the next WP until it has verified that all drones have reached the WP


int formation(std::string mode)
{
  //Assign drone number to formation number with loop oir some shit
  //Something count number of drones and create offset waypoint
  if(mode == "V"){
    ROSINFO("Formation set to %s", mode);
    //Do formation shit
  }
  if(mode == "line"){
    ROSINFO("Formation set to %s", mode);
    //Do formation shit
  }
}


void newWP (float something oneday){
  int k;
  for(k=0; k < size(waypointList); k++){ //Basically take existing WP, create a new array where angle between each line of WP is checked to see if a rotation WP is required.

    float newangle = (waypointList[n+1].y - waypointList[n].y) / (waypointList[n+1].x - waypointList[n].x)
    float curangle = (waypointList[n].y - waypointList[n-1].y) / (waypointList[n].x - waypointList[n-1].x)
    float diffangle = newangle - curangle;
  }
}

/// In future do a set yaw rate to match whatever has the lowest limit

int main (int argc, char**argv)
{

  formation("square");

  std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0; //update this to 0,1
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5; //1
	nextWayPoint.y = 1;
	nextWayPoint.z = 3;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 4; //2
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 4; //3
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5; //4
	nextWayPoint.y = 4;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0; //5
	nextWayPoint.y = 4;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 1; //6
	nextWayPoint.y = 0;
	nextWayPoint.z = 5;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 1; //7
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint); //8 WP?

  int k;
  for (k=0; k<8; k++){

  }


}