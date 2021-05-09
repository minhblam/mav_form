#include <iostream>
#include <vector>
#include <math.h>
#include <control_functions.hpp>

// Navigation Manager

// Input primary waypoints to reach and modify to accomodate flight capabilities



//Implementation
// #include <geometry_msgs/PoseStamped.h>

// ros::Publisher pose_pub;                  // Drone 2 State Control Must be from main variable pose_pub.publish(value)

// geometry_msgs::PoseStamped fp_pose;        // Probably Redundant Master Message


//Independent Test
#include <geometry_msgs/PoseArray.h>
ros::Publisher wp_pub;
geometry_msgs::PoseArray wp_pose;


using namespace std;

vector<gnc_WP> func_wplist (){
	vector<gnc_WP> wp_in;
	gnc_WP wp_list;
	wp_list.x = 0; //update this to 0,1
	wp_list.y = 0;
	wp_list.z = 2;
	wp_in.push_back(wp_list);
	wp_list.x = 3; //1
	wp_list.y = 1;
	wp_list.z = 2;
	wp_in.push_back(wp_list);
	wp_list.x = 9; //2
	wp_list.y = 3;
	wp_list.z = 2;
	wp_in.push_back(wp_list);
	wp_list.x = 9; //3
	wp_list.y = 5;
	wp_list.z = 2;
	wp_in.push_back(wp_list);
	wp_list.x = 9; //4
	wp_list.y = 7;
	wp_list.z = 2;
	wp_in.push_back(wp_list);
	wp_list.x = 5; //5
	wp_list.y = 9;
	wp_list.z = 2;
	wp_in.push_back(wp_list);

	return wp_in;
}

void push_wp (vector<gnc_WP> wp_in, int k){

	vector<gnc_WP>wp_out;
	gnc_WP newWP;
	for (int n=0 ; n<wp_in.size()-1 ; n++ ){
        
        newWP.x = wp_in[n].x;
        newWP.y = wp_in[n].y;
        newWP.z = wp_in[n].z;
        // wp_out.push_back(wp_in[n]); try using this
		wp_out.push_back(newWP);

		float angle1 = atan( (wp_in[n+2].x - wp_in[n+1].x)/(wp_in[n+2].y - wp_in[n+1].y) )*180/M_PI;
		float angle2= atan((wp_in[n+1].x - wp_in[n].x)/(wp_in[n+1].y - wp_in[n].y))*180/M_PI;
		float angle = angle1 - angle2;
		// cout <<"Angle WP" <<n+2 << "_" << n+1 << " is " << angle1 << " degrees, WP" << n+1 << "_" << n << " is " << angle2 << " degrees, difference is " << angle <<endl;
		
		if (angle != 0){
			newWP.psi = atan( (wp_in[n+2].x - wp_in[n+1].x)/(wp_in[n+2].y - wp_in[n+1].y) )*180/M_PI;
			wp_out.push_back(newWP);
		}
    }

	// fp_pose.pose.pose.x = wp_out[n].x;
	// fp_pose.pose.pose.y = wp_out[n].y;
	// fp_pose.pose.pose.z = wp_out[n].z;
	// pose_pub.publish(fp_pose);
	cout << wp_out[k].x << endl;

}


// new WP adds the base WP. formation creates the new offset WPs for drones to follow.
// The active WP is selected by nav manager and sent to form_control.

// In future do a set yaw rate to match whatever has the lowest limit




//*Main
// waypointlist = n.serviceCLient<mavros_msgs::WaypointList>"(/drone2/mavros/mission/waypoints");


int main (int argc, char**argv)
{
	//Replace int n with whatever was passed
	int n = 0;
	vector<gnc_WP> wp_in = func_wplist();
	push_wp(wp_in, n);
	wp_pub = n.advertise<geometry_msgs::PoseArray>("/wp_pre/setpoint_position/local",10);
	// pose_pub = n.advertise<geometry_msgs::PoseStamped>("/drone1/mavros/setpoint_position/local", 10); // For Built in setpoint WP control
	return 0;


}