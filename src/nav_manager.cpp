// Navigation Manager
// Input primary waypoints to reach and modify to accomodate flight capabilities

#include <iostream>
#include <vector>
#include <math.h>

#include <control_functions.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

ros::Publisher wp_pub;
geometry_msgs::PoseArray wp_posearray;
geometry_msgs::Pose wp_pose;

using namespace std;

vector<gnc_WP> func_wplist()
{
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

void push_wp(vector<gnc_WP> wp_in)
{
	vector<gnc_WP> wp_out;
	gnc_WP newWP;

	for (int n = 0; n < wp_in.size() - 1; n++)
	{
		// // Old solution?
		// newWP.x = wp_in[n].x;
		// newWP.y = wp_in[n].y;
		// newWP.z = wp_in[n].z;
		// wp_out.push_back(newWP);
		// float angle1 = atan( (wp_in[n+2].x - wp_in[n+1].x)/(wp_in[n+2].y - wp_in[n+1].y) )*180/M_PI;
		// float angle2= atan((wp_in[n+1].x - wp_in[n].x)/(wp_in[n+1].y - wp_in[n].y))*180/M_PI;
		// float angle = angle1 - angle2;
		// // cout <<"Angle WP" <<n+2 << "_" << n+1 << " is " << angle1 << " degrees, WP" << n+1 << "_" << n << " is " << angle2 << " degrees, difference is " << angle <<endl;
		// if (angle != 0){
		// 	newWP.psi = atan( (wp_in[n+2].x - wp_in[n+1].x)/(wp_in[n+2].y - wp_in[n+1].y) )*180/M_PI;
		// 	wp_out.push_back(newWP);
		// }

		//new solution?
		wp_pose.position.x = wp_in[n].x;
		wp_pose.position.y = wp_in[n].y;
		wp_pose.position.z = wp_in[n].z;
		wp_posearray.poses.push_back(wp_pose);
		float angle1 = atan((wp_in[n + 2].x - wp_in[n + 1].x) / (wp_in[n + 2].y - wp_in[n + 1].y)) * 180 / M_PI;
		float angle2 = atan((wp_in[n + 1].x - wp_in[n].x) / (wp_in[n + 1].y - wp_in[n].y)) * 180 / M_PI;
		float angle = angle1 - angle2;

		if (angle != 0)
		{
			float yaw = atan((wp_in[n + 2].x - wp_in[n + 1].x) / (wp_in[n + 2].y - wp_in[n + 1].y)) * 180 / M_PI;
			float pitch = 0;
			float roll = 0;

			float cy = cos(yaw * 0.5);
			float sy = sin(yaw * 0.5);
			float cr = cos(roll * 0.5);
			float sr = sin(roll * 0.5);
			float cp = cos(pitch * 0.5);
			float sp = sin(pitch * 0.5);

			float qw = cy * cr * cp + sy * sr * sp;
			float qx = cy * sr * cp - sy * cr * sp;
			float qy = cy * cr * sp + sy * sr * cp;
			float qz = sy * cr * cp - cy * sr * sp;

			wp_pose.orientation.w = qw;
			wp_pose.orientation.x = qx;
			wp_pose.orientation.y = qy;
			wp_pose.orientation.z = qz;

			wp_posearray.poses.push_back(wp_pose);
		}
	}

	// cout << wp_out[k].x << endl;
	wp_pub.publish(wp_posearray);
}

// new WP adds the base WP. formation creates the new offset WPs for drones to follow.
// The active WP is selected by nav manager and sent to form_control.

// In future do a set yaw rate to match whatever has the lowest limit

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	//Something to make this run once for ROScore(until the function is completed)
	vector<gnc_WP> wp_in = func_wplist();
	push_wp(wp_in);
	wp_pub = gnc_node.advertise<geometry_msgs::PoseArray>("/gnc/goal", 10);
	// pose_pub = n.advertise<geometry_msgs::PoseStamped>("/drone1/mavros/setpoint_position/local", 10); // For Built in setpoint WP control
	return 0;
}