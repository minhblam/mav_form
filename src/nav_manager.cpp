// Navigation Manager
// Input primary waypoints to reach and modify to accomodate flight capabilities

// #include <iostream>
// #include <vector>
// #include <math.h>

#include <control_functions.hpp>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/Pose.h>

ros::Publisher wp_pub;
// geometry_msgs::PoseArray wp_posearray;
geometry_msgs::Pose wp_pose;

ros::Subscriber bool_sub;
std_msgs::Bool wp_update;

void wp_cb(const std_msgs::Bool::ConstPtr &msg)
{
	wp_update=*msg;
}

// using namespace std;

std::vector<gnc_WP> func_wplist() //Create a separate function for a list of waypoints
{
	std::vector<gnc_WP> wp_in;
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

std::vector<gnc_wppose> mod_wp(std::vector<gnc_WP> wp_in)
{
	std::vector<gnc_wppose> wp_out;
	gnc_wppose wp_list;

	for (int n = 0; n < wp_in.size(); n++)
	{
		wp_list.x = wp_in[n].x;
		wp_list.y = wp_in[n].y;
		wp_list.z = wp_in[n].z;
		wp_out.push_back(wp_list);

		float angle1 = atan((wp_in[n + 2].x - wp_in[n + 1].x) / (wp_in[n + 2].y - wp_in[n + 1].y)) * 180 / M_PI;
		float angle2 = atan((wp_in[n + 1].x - wp_in[n].x) / (wp_in[n + 1].y - wp_in[n].y)) * 180 / M_PI;
		float angle = angle1 - angle2;

		if (angle != 0)
		{
			float yaw = atan((wp_in[n + 2].x - wp_in[n + 1].x) / (wp_in[n + 2].y - wp_in[n + 1].y)) * 180 / M_PI;

			wp_list.qw = yaw_to_q(yaw).w;
			wp_list.qx = yaw_to_q(yaw).x;
			wp_list.qy = yaw_to_q(yaw).y;
			wp_list.qz = yaw_to_q(yaw).z;

			wp_out.push_back(wp_list);
		}
	}
}








void push_wp(std::vector<gnc_wppose> wp_in, int n)
{
	// int size_wp = func_wplist().size();

	wp_pose.position.x = wp_in[n].x;
	wp_pose.position.x = wp_in[n].y;
	wp_pose.position.x = wp_in[n].z;
	wp_pose.orientation.w = wp_in[n].qw;
	wp_pose.orientation.x = wp_in[n].qx;
	wp_pose.orientation.y = wp_in[n].qy;
	wp_pose.orientation.z = wp_in[n].qz;

	wp_pub.publish(wp_pose);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gnc_node");
	// ros::NodeHandle gnc_node("~");
	ros::NodeHandle gnc_node;
	//Something to make this run once for ROScore(until the function is completed)
	wp_pub = gnc_node.advertise<geometry_msgs::Pose>("/gnc/goal", 2);
	bool_sub = gnc_node.subscribe<std_msgs::Bool>("/gnc/wpreach",10,wp_cb);

	int n = 0;
	ros::Rate loop_rate(0.5);
	while (ros::ok())
	{
		std::vector<gnc_wppose> wp_in = mod_wp(func_wp_list());
		
		if (wp_update.data){ //something needs to be done about if limit is reached
			n++ 
			push_wp(wp_in, n);
		}
		
		push_wp(wp_in);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}