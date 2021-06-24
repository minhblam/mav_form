// Navigation Manager
// Input primary waypoints to reach and modify to accomodate flight capabilities

#include <gnc_functions.hpp>

#include <geometry_msgs/PoseArray.h>
geometry_msgs::PoseArray wp_posearray;
geometry_msgs::Pose wp_pose;

ros::Publisher wp_pub;
geometry_msgs::Point wplist;


std::vector<gnc_WP> func_wplist() //Create a separate function for a list of waypoints
{
	std::vector<gnc_WP> wp_in;
	gnc_WP wp_list;
	wp_list.x = 0;
	wp_list.y = 0;
	wp_list.z = 5;
	wp_in.push_back(wp_list);
	wp_list.x = 10;
	wp_list.y = 0;
	wp_list.z = 5;
	wp_in.push_back(wp_list);
	wp_list.x = 10;
	wp_list.y = 10;
	wp_list.z = 5;
	wp_in.push_back(wp_list);
	wp_list.x = 0;
	wp_list.y = 10;
	wp_list.z = 5;
	wp_in.push_back(wp_list);
	wp_list.x = 0;
	wp_list.y = 0;
	wp_list.z = 10;
	wp_in.push_back(wp_list);
	wp_list.x = -10;
	wp_list.y = -10;
	wp_list.z = 5;
	wp_in.push_back(wp_list);
	return wp_in;
}

void push_wp(std::vector<gnc_WP> wp_in)
{

	for (int n = 0; n < wp_in.size(); n++)
	{
		wp_pose.position.x = wp_in[n].x;
		wp_pose.position.y = wp_in[n].y;
		wp_pose.position.z = wp_in[n].z;
		wp_posearray.poses.push_back(wp_pose);
	}

	// cout << wp_out[k].x << endl;
	wp_pub.publish(wp_posearray);
	// ROS_INFO("Published initial waypoints x:%f y:%f z:%f",wp_pose.position.x,wp_pose.position.y,wp_pose.position.z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");

	wp_pub = gnc_node.advertise<geometry_msgs::PoseArray>("/gnc/goal", 2,true);

	std::vector<gnc_WP> wp_in = func_wplist();
	push_wp(wp_in);
	int size = wp_in.size();
	ROS_INFO("Published %i waypoints",size);

	ros::spin();
	return 0;
}