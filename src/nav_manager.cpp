// Navigation Manager
// Input primary waypoints to reach and modify to accomodate flight capabilities

#include <gnc_functions.hpp>

ros::Publisher wp_pub;
geometry_msgs::Point wplist;

int i=0;

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

// void push_waypoints(std::vector<gnc_WP> wp_in)
// {
// 	int wp_size = wp_in.size();
	
// 	for (int i = 0;i < wp_size; i++)
// 	{
// 		wplist.x = wp_in[i].x;
// 		wplist.y = wp_in[i].y;
// 		wplist.z = wp_in[i].z;
// 		wp_pub.publish(wplist);
// 		ROS_INFO("Sending Waypoints");
// 	}
	
// }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	wp_pub = gnc_node.advertise<geometry_msgs::Point>("/gnc/goal", 10);

	std::vector<gnc_WP> wp_in = func_wplist();
	int wp_size = wp_in.size();
	ROS_INFO("Processing Waypoints %i",wp_size);
	

	ros::Rate loop_rate(2);

	// push_waypoints(wp_in);
	ros::Duration(5.0).sleep();
	bool publish = 1;
	while (ros::ok() && publish)
	{
		if (i < wp_size)
	{
		wplist.x = wp_in[i].x;
		wplist.y = wp_in[i].y;
		wplist.z = wp_in[i].z;
		wp_pub.publish(wplist);
		ROS_INFO("Sending Waypoint %i",i);
		i++;
	}else{
		publish = 0;
	}
	
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}