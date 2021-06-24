#include <gnc_functions.hpp>
#include <geometry_msgs/PoseArray.h>

ros::Subscriber wp_sub;
geometry_msgs::PoseArray wp_subpose; //Original Waypoint Array

int counter = 0;

void nav_wp(const geometry_msgs::PoseArray::ConstPtr &msg)
{
	wp_subpose = *msg;
	int size = wp_subpose.poses.size();
	ROS_INFO("Received %i Waypoints", size);
}

void set_destination(float x, float y, float z)
{
	cmd_pose.pose.position.x = x;
	cmd_pose.pose.position.y = y;
	cmd_pose.pose.position.z = z;
	pose_pub.publish(cmd_pose);
}

bool check_waypoint_reached(float pos_tolerance = 0.3)
{
	float deltaX = abs(wp_subpose.poses[counter].position.x - d_pose.pose.pose.position.x);
	float deltaY = abs(wp_subpose.poses[counter].position.y - d_pose.pose.pose.position.y);
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
	//initialize ros
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");

	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);
	wp_sub = gnc_node.subscribe<geometry_msgs::PoseArray>("/gnc/goal", 10, nav_wp);

	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	set_mode("GUIDED");

	//request takeoff
	takeoff(3);

	int wp_size = wp_subpose.poses.size();
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	
	bool navigate = 1;
	while (ros::ok() && navigate)
	{
		ros::spinOnce();
		rate.sleep();
		if (counter < wp_size)
		{
			set_destination(wp_subpose.poses[counter].position.x, wp_subpose.poses[counter].position.y, wp_subpose.poses[counter].position.z);
			// ROS_INFO("Moving To Waypoint (%.1f,%.1f,%.1f)",wp_subpose.poses[counter].position.x, wp_subpose.poses[counter].position.y, wp_subpose.poses[counter].position.z);
			if (check_waypoint_reached(0.3))
			{
			counter++;
			ROS_INFO("Waypoint Reached, moving to Waypoint %i (%.1f,%.1f,%.1f)",counter,wp_subpose.poses[counter].position.x,wp_subpose.poses[counter].position.y,wp_subpose.poses[counter].position.z);
			}
		}
		else
		{
			land();
			navigate = 0;
		}
	}
	return 0;
}