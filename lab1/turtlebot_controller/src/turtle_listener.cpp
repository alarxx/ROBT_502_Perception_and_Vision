#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"

ros::Publisher pub;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg){
	ROS_INFO("Turtle subscriber@[%f, %f, %f]", msg->x, msg->y, msg->theta);
	geometry_msgs::Twist my_vel;
	my_vel.linear.x = 1.0;
	my_vel.angular.z = 1.0;
	pub.publish(my_vel);
}


int main (int argc, char **argv){
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "turtlebot_subscriber");
	ros::NodeHandle nh;

	// ---- Exercise 4 ----
	ros::service::waitForService("/spawn");
	ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");

	turtlesim::Spawn srv1;
	srv1.request.x = 1.0;
	srv1.request.y = 5.0;
	srv1.request.theta = 0.0;
	srv1.request.name = "Turtle_Alar";

	if (client1.call(srv1)) {
		ROS_INFO("Spawned: %s", srv1.response.name.c_str());
	} else {
		ROS_ERROR("Failed to call /spawn");
	}

	// ---- Exercise 3 ----
	pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

	// Define the subscriber to turtle's position
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1, turtleCallback);

	ros::spin();
	return 0;
}
