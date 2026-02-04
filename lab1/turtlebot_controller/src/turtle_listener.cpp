#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

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

	pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

	// Define the subscriber to turtle's position
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1, turtleCallback);

	ros::spin();
	return 0;
}
