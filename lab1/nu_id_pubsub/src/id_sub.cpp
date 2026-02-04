#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <string>

static void cb(const std_msgs::Int32::ConstPtr& msg) {
  ROS_INFO("Received digit: %d", msg->data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "nu_id_subscriber");
  ros::NodeHandle nh("~");

  std::string TOPIC = "/akilbekov";
  nh.param("topic", TOPIC, TOPIC);

  ros::Subscriber sub = nh.subscribe(TOPIC, 10, cb);
  ros::spin();
  return 0;
}
