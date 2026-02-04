#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <string>
#include <vector>

static std::vector<int> digits(const std::string& s) {
  std::vector<int> d;
  for (char c : s) if (c >= '0' && c <= '9') d.push_back(c - '0');
  return d;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "nu_id_publisher");
  ros::NodeHandle nh("~");

  std::string TOPIC = "/akilbekov";   // <-- твоя фамилия
  std::string NU_ID = "211499";       // <-- твой NU ID

  nh.param("topic", TOPIC, TOPIC);
  nh.param("nu_id", NU_ID, NU_ID);

  double hz = 1.0;                   // <-- 1 Hz или 50 Hz
  nh.param("hz", hz, hz);

  auto d = digits(NU_ID);
  if (d.empty()) { ROS_ERROR("nu_id has no digits"); return 1; }

  ros::Publisher pub = nh.advertise<std_msgs::Int32>(TOPIC, 10);
  ros::Rate rate(hz);

  size_t i = 0;
  while (ros::ok()) {
    std_msgs::Int32 msg;
    msg.data = d[i];
    pub.publish(msg);
    ROS_INFO("Published %d on %s", msg.data, TOPIC.c_str());

    i = (i + 1) % d.size();

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

