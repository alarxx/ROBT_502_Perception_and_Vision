#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

#include "turtlesim/Kill.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/TeleportAbsolute.h"

turtlesim::Pose current_pose;

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    current_pose = *msg;
}

void move(ros::Publisher &pub, double lin, double ang, double time)
{
    geometry_msgs::Twist vel;
    vel.linear.x = lin;
    vel.angular.z = ang;

    ros::Rate r(10);
    int ticks = time * 10;

    for(int i = 0; i < ticks; i++)
    {
        pub.publish(vel);
        r.sleep();
    }

    // stop
    vel.linear.x = 0;
    vel.angular.z = 0;
    pub.publish(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_controller");
    ros::NodeHandle nh;

    std::string my_turtle = "turtle_Alar";

    ros::service::waitForService("/kill");
    ros::ServiceClient kill_client =
        nh.serviceClient<turtlesim::Kill>("/kill");

    turtlesim::Kill kill_srv;
    kill_srv.request.name = "turtle1";
    kill_client.call(kill_srv);

    ros::service::waitForService("/spawn");
    ros::ServiceClient spawn_client =
        nh.serviceClient<turtlesim::Spawn>("/spawn");

    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.5;
    spawn_srv.request.y = 5.5;
    spawn_srv.request.theta = 0;
    spawn_srv.request.name = my_turtle;

    spawn_client.call(spawn_srv);

    std::string pose_topic = "/" + my_turtle + "/pose";
    std::string cmd_topic  = "/" + my_turtle + "/cmd_vel";
    std::string tp_topic   = "/" + my_turtle + "/teleport_absolute";

    ros::Subscriber sub =
        nh.subscribe(pose_topic, 10, poseCallback);

    ros::Publisher pub =
        nh.advertise<geometry_msgs::Twist>(cmd_topic, 10);

    ros::service::waitForService(tp_topic);
    ros::ServiceClient tp_client =
        nh.serviceClient<turtlesim::TeleportAbsolute>(tp_topic);

    turtlesim::TeleportAbsolute tp;
    tp.request.x = 1;
    tp.request.y = 1;
    tp.request.theta = 0;
    tp_client.call(tp);

    ros::Duration(1).sleep();

    ROS_INFO("SQUARE");

    for(int i = 0; i < 4; i++)
    {
        move(pub, 2.0, 0.0, 3.0);   // forward
        move(pub, 0.0, 1.57, 1.0);  // turn 90 deg
    }

    ROS_INFO("TRIANGLE");

    tp_client.call(tp);
    ros::Duration(1).sleep();

    for(int i = 0; i < 3; i++)
    {
        move(pub, 2.0, 0.0, 4.0);
        move(pub, 0.0, 2.09, 1.0);  // 120
    }

    ROS_INFO("FINISHED");

    return 0;
}
