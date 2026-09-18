#include <ros/ros.h>
#include <ros1_hello/HelloStatus.h>

static void statusCallback(const ros1_hello::HelloStatus::ConstPtr& msg)
{
    ROS_INFO_STREAM("received: sequence=" << msg->sequence << " text=" << msg->text);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_msg_subscriber");

    ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("hello_status", 10, statusCallback);

    ros::spin();
    return 0;
}
